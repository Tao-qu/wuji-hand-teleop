"""Desktop keyboard session; controls recording only."""
from contextlib import contextmanager
import json
import select
import sys
import termios
import threading
import time
import tty

from .configuration import parse_settings


@contextmanager
def keyboard():
    if not sys.stdin.isatty():
        raise ValueError("record requires an interactive terminal for x / s / r / q")
    original = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        yield
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)


class Session:
    def __init__(self, node, settings):
        self.node = node
        self.settings = settings
        self.episode = settings.start
        self.state = "ready"
        self.owns_candidate = False
        self.failed_candidate = False

    def command(self, command, payload=None):
        from data_collection_interfaces.srv import SessionCommand
        request = SessionCommand.Request()
        request.command = command
        request.payload_json = json.dumps(payload or {})
        future = self.node.client.call_async(request)
        started = time.monotonic()
        interrupted = False
        while not future.done():
            try:
                if not self.node.context.ok():
                    raise RuntimeError("ROS context closed while waiting for the writer")
                if command not in ("stop", "save") and time.monotonic() - started > 30:
                    self.node.client.remove_pending_request(future)
                    raise RuntimeError(f"writer did not answer {command} within 30 seconds")
                time.sleep(0.02)
            except KeyboardInterrupt:
                if command not in ("start", "stop", "save", "discard"):
                    raise
                # Finish the requested mutation before deciding what cleanup owns.
                interrupted = True
        response = future.result()
        if response is None:
            raise RuntimeError("writer service returned no response")
        result = json.loads(response.message_json)
        if response.success and command == "start":
            self.owns_candidate = True
        elif response.success and command in ("save", "discard"):
            self.owns_candidate = False
        if (result.get("state") == "error" and self.owns_candidate
                and self.state in ("recording", "pending")):
            self.failed_candidate = True
        if interrupted:
            raise KeyboardInterrupt
        if not response.success:
            raise ValueError(result.get("error") or result.get("message") or str(result))
        return result

    def initialize(self):
        if not self.node.client.wait_for_service(timeout_sec=8.0):
            raise ValueError("start `ros2 run data_collection writer` on the Jetson; /data_collection/command is unavailable")
        status = self.command("status")
        if status["state"] != "idle":
            raise ValueError(f"writer already has a session ({status['state']}); finish it before starting another recorder")
        status = self.command("configure", {"output": self.settings.output})
        if self.episode != status["next_episode_index"]:
            raise ValueError(f"--start must be {status['next_episode_index']} for this dataset (contiguous LeRobot episode indices)")
        print(f"Dataset on Jetson: {status['output']}\nTask: {self.settings.task}", flush=True)
        self.prompt()

    def prompt(self):
        choices = {"ready": "x: start", "recording": "x: stop",
                   "pending": "s: save and continue | r: discard and retry | q: save and exit"}
        if self.state == "pending" and self.failed_candidate:
            choices["pending"] = "r: discard and retry | Ctrl-C: discard and exit"
        print(f"Episode {self.episode} [{self.state.upper()}] {choices[self.state]}", flush=True)

    def stop(self):
        count = self.node.stop_recording()
        self.state = "pending"
        print("Finishing captured frames...", flush=True)
        try:
            self.command("stop", {"frame_count": count})
            print(f"Captured {count} frames ({count / 30:.2f} s).", flush=True)
        except Exception:
            self.failed_candidate = True
            raise
        finally:
            self.prompt()

    def key(self, key):
        """Return True only after a successful save that ends the session."""
        if key == "x" and self.state == "ready":
            self.node.readiness()
            self.command("start", {"episode_index": self.episode, "task": self.settings.task,
                                   "output": self.settings.output})
            self.owns_candidate = True
            self.failed_candidate = False
            self.node.start_recording()
            self.state = "recording"
            self.prompt()
        elif key == "x" and self.state == "recording":
            self.stop()
        elif key == "r" and self.state == "pending":
            self.command("discard")
            self.owns_candidate = False
            self.failed_candidate = False
            self.state = "ready"
            self.prompt()
        elif key in ("s", "q") and self.state == "pending":
            if self.failed_candidate:
                print("This recording failed and cannot be saved. Press r to discard and retry, or Ctrl-C to discard and exit.", flush=True)
                return False
            print("Saving episode on Jetson...", flush=True)
            try:
                result = self.command("save")
            except ValueError:
                if self.failed_candidate:
                    self.prompt()
                raise
            self.owns_candidate = False
            print(f"Saved episode {self.episode}: {result.get('saved_episode', result)}", flush=True)
            if key == "q" or self.episode == self.settings.last_episode:
                return True
            self.episode += 1
            self.state = "ready"
            self.prompt()
        elif key in ("s", "r", "q", "x"):
            print(f"'{key}' is not available in {self.state.upper()}.", flush=True)
        return False

    def check_recording(self):
        if self.state != "recording":
            return
        error = self.node.recording_error
        status = self.command("status")
        if status["state"] == "error":
            self.node.stop_recording()
            self.state = "pending"
            self.failed_candidate = True
            print(f"Recording error: {status.get('error')}. Press r to discard and retry.", flush=True)
            self.prompt()
        elif error:
            print(f"Robot data interrupted: {error}", flush=True)
            self.stop()

    def abort(self):
        count = self.node.stop_recording()
        if self.owns_candidate:
            status = self.command("status")
            if status["state"] == "recording":
                try:
                    self.command("stop", {"frame_count": count})
                except ValueError:
                    # A zero-frame or incomplete candidate can still be discarded.
                    pass
                status = self.command("status")
            if status["state"] in ("pending", "error"):
                self.command("discard")
                print("Unsaved candidate discarded. Teleoperation continues.", flush=True)
            elif status["state"] in ("saving", "stopping"):
                print("Writer is finishing the requested operation on Jetson.", flush=True)
            self.owns_candidate = False


def main(args=None):
    # Parse --help before loading ROS interfaces or the desktop FK library.
    settings = parse_settings(args)
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from .robot import RobotRecorder
    node = executor = session = thread = None
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        with keyboard():
            node = RobotRecorder()
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            def spin():
                try:
                    executor.spin()
                except ExternalShutdownException:
                    pass

            thread = threading.Thread(target=spin, daemon=True)
            thread.start()
            session = Session(node, settings)
            session.initialize()
            checked = time.monotonic()
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    if not key or key == "\x04":
                        break
                    try:
                        if session.key(key):
                            break
                    except ValueError as exc:
                        print(f"Recording: {exc}", flush=True)
                if time.monotonic() - checked >= 1.0:
                    try:
                        session.check_recording()
                    except ValueError as exc:
                        print(f"Recording: {exc}", flush=True)
                    checked = time.monotonic()
    except KeyboardInterrupt:
        print("\nRecording interrupted.", flush=True)
    except (ValueError, RuntimeError, OSError) as exc:
        print(f"Recording: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if session is not None and session.owns_candidate:
            try:
                session.abort()
            except Exception as exc:
                print(f"Writer cleanup incomplete: {exc}", file=sys.stderr, flush=True)
        if executor is not None:
            executor.shutdown(timeout_sec=3)
        if thread is not None:
            thread.join(timeout=3)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
