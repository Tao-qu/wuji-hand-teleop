"""Publishes observations only. Never calls robot or recording interfaces."""
import json
import fcntl
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

from .gestures import Gestures, KEYS


class PedalNode(Node):
    def __init__(self):
        super().__init__('foot_pedal')
        for name, value in [('device', ''), ('vendor_id', 0x3553),
                            ('product_id', 0xb001), ('exclusive', True),
                            ('simulate', False)]:
            self.declare_parameter(name, value)
        self.gestures = Gestures()
        self.simulate = self.get_parameter('simulate').value
        self.device = None
        self.last_status = None
        self.retry_at = 0.0
        self.sequence = 0
        self.pending = []
        self.pub = self.create_publisher(String, '~/events', QoSProfile(
            depth=32, durability=DurabilityPolicy.VOLATILE))
        self.status_pub = self.create_publisher(String, '~/status', 10)
        if self.simulate:
            self.create_subscription(String, '~/simulate', self.inject, 10)
            self.status('simulation: physical device is not opened')
        else:
            import evdev
            self.evdev = evdev
        self.create_timer(0.01, self.poll)
        self.create_timer(1.0, self.heartbeat)

    def status(self, value):
        if value != self.last_status:
            self.get_logger().info(value)
        self.last_status = value
        self.heartbeat()

    def heartbeat(self):
        if self.last_status:
            self.status_pub.publish(String(data=self.last_status))

    def emit(self, code, value, timestamp):
        for event in self.gestures.feed(code, value, timestamp):
            if not self.simulate and (event['event'] != 'press' or self.pending):
                # Linux may synthesize key-up just before ENODEV on unplug.
                self.pending.append((time.monotonic() + 0.1, event))
            else:
                self.publish_event(event)

    def publish_event(self, event):
        self.sequence += 1
        event.update(sequence=self.sequence,
                     source='simulation' if self.simulate else 'device')
        payload = json.dumps(event)
        self.pub.publish(String(data=payload))
        self.get_logger().info(payload)

    def inject(self, message):
        try:
            data = json.loads(message.data)
            code = {'A': 30, 'B': 48, 'C': 46}[data['key']]
            value = data['value']
            if type(value) is not int or value not in (0, 1, 2):
                raise ValueError('value must be 0, 1, or 2')
            self.emit(code, value, time.monotonic())
        except (ValueError, KeyError, TypeError) as error:
            self.get_logger().warning(f'Invalid simulation event: {error}')

    def connect(self):
        requested = self.get_parameter('device').value
        paths = [requested] if requested else self.evdev.list_devices()
        matches, denied = [], []
        for path in paths:
            candidate = None
            try:
                candidate = self.evdev.InputDevice(path)
                if (candidate.info.vendor == self.get_parameter('vendor_id').value
                        and candidate.info.product == self.get_parameter('product_id').value
                        and set(KEYS).issubset(candidate.capabilities().get(1, []))):
                    matches.append(candidate)
                    candidate = None
            except OSError as error:
                if requested or isinstance(error, PermissionError):
                    denied.append(f'{path}: {error.strerror}')
            finally:
                if candidate is not None:
                    candidate.close()
        if len(matches) != 1:
            for candidate in matches:
                candidate.close()
            detail = '; check USB connection and udev access rules' if not matches else '; specify device path'
            self.status(f'disconnected: found {len(matches)} matching keyboards{detail}')
            return
        candidate = matches[0]
        try:
            # Use kernel monotonic event timestamps, including events read in batches.
            # Linux EVIOCSCLOCKID = _IOW('E', 0xa0, int). python-evdev
            # does not expose this ioctl as an InputDevice method.
            fcntl.ioctl(candidate.fd, 0x400445A0, struct.pack('i', time.CLOCK_MONOTONIC))
            if self.get_parameter('exclusive').value:
                candidate.grab()
            self.gestures.reset(candidate.active_keys())
        except OSError as error:
            candidate.close()
            self.status(f'disconnected: cannot initialize/grab device: {error}')
            return
        self.device = candidate
        self.status(f'connected: {candidate.path} {candidate.name}; exclusive='
                    f'{self.get_parameter("exclusive").value}')

    def disconnect(self, reason):
        if self.device:
            self.device.close()  # Also releases exclusive grab.
            self.device = None
        self.gestures.reset()
        self.pending.clear()
        self.retry_at = time.monotonic() + 1.0
        self.status(f'disconnected: {reason}; pending gestures cancelled')

    def poll(self):
        if self.simulate:
            return
        if self.device is None:
            if time.monotonic() >= self.retry_at:
                self.retry_at = time.monotonic() + 2.0
                self.connect()
            return
        try:
            for event in self.device.read():
                if event.type == 0 and event.code == 3:  # SYN_DROPPED
                    self.disconnect('input queue overflow')
                    return
                if event.type == 1:
                    self.emit(event.code, event.value, event.timestamp())
        except BlockingIOError:
            pass
        except OSError as error:
            self.disconnect(str(error))
        # Read first: a disconnect cancels buffered synthetic releases before publication.
        now = time.monotonic()
        while self.device is not None and self.pending and self.pending[0][0] <= now:
            _, event = self.pending.pop(0)
            self.publish_event(event)

    def destroy_node(self):
        if self.device:
            self.device.close()
            self.device = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PedalNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
