"""Recording defaults and episode numbering."""
import argparse
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_OUTPUT = "~/Datasets/tianji-wuji2"


def validate_output(value):
    """Keep the path unexpanded: the Jetson writer resolves it on its host."""
    if not isinstance(value, str) or not value.strip():
        raise ValueError("output must be a nonempty directory path")
    return value.strip()


def config_path():
    source = Path(__file__).resolve().parents[1]
    if (source / "package.xml").is_file():
        return source / "config" / "recording.yaml"
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("data_collection")) / "config" / "recording.yaml"


@dataclass(frozen=True)
class Settings:
    task: str
    start: int
    end: int | None
    num: int | None
    output: str = DEFAULT_OUTPUT

    @property
    def last_episode(self):
        return self.end if self.end is not None else (
            self.start + self.num - 1 if self.num is not None else None)


def parse_settings(args=None, path=None):
    parser = argparse.ArgumentParser(description="Record continuous LeRobot v2.1 episodes; teleoperation must already be running.")
    parser.add_argument("--start", type=int, help="First episode index; default from recording.yaml")
    limits = parser.add_mutually_exclusive_group()
    limits.add_argument("--end", type=int, help="Inclusive final episode index")
    limits.add_argument("--num", type=int, help="Number of episodes to save")
    options = parser.parse_args(args)
    filename = Path(path) if path is not None else config_path()
    try:
        with filename.open() as stream:
            config = yaml.safe_load(stream)
        if not isinstance(config, dict):
            raise ValueError("recording configuration must be a mapping")
        unknown = set(config) - {"task", "start", "end", "num", "output"}
        if unknown:
            raise ValueError(f"unknown recording settings: {', '.join(sorted(unknown))}")
        values = {"start": 0, "end": None, "num": None, "output": DEFAULT_OUTPUT, **config}
        output = validate_output(values["output"])
        for key in ("start", "end", "num"):
            if getattr(options, key) is not None:
                values[key] = getattr(options, key)
        for key in ("start", "end", "num"):
            if values[key] is not None and type(values[key]) is not int:
                raise ValueError(f"{key} must be an integer")
        if not isinstance(values.get("task"), str) or not values["task"].strip():
            raise ValueError(f"set a nonempty task in {filename}")
        if values["start"] is None or values["start"] < 0:
            raise ValueError("start must be >= 0")
        if values["end"] is not None and values["num"] is not None:
            raise ValueError("end and num are mutually exclusive, including configuration defaults")
        if values["end"] is not None and values["end"] < values["start"]:
            raise ValueError("end must be >= start")
        if values["num"] is not None and values["num"] <= 0:
            raise ValueError("num must be > 0")
        return Settings(values["task"].strip(), values["start"], values["end"], values["num"], output)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        parser.error(str(exc))
