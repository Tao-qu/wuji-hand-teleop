#!/usr/bin/env python3
"""Create local hardware YAML files without overwriting existing settings."""
from pathlib import Path
import shutil


def configure(root):
    created = []
    for template in sorted((root / "src").rglob("*.yaml.template")):
        destination = template.with_suffix("")
        if not destination.exists():
            shutil.copyfile(template, destination)
            created.append(destination)
    return created


if __name__ == "__main__":
    for path in configure(Path(__file__).resolve().parents[1]):
        print(f"Created {path}")
