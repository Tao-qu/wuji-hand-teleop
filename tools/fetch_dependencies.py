#!/usr/bin/env python3
"""Fetch exact external source revisions; never reset an existing checkout."""
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def run(*args, cwd=None):
    return subprocess.check_output(args, cwd=cwd, text=True).strip()


def main():
    lock = json.loads((ROOT / "dependencies.lock.json").read_text())
    for name, spec in lock.items():
        path = ROOT / "src" / name
        if path.exists():
            if run("git", "rev-parse", "HEAD", cwd=path) != spec["commit"]:
                raise SystemExit(f"{path}: revision differs from lock; inspect it before updating")
            if run("git", "status", "--porcelain", cwd=path):
                raise SystemExit(f"{path}: local changes exist; inspect them before fetching")
        else:
            run("git", "clone", "--filter=blob:none", "--no-checkout", spec["url"], str(path))
            if spec.get("sparse_paths"):
                run("git", "sparse-checkout", "init", "--cone", cwd=path)
                run("git", "sparse-checkout", "set", *spec["sparse_paths"], cwd=path)
            run("git", "checkout", "--detach", spec["commit"], cwd=path)
        if spec.get("submodule"):
            run("git", "submodule", "update", "--init", "--recursive", "--", spec["submodule"], cwd=path)
        print(f"{name}: {spec['commit']}")


if __name__ == "__main__":
    main()
