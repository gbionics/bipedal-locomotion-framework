#!/usr/bin/env python3

# Copyright (C) 2026 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

import argparse
import subprocess
from pathlib import Path

from git import Repo


def run(command: list[str], cwd: Path | None = None) -> None:
    subprocess.check_call(command, cwd=str(cwd) if cwd is not None else None)


def ensure_doxygen_awesome(docs_dir: Path, repo_url: str, ref: str) -> Path:
    awesome_dir = docs_dir.parent / "doxygen-awesome-css"

    if not awesome_dir.exists():
        repo = Repo.clone_from(repo_url, str(awesome_dir))
    else:
        repo = Repo(str(awesome_dir))
        repo.remotes.origin.fetch(tags=True)

    repo.git.checkout(ref)
    return awesome_dir


def main() -> None:
    parser = argparse.ArgumentParser(description="Clone/update doxygen-awesome-css and build docs.")
    parser.add_argument("--doxygen-awesome-ref", default="v2.4.2", help="Tag/branch/commit to checkout.")
    parser.add_argument("--doxygen-awesome-repo",
                        default="https://github.com/jothepro/doxygen-awesome-css.git",
                        help="Git URL for doxygen-awesome-css.")
    args = parser.parse_args()

    docs_dir = Path(__file__).resolve().parent
    ensure_doxygen_awesome(docs_dir, args.doxygen_awesome_repo, args.doxygen_awesome_ref)

    run(["doxygen", "Doxyfile"], cwd=docs_dir)


if __name__ == "__main__":
    main()
