#!/usr/bin/env python3
"""Render legacy Nav2 params without changing any navigation or recovery policy."""
import argparse
import json
import re
from pathlib import Path
import xml.etree.ElementTree as ET

WS = Path(__file__).resolve().parents[1]
KEY = re.compile(r"^(\s*default_nav_to_pose_bt_xml:\s*).*$", re.MULTILINE)
PLACEHOLDER = "behavior_trees/competition_nav_to_pose.xml"


def render(source: Path, behavior_tree: Path, output: Path):
    tree = behavior_tree.resolve(strict=True)
    if not tree.is_file() or ET.parse(tree).getroot().tag != "root":
        raise ValueError("expected Nav2 behavior-tree XML")
    text = source.read_text(encoding="utf-8")
    matches = list(KEY.finditer(text))
    if len(matches) != 1 or not matches[0].group().split(":", 1)[1].strip().split(" #", 1)[0] == PLACEHOLDER:
        raise ValueError("expected one portable competition behavior tree entry")
    text = KEY.sub(lambda m: m.group(1) + json.dumps(str(tree), ensure_ascii=False), text)
    # Output must be new, never overwrite source or previous generated config.
    with output.open("x", encoding="utf-8") as stream:
        stream.write(text)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--params", type=Path, default=WS / "config/nav2_competition_params.yaml")
    p.add_argument("--behavior-tree", type=Path, default=WS / PLACEHOLDER)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    try:
        render(a.params, a.behavior_tree, a.output)
    except (OSError, ValueError, ET.ParseError) as exc:
        p.exit(1, "ERROR: " + str(exc) + "\n")
    print(a.output.resolve())


if __name__ == "__main__":
    main()
