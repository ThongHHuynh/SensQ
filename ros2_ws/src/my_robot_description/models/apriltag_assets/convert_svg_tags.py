#!/usr/bin/env python3
"""Convert the resized grid-based AprilTag SVGs into crisp RGB PNGs."""

from __future__ import annotations

import argparse
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from PIL import Image, ImageDraw


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_INPUT_DIR = SCRIPT_DIR.parents[1] / "worlds" / "tag36h11-resize"


def parse_color(value: str) -> tuple[int, int, int]:
    numbers = re.findall(r"[\d.]+", value)
    if value.startswith(("rgb(", "rgba(")) and len(numbers) >= 3:
        return tuple(round(float(channel)) for channel in numbers[:3])
    if value.startswith("#") and len(value) in (4, 7):
        value = value.lstrip("#")
        if len(value) == 3:
            value = "".join(channel * 2 for channel in value)
        return tuple(int(value[index : index + 2], 16) for index in (0, 2, 4))
    raise ValueError(f"Unsupported SVG fill color: {value}")


def convert(svg_path: Path, output_dir: Path, size: int) -> Path:
    root = ET.parse(svg_path).getroot()
    view_box = root.attrib.get("viewBox")
    if not view_box:
        raise ValueError(f"{svg_path}: missing viewBox")

    min_x, min_y, width, height = map(float, re.split(r"[\s,]+", view_box))
    if width <= 0 or height <= 0:
        raise ValueError(f"{svg_path}: invalid viewBox")

    image = Image.new("RGB", (size, size), "white")
    draw = ImageDraw.Draw(image)
    rectangle_count = 0

    for element in root.iter():
        if element.tag.rsplit("}", 1)[-1] != "rect":
            continue

        x = float(element.attrib.get("x", 0))
        y = float(element.attrib.get("y", 0))
        rect_width = float(element.attrib["width"])
        rect_height = float(element.attrib["height"])
        fill = parse_color(element.attrib.get("fill", "#000000"))

        left = round((x - min_x) * size / width)
        top = round((y - min_y) * size / height)
        right = round((x + rect_width - min_x) * size / width)
        bottom = round((y + rect_height - min_y) * size / height)
        draw.rectangle((left, top, right - 1, bottom - 1), fill=fill)
        rectangle_count += 1

    if rectangle_count == 0:
        raise ValueError(f"{svg_path}: no rectangles found")

    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"{svg_path.stem}_rgb.png"
    image.save(output_path, format="PNG")
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "inputs",
        nargs="*",
        type=Path,
        help=f"SVG files; defaults to every SVG in {DEFAULT_INPUT_DIR}",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=SCRIPT_DIR,
        help="PNG destination (default: directory containing this script)",
    )
    parser.add_argument(
        "--size",
        type=int,
        default=1000,
        help="Square output size in pixels (default: 1000)",
    )
    args = parser.parse_args()

    if args.size <= 0:
        parser.error("--size must be positive")

    inputs = args.inputs or sorted(DEFAULT_INPUT_DIR.glob("*.svg"))
    if not inputs:
        parser.error("no SVG input files found")

    for svg_path in inputs:
        output_path = convert(svg_path.resolve(), args.output_dir.resolve(), args.size)
        print(f"{svg_path} -> {output_path}")


if __name__ == "__main__":
    main()
