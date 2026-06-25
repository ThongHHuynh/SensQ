#!/usr/bin/env python3

import argparse
from collections import deque
from pathlib import Path


OCCUPIED_VALUE = 0
UNKNOWN_VALUE = 205
FREE_VALUE = 254


def read_map_yaml(path):
    values = {}
    lines = path.read_text(encoding="utf-8").splitlines()
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        values[key.strip()] = value.strip()
    if "image" not in values:
        raise ValueError(f"{path} does not contain an image entry")
    return values, lines


def replace_yaml_value(lines, key, value):
    prefix = f"{key}:"
    for index, line in enumerate(lines):
        if line.strip().startswith(prefix):
            indent = line[: len(line) - len(line.lstrip())]
            lines[index] = f"{indent}{key}: {value}"
            return lines
    lines.append(f"{key}: {value}")
    return lines


def _next_token(data, index):
    length = len(data)
    while index < length:
        byte = data[index]
        if byte == ord("#"):
            while index < length and data[index] not in (10, 13):
                index += 1
        elif chr(byte).isspace():
            index += 1
        else:
            break

    start = index
    while index < length and not chr(data[index]).isspace():
        index += 1
    return data[start:index].decode("ascii"), index


def read_pgm(path):
    data = path.read_bytes()
    magic, index = _next_token(data, 0)
    if magic not in ("P2", "P5"):
        raise ValueError(f"{path} is not a PGM file")

    width_token, index = _next_token(data, index)
    height_token, index = _next_token(data, index)
    max_value_token, index = _next_token(data, index)
    width = int(width_token)
    height = int(height_token)
    max_value = int(max_value_token)
    if max_value > 255:
        raise ValueError("Only 8-bit PGM files are supported")

    if magic == "P5":
        while index < len(data) and chr(data[index]).isspace():
            index += 1
        pixels = bytearray(data[index : index + width * height])
    else:
        pixels = bytearray()
        for _ in range(width * height):
            token, index = _next_token(data, index)
            pixels.append(int(token))

    if len(pixels) != width * height:
        raise ValueError(f"{path} pixel data is incomplete")
    return width, height, pixels


def write_pgm(path, width, height, pixels):
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    path.write_bytes(header + bytes(pixels))


def neighbors(index, width, height):
    x = index % width
    y = index // width
    if x > 0:
        yield index - 1
    if x + 1 < width:
        yield index + 1
    if y > 0:
        yield index - width
    if y + 1 < height:
        yield index + width


def component_fill(mask, width, height):
    seen = bytearray(len(mask))
    for start, is_target in enumerate(mask):
        if not is_target or seen[start]:
            continue
        queue = deque([start])
        seen[start] = 1
        component = []
        while queue:
            current = queue.popleft()
            component.append(current)
            for candidate in neighbors(current, width, height):
                if mask[candidate] and not seen[candidate]:
                    seen[candidate] = 1
                    queue.append(candidate)
        yield component


def majority_neighbor_value(component, pixels, width, height):
    component_set = set(component)
    counts = {FREE_VALUE: 0, UNKNOWN_VALUE: 0, OCCUPIED_VALUE: 0}
    for index in component:
        for candidate in neighbors(index, width, height):
            if candidate in component_set:
                continue
            value = pixels[candidate]
            if value <= 100:
                counts[OCCUPIED_VALUE] += 1
            elif value >= 240:
                counts[FREE_VALUE] += 1
            else:
                counts[UNKNOWN_VALUE] += 1
    return max(counts, key=counts.get)


def clean_pixels(pixels, width, height, min_occupied_pixels, min_free_pixels):
    output = bytearray(pixels)
    occupied = bytearray(1 if value <= 100 else 0 for value in output)
    free = bytearray(1 if value >= 240 else 0 for value in output)
    removed_occupied = 0
    filled_free = 0

    if min_occupied_pixels > 1:
        for component in component_fill(occupied, width, height):
            if len(component) < min_occupied_pixels:
                replacement = majority_neighbor_value(component, output, width, height)
                for index in component:
                    output[index] = replacement
                removed_occupied += len(component)

    if min_free_pixels > 1:
        for component in component_fill(free, width, height):
            if len(component) < min_free_pixels:
                replacement = majority_neighbor_value(component, output, width, height)
                for index in component:
                    output[index] = replacement
                filled_free += len(component)

    return output, removed_occupied, filled_free


def default_output_path(input_yaml):
    return input_yaml.with_name(f"{input_yaml.stem}_cleaned.yaml")


def main():
    parser = argparse.ArgumentParser(description="Clean small specks from a Nav2 PGM occupancy map.")
    parser.add_argument("map_yaml", type=Path, help="Input Nav2 map YAML.")
    parser.add_argument("-o", "--output", type=Path, help="Output YAML path.")
    parser.add_argument("--min-occupied-pixels", type=int, default=4, help="Remove occupied blobs smaller than this.")
    parser.add_argument("--min-free-pixels", type=int, default=0, help="Fill free blobs smaller than this; 0 disables it.")
    args = parser.parse_args()

    input_yaml = args.map_yaml.resolve()
    output_yaml = (args.output or default_output_path(input_yaml)).resolve()
    values, yaml_lines = read_map_yaml(input_yaml)
    input_image = (input_yaml.parent / values["image"]).resolve()
    output_image = output_yaml.with_suffix(".pgm")

    width, height, pixels = read_pgm(input_image)
    cleaned, removed_occupied, filled_free = clean_pixels(
        pixels,
        width,
        height,
        args.min_occupied_pixels,
        args.min_free_pixels,
    )

    output_yaml.parent.mkdir(parents=True, exist_ok=True)
    write_pgm(output_image, width, height, cleaned)
    yaml_lines = replace_yaml_value(yaml_lines, "image", output_image.name)
    output_yaml.write_text("\n".join(yaml_lines) + "\n", encoding="utf-8")

    print(f"Wrote {output_yaml}")
    print(f"Wrote {output_image}")
    print(f"Removed occupied pixels: {removed_occupied}")
    print(f"Filled free pixels: {filled_free}")


if __name__ == "__main__":
    main()
