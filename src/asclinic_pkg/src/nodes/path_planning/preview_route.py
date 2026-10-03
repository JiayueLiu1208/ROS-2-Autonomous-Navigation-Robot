#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path
from xml.sax.saxutils import escape

from final_demo_layout import (
    HEIGHT_M,
    LECTERNS,
    MARKERS,
    PLANTS,
    ROUTE_WAYPOINTS,
    STOP_POINTS,
    TABLES,
    WIDTH_M,
)


PATH_STEP = 0.08
CORNER_TRIM = 0.45
WALL_THICKNESS = 0.10


def clamp(value, low, high):
    return max(low, min(high, value))


def segment_length(a, b):
    return math.hypot(b[1] - a[1], b[2] - a[2])


def yaw_deg(x1, y1, x2, y2):
    return math.degrees(math.atan2(y2 - y1, x2 - x1))


def can_smooth_waypoint(index, route_waypoints):
    if index <= 0 or index >= len(route_waypoints) - 1:
        return False

    label, x, y = route_waypoints[index]
    if label.endswith('_STOP'):
        return False

    _, prev_x, prev_y = route_waypoints[index - 1]
    _, next_x, next_y = route_waypoints[index + 1]
    in_len = math.hypot(x - prev_x, y - prev_y)
    out_len = math.hypot(next_x - x, next_y - y)

    if in_len < 0.05 or out_len < 0.05:
        return False

    in_unit = ((x - prev_x) / in_len, (y - prev_y) / in_len)
    out_unit = ((next_x - x) / out_len, (next_y - y) / out_len)
    dot = clamp(in_unit[0] * out_unit[0] + in_unit[1] * out_unit[1], -1.0, 1.0)
    turn_angle = math.acos(dot)
    return turn_angle > math.radians(8.0)


def corner_points(index, route_waypoints):
    _, prev_x, prev_y = route_waypoints[index - 1]
    _, x, y = route_waypoints[index]
    _, next_x, next_y = route_waypoints[index + 1]

    in_len = math.hypot(x - prev_x, y - prev_y)
    out_len = math.hypot(next_x - x, next_y - y)
    trim = min(CORNER_TRIM, 0.4 * in_len, 0.4 * out_len)

    in_unit = ((x - prev_x) / in_len, (y - prev_y) / in_len)
    out_unit = ((next_x - x) / out_len, (next_y - y) / out_len)

    entry_x = x - in_unit[0] * trim
    entry_y = y - in_unit[1] * trim
    exit_x = x + out_unit[0] * trim
    exit_y = y + out_unit[1] * trim

    return entry_x, entry_y, exit_x, exit_y


def append_straight_points(points, label, x, y):
    _, start_x, start_y = points[-1]
    distance = math.hypot(x - start_x, y - start_y)
    steps = max(1, int(math.ceil(distance / PATH_STEP)))

    for step in range(1, steps + 1):
        ratio = step / steps
        dense_x = start_x + (x - start_x) * ratio
        dense_y = start_y + (y - start_y) * ratio
        point_label = label if step == steps else f'{label}_SEGMENT'
        points.append((point_label, dense_x, dense_y))


def append_curve_points(points, label, control_x, control_y, exit_x, exit_y):
    _, entry_x, entry_y = points[-1]
    chord = math.hypot(exit_x - entry_x, exit_y - entry_y)
    steps = max(4, int(math.ceil(chord / PATH_STEP)))

    for step in range(1, steps + 1):
        t = step / steps
        one_minus_t = 1.0 - t
        curve_x = (
            one_minus_t * one_minus_t * entry_x
            + 2.0 * one_minus_t * t * control_x
            + t * t * exit_x
        )
        curve_y = (
            one_minus_t * one_minus_t * entry_y
            + 2.0 * one_minus_t * t * control_y
            + t * t * exit_y
        )
        point_label = f'{label}_CURVE_EXIT' if step == steps else f'{label}_CURVE'
        points.append((point_label, curve_x, curve_y))


def densify_route(route_waypoints, smooth_corners=False):
    first_label, first_x, first_y = route_waypoints[0]
    points = [(first_label, first_x, first_y)]

    for idx in range(1, len(route_waypoints)):
        label, x, y = route_waypoints[idx]

        if smooth_corners and can_smooth_waypoint(idx, route_waypoints):
            entry_x, entry_y, exit_x, exit_y = corner_points(idx, route_waypoints)
            append_straight_points(
                points,
                f'{label}_CURVE_ENTRY',
                entry_x,
                entry_y,
            )
            append_curve_points(points, label, x, y, exit_x, exit_y)
        else:
            append_straight_points(points, label, x, y)

    return points


def route_length(points):
    return sum(segment_length(a, b) for a, b in zip(points, points[1:]))


def in_rect(x, y, cx, cy, sx, sy):
    return (cx - sx / 2.0) <= x <= (cx + sx / 2.0) and (
        cy - sy / 2.0
    ) <= y <= (cy + sy / 2.0)


def nearest_label(x, y, items, radius):
    best = None
    best_dist = radius
    for item in items:
        label, item_x, item_y = item[:3]
        dist = math.hypot(x - item_x, y - item_y)
        if dist <= best_dist:
            best = label
            best_dist = dist
    return best


def build_ascii_map(points, scale):
    cols = int(math.ceil(WIDTH_M / scale)) + 1
    rows = int(math.ceil(HEIGHT_M / scale)) + 1
    grid = [[' ' for _ in range(cols)] for _ in range(rows)]

    def to_cell(x, y):
        col = int(round(x / scale))
        row = int(round((HEIGHT_M - y) / scale))
        return row, col

    for row in range(rows):
        for col in range(cols):
            x = col * scale
            y = HEIGHT_M - row * scale

            if (
                x <= WALL_THICKNESS
                or x >= WIDTH_M - WALL_THICKNESS
                or y <= WALL_THICKNESS
                or y >= HEIGHT_M - WALL_THICKNESS
            ):
                grid[row][col] = '#'
                continue

            for _, tx, ty, sx, sy in TABLES + LECTERNS:
                if in_rect(x, y, tx, ty, sx, sy):
                    grid[row][col] = '#'
                    break

    for _, x, y in points:
        row, col = to_cell(x, y)
        if 0 <= row < rows and 0 <= col < cols and grid[row][col] == ' ':
            grid[row][col] = '.'

    for idx, (_, x, y) in enumerate(STOP_POINTS, start=1):
        row, col = to_cell(x, y)
        if 0 <= row < rows and 0 <= col < cols:
            grid[row][col] = 'S'

    for name, x, y in PLANTS:
        row, col = to_cell(x, y)
        if 0 <= row < rows and 0 <= col < cols:
            grid[row][col] = name[-1]

    output = []
    for row, line in enumerate(grid):
        y = HEIGHT_M - row * scale
        if row % max(1, int(round(1.0 / scale))) == 0:
            prefix = f'{y:4.1f}|'
        else:
            prefix = '    |'
        output.append(prefix + ''.join(line))

    axis = '    +' + '-' * cols
    x_labels = ['     ']
    for col in range(cols):
        x = col * scale
        if abs(x - round(x)) < scale / 2.0:
            x_labels.append(str(int(round(x)))[:1])
        else:
            x_labels.append(' ')
    output.append(axis)
    output.append(''.join(x_labels))
    return '\n'.join(output)


def write_csv(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['index', 'label', 'x_m', 'y_m'])
        for idx, (label, x, y) in enumerate(points):
            writer.writerow([idx, label, f'{x:.4f}', f'{y:.4f}'])


def write_svg(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)

    px_per_m = 70.0
    margin_left = 76.0
    margin_top = 70.0
    plot_w = WIDTH_M * px_per_m
    plot_h = HEIGHT_M * px_per_m
    svg_w = margin_left + plot_w + 260.0
    svg_h = margin_top + plot_h + 88.0

    def sx(x):
        return margin_left + x * px_per_m

    def sy(y):
        return margin_top + (HEIGHT_M - y) * px_per_m

    def rect_from_center(cx, cy, width, height):
        x = sx(cx - width / 2.0)
        y = sy(cy + height / 2.0)
        return x, y, width * px_per_m, height * px_per_m

    route_points = ' '.join(f'{sx(x):.1f},{sy(y):.1f}' for _, x, y in points)
    raw_points = ' '.join(
        f'{sx(x):.1f},{sy(y):.1f}' for _, x, y in ROUTE_WAYPOINTS
    )

    route_len = route_length(points)
    raw_len = sum(
        segment_length(start, end)
        for start, end in zip(ROUTE_WAYPOINTS, ROUTE_WAYPOINTS[1:])
    )

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" '
            f'width="{svg_w:.0f}" height="{svg_h:.0f}" '
            f'viewBox="0 0 {svg_w:.0f} {svg_h:.0f}">'
        ),
        '<defs>',
        (
            '<marker id="arrow" markerWidth="10" markerHeight="10" '
            'refX="7" refY="3" orient="auto" markerUnits="strokeWidth">'
            '<path d="M0,0 L0,6 L8,3 z" fill="#2563eb"/>'
            '</marker>'
        ),
        (
            '<marker id="face-arrow" markerWidth="8" markerHeight="8" '
            'refX="6" refY="3" orient="auto" markerUnits="strokeWidth">'
            '<path d="M0,0 L0,6 L7,3 z" fill="#059669"/>'
            '</marker>'
        ),
        (
            '<marker id="aruco-arrow" markerWidth="8" markerHeight="8" '
            'refX="6" refY="3" orient="auto" markerUnits="strokeWidth">'
            '<path d="M0,0 L0,6 L7,3 z" fill="#1d4ed8"/>'
            '</marker>'
        ),
        (
            '<style>'
            'text{font-family:Arial,sans-serif;fill:#111827}'
            '.small{font-size:12px}.tiny{font-size:10px}.label{font-size:13px;font-weight:700}'
            '.title{font-size:22px;font-weight:700}.sub{font-size:13px;fill:#4b5563}'
            '</style>'
        ),
        '</defs>',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        (
            f'<text class="title" x="{margin_left:.1f}" y="30">'
            'Final Demo Route Preview</text>'
        ),
        (
            f'<text class="sub" x="{margin_left:.1f}" y="51">'
            f'map={WIDTH_M:.1f}m x {HEIGHT_M:.1f}m, '
            f'dense_length={route_len:.2f}m, raw_waypoint_length={raw_len:.2f}m'
            '</text>'
        ),
        (
            f'<rect x="{margin_left:.1f}" y="{margin_top:.1f}" '
            f'width="{plot_w:.1f}" height="{plot_h:.1f}" '
            'fill="#ffffff" stroke="#111827" stroke-width="2"/>'
        ),
    ]

    for x in range(int(WIDTH_M) + 1):
        width = 1.2 if x % 2 == 0 else 0.6
        lines.append(
            f'<line x1="{sx(x):.1f}" y1="{sy(0):.1f}" '
            f'x2="{sx(x):.1f}" y2="{sy(HEIGHT_M):.1f}" '
            f'stroke="#d1d5db" stroke-width="{width}"/>'
        )
        lines.append(
            f'<text class="tiny" x="{sx(x) - 3:.1f}" y="{sy(0) + 18:.1f}">{x}</text>'
        )

    for y in range(int(math.floor(HEIGHT_M)) + 1):
        width = 1.2 if y % 2 == 0 else 0.6
        lines.append(
            f'<line x1="{sx(0):.1f}" y1="{sy(y):.1f}" '
            f'x2="{sx(WIDTH_M):.1f}" y2="{sy(y):.1f}" '
            f'stroke="#d1d5db" stroke-width="{width}"/>'
        )
        lines.append(
            f'<text class="tiny" x="{sx(0) - 28:.1f}" y="{sy(y) + 4:.1f}">{y}</text>'
        )

    for name, cx, cy, tx, ty in TABLES:
        x, y, w, h = rect_from_center(cx, cy, tx, ty)
        lines.append(
            f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
            'rx="4" fill="#6b7280" opacity="0.35" stroke="#374151"/>'
        )
        lines.append(
            f'<text class="tiny" text-anchor="middle" x="{sx(cx):.1f}" '
            f'y="{sy(cy) + 4:.1f}">{escape(name)}</text>'
        )

    for name, cx, cy, tx, ty in LECTERNS:
        x, y, w, h = rect_from_center(cx, cy, tx, ty)
        lines.append(
            f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
            'rx="4" fill="#a16207" opacity="0.35" stroke="#713f12"/>'
        )
        lines.append(
            f'<text class="tiny" text-anchor="middle" x="{sx(cx):.1f}" '
            f'y="{sy(cy) + 4:.1f}">{escape(name)}</text>'
        )

    marker_line_len = 0.36
    marker_arrow_len = 0.42
    for marker_id, x, y, _z, phi_deg in MARKERS:
        phi = math.radians(phi_deg)
        normal_x = math.cos(phi)
        normal_y = math.sin(phi)
        tangent_x = -normal_y
        tangent_y = normal_x
        x1 = x - tangent_x * marker_line_len / 2.0
        y1 = y - tangent_y * marker_line_len / 2.0
        x2 = x + tangent_x * marker_line_len / 2.0
        y2 = y + tangent_y * marker_line_len / 2.0
        ax0 = x
        ay0 = y
        ax1 = x + normal_x * marker_arrow_len
        ay1 = y + normal_y * marker_arrow_len
        label_dx = 0.10 if x < WIDTH_M - 0.4 else -0.34
        label_dy = 0.18 if y < HEIGHT_M - 0.4 else -0.22
        lines.append(
            f'<line x1="{sx(x1):.1f}" y1="{sy(y1):.1f}" '
            f'x2="{sx(x2):.1f}" y2="{sy(y2):.1f}" '
            'stroke="#111827" stroke-width="4" stroke-linecap="round"/>'
        )
        lines.append(
            f'<line x1="{sx(ax0):.1f}" y1="{sy(ay0):.1f}" '
            f'x2="{sx(ax1):.1f}" y2="{sy(ay1):.1f}" '
            'stroke="#1d4ed8" stroke-width="2.5" marker-end="url(#aruco-arrow)"/>'
        )
        lines.append(
            f'<text class="tiny" x="{sx(x + label_dx):.1f}" '
            f'y="{sy(y + label_dy):.1f}" fill="#111827">M{marker_id}</text>'
        )

    lines.append(
        f'<polyline points="{raw_points}" fill="none" stroke="#93c5fd" '
        'stroke-width="2" stroke-dasharray="8 8"/>'
    )
    lines.append(
        f'<polyline points="{route_points}" fill="none" stroke="#2563eb" '
        'stroke-width="4" stroke-linecap="round" stroke-linejoin="round" '
        'marker-end="url(#arrow)"/>'
    )

    for idx in range(14, len(points) - 1, 38):
        _, x1, y1 = points[idx]
        _, x2, y2 = points[idx + 1]
        dx = x2 - x1
        dy = y2 - y1
        norm = math.hypot(dx, dy)
        if norm <= 1e-6:
            continue
        arrow_len = 0.22
        x_start = x1 - dx / norm * arrow_len
        y_start = y1 - dy / norm * arrow_len
        x_end = x1 + dx / norm * arrow_len
        y_end = y1 + dy / norm * arrow_len
        lines.append(
            f'<line x1="{sx(x_start):.1f}" y1="{sy(y_start):.1f}" '
            f'x2="{sx(x_end):.1f}" y2="{sy(y_end):.1f}" '
            'stroke="#2563eb" stroke-width="3" marker-end="url(#arrow)"/>'
        )

    for idx, (label, x, y) in enumerate(ROUTE_WAYPOINTS, start=1):
        lines.append(
            f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="7" '
            'fill="#ffffff" stroke="#1d4ed8" stroke-width="2"/>'
        )
        lines.append(
            f'<text class="tiny" text-anchor="middle" x="{sx(x):.1f}" '
            f'y="{sy(y) + 3.5:.1f}">{idx}</text>'
        )
        if idx in (1, len(ROUTE_WAYPOINTS)) or label.endswith('_STOP'):
            lines.append(
                f'<text class="tiny" x="{sx(x) + 8:.1f}" y="{sy(y) - 8:.1f}">'
                f'{escape(label)}</text>'
            )

    plant_lookup = {name: (x, y) for name, x, y in PLANTS}
    for plant_id, stop_x, stop_y in STOP_POINTS:
        plant_x, plant_y = plant_lookup[plant_id]
        lines.append(
            f'<line x1="{sx(stop_x):.1f}" y1="{sy(stop_y):.1f}" '
            f'x2="{sx(plant_x):.1f}" y2="{sy(plant_y):.1f}" '
            'stroke="#059669" stroke-width="2" stroke-dasharray="4 4" '
            'marker-end="url(#face-arrow)"/>'
        )
        lines.append(
            f'<circle cx="{sx(stop_x):.1f}" cy="{sy(stop_y):.1f}" r="8" '
            'fill="#f59e0b" stroke="#92400e" stroke-width="2"/>'
        )
        lines.append(
            f'<text class="tiny" text-anchor="middle" x="{sx(stop_x):.1f}" '
            f'y="{sy(stop_y) + 4:.1f}">S</text>'
        )

    for name, x, y in PLANTS:
        lines.append(
            f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="12" '
            'fill="#22c55e" stroke="#166534" stroke-width="2"/>'
        )
        lines.append(
            f'<text class="label" text-anchor="middle" x="{sx(x):.1f}" '
            f'y="{sy(y) + 4.5:.1f}" fill="#052e16">{escape(name)}</text>'
        )

    panel_x = margin_left + plot_w + 28.0
    panel_y = margin_top
    lines.extend(
        [
            (
                f'<rect x="{panel_x:.1f}" y="{panel_y:.1f}" '
                'width="210" height="430" rx="8" '
                'fill="#ffffff" stroke="#cbd5e1"/>'
            ),
            f'<text class="label" x="{panel_x + 14:.1f}" y="{panel_y + 26:.1f}">Legend</text>',
            (
                f'<line x1="{panel_x + 16:.1f}" y1="{panel_y + 52:.1f}" '
                f'x2="{panel_x + 66:.1f}" y2="{panel_y + 52:.1f}" '
                'stroke="#2563eb" stroke-width="4"/>'
            ),
            f'<text class="small" x="{panel_x + 78:.1f}" y="{panel_y + 56:.1f}">smoothed path</text>',
            (
                f'<line x1="{panel_x + 16:.1f}" y1="{panel_y + 82:.1f}" '
                f'x2="{panel_x + 66:.1f}" y2="{panel_y + 82:.1f}" '
                'stroke="#93c5fd" stroke-width="2" stroke-dasharray="8 8"/>'
            ),
            f'<text class="small" x="{panel_x + 78:.1f}" y="{panel_y + 86:.1f}">raw waypoints</text>',
            (
                f'<rect x="{panel_x + 18:.1f}" y="{panel_y + 106:.1f}" '
                'width="30" height="18" fill="#6b7280" opacity="0.35" stroke="#374151"/>'
            ),
            f'<text class="small" x="{panel_x + 78:.1f}" y="{panel_y + 120:.1f}">table / lectern</text>',
            (
                f'<circle cx="{panel_x + 33:.1f}" cy="{panel_y + 150:.1f}" r="10" '
                'fill="#22c55e" stroke="#166534" stroke-width="2"/>'
            ),
            f'<text class="small" x="{panel_x + 78:.1f}" y="{panel_y + 154:.1f}">plant</text>',
            (
                f'<circle cx="{panel_x + 33:.1f}" cy="{panel_y + 181:.1f}" r="8" '
                'fill="#f59e0b" stroke="#92400e" stroke-width="2"/>'
            ),
            f'<text class="small" x="{panel_x + 78:.1f}" y="{panel_y + 185:.1f}">stop pose</text>',
            (
                f'<line x1="{panel_x + 18:.1f}" y1="{panel_y + 212:.1f}" '
                f'x2="{panel_x + 48:.1f}" y2="{panel_y + 212:.1f}" '
                'stroke="#111827" stroke-width="4" stroke-linecap="round"/>'
            ),
            (
                f'<line x1="{panel_x + 33:.1f}" y1="{panel_y + 212:.1f}" '
                f'x2="{panel_x + 61:.1f}" y2="{panel_y + 212:.1f}" '
                'stroke="#1d4ed8" stroke-width="2.5" marker-end="url(#aruco-arrow)"/>'
            ),
            f'<text class="small" x="{panel_x + 78:.1f}" y="{panel_y + 216:.1f}">ArUco marker</text>',
            (
                f'<text class="small" x="{panel_x + 14:.1f}" y="{panel_y + 254:.1f}">'
                f'Dense points: {len(points)}</text>'
            ),
            (
                f'<text class="small" x="{panel_x + 14:.1f}" y="{panel_y + 278:.1f}">'
                f'Dense length: {route_len:.2f} m</text>'
            ),
            (
                f'<text class="small" x="{panel_x + 14:.1f}" y="{panel_y + 302:.1f}">'
                f'Raw length: {raw_len:.2f} m</text>'
            ),
            (
                f'<text class="small" x="{panel_x + 14:.1f}" y="{panel_y + 336:.1f}">'
                'Route order:</text>'
            ),
        ]
    )

    y_cursor = panel_y + 360.0
    for idx, (label, x, y) in enumerate(ROUTE_WAYPOINTS, start=1):
        short_label = label.replace('_', ' ')
        if len(short_label) > 24:
            short_label = short_label[:21] + '...'
        lines.append(
            f'<text class="tiny" x="{panel_x + 14:.1f}" y="{y_cursor:.1f}">'
            f'{idx:02d}. {escape(short_label)} ({x:.2f}, {y:.2f})</text>'
        )
        y_cursor += 15.0

    lines.append('</svg>')
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def print_summary(points, args):
    plant_lookup = {name: (x, y) for name, x, y in PLANTS}

    print('Final demo route preview')
    print(f'Map size: {WIDTH_M:.2f} m x {HEIGHT_M:.2f} m')
    print(f'Route key waypoints: {len(ROUTE_WAYPOINTS)}')
    print(f'Dense simulated points: {len(points)} at step about {PATH_STEP:.2f} m')
    print(f'Dense route length: {route_length(points):.2f} m')
    print()

    print('Plant stop poses:')
    for idx, (plant_id, stop_x, stop_y) in enumerate(STOP_POINTS, start=1):
        plant_x, plant_y = plant_lookup[plant_id]
        facing = yaw_deg(stop_x, stop_y, plant_x, plant_y)
        print(
            f'  {idx}. {plant_id}: stop=({stop_x:.2f}, {stop_y:.2f}), '
            f'plant=({plant_x:.2f}, {plant_y:.2f}), face_yaw={facing:+.1f} deg'
        )
    print()

    print('Route waypoint segments:')
    total = 0.0
    for idx, (start, end) in enumerate(zip(ROUTE_WAYPOINTS, ROUTE_WAYPOINTS[1:]), start=1):
        length = segment_length(start, end)
        heading = yaw_deg(start[1], start[2], end[1], end[2])
        total += length
        print(
            f'  {idx:02d}. {start[0]} -> {end[0]}: '
            f'{length:.2f} m, heading={heading:+.1f} deg'
        )
    print(f'  Raw waypoint length before smoothing: {total:.2f} m')
    print()

    if not args.no_ascii:
        print('ASCII map legend: # wall/table, . path, S stop pose, 1-6 plant')
        print(build_ascii_map(points, args.scale))


def main():
    parser = argparse.ArgumentParser(description='Preview the final demo path route.')
    parser.add_argument(
        '--scale',
        type=float,
        default=0.25,
        help='ASCII map cell size in metres. Default: 0.25',
    )
    parser.add_argument(
        '--csv',
        type=Path,
        default=None,
        help='Optional output CSV path for dense route points.',
    )
    parser.add_argument(
        '--svg',
        type=Path,
        default=None,
        help='Optional output SVG path for a detailed route diagram.',
    )
    parser.add_argument(
        '--no-ascii',
        action='store_true',
        help='Only print the numeric route summary.',
    )
    parser.add_argument(
        '--smooth-corners',
        action='store_true',
        help='Preview the optional smoothed-corner route instead of the default right-angle route.',
    )
    args = parser.parse_args()

    if args.scale <= 0:
        raise SystemExit('--scale must be positive')

    points = densify_route(ROUTE_WAYPOINTS, smooth_corners=args.smooth_corners)
    print_summary(points, args)

    if args.csv is not None:
        write_csv(args.csv, points)
        print()
        print(f'Wrote dense route CSV: {args.csv}')

    if args.svg is not None:
        write_svg(args.svg, points)
        print()
        print(f'Wrote detailed route SVG: {args.svg}')


if __name__ == '__main__':
    main()
