#!/usr/bin/env python3

import argparse
import csv
import os
import sys
from dataclasses import dataclass
from pathlib import Path


HEADER = ['id', 'x', 'y', 'z', 'yaw_deg']


@dataclass
class Marker:
    marker_id: int
    x: float
    y: float
    z: float
    yaw_deg: float

    def as_row(self) -> dict[str, str]:
        return {
            'id': str(self.marker_id),
            'x': f'{self.x:.3f}',
            'y': f'{self.y:.3f}',
            'z': f'{self.z:.3f}',
            'yaw_deg': f'{self.yaw_deg:.3f}',
        }

    def as_map_entry(self) -> str:
        return (
            f'{self.marker_id}:'
            f'{self.x:.3f},{self.y:.3f},{self.z:.3f},{self.yaw_deg:.3f}'
        )


def default_marker_file() -> Path:
    package_dir = Path(__file__).resolve().parents[3]
    return package_dir / 'config' / 'marker_map_final.csv'


def marker_string(markers: list[Marker]) -> str:
    return ';'.join(marker.as_map_entry() for marker in sorted(markers, key=lambda m: m.marker_id))


def load_markers(path: Path) -> list[Marker]:
    if not path.exists():
        raise FileNotFoundError(f'Marker map file not found: {path}')

    markers: list[Marker] = []
    with path.open(newline='') as f:
        reader = csv.DictReader(f)
        missing = [name for name in HEADER if name not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f'Marker map is missing columns: {", ".join(missing)}')

        for line_number, row in enumerate(reader, start=2):
            if not any((row.get(name) or '').strip() for name in HEADER):
                continue
            try:
                markers.append(
                    Marker(
                        marker_id=int((row.get('id') or '').strip()),
                        x=float((row.get('x') or '').strip()),
                        y=float((row.get('y') or '').strip()),
                        z=float((row.get('z') or '').strip()),
                        yaw_deg=float((row.get('yaw_deg') or '').strip()),
                    )
                )
            except ValueError as exc:
                raise ValueError(f'Invalid marker row at line {line_number}: {row}') from exc

    seen: set[int] = set()
    duplicates: set[int] = set()
    for marker in markers:
        if marker.marker_id in seen:
            duplicates.add(marker.marker_id)
        seen.add(marker.marker_id)
    if duplicates:
        raise ValueError(f'Duplicate marker ids: {sorted(duplicates)}')

    return sorted(markers, key=lambda m: m.marker_id)


def save_markers(path: Path, markers: list[Marker]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=HEADER)
        writer.writeheader()
        for marker in sorted(markers, key=lambda m: m.marker_id):
            writer.writerow(marker.as_row())


def export_marker_string(path: Path, markers: list[Marker]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(marker_string(markers) + '\n')


def print_markers(markers: list[Marker]) -> None:
    print()
    print(' id        x        y        z   yaw_deg')
    print('-----------------------------------------')
    for marker in sorted(markers, key=lambda m: m.marker_id):
        print(
            f'{marker.marker_id:3d} '
            f'{marker.x:8.3f} '
            f'{marker.y:8.3f} '
            f'{marker.z:8.3f} '
            f'{marker.yaw_deg:9.3f}'
        )
    print()


def prompt_float(label: str, default: float | None = None) -> float:
    suffix = f' [{default:.3f}]' if default is not None else ''
    while True:
        text = input(f'{label}{suffix}: ').strip()
        if text == '' and default is not None:
            return default
        try:
            return float(text)
        except ValueError:
            print('Please enter a number.')


def prompt_int(label: str, default: int | None = None) -> int:
    suffix = f' [{default}]' if default is not None else ''
    while True:
        text = input(f'{label}{suffix}: ').strip()
        if text == '' and default is not None:
            return default
        try:
            return int(text)
        except ValueError:
            print('Please enter an integer.')


def run_terminal_editor(csv_path: Path, export_path: Path, markers: list[Marker]) -> None:
    print('[marker_map_editor] No graphical display found; using terminal editor.')
    print(f'CSV: {csv_path}')
    print(f'ROS string export: {export_path}')

    while True:
        print_markers(markers)
        print('Menu: [a] add/update  [d] delete  [s] save/export  [p] print ROS string  [q] quit')
        choice = input('Choice: ').strip().lower()

        if choice in ('q', 'quit', 'exit'):
            break

        if choice in ('a', 'add', 'u', 'update'):
            marker_id = prompt_int('marker id')
            existing = next((m for m in markers if m.marker_id == marker_id), None)
            marker = Marker(
                marker_id=marker_id,
                x=prompt_float('x', existing.x if existing else None),
                y=prompt_float('y', existing.y if existing else None),
                z=prompt_float('z', existing.z if existing else 0.30),
                yaw_deg=prompt_float('yaw_deg', existing.yaw_deg if existing else None),
            )
            markers = [m for m in markers if m.marker_id != marker.marker_id]
            markers.append(marker)
            markers.sort(key=lambda m: m.marker_id)
            print(f'Updated marker {marker.marker_id}.')
            continue

        if choice in ('d', 'delete', 'del'):
            marker_id = prompt_int('marker id to delete')
            old_count = len(markers)
            markers = [m for m in markers if m.marker_id != marker_id]
            if len(markers) == old_count:
                print(f'Marker {marker_id} was not found.')
            else:
                print(f'Deleted marker {marker_id}.')
            continue

        if choice in ('s', 'save', 'export'):
            save_markers(csv_path, markers)
            export_marker_string(export_path, markers)
            print(f'Saved {csv_path}')
            print(f'Exported {export_path}')
            continue

        if choice in ('p', 'print'):
            print(marker_string(markers))
            continue

        print('Unknown choice.')

    save_now = input('Save/export before exit? [Y/n]: ').strip().lower()
    if save_now in ('', 'y', 'yes'):
        save_markers(csv_path, markers)
        export_marker_string(export_path, markers)
        print(f'Saved {csv_path}')
        print(f'Exported {export_path}')


class MarkerMapEditor:
    def __init__(self, csv_path: Path, export_path: Path | None):
        import tkinter as tk
        from tkinter import messagebox, ttk

        self.tk = tk
        self.ttk = ttk
        self.messagebox = messagebox
        self.csv_path = csv_path
        self.export_path = export_path or csv_path.with_suffix('.txt')
        self.markers = load_markers(csv_path)

        self.root = tk.Tk()
        self.root.title('Marker Map Editor')
        self.root.geometry('760x480')

        self.tree = ttk.Treeview(self.root, columns=HEADER, show='headings', height=14)
        for column in HEADER:
            self.tree.heading(column, text=column)
            self.tree.column(column, width=110, anchor='center')
        self.tree.pack(fill='both', expand=True, padx=10, pady=(10, 4))
        self.tree.bind('<<TreeviewSelect>>', self.on_select)

        form = ttk.Frame(self.root)
        form.pack(fill='x', padx=10, pady=4)
        self.entries: dict[str, tk.Entry] = {}
        for index, column in enumerate(HEADER):
            ttk.Label(form, text=column).grid(row=0, column=index, padx=4, sticky='w')
            entry = ttk.Entry(form, width=12)
            entry.grid(row=1, column=index, padx=4, sticky='ew')
            self.entries[column] = entry

        buttons = ttk.Frame(self.root)
        buttons.pack(fill='x', padx=10, pady=(4, 10))
        ttk.Button(buttons, text='Add / Update', command=self.add_or_update).pack(side='left', padx=4)
        ttk.Button(buttons, text='Delete', command=self.delete_selected).pack(side='left', padx=4)
        ttk.Button(buttons, text='Clear', command=self.clear_entries).pack(side='left', padx=4)
        ttk.Button(buttons, text='Save CSV', command=self.save_csv).pack(side='right', padx=4)
        ttk.Button(buttons, text='Export ROS String', command=self.export_string).pack(side='right', padx=4)
        ttk.Button(buttons, text='Copy ROS String', command=self.copy_string).pack(side='right', padx=4)

        self.status = tk.StringVar(value=f'Editing: {self.csv_path}')
        ttk.Label(self.root, textvariable=self.status).pack(fill='x', padx=10, pady=(0, 8))

        self.refresh_tree()

    def refresh_tree(self) -> None:
        for item in self.tree.get_children():
            self.tree.delete(item)
        for marker in sorted(self.markers, key=lambda m: m.marker_id):
            self.tree.insert(
                '',
                'end',
                iid=str(marker.marker_id),
                values=(
                    marker.marker_id,
                    f'{marker.x:.3f}',
                    f'{marker.y:.3f}',
                    f'{marker.z:.3f}',
                    f'{marker.yaw_deg:.3f}',
                ),
            )

    def selected_marker_id(self) -> int | None:
        selected = self.tree.selection()
        if not selected:
            return None
        return int(selected[0])

    def on_select(self, _event=None) -> None:
        marker_id = self.selected_marker_id()
        if marker_id is None:
            return
        marker = next((m for m in self.markers if m.marker_id == marker_id), None)
        if marker is None:
            return
        values = marker.as_row()
        for column in HEADER:
            self.entries[column].delete(0, self.tk.END)
            self.entries[column].insert(0, values[column])

    def read_entry_marker(self) -> Marker:
        try:
            return Marker(
                marker_id=int(self.entries['id'].get().strip()),
                x=float(self.entries['x'].get().strip()),
                y=float(self.entries['y'].get().strip()),
                z=float(self.entries['z'].get().strip()),
                yaw_deg=float(self.entries['yaw_deg'].get().strip()),
            )
        except ValueError as exc:
            raise ValueError('id must be int; x, y, z, yaw_deg must be numbers') from exc

    def add_or_update(self) -> None:
        try:
            marker = self.read_entry_marker()
        except ValueError as exc:
            self.messagebox.showerror('Invalid marker', str(exc))
            return

        self.markers = [m for m in self.markers if m.marker_id != marker.marker_id]
        self.markers.append(marker)
        self.markers.sort(key=lambda m: m.marker_id)
        self.refresh_tree()
        self.tree.selection_set(str(marker.marker_id))
        self.status.set(f'Updated marker {marker.marker_id}; remember to save/export.')

    def delete_selected(self) -> None:
        marker_id = self.selected_marker_id()
        if marker_id is None:
            self.messagebox.showinfo('Delete marker', 'Select a marker first.')
            return
        self.markers = [m for m in self.markers if m.marker_id != marker_id]
        self.refresh_tree()
        self.clear_entries()
        self.status.set(f'Deleted marker {marker_id}; remember to save/export.')

    def clear_entries(self) -> None:
        for entry in self.entries.values():
            entry.delete(0, self.tk.END)

    def save_csv(self) -> None:
        save_markers(self.csv_path, self.markers)
        self.status.set(f'Saved CSV: {self.csv_path}')

    def export_string(self) -> None:
        save_markers(self.csv_path, self.markers)
        export_marker_string(self.export_path, self.markers)
        self.status.set(f'Exported ROS marker string: {self.export_path}')

    def copy_string(self) -> None:
        text = marker_string(self.markers)
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self.status.set('Copied ROS marker string to clipboard.')

    def run(self) -> None:
        self.root.mainloop()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Edit/export ASClinic ArUco marker maps.')
    parser.add_argument('--file', type=Path, default=default_marker_file(), help='Marker CSV file.')
    parser.add_argument('--export-txt', type=Path, default=None, help='Where to write the ROS marker string.')
    parser.add_argument('--print-string', action='store_true', help='Print id:x,y,z,yaw;... and exit.')
    parser.add_argument('--set', nargs=5, metavar=('ID', 'X', 'Y', 'Z', 'YAW_DEG'), help='Add/update one marker, then save/export.')
    parser.add_argument('--delete', type=int, metavar='ID', help='Delete one marker, then save/export.')
    parser.add_argument('--no-ui', action='store_true', help='Do not open the Tk UI.')
    parser.add_argument('--terminal', action='store_true', help='Use the terminal editor instead of Tk.')
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    csv_path = args.file.expanduser().resolve()
    export_path = (args.export_txt.expanduser().resolve() if args.export_txt else csv_path.with_suffix('.txt'))

    try:
        markers = load_markers(csv_path)
        changed = False

        if args.set:
            marker = Marker(
                marker_id=int(args.set[0]),
                x=float(args.set[1]),
                y=float(args.set[2]),
                z=float(args.set[3]),
                yaw_deg=float(args.set[4]),
            )
            markers = [m for m in markers if m.marker_id != marker.marker_id]
            markers.append(marker)
            changed = True

        if args.delete is not None:
            markers = [m for m in markers if m.marker_id != args.delete]
            changed = True

        if changed:
            save_markers(csv_path, markers)
            export_marker_string(export_path, markers)

        if args.print_string:
            print(marker_string(markers))
            return 0

        if args.no_ui:
            export_marker_string(export_path, markers)
            print(f'Wrote {export_path}')
            return 0

        if args.terminal or not os.environ.get('DISPLAY'):
            run_terminal_editor(csv_path, export_path, markers)
            return 0

        editor = MarkerMapEditor(csv_path=csv_path, export_path=export_path)
        editor.run()
        return 0
    except Exception as exc:
        print(f'[marker_map_editor] {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
