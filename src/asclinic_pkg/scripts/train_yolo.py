#!/usr/bin/env python3

import argparse
import os


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Optional Ultralytics YOLO training wrapper for plant detection.'
    )
    parser.add_argument('--data', required=True, help='Path to YOLO data.yaml.')
    parser.add_argument('--model', default='yolov8n.pt', help='Baseline model checkpoint.')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--batch', default='auto', help='Batch size or auto.')
    parser.add_argument('--project', default='models/plant_detector/runs')
    parser.add_argument('--name', default='train')
    parser.add_argument('--device', default='auto')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError('ultralytics is not installed. Try: pip3 install ultralytics') from exc

    model = YOLO(os.path.expanduser(args.model))
    batch = args.batch
    try:
        batch = int(batch)
    except ValueError:
        pass

    model.train(
        data=os.path.expanduser(args.data),
        imgsz=args.imgsz,
        epochs=args.epochs,
        batch=batch,
        project=os.path.expanduser(args.project),
        name=args.name,
        device=None if args.device == 'auto' else args.device,
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
