#!/usr/bin/env python3

import argparse
import os
from pathlib import Path

import cv2


def select_device(requested: str) -> str:
    requested = requested.lower()
    if requested in ('cpu', 'none'):
        return 'cpu'
    cuda_available = False
    try:
        import torch

        cuda_available = bool(torch.cuda.is_available())
    except Exception:
        cuda_available = False

    if requested == 'auto':
        return 'cuda:0' if cuda_available else 'cpu'
    if requested == 'cuda':
        return 'cuda:0' if cuda_available else 'cpu'
    return requested


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Standalone YOLO plant detector smoke test.')
    parser.add_argument('--model', required=True, help='Path to trained YOLO .pt model.')
    parser.add_argument('--image', required=True, help='Path to one test image.')
    parser.add_argument('--output', default='', help='Annotated output image path.')
    parser.add_argument('--confidence', type=float, default=0.25)
    parser.add_argument('--iou', type=float, default=0.45)
    parser.add_argument('--device', default='auto', help='auto, cpu, cuda, cuda:0, ...')
    parser.add_argument(
        '--classes',
        default='',
        help='Optional comma-separated class names to print/save, for example plant,pot.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model_path = os.path.expanduser(args.model)
    image_path = os.path.expanduser(args.image)
    output_path = os.path.expanduser(args.output) if args.output else ''
    target_classes = {item.strip() for item in args.classes.split(',') if item.strip()}

    if not os.path.isfile(model_path):
        raise FileNotFoundError(model_path)
    if not os.path.isfile(image_path):
        raise FileNotFoundError(image_path)

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError('ultralytics is not installed. Try: pip3 install ultralytics') from exc

    image = cv2.imread(image_path)
    if image is None:
        raise RuntimeError('cv2.imread failed for %s' % image_path)

    device = select_device(args.device)
    model = YOLO(model_path)
    results = model.predict(
        source=image,
        conf=args.confidence,
        iou=args.iou,
        device=device,
        verbose=False,
    )
    result = results[0]
    names = getattr(result, 'names', None) or getattr(model, 'names', {})

    printed = 0
    annotated = image.copy()
    if result.boxes is not None and len(result.boxes) > 0:
        xyxy = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy().astype(int)
        for box, confidence, class_id in zip(xyxy, confs, class_ids):
            class_name = str(names.get(int(class_id), int(class_id)))
            if target_classes and class_name not in target_classes:
                continue
            xmin, ymin, xmax, ymax = [int(round(v)) for v in box.tolist()]
            print(
                '%s class_id=%d confidence=%.3f bbox=[%d,%d,%d,%d]'
                % (class_name, class_id, float(confidence), xmin, ymin, xmax, ymax)
            )
            cv2.rectangle(annotated, (xmin, ymin), (xmax, ymax), (0, 220, 0), 2)
            cv2.putText(
                annotated,
                '%s %.2f' % (class_name, float(confidence)),
                (xmin, max(18, ymin - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 220, 0),
                2,
                cv2.LINE_AA,
            )
            printed += 1

    if printed == 0:
        print('No detections passed filters.')

    if output_path:
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(output_path, annotated)
        print('Saved annotated image: %s' % output_path)

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
