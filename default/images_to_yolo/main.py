#!/usr/bin/env python3
"""
Semi-automatic labeling tool for YOLO training.

Workflow:
1) Pre-label a folder of images using an existing YOLO model (Ultralytics).
2) Review + adjust labels in a browser UI (Streamlit) and save YOLO .txt files.

This avoids desktop GUI backends (GTK/Qt) and works well on Jetson/headless setups.
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass
from pathlib import Path


def _eprint(msg: str) -> None:
    print(msg, file=sys.stderr)


def _require_import(module: str, install_hint: str) -> None:
    try:
        __import__(module)
    except Exception as exc:  # noqa: BLE001 - show actionable hint
        raise RuntimeError(f"Missing dependency `{module}`. Install with: {install_hint}") from exc


def _resolve_dir(p: str | Path) -> Path:
    path = Path(p).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Path does not exist: {path}")
    if not path.is_dir():
        raise NotADirectoryError(f"Not a directory: {path}")
    return path


def _list_images(images_dir: Path) -> list[Path]:
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    images = [p for p in sorted(images_dir.rglob("*")) if p.is_file() and p.suffix.lower() in exts]
    return images


@dataclass(frozen=True)
class DatasetLayout:
    images_dir: Path
    labels_dir: Path

    def label_path_for_image(self, img_path: Path) -> Path:
        rel = img_path.relative_to(self.images_dir)
        return (self.labels_dir / rel).with_suffix(".txt")


def cmd_prelabel(args: argparse.Namespace) -> int:
    # Lazy import so `annotate` can run without ultralytics installed.
    _require_import("ultralytics", "pip install ultralytics")
    from ultralytics import YOLO  # type: ignore

    from yolo_utils import detections_to_yolo_lines  # noqa: PLC0415

    images_dir = _resolve_dir(args.images_dir)
    labels_dir = Path(args.labels_dir).expanduser().resolve()
    labels_dir.mkdir(parents=True, exist_ok=True)
    layout = DatasetLayout(images_dir=images_dir, labels_dir=labels_dir)

    images = _list_images(images_dir)
    if not images:
        _eprint(f"No images found under: {images_dir}")
        return 2

    model = YOLO(args.model)

    # Classes:
    # - If you pass --class-map "0:2,1:0" we will remap predicted class ids to dataset ids.
    class_map: dict[int, int] = {}
    if args.class_map:
        for pair in args.class_map.split(","):
            src_s, dst_s = pair.split(":")
            class_map[int(src_s)] = int(dst_s)

    for img_path in images:
        results = model.predict(
            source=str(img_path),
            conf=args.conf,
            iou=args.iou,
            imgsz=args.imgsz,
            device=args.device,
            verbose=False,
            max_det=args.max_det,
        )
        if not results:
            continue
        r0 = results[0]
        # Ultralytics returns boxes with xyxy in pixel coords + cls/conf.
        # We'll write YOLO normalized lines.
        try:
            boxes = r0.boxes
            if boxes is None or len(boxes) == 0:
                yolo_lines: list[str] = []
            else:
                w = int(r0.orig_shape[1])
                h = int(r0.orig_shape[0])
                xyxy = boxes.xyxy.cpu().numpy().tolist()
                cls = boxes.cls.cpu().numpy().tolist()
                conf = boxes.conf.cpu().numpy().tolist()
                dets = []
                for (x1, y1, x2, y2), c, s in zip(xyxy, cls, conf, strict=True):
                    ci = int(c)
                    if class_map:
                        if ci not in class_map:
                            continue
                        ci = class_map[ci]
                    dets.append((ci, float(s), float(x1), float(y1), float(x2), float(y2)))
                yolo_lines = detections_to_yolo_lines(dets, img_w=w, img_h=h)
        except Exception as exc:  # noqa: BLE001
            _eprint(f"Failed on {img_path}: {exc}")
            continue

        out_path = layout.label_path_for_image(img_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        if out_path.exists() and not args.overwrite:
            continue
        out_path.write_text("\n".join(yolo_lines) + ("\n" if yolo_lines else ""), encoding="utf-8")

    print(f"Prelabel complete. Labels in: {labels_dir}")
    return 0


def cmd_annotate(args: argparse.Namespace) -> int:
    # Streamlit app is launched in a subprocess, which keeps this script simple.
    _require_import("streamlit", "pip install streamlit streamlit-drawable-canvas pillow")

    # Run streamlit with env vars that point at dataset paths.
    env = os.environ.copy()
    env["I2Y_IMAGES_DIR"] = str(Path(args.images_dir).expanduser().resolve())
    env["I2Y_LABELS_DIR"] = str(Path(args.labels_dir).expanduser().resolve())
    env["I2Y_CLASSES"] = args.classes or ""
    env["I2Y_PORT"] = str(args.port)

    # Defer imports; we want to execute `streamlit run` even if module import paths differ.
    import subprocess  # noqa: PLC0415

    app_path = Path(__file__).resolve().parent / "streamlit_app.py"
    cmd = [
        sys.executable,
        "-m",
        "streamlit",
        "run",
        str(app_path),
        "--server.port",
        str(args.port),
        "--server.headless",
        "true",
    ]
    return subprocess.call(cmd, env=env)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="images_to_yolo", description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    pre = sub.add_parser("prelabel", help="Run a YOLO model over images and write YOLO .txt labels")
    pre.add_argument("--images-dir", required=True, help="Folder containing images (recursively searched)")
    pre.add_argument("--labels-dir", required=True, help="Folder to write YOLO label .txt files into")
    pre.add_argument("--model", required=True, help="Ultralytics model name or weights path (e.g. yolov8n.pt)")
    pre.add_argument("--conf", type=float, default=0.25)
    pre.add_argument("--iou", type=float, default=0.7)
    pre.add_argument("--imgsz", type=int, default=640)
    pre.add_argument("--max-det", type=int, default=300)
    pre.add_argument("--device", default=None, help="Ultralytics device string (e.g. 0, 'cpu')")
    pre.add_argument(
        "--class-map",
        default=None,
        help="Optional remap of predicted class ids to your dataset ids, e.g. '0:2,1:0'",
    )
    pre.add_argument("--overwrite", action="store_true", help="Overwrite existing label files")
    pre.set_defaults(func=cmd_prelabel)

    ann = sub.add_parser("annotate", help="Launch browser UI to review/edit YOLO labels")
    ann.add_argument("--images-dir", required=True)
    ann.add_argument("--labels-dir", required=True)
    ann.add_argument(
        "--classes",
        default=None,
        help="Comma-separated class names in dataset order (e.g. 'cube,hand,tool'). Optional but recommended.",
    )
    ann.add_argument("--port", type=int, default=8501)
    ann.set_defaults(func=cmd_annotate)

    return p


def main() -> int:
    args = build_parser().parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())