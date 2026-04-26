from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class YoloBox:
    cls: int
    xc: float
    yc: float
    w: float
    h: float


def clamp01(x: float) -> float:
    if x < 0.0:
        return 0.0
    if x > 1.0:
        return 1.0
    return x


def xyxy_to_yolo(x1: float, y1: float, x2: float, y2: float, img_w: int, img_h: int) -> tuple[float, float, float, float]:
    x1 = max(0.0, min(float(img_w), x1))
    x2 = max(0.0, min(float(img_w), x2))
    y1 = max(0.0, min(float(img_h), y1))
    y2 = max(0.0, min(float(img_h), y2))
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1

    xc = (x1 + x2) / 2.0 / float(img_w)
    yc = (y1 + y2) / 2.0 / float(img_h)
    w = (x2 - x1) / float(img_w)
    h = (y2 - y1) / float(img_h)
    return clamp01(xc), clamp01(yc), clamp01(w), clamp01(h)


def yolo_to_xyxy(xc: float, yc: float, w: float, h: float, img_w: int, img_h: int) -> tuple[float, float, float, float]:
    xc = float(xc) * float(img_w)
    yc = float(yc) * float(img_h)
    w = float(w) * float(img_w)
    h = float(h) * float(img_h)
    x1 = xc - w / 2.0
    y1 = yc - h / 2.0
    x2 = xc + w / 2.0
    y2 = yc + h / 2.0
    return x1, y1, x2, y2


def read_yolo_label_file(path: Path) -> list[YoloBox]:
    if not path.exists():
        return []
    boxes: list[YoloBox] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 5:
            continue
        cls = int(parts[0])
        xc, yc, w, h = map(float, parts[1:5])
        boxes.append(YoloBox(cls=cls, xc=xc, yc=yc, w=w, h=h))
    return boxes


def write_yolo_label_file(path: Path, boxes: list[YoloBox]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"{b.cls} {b.xc:.6f} {b.yc:.6f} {b.w:.6f} {b.h:.6f}" for b in boxes]
    path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def detections_to_yolo_lines(
    dets: list[tuple[int, float, float, float, float, float]],
    *,
    img_w: int,
    img_h: int,
) -> list[str]:
    """
    Convert detections into YOLO lines.

    dets: list of (cls, conf, x1, y1, x2, y2) in pixel xyxy.
    """
    lines: list[str] = []
    for cls, _conf, x1, y1, x2, y2 in dets:
        xc, yc, w, h = xyxy_to_yolo(x1, y1, x2, y2, img_w=img_w, img_h=img_h)
        # Keep cls as int, 6 decimal places for coords.
        lines.append(f"{int(cls)} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}")
    return lines

