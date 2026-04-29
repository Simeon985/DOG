from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import streamlit as st
from PIL import Image
from streamlit_drawable_canvas import st_canvas

from yolo_utils import YoloBox, read_yolo_label_file, write_yolo_label_file, xyxy_to_yolo, yolo_to_xyxy


@dataclass(frozen=True)
class Layout:
    images_dir: Path
    labels_dir: Path

    def label_path(self, image_path: Path) -> Path:
        rel = image_path.relative_to(self.images_dir)
        return (self.labels_dir / rel).with_suffix(".txt")


def list_images(images_dir: Path) -> list[Path]:
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    return [p for p in sorted(images_dir.rglob("*")) if p.is_file() and p.suffix.lower() in exts]


def boxes_to_canvas_objects(boxes: list[YoloBox], *, img_w: int, img_h: int) -> list[dict]:
    objects: list[dict] = []
    for b in boxes:
        x1, y1, x2, y2 = yolo_to_xyxy(b.xc, b.yc, b.w, b.h, img_w=img_w, img_h=img_h)
        objects.append(
            {
                "type": "rect",
                "left": float(x1),
                "top": float(y1),
                "width": float(x2 - x1),
                "height": float(y2 - y1),
                "fill": "rgba(0, 0, 0, 0.0)",
                "stroke": "rgba(0, 255, 0, 0.9)",
                "strokeWidth": 2,
                "transparentCorners": False,
            }
        )
    return objects


def canvas_objects_to_boxes(objs: list[dict], *, img_w: int, img_h: int, class_ids: list[int]) -> list[YoloBox]:
    boxes: list[YoloBox] = []
    rects = [o for o in objs if o.get("type") == "rect"]
    for i, r in enumerate(rects):
        left = float(r.get("left", 0.0))
        top = float(r.get("top", 0.0))
        w = float(r.get("width", 0.0))
        h = float(r.get("height", 0.0))
        x1, y1, x2, y2 = left, top, left + w, top + h
        xc, yc, bw, bh = xyxy_to_yolo(x1, y1, x2, y2, img_w=img_w, img_h=img_h)
        cls = int(class_ids[i]) if i < len(class_ids) else 0
        boxes.append(YoloBox(cls=cls, xc=xc, yc=yc, w=bw, h=bh))
    return boxes


def parse_classes(raw: str) -> list[str]:
    raw = (raw or "").strip()
    if not raw:
        return []
    return [c.strip() for c in raw.split(",") if c.strip()]


def main() -> None:
    st.set_page_config(page_title="images_to_yolo", layout="wide")

    images_dir = Path(os.environ.get("I2Y_IMAGES_DIR", "")).expanduser().resolve()
    labels_dir = Path(os.environ.get("I2Y_LABELS_DIR", "")).expanduser().resolve()
    classes = parse_classes(os.environ.get("I2Y_CLASSES", ""))

    if not images_dir.exists():
        st.error("Missing/invalid images dir. Launch via `python main.py annotate --images-dir ... --labels-dir ...`.")
        st.stop()

    labels_dir.mkdir(parents=True, exist_ok=True)
    layout = Layout(images_dir=images_dir, labels_dir=labels_dir)

    images = list_images(images_dir)
    if not images:
        st.error(f"No images found under: {images_dir}")
        st.stop()

    # Sidebar controls
    st.sidebar.title("Dataset")
    st.sidebar.write(f"Images: `{images_dir}`")
    st.sidebar.write(f"Labels: `{labels_dir}`")

    default_idx = 0
    if "img_idx" not in st.session_state:
        st.session_state.img_idx = default_idx

    img_idx = st.sidebar.number_input("Image index", min_value=0, max_value=len(images) - 1, value=int(st.session_state.img_idx))
    st.session_state.img_idx = int(img_idx)

    image_path = images[int(img_idx)]
    label_path = layout.label_path(image_path)

    img = Image.open(image_path).convert("RGB")
    img_w, img_h = img.size

    st.title("YOLO label editor")
    st.caption("Draw / resize / move rectangles, then assign classes and Save.")

    existing = read_yolo_label_file(label_path)

    # Initialize per-image class IDs based on existing boxes.
    key_prefix = str(image_path)
    cls_key = f"cls_ids::{key_prefix}"
    if cls_key not in st.session_state:
        st.session_state[cls_key] = [b.cls for b in existing]

    # Canvas
    init_drawing = {"version": "4.4.0", "objects": boxes_to_canvas_objects(existing, img_w=img_w, img_h=img_h)}

    col_left, col_right = st.columns([0.72, 0.28], gap="large")
    with col_left:
        canvas = st_canvas(
            fill_color="rgba(0, 0, 0, 0.0)",
            stroke_width=2,
            stroke_color="rgba(0, 255, 0, 0.9)",
            background_image=img,
            update_streamlit=True,
            height=img_h,
            width=img_w,
            drawing_mode="rect",
            initial_drawing=init_drawing,
            key=f"canvas::{key_prefix}",
        )

    rects: list[dict] = []
    if canvas.json_data and "objects" in canvas.json_data:
        rects = [o for o in canvas.json_data["objects"] if o.get("type") == "rect"]

    # Keep class-id list length aligned with number of rects.
    cls_ids: list[int] = list(st.session_state[cls_key])
    if len(cls_ids) < len(rects):
        cls_ids.extend([0] * (len(rects) - len(cls_ids)))
    if len(cls_ids) > len(rects):
        cls_ids = cls_ids[: len(rects)]
    st.session_state[cls_key] = cls_ids

    with col_right:
        st.subheader("Boxes")
        st.write(f"Image: `{image_path.relative_to(images_dir)}`")
        st.write(f"Label: `{label_path.relative_to(labels_dir)}`")

        if not rects:
            st.info("No boxes yet. Draw rectangles on the image.")
        else:
            for i in range(len(rects)):
                if classes:
                    name = classes[cls_ids[i]] if 0 <= cls_ids[i] < len(classes) else f"class {cls_ids[i]}"
                    chosen = st.selectbox(
                        f"Box {i} class",
                        options=list(range(len(classes))),
                        format_func=lambda j: f"{j}: {classes[j]}",
                        index=int(cls_ids[i]) if 0 <= cls_ids[i] < len(classes) else 0,
                        key=f"sel::{key_prefix}::{i}",
                    )
                    cls_ids[i] = int(chosen)
                else:
                    cls_ids[i] = int(
                        st.number_input(
                            f"Box {i} class id",
                            min_value=0,
                            value=int(cls_ids[i]),
                            step=1,
                            key=f"num::{key_prefix}::{i}",
                        )
                    )
            st.session_state[cls_key] = cls_ids

        st.divider()
        if st.button("Save labels", type="primary"):
            objs = rects
            boxes = canvas_objects_to_boxes(objs, img_w=img_w, img_h=img_h, class_ids=cls_ids)
            write_yolo_label_file(label_path, boxes)
            st.success(f"Saved {len(boxes)} boxes → {label_path}")

        c1, c2 = st.columns(2)
        with c1:
            if st.button("Prev") and st.session_state.img_idx > 0:
                st.session_state.img_idx -= 1
                st.rerun()
        with c2:
            if st.button("Next") and st.session_state.img_idx < len(images) - 1:
                st.session_state.img_idx += 1
                st.rerun()


if __name__ == "__main__":
    main()

