## images_to_yolo (semi-auto labeling)

This tool helps you label an existing image dataset for YOLO training:

- **Prelabel**: run an existing YOLO model on your images → writes YOLO `.txt` labels.
- **Annotate**: open a **browser UI** to adjust/add/remove boxes → saves YOLO labels.

### Install (in your `lerobot` env)

```bash
pip install ultralytics streamlit streamlit-drawable-canvas pillow
```

### 1) Prelabel with an existing YOLO model

```bash
python /home/dog/DOG/lerobot/examples/phone_to_so100/images_to_yolo/main.py prelabel \
  --images-dir /ABS/PATH/TO/IMAGES \
  --labels-dir /ABS/PATH/TO/LABELS \
  --model yolov8n.pt \
  --conf 0.25
```

Optional class remapping (if the model’s class IDs don’t match your dataset IDs):

```bash
python /home/dog/DOG/lerobot/examples/phone_to_so100/images_to_yolo/main.py prelabel \
  --images-dir /ABS/PATH/TO/IMAGES \
  --labels-dir /ABS/PATH/TO/LABELS \
  --model /path/to/weights.pt \
  --class-map "0:2,1:0"
```

### 2) Review/edit in the browser UI

```bash
python /home/dog/DOG/lerobot/examples/phone_to_so100/images_to_yolo/main.py annotate \
  --images-dir /ABS/PATH/TO/IMAGES \
  --labels-dir /ABS/PATH/TO/LABELS \
  --classes "class0,class1,class2" \
  --port 8501
```

Then open `http://<your-ip>:8501` in a browser.

### Output format

For each image `something.jpg`, the label file is written as:

- `LABELS_DIR/something.txt` (same relative subfolders as your images)

Each line is standard YOLO format:

`class_id x_center y_center width height` (all normalized to `[0,1]`).

