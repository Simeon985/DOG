from ultralytics import YOLO

model = YOLO("balls_ourdata_augmented.pt")
model.export(
    format="engine",
    device=0,          # GPU
    half=True,         # FP16 — big speedup on Jetson, usually negligible accuracy loss
    imgsz=640,         # must match what you'll inference at
    workspace=4,       # GiB of TRT builder workspace
)