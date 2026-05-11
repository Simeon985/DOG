import torch
import cv2
import onnxruntime as ort
import lerobot
import numpy as np
import time

def test_jetson_stack():
    print("--- Jetson Orin Nano AI Stack Check ---")
    
    # 1. Check PyTorch + CUDA
    print(f"\n[1/4] Checking PyTorch...")
    torch_gpu = torch.cuda.is_available()
    if torch_gpu:
        print(f"  ✅ PyTorch sees GPU: {torch.cuda.get_device_name(0)}")
        # Simple math stress test
        x = torch.rand(2000, 2000).cuda()
        y = torch.matmul(x, x)
        print("  ✅ Tensor Math on GPU: Success")
    else:
        print("  ❌ PyTorch CUDA not found! Check your torch installation.")

    # 2. Check ONNX Runtime (For InsightFace/YOLO)
    print(f"\n[2/4] Checking ONNX Runtime...")
    providers = ort.get_available_providers()
    if 'CUDAExecutionProvider' in providers:
        print(f"  ✅ ONNX CUDA Provider: Found")
    else:
        print(f"  ❌ ONNX CUDA Provider: MISSING (Found: {providers})")

    # 3. Check OpenCV (Hardware Accelerated)
    print(f"\n[3/4] Checking OpenCV...")
    print(f"  - Version: {cv2.__version__}")
    try:
        cuda_count = cv2.cuda.getCudaEnabledDeviceCount()
        if cuda_count > 0:
            print(f"  ✅ OpenCV CUDA Devices: {cuda_count}")
        else:
            print("  ⚠️ OpenCV: CPU only (No CUDA detected in build)")
    except AttributeError:
        print("  ❌ OpenCV: CUDA module not found in this CV2 build.")
    
    # Check GStreamer (Required for Jetson Cameras)
    build_info = cv2.getBuildInformation()
    if "GStreamer:" in build_info and "YES" in build_info.split("GStreamer:")[1].split("\n")[0]:
        print("  ✅ GStreamer Support: Yes")
    else:
        print("  ⚠️ GStreamer Support: No (Camera input might be slow)")

    # 4. Check LeRobot
    print(f"\n[4/4] Checking LeRobot...")
    try:
        print(f"  ✅ LeRobot Version: {lerobot.__version__}")
        print(f"  ✅ LeRobot Path: {lerobot.__file__}")
    except Exception as e:
        print(f"  ❌ LeRobot Error: {e}")

    print("\n" + "="*40)
    if torch_gpu and 'CUDAExecutionProvider' in providers:
        print("🚀 STATUS: SYSTEM READY FOR YOLO & ROBOTICS")
    else:
        print("⚠️ STATUS: DEGRADED PERFORMANCE (Check CUDA/Drivers)")
    print("="*40)

if __name__ == "__main__":
    test_jetson_stack()