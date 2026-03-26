import time
import threading

def control(stop_signal : threading.Event, counter : list[int]):
    while not stop_signal.is_set():
        counter[0] += 1
        time.sleep(0.05)
    print("control thread closing")