import time
import threading

def control(stop_signal : threading.Event):
    while not stop_signal.is_set():
        time.sleep(1.2)