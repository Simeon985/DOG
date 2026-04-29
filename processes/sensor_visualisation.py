import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time

class RobotSim:
    def __init__(self):
        # De gedeelde data
        self.history = [(0, 0, 0, 0)] # x, y, p_a, time
        self.pose = [0, 0]
        self.running = True

    def data_loop(self):
        """ Deze functie draait in een aparte thread (Bestand/Robot) """
        dt = 0.1
        while self.running:


            time.sleep(0.05) # Simuleert een sensor/robot vertraging

# --- Visualisatie Gedeelte ---

def setup_live_plot(estimator_object):
    fig, ax = plt.subplots(figsize=(8,8))
    line, = ax.plot([], [], 'b-', label='Peripheral')

    def init():
        ax.set_xlim(-5, 5) # Pas aan naar jouw bereik
        ax.set_ylim(-5, 5)
        return line,

    def update_frame(frame):
        print("--------------------------------------------------")
        # Haal de history uit het object dat in de andere thread wordt bijgewerkt
        if estimator_object.history:
            # We maken een kopie om 'thread collisions' te voorkomen tijdens het itereren
            current_history = list(estimator_object.history)
            x_peri = [pos[0] for pos in current_history]
            y_peri = [pos[1] for pos in current_history]

            line.set_data(x_peri, y_peri)

            # Schaal de assen dynamisch
            ax.relim()
            ax.autoscale_view()
        return line,

    print("-------------/------------------------------------")
    import itertools
    ani = FuncAnimation(fig, update_frame, frames=itertools.count(),
                        blit=True, interval=100, cache_frame_data=False)
    print("------------+-------------------------------------")
    plt.legend()
    plt.show() # Houdt de Main Thread bezig
    print("eee")
