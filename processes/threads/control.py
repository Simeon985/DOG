import time
import threading
import numpy as np
from multiprocessing.sharedctypes import SynchronizedArray


# ── shared_array layout ──────────────────────────────────────────
# [0] ball_detected       (0.0 / 1.0)
# [1] ball_coordinates_x  (world x)
# [2] ball_coordinates_y  (world y)
# [3] ball_thrown         (0.0 / 1.0)
# [4] estimated_ball_x    (world x)
# [5] estimated_ball_y    (world y)
# [6] person_detected     (0.0 / 1.0)
# [7] estimated_person_x  (world x)
# [8] estimated_person_y  (world y)
# [9] ball_grabbed        (0.0 / 1.0)
# [10] camera initialized (0.0 / 1.0)
# ────────────────────────────────────────────────────────────────


def _search_for_ball(shared_array: SynchronizedArray) -> bool:
    """Tell camera process to search for ball. Returns True when found."""
    # TODO: trigger camera search mode via shared_array
    raise NotImplementedError


def _center_ball_on_screen(shared_array: SynchronizedArray) -> tuple[float, float]:
    """Align robot so ball is centered. Returns ball screen coordinates."""
    # TODO: read ball_coordinates from shared_array, adjust heading
    raise NotImplementedError


def _drive_to_coordinates(x: float, y: float) -> bool:
    """Drive to world coordinates. Returns True when arrived."""
    # TODO: use estimator pose + motor control
    raise NotImplementedError


def _drive_to_predefined_distance_from_ball(shared_array: SynchronizedArray) -> None:
    """Drive to predefined distance from ball (prep for pickup)."""
    # TODO: stop at pickup distance using camera feedback
    raise NotImplementedError


def _scan_360() -> None:
    """Rotate 360° to scan environment."""
    # TODO: rotate robot using motor control
    raise NotImplementedError


def _ball_pickup_script(shared_array: SynchronizedArray) -> bool:
    """Execute ball pickup. Returns True when ball is grabbed."""
    # TODO: trigger arm/gripper, confirm via shared_array[7]
    raise NotImplementedError


def _drive_to_start() -> None:
    """Drive back to start position."""
    # TODO: navigate to origin using history/pose
    raise NotImplementedError


def _drive_to_person(shared_array: SynchronizedArray) -> None:
    """Drive to person coordinates provided by camera process."""
    # TODO: read person coordinates from shared_array
    raise NotImplementedError


def _drive_to_predefined_position_near_person(shared_array: SynchronizedArray) -> None:
    """Drive to predefined presentation position near person."""
    # TODO: stop at presentation distance
    raise NotImplementedError


def _ball_presentation_script() -> None:
    """Present ball to person (e.g. extend arm)."""
    # TODO: trigger arm/gripper for presentation
    raise NotImplementedError


def _wait_for_ball_grabbed(shared_array: SynchronizedArray, stop_event: threading.Event) -> bool:
    """Block until camera confirms ball is grabbed. Returns False if stopped."""
    while not stop_event.is_set():
        with shared_array.get_lock():
            if shared_array[7] == 1.0:
                return True
        time.sleep(0.05)
    return False


# ── main control loop ────────────────────────────────────────────

def control(
    stop_event: threading.Event,
    counter: list[int],
    shared_array: SynchronizedArray,
) -> None:
    """
    Main control thread. Runs the full mission loop until stop_event is set.

    Mission loop:
        search for ball
        → center + get coordinates
        → [if ball thrown] drive to estimated coordinates
        → [if arrived, no ball] 360° scan
        → drive to predefined distance from ball
        → ball pickup
        → return to start
        → [person detected] drive to person
        → [no person] 360° scan
        → drive to presentation position
        → ball presentation
        → wait for grab
        → history.clear()
        → restart
    """

    mission    = False
    returning  = False

    while not stop_event.is_set():
        #print(f"Person detected: {shared_array[6]}")
        time.sleep(0.1)
        continue
        counter[0] += 1

        # ── PHASE 1: find ball ───────────────────────────────────
        ball_detected = _search_for_ball(shared_array)

        if not ball_detected:
            # Should not happen at start — just retry
            time.sleep(0.1)
            continue

        ball_x, ball_y = _center_ball_on_screen(shared_array)
        mission = True

        # ── PHASE 2: ball thrown? ────────────────────────────────
        with shared_array.get_lock():
            ball_thrown = shared_array[3] == 1.0
            est_x       = shared_array[4]
            est_y       = shared_array[5]

        if ball_thrown:
            arrived = _drive_to_coordinates(est_x, est_y)

            if arrived:
                with shared_array.get_lock():
                    ball_detected = shared_array[0] == 1.0

                if not ball_detected:
                    # ── PHASE 3: 360° scan ───────────────────────
                    _scan_360()

                    with shared_array.get_lock():
                        ball_detected = shared_array[0] == 1.0

                    if not ball_detected:
                        # IDK — no ball found after scan
                        # TODO: define fallback behaviour
                        print("Ball not found after 360° scan — IDK")
                        mission = False
                        continue

        # ── PHASE 4: approach + pickup ───────────────────────────
        _drive_to_predefined_distance_from_ball(shared_array)
        pickup_success = _ball_pickup_script(shared_array)

        if not pickup_success:
            # TODO: handle pickup failure
            continue

        # ── PHASE 5: return to start ─────────────────────────────
        _drive_to_start()
        mission   = False
        returning = True

        # ── PHASE 6: find person ─────────────────────────────────
        with shared_array.get_lock():
            person_detected = shared_array[6] == 1.0

        if person_detected:
            _drive_to_person(shared_array)
        else:
            # ── PHASE 7: 360° scan for person ────────────────────
            _scan_360()

            with shared_array.get_lock():
                person_detected = shared_array[6] == 1.0

            if not person_detected:
                # IDK — no person found after scan
                # TODO: define fallback behaviour
                print("Person not found after 360° scan — IDK")
                returning = False
                continue

        # ── PHASE 8: present ball ────────────────────────────────
        _drive_to_predefined_position_near_person(shared_array)
        returning = False

        _ball_presentation_script()
        grabbed = _wait_for_ball_grabbed(shared_array, stop_event)

        if not grabbed:
            break  # stop_event was set during wait

        # ── PHASE 9: restart ─────────────────────────────────────
        # history.clear() equivalent — reset shared state
        with shared_array.get_lock():
            for i in range(len(shared_array)):
                shared_array[i] = 0.0

        print(f"Mission complete — restarting. Cycle count: {counter[0]}")
        # loop back to top → PHASE 1

    print("control thread closing")
