import sys
import time
import logging
import signal
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger
import matplotlib.pyplot as plt

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E2')
logging.basicConfig(level=logging.ERROR)

# PARAMETERS
DEFAULT_HEIGHT = 3.0
BOX_LIMIT = 4.0
MAX_VEL = 0.12
MAX_RUN_TIME = 180
MIN_DISTANCE = 0.3
FINISH_LINE_X = 2.5
BACK_DISTANCE = 0.3
SIDE_DISTANCE = 0.3
FORWARD_PAST_OBSTACLE = 0.5
FORWARD_MOVE = 0.15

# tuning
UP_MIN_DISTANCE = 0.15
AVOID_COOLDOWN = 0.15

# Extra-credit: fly over obstacle params
OVER_HEIGHT = 0.25        # extra meters above DEFAULT_HEIGHT to clear the box
OVER_CLEARANCE = 0.20     # required extra headroom reported by up sensor before ascending

# Use this as original flight height (OG = 0.3m) and take off to this height
ORIGINAL_HEIGHT = 0.3
# incremental ascent step and max ascend
ASCEND_STEP = 0.05

# GLOBALS
position_log = []
collision_count = 0
start_time = None
keep_flying = True
current_x = 0.0
current_y = 0.0
stuck_counter = 0
last_avoid_direction = None
last_avoid_time = 0.0

def signal_handler(sig, frame):
    global keep_flying
    print('\nCtrl+C detected! Landing...')
    keep_flying = False

signal.signal(signal.SIGINT, signal_handler)

def is_close(range_value, thresh=None):
    if range_value is None:
        return False
    if thresh is None:
        thresh = MIN_DISTANCE
    return range_value < thresh

def front_is_clear(multiranger, samples=2, delay=0.08):
    for _ in range(samples):
        # refresh sensors if available (helps dry-run mocks)
        if hasattr(multiranger, 'update'):
            multiranger.update()
        if is_close(multiranger.front):
            return False
        time.sleep(delay)
    return True

def is_within_boundary(x, y):
    return abs(x) <= BOX_LIMIT and abs(y) <= BOX_LIMIT

def has_reached_finish_line(x):
    return x >= FINISH_LINE_X

def update_position(distance, direction):
    global current_x, current_y
    if direction == 'forward':
        current_x += distance
    elif direction == 'back':
        current_x -= distance
    elif direction == 'left':
        current_y += distance
    elif direction == 'right':
        current_y -= distance

def log_position():
    position_log.append((current_x, current_y))
    return current_x, current_y

def plot_path():
    if len(position_log) < 2:
        print("Not enough position data to plot.")
        return
    x_coords = [p[0] for p in position_log]
    y_coords = [p[1] for p in position_log]
    plt.figure(figsize=(8,6))
    plt.plot(x_coords, y_coords, '-o', markersize=4)
    plt.plot(x_coords[0], y_coords[0], 'go', label='Start')
    plt.plot(x_coords[-1], y_coords[-1], 'ro', label='End')
    boundary_x = [-BOX_LIMIT, BOX_LIMIT, BOX_LIMIT, -BOX_LIMIT, -BOX_LIMIT]
    boundary_y = [-BOX_LIMIT, -BOX_LIMIT, BOX_LIMIT, BOX_LIMIT, -BOX_LIMIT]
    plt.plot(boundary_x, boundary_y, 'k--', linewidth=1)
    plt.axvline(FINISH_LINE_X, color='g', linestyle='--')
    plt.xlabel('X (m)'); plt.ylabel('Y (m)'); plt.axis('equal'); plt.grid(True)
    plt.legend()
    plt.savefig('flight_path.png', dpi=200, bbox_inches='tight')
    print("Saved flight_path.png")

def smooth_wait(distance, velocity):
    return (distance / velocity) * 0.7

def navigate_with_avoidance(motion_commander, multiranger):
    global collision_count, stuck_counter, last_avoid_direction, last_avoid_time
    if time.time() - last_avoid_time < AVOID_COOLDOWN:
        return False

    # refresh sensors before making avoidance decisions
    if hasattr(multiranger, 'update'):
        multiranger.update()

    # debug: show which sensors look blocked when avoidance starts
    print(f"[NAV] front={multiranger.front} left={multiranger.left} right={multiranger.right} back={multiranger.back}")

    front_blocked = is_close(multiranger.front)
    back_blocked = is_close(multiranger.back)
    left_blocked = is_close(multiranger.left)
    right_blocked = is_close(multiranger.right)
    obstacle_detected = False

    # If any forward blocking is detected, always try to go over the obstacle
    if front_blocked:
        obstacle_detected = True
        collision_count += 1
        stuck_counter += 1
        last_avoid_time = time.time()

        print(f"Obstacle ahead ({collision_count}) — attempting to fly over")

        # small retreat to get clearance before ascending
        motion_commander.back(BACK_DISTANCE, velocity=MAX_VEL)
        update_position(BACK_DISTANCE, 'back')
        time.sleep(smooth_wait(BACK_DISTANCE, MAX_VEL))
        time.sleep(0.12)  # allow sensors to stabilize

        # Decide if it's safe to ascend: if up reading is None assume ok, otherwise require headroom
        can_ascend = False
        # refresh up sensor before decision
        if hasattr(multiranger, 'update'):
            multiranger.update()
        up_reading = getattr(multiranger, 'up', None)
        print(f"[NAV] up={up_reading}")  # debug - remove later
        if up_reading is None:
            can_ascend = True
        else:
            # Use ORIGINAL_HEIGHT (actual takeoff height) when checking headroom
            required_headroom = ORIGINAL_HEIGHT + OVER_HEIGHT + OVER_CLEARANCE
            if up_reading > required_headroom:
                can_ascend = True

        if can_ascend and hasattr(motion_commander, 'up') and hasattr(motion_commander, 'down'):
            # incremental ascent until front is clear or we reached OVER_HEIGHT
            ascended = 0.0
            print("  Attempting incremental ascent to clear obstacle")
            # set avoidance lock so we don't re-enter repeatedly
            last_avoid_time = time.time()
            while ascended < OVER_HEIGHT:
                step = min(ASCEND_STEP, OVER_HEIGHT - ascended)
                try:
                    motion_commander.up(step, velocity=MAX_VEL)
                except TypeError:
                    motion_commander.up(step)
                ascended += step
                # allow motion and sensors to stabilize (longer wait helps real hardware)
                time.sleep(0.25)
                if hasattr(multiranger, 'update'):
                    multiranger.update()
                # require the front to be clearly free before proceeding
                if not is_close(multiranger.front):
                    print(f"  Front clear after ascending {ascended:.2f}m")
                    break

            # safety: if front still blocked after full ascend, descend back and retreat
            if is_close(multiranger.front):
                print("  Front still blocked after full ascend — descending and retreating")
                if ascended > 0:
                    try:
                        motion_commander.down(ascended, velocity=MAX_VEL)
                    except TypeError:
                        motion_commander.down(ascended)
                    time.sleep(0.25)
                motion_commander.back(BACK_DISTANCE, velocity=MAX_VEL)
                update_position(BACK_DISTANCE, 'back')
                time.sleep(smooth_wait(BACK_DISTANCE, MAX_VEL))
                last_avoid_time = time.time()
            

            # move forward over the obstacle
            print(f"  Moving forward {FORWARD_PAST_OBSTACLE}m over obstacle")
            motion_commander.forward(FORWARD_PAST_OBSTACLE, velocity=MAX_VEL)
            update_position(FORWARD_PAST_OBSTACLE, 'forward')
            time.sleep(smooth_wait(FORWARD_PAST_OBSTACLE, MAX_VEL))

            # descend back to ORIGINAL_HEIGHT (descend by ascended amount)
            if ascended > 0.0:
                print(f"  Descending {ascended:.2f}m back to original height ({ORIGINAL_HEIGHT}m)")
                try:
                    motion_commander.down(ascended, velocity=MAX_VEL)
                except TypeError:
                    motion_commander.down(ascended)
                time.sleep(0.18)

            last_avoid_time = time.time()
            stuck_counter = 0
        else:
            # cannot ascend safely — fall back to a short retreat
            print("  Insufficient headroom to ascend — retreating")
            motion_commander.back(BACK_DISTANCE, velocity=MAX_VEL)
            update_position(BACK_DISTANCE, 'back')
            time.sleep(smooth_wait(BACK_DISTANCE, MAX_VEL))

    elif back_blocked or left_blocked or right_blocked:
        # treat other blocked directions conservatively by attempting a short forward/back correction,
        # then try the same over strategy on the next loop if front becomes blocked
        obstacle_detected = True
        collision_count += 1
        print("Side/back sensor triggered — performing conservative correction")
        if back_blocked:
            motion_commander.forward(0.2, velocity=MAX_VEL)
            update_position(0.2, 'forward')
            time.sleep(smooth_wait(0.2, MAX_VEL))
        else:
            # small lateral correction away from the blocked side
            if left_blocked and hasattr(motion_commander, 'right'):
                motion_commander.right(0.2, velocity=MAX_VEL); update_position(0.2, 'right'); time.sleep(smooth_wait(0.2, MAX_VEL))
            if right_blocked and hasattr(motion_commander, 'left'):
                motion_commander.left(0.2, velocity=MAX_VEL); update_position(0.2, 'left'); time.sleep(smooth_wait(0.2, MAX_VEL))

    else:
        if stuck_counter > 0:
            stuck_counter = 0

    return obstacle_detected

if __name__ == '__main__':
    print("Starting SkySensor (race.py)")
    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache='./cache')
    elapsed_time = 0.0

    try:
        with SyncCrazyflie(URI, cf=cf) as scf:
            scf.cf.platform.send_arming_request(True)
            # start at ORIGINAL_HEIGHT (takeoff height = 0.3m)
            with MotionCommander(scf, default_height=ORIGINAL_HEIGHT) as motion_commander:
                with Multiranger(scf) as multiranger:
                    print("Stabilizing sensors...")
                    # small stabilization: update sensors a few times and wait
                    for _ in range(6):
                        if hasattr(multiranger, 'update'):
                            multiranger.update()
                        time.sleep(0.08)
                    time.sleep(0.4)  # give IMU / controller a moment too

                    start_time = time.time()
                    current_x = 0.0
                    current_y = 0.0

                    # mark time when we consider sensors stable
                    sensors_stable_time = time.time()
                    while keep_flying:
                        # refresh sensors and print debug telemetry (remove or lower verbosity later)
                        if hasattr(multiranger, 'update'):
                            multiranger.update()
                        print(f"[SENSORS] front={multiranger.front} left={multiranger.left} right={multiranger.right} up={multiranger.up} back={multiranger.back}")
                        elapsed_time = time.time() - start_time

                        # require a short stabilization window before any avoidance sidestep
                        if time.time() - sensors_stable_time < 0.8:
                            # only allow gentle forward after sensors stabilized; skip avoidance for first ~0.8s
                            if front_is_clear(multiranger, samples=2, delay=0.06):
                                motion_commander.forward(FORWARD_MOVE / 2.0, velocity=MAX_VEL)  # very small initial step
                                update_position(FORWARD_MOVE / 2.0, 'forward')
                                time.sleep(0.06)
                            else:
                                print("Waiting for stable front sensor...")
                            continue

                        if elapsed_time > MAX_RUN_TIME:
                            break
                        if is_close(multiranger.up, thresh=UP_MIN_DISTANCE):
                            print("Ceiling too close — landing")
                            break
                        pos_x, pos_y = log_position()
                        if has_reached_finish_line(pos_x):
                            print("Finish line reached")
                            break
                        if not is_within_boundary(pos_x, pos_y):
                            if abs(pos_x) > BOX_LIMIT:
                                if pos_x > 0:
                                    motion_commander.back(0.3, velocity=MAX_VEL); update_position(0.3,'back')
                                else:
                                    motion_commander.forward(0.3, velocity=MAX_VEL); update_position(0.3,'forward')
                                time.sleep(smooth_wait(0.3, MAX_VEL))
                            if abs(pos_y) > BOX_LIMIT:
                                if pos_y > 0:
                                    motion_commander.right(0.3, velocity=MAX_VEL); update_position(0.3,'right')
                                else:
                                    motion_commander.left(0.3, velocity=MAX_VEL); update_position(0.3,'left')
                                time.sleep(smooth_wait(0.3, MAX_VEL))
                            continue
                        obstacle = navigate_with_avoidance(motion_commander, multiranger)
                        if not obstacle:
                            # require a short debounce that front is clear before moving forward
                            if front_is_clear(multiranger, samples=2, delay=0.06):
                                motion_commander.forward(FORWARD_MOVE, velocity=MAX_VEL)
                                update_position(FORWARD_MOVE, 'forward')
                                time.sleep(0.08)
                            else:
                                print("Holding: front not stable-clear")
    except Exception as e:
        print("Error:", e)
    finally:
        print("Run complete — saving log/plot")
        print(f"Duration: {elapsed_time:.2f}s, collisions: {collision_count}, samples: {len(position_log)}")
        plot_path()