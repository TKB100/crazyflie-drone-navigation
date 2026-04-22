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

# Define the default URI for communication with the Crazyflie
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E2')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# ==================== PARAMETERS - ULTRA SMOOTH ====================
DEFAULT_HEIGHT = 0.3    # Default flight height in meters
BOX_LIMIT = 4.0         # Boundary limits for movement in meters
MAX_VEL = 0.12          # SLOWER for smoothness (was 0.15)
MAX_RUN_TIME = 180      # Maximum run time in seconds
MIN_DISTANCE = 0.3      # Detect obstacles earlier (was 0.25)
FINISH_LINE_X = 3.5     # X-coordinate of the finish line
BACK_DISTANCE = 0.3     # How far to back up
SIDE_DISTANCE = 0.5     # How far to move sideways
FORWARD_PAST_OBSTACLE = 0.5  # How far forward past obstacle
FORWARD_MOVE = 0.15     # Larger normal forward steps (was 0.12)

# ==================== GLOBAL VARIABLES ====================
position_log = []
collision_count = 0
start_time = None
keep_flying = True
current_x = 0.0
current_y = 0.0
stuck_counter = 0
last_avoid_direction = None  # Remember which way we went

# ==================== SIGNAL HANDLER ====================
def signal_handler(sig, frame):
    global keep_flying
    print('\nCtrl+C detected! Landing...')
    keep_flying = False

signal.signal(signal.SIGINT, signal_handler)

# ==================== HELPER FUNCTIONS ====================
def is_close(range_value):
    return range_value is not None and range_value < MIN_DISTANCE

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
    global position_log
    position_log.append((current_x, current_y))
    return current_x, current_y

def plot_path():
    if len(position_log) < 2:
        print("Not enough position data to plot.")
        return
    
    x_coords = [pos[0] for pos in position_log]
    y_coords = [pos[1] for pos in position_log]
    
    plt.figure(figsize=(10, 8))
    plt.plot(x_coords, y_coords, 'b-', linewidth=2, label='Drone Path')
    plt.plot(x_coords, y_coords, 'bo', markerfacecolor='none', 
             markersize=8, markeredgewidth=1.5, label='Position Markers')
    plt.plot(x_coords[0], y_coords[0], 'go', markersize=12, label='Start')
    plt.plot(x_coords[-1], y_coords[-1], 'ro', markersize=12, label='End')
    
    boundary_x = [-BOX_LIMIT, BOX_LIMIT, BOX_LIMIT, -BOX_LIMIT, -BOX_LIMIT]
    boundary_y = [-BOX_LIMIT, -BOX_LIMIT, BOX_LIMIT, BOX_LIMIT, -BOX_LIMIT]
    plt.plot(boundary_x, boundary_y, 'k--', linewidth=2, label='Virtual Boundary')
    plt.axvline(x=FINISH_LINE_X, color='g', linestyle='--', linewidth=2, label='Finish Line')
    
    plt.xlabel('X Position (m)', fontsize=12)
    plt.ylabel('Y Position (m)', fontsize=12)
    plt.title(f'Drone Flight Path\nCollisions: {collision_count}', fontsize=14, fontweight='bold')
    plt.legend(loc='best')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    plt.savefig('flight_path.png', dpi=300, bbox_inches='tight')
    print(f"\nFlight path plot saved to: flight_path.png")
    plt.show()
    
    print(f"\n===== Flight Statistics =====")
    print(f"Total Collisions: {collision_count}")
    print(f"Total Position Samples: {len(position_log)}")
    print(f"Final Position: ({x_coords[-1]:.3f}, {y_coords[-1]:.3f})")
    print(f"=============================\n")

def smooth_wait(distance, velocity):
    """
    Calculate wait time for smooth continuous motion.
    Wait for 70% of movement - allows smooth overlap.
    """
    full_time = distance / velocity
    return full_time * 0.7  # 70% for very smooth transitions

def navigate_with_avoidance(motion_commander, multiranger):
    """
    Ultra-smooth obstacle avoidance - like water flowing around rocks.
    """
    global collision_count, stuck_counter, last_avoid_direction
    
    obstacle_detected = False
    
    front_blocked = is_close(multiranger.front)
    back_blocked = is_close(multiranger.back)
    left_blocked = is_close(multiranger.left)
    right_blocked = is_close(multiranger.right)
    
    if front_blocked:
        obstacle_detected = True
        collision_count += 1
        stuck_counter += 1
        
        print(f"🌊 Obstacle ahead - flowing around... (#{collision_count})")
        
        # Gentle back-up - smooth like water pulling back
        motion_commander.back(BACK_DISTANCE, velocity=MAX_VEL)
        update_position(BACK_DISTANCE, 'back')
        time.sleep(smooth_wait(BACK_DISTANCE, MAX_VEL))
        
        # Decide direction - prefer consistency (keep flowing same direction)
        can_go_right = not is_close(multiranger.right) and is_within_boundary(current_x, current_y - SIDE_DISTANCE)
        can_go_left = not is_close(multiranger.left) and is_within_boundary(current_x, current_y + SIDE_DISTANCE)
        
        # If we're stuck, try opposite direction
        if stuck_counter >= 3:
            print(f"  💫 Stuck! Trying opposite direction...")
            if last_avoid_direction == 'right':
                can_go_right = False  # Force left
            elif last_avoid_direction == 'left':
                can_go_left = False  # Force right
            stuck_counter = 0
        
        # Flow around the obstacle
        if can_go_right:
            print(f"  → Flowing RIGHT {SIDE_DISTANCE}m")
            last_avoid_direction = 'right'
            
            # Smooth sideways flow
            motion_commander.right(SIDE_DISTANCE, velocity=MAX_VEL)
            update_position(SIDE_DISTANCE, 'right')
            time.sleep(smooth_wait(SIDE_DISTANCE, MAX_VEL))
            
            # Brief sensor check
            time.sleep(0.2)
            
            # Continue flowing forward if clear
            if not is_close(multiranger.front):
                print(f"  ↑ Flowing forward {FORWARD_PAST_OBSTACLE}m")
                motion_commander.forward(FORWARD_PAST_OBSTACLE, velocity=MAX_VEL)
                update_position(FORWARD_PAST_OBSTACLE, 'forward')
                time.sleep(smooth_wait(FORWARD_PAST_OBSTACLE, MAX_VEL))
                stuck_counter = 0  # Success!
                
        elif can_go_left:
            print(f"  ← Flowing LEFT {SIDE_DISTANCE}m")
            last_avoid_direction = 'left'
            
            # Smooth sideways flow
            motion_commander.left(SIDE_DISTANCE, velocity=MAX_VEL)
            update_position(SIDE_DISTANCE, 'left')
            time.sleep(smooth_wait(SIDE_DISTANCE, MAX_VEL))
            
            # Brief sensor check
            time.sleep(0.2)
            
            # Continue flowing forward if clear
            if not is_close(multiranger.front):
                print(f"  ↑ Flowing forward {FORWARD_PAST_OBSTACLE}m")
                motion_commander.forward(FORWARD_PAST_OBSTACLE, velocity=MAX_VEL)
                update_position(FORWARD_PAST_OBSTACLE, 'forward')
                time.sleep(smooth_wait(FORWARD_PAST_OBSTACLE, MAX_VEL))
                stuck_counter = 0  # Success!
        
        else:
            # Both sides blocked - retreat more
            print(f"  ⬅ Both sides blocked - retreating")
            motion_commander.back(BACK_DISTANCE, velocity=MAX_VEL)
            update_position(BACK_DISTANCE, 'back')
            time.sleep(smooth_wait(BACK_DISTANCE, MAX_VEL))
    
    elif back_blocked:
        obstacle_detected = True
        collision_count += 1
        motion_commander.forward(0.2, velocity=MAX_VEL)
        update_position(0.2, 'forward')
        time.sleep(smooth_wait(0.2, MAX_VEL))
    
    elif left_blocked:
        obstacle_detected = True
        collision_count += 1
        if is_within_boundary(current_x, current_y - 0.2):
            motion_commander.right(0.2, velocity=MAX_VEL)
            update_position(0.2, 'right')
            time.sleep(smooth_wait(0.2, MAX_VEL))
    
    elif right_blocked:
        obstacle_detected = True
        collision_count += 1
        if is_within_boundary(current_x, current_y + 0.2):
            motion_commander.left(0.2, velocity=MAX_VEL)
            update_position(0.2, 'left')
            time.sleep(smooth_wait(0.2, MAX_VEL))
    
    else:
        # Clear path - reset counters
        if stuck_counter > 0:
            stuck_counter = 0
    
    return obstacle_detected

# ==================== MAIN PROGRAM ====================
if __name__ == '__main__':
    print("=" * 60)
    print("SkySensor: Ultra-Smooth Navigation 🌊")
    print("=" * 60)
    print(f"Parameters:")
    print(f"  - Default Height: {DEFAULT_HEIGHT}m")
    print(f"  - Max Velocity: {MAX_VEL}m/s (slower for smoothness)")
    print(f"  - Forward Step: {FORWARD_MOVE}m (larger steps)")
    print(f"  - Wait Strategy: 70% overlap for continuous flow")
    print(f"  - Side Distance: {SIDE_DISTANCE}m")
    print(f"  - Finish Line: X = {FINISH_LINE_X}m")
    print("=" * 60)
    print("Press Ctrl+C to stop the drone at any time.\n")
    
    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache='./cache')
    elapsed_time = 0
    
    try:
        with SyncCrazyflie(URI, cf=cf) as scf:
            print("Connected to Crazyflie!")
            scf.cf.platform.send_arming_request(True)
            print("Drone armed. Waiting for stabilization...")
            time.sleep(1.0)
            
            with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as motion_commander:
                print(f"Taking off to {DEFAULT_HEIGHT}m altitude...")
                time.sleep(2)
                
                with Multiranger(scf) as multiranger:
                    print("Multi-ranger activated. Starting ultra-smooth navigation...")
                    print(f"Virtual boundary: ±{BOX_LIMIT}m")
                    print(f"Finish line at X = {FINISH_LINE_X}m\n")
                    
                    start_time = time.time()
                    current_x = 0.0
                    current_y = 0.0
                    
                    while keep_flying:
                        elapsed_time = time.time() - start_time
                        if elapsed_time > MAX_RUN_TIME:
                            print(f"\nMaximum run time ({MAX_RUN_TIME}s) reached. Landing...")
                            break
                        
                        if is_close(multiranger.up):
                            print("\nObject detected above drone. Landing...")
                            break
                        
                        pos_x, pos_y = log_position()
                        
                        if has_reached_finish_line(pos_x):
                            print(f"\n🎉 Finish line reached at X = {pos_x:.3f}m!")
                            break
                        
                        if not is_within_boundary(pos_x, pos_y):
                            print(f"⚠️  Boundary at ({pos_x:.2f}, {pos_y:.2f})")
                            if abs(pos_x) > BOX_LIMIT:
                                if pos_x > 0:
                                    motion_commander.back(0.3, velocity=MAX_VEL)
                                    update_position(0.3, 'back')
                                else:
                                    motion_commander.forward(0.3, velocity=MAX_VEL)
                                    update_position(0.3, 'forward')
                                time.sleep(smooth_wait(0.3, MAX_VEL))
                            if abs(pos_y) > BOX_LIMIT:
                                if pos_y > 0:
                                    motion_commander.right(0.3, velocity=MAX_VEL)
                                    update_position(0.3, 'right')
                                else:
                                    motion_commander.left(0.3, velocity=MAX_VEL)
                                    update_position(0.3, 'left')
                                time.sleep(smooth_wait(0.3, MAX_VEL))
                            continue
                        
                        # Check for obstacles and navigate smoothly
                        obstacle_detected = navigate_with_avoidance(motion_commander, multiranger)
                        
                        # Continuous forward flow when clear
                        if not obstacle_detected:
                            motion_commander.forward(FORWARD_MOVE, velocity=MAX_VEL)
                            update_position(FORWARD_MOVE, 'forward')
                            # Minimal pause for continuous feel
                            time.sleep(0.08)  # Very brief - keeps motion fluid
                    
                    print("\nLanding drone...")
        
        print("\n" + "=" * 60)
        print("MISSION COMPLETE")
        print("=" * 60)
        print(f"Flight duration: {elapsed_time:.2f} seconds ⏱️")
        print(f"Total collisions: {collision_count}")
        print(f"Total positions logged: {len(position_log)}")
        if position_log:
            print(f"Final position: X = {position_log[-1][0]:.3f}m, Y = {position_log[-1][1]:.3f}m")
        print("=" * 60 + "\n")
        
        print("Generating flight path visualization...")
        plot_path()
        
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\nProgram terminated successfully!")