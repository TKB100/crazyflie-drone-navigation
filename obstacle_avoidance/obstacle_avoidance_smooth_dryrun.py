import time
import race as fast

class MockMotionCommander:
    def forward(self, distance, velocity=None):
        print(f"[DRYRUN] forward {distance}m")
    def back(self, distance, velocity=None):
        print(f"[DRYRUN] back {distance}m")
    def left(self, distance, velocity=None):
        print(f"[DRYRUN] left {distance}m")
    def right(self, distance, velocity=None):
        print(f"[DRYRUN] right {distance}m")

class MockMultiranger:
    def __init__(self):
        self.front = None; self.back = None; self.left = None; self.right = None; self.up = None
    def update(self):
        x = fast.current_x; y = fast.current_y
        self.front = 0.2 if (0.9 <= x <= 1.1 or 2.0 <= x <= 2.2) else None
        self.right = 0.2 if y < -0.45 else None
        self.left = 0.2 if y > 0.45 else None
        self.up = None; self.back = None

def run_dryrun(max_steps=300):
    motion = MockMotionCommander()
    multiranger = MockMultiranger()
    fast.position_log.clear()
    fast.collision_count = 0
    fast.current_x = 0.0; fast.current_y = 0.0; fast.keep_flying = True

    step = 0; start = time.time()
    while fast.keep_flying and step < max_steps:
        if time.time() - start > fast.MAX_RUN_TIME:
            print("[DRYRUN] max run time")
            break
        multiranger.update()
        fast.log_position()
        if fast.is_close(multiranger.up, thresh=fast.UP_MIN_DISTANCE):
            print("[DRYRUN] ceiling detected; landing")
            break
        if fast.has_reached_finish_line(fast.current_x):
            print(f"[DRYRUN] finish at x={fast.current_x:.2f}")
            break
        if not fast.is_within_boundary(fast.current_x, fast.current_y):
            print("[DRYRUN] out of bounds; correcting")
            # simple correction
            if abs(fast.current_x) > fast.BOX_LIMIT:
                if fast.current_x > 0:
                    motion.back(0.3, velocity=fast.MAX_VEL); fast.update_position(0.3,'back')
                else:
                    motion.forward(0.3, velocity=fast.MAX_VEL); fast.update_position(0.3,'forward')
            if abs(fast.current_y) > fast.BOX_LIMIT:
                if fast.current_y > 0:
                    motion.right(0.3, velocity=fast.MAX_VEL); fast.update_position(0.3,'right')
                else:
                    motion.left(0.3, velocity=fast.MAX_VEL); fast.update_position(0.3,'left')
            time.sleep(0.05); step += 1; continue
        obstacle = fast.navigate_with_avoidance(motion, multiranger)
        if not obstacle:
            motion.forward(fast.FORWARD_MOVE, velocity=fast.MAX_VEL)
            fast.update_position(fast.FORWARD_MOVE, 'forward')
            time.sleep(0.05)
        step += 1

    print("[DRYRUN] done; saving plot")
    fast.plot_path()

if __name__ == "__main__":
    run_dryrun()