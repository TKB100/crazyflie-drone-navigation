# Crazyflie Autonomous Drone Navigation

A collection of autonomous flight programs developed for the Crazyflie 2.1 drone using Python and the cflib library. This project covers basic flight control, random waypoint mobility, privacy-preserving navigation algorithms, real-time obstacle avoidance, and interactive drone games — all running on resource-constrained embedded hardware.

---

## Project Structure

```
crazyflie-drone-navigation/
├── labs/
│   └── lab0_basic_movement.py              # Basic takeoff, hover, and directional movement
├── project1/
│   ├── project1_random_waypoint_mobility.py         # Random Waypoint Mobility algorithm
│   └── project1_privacy_preserving_navigation.py    # Privacy-Preserving Random Waypoint Mobility
├── obstacle_avoidance/
│   ├── obstacle_avoidance_v1.py            # Basic front-sensor obstacle detection
│   ├── obstacle_avoidance_v2.py            # Improved avoidance with boundary detection
│   ├── obstacle_avoidance_v3.py            # Enhanced path planning with collision counting
│   ├── obstacle_avoidance_v4.py            # Obstacle avoidance with position logging
│   ├── obstacle_avoidance_smooth.py        # Ultra-smooth "water flow" navigation algorithm
│   ├── obstacle_avoidance_smooth_dryrun.py # Dry-run simulator (no drone required)
│   └── obstacle_avoidance_advanced.py      # Advanced avoidance — flies OVER obstacles vertically
└── games/
    └── drone_rps_game.py                   # Rock-Paper-Scissors game with drone movement and LED feedback
```

---

## Key Projects

### Project 1: Random Waypoint Mobility
`project1/project1_random_waypoint_mobility.py`

Implements the **Random Waypoint Mobility model** — a standard algorithm used in mobile network simulation. The drone generates random destination coordinates within a defined boundary and navigates to each waypoint sequentially, logging position data and plotting the flight path after landing.

- Random waypoint generation within configurable boundary limits
- Position logging and 2D matplotlib flight path visualization
- Configurable flight time, velocity, and boundary parameters

---

### Project 1 Part 2: Privacy-Preserving Navigation
`project1/project1_privacy_preserving_navigation.py`

Extends the Random Waypoint Mobility model with a **privacy-preserving algorithm**. Instead of flying directly to a destination, the drone generates random dummy waypoints within a virtual rectangle spanning the current position to the destination. The drone visits dummy waypoints before the actual target, obscuring the true navigation path from observers.

- Dummy waypoint generation within a dynamically calculated bounding rectangle
- Waypoints sorted by distance for efficient routing
- Distinct visualization — destinations shown as solid circles, dummy waypoints as hollow circles
- Implements concepts from privacy-preserving mobility research

---

### Obstacle Avoidance Suite
`obstacle_avoidance/`

A progression of obstacle avoidance algorithms developed and refined across multiple iterations:

- **v1-v2:** Basic multiranger sensor detection — stops and redirects when front sensor detects an obstacle
- **v3-v4:** Improved path planning — checks all four directions, tracks collision count, respects virtual boundaries
- **Smooth:** Ultra-smooth navigation inspired by water flowing around obstacles — consistent avoidance direction with stuck detection and recovery
- **Advanced:** Most sophisticated version — drone attempts to fly **vertically over obstacles** using incremental ascent, descending back to original altitude after clearing

All versions include:
- Real-time position tracking and boundary enforcement
- Matplotlib flight path visualization saved to PNG
- Graceful Ctrl+C handling for safe emergency landing
- Configurable velocity, boundary limits, and finish line

---

### Dry Run Simulator
`obstacle_avoidance/obstacle_avoidance_smooth_dryrun.py`

A mock simulator for testing the smooth navigation algorithm without a physical drone. Uses `MockMotionCommander` and `MockMultiranger` classes to simulate sensor readings and drone movement, allowing algorithm testing and debugging in a safe environment.

---

### Rock-Paper-Scissors Drone Game
`games/drone_rps_game.py`

An interactive game where the drone plays Rock-Paper-Scissors against the user:
- Drone moves **up** on a win, **down** on a loss, stays on a tie
- LED ring changes color — blue for win, red for loss, green for tie
- First to 2 wins or 2 losses ends the game
- Full game log and statistics displayed after landing

---

## Hardware Requirements

- **Crazyflie 2.1** drone
- **Flow Deck v2** — for optical flow positioning
- **Multiranger Deck** — for 360° obstacle detection (front, back, left, right, up)
- **Crazyradio PA** — for wireless communication

---

## Software Requirements

```bash
pip install cflib matplotlib
```

---

## How to Run

### Basic Movement (Lab 0)
```bash
python3 labs/lab0_basic_movement.py
```

### Privacy-Preserving Navigation (Main Project)
```bash
python3 project1/project1_privacy_preserving_navigation.py
```

### Obstacle Avoidance (Advanced)
```bash
python3 obstacle_avoidance/obstacle_avoidance_advanced.py
```

### Test Without a Drone (Dry Run)
```bash
python3 obstacle_avoidance/obstacle_avoidance_smooth_dryrun.py
```

### Rock-Paper-Scissors Game
```bash
python3 games/drone_rps_game.py
```

---

## Skills Demonstrated

- **Python** — real-time flight logic, signal handling, event-driven programming
- **Embedded Systems** — Crazyflie 2.1 hardware integration via cflib
- **Autonomous Navigation** — waypoint mobility, obstacle avoidance, path planning
- **Privacy-Preserving Algorithms** — dummy waypoint generation to obscure true navigation paths
- **Real-Time Sensor Processing** — multiranger deck integration for live obstacle detection
- **Data Visualization** — matplotlib flight path plots with position logging
- **Algorithm Design** — iterative development from basic to advanced avoidance strategies

---

## Author

**Triston Barrientos**
CS Senior — Texas Tech University (May 2026)
[LinkedIn](https://linkedin.com/in/triston00barrientos) | [GitHub](https://github.com/TKB100)
