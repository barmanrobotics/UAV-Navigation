# UAV Navigation Classes

This package contains well-structured, well-documented classes for UAV navigation that encapsulate all functionality from the `drone_pi` and `tower_pi` folders.

## Overview

The package provides two main classes:

- **Drone**: Comprehensive drone control class with MAVLink communication, flight operations, precision landing, and more
- **Tower**: Multi-drone coordination class with GPS data processing, collision avoidance, flap control, and server management

## Installation

### Prerequisites

```bash
pip install pymavlink opencv-python numpy RPi.GPIO
```

### Import

```python
from classes import Drone, Tower
```

## Drone Class

The `Drone` class provides comprehensive drone control functionality.

### Basic Usage

```python
# Create drone instance
drone = Drone(
    mavlink_port=14551,
    hub_ip='10.203.121.89',
    hub_port=14551,
    use_pi_camera=True
)

# Start the drone system
if drone.start():
    print("Drone system started successfully")
    
    # Perform flight operations
    drone.takeoff(altitude=5.0)
    drone.waypoint(10, 10, 5)
    drone.land()
    
    # Stop the system
    drone.stop()
```

### Context Manager Usage

```python
with Drone() as drone:
    # All operations within this block
    drone.takeoff(3.0)
    drone.send_velocity(1, 0, 0)  # Move forward
    time.sleep(5)
    drone.land()
```

### Key Methods

#### Connection and Setup
- `connect_mavlink()`: Connect to drone via MAVLink
- `connect_to_hub()`: Connect to tower/hub
- `start()`: Start all threads and connections
- `stop()`: Stop all threads and clean up

#### Flight Operations
- `takeoff(altitude)`: Take off to specified altitude
- `land()`: Land the drone
- `waypoint(x, y, z)`: Navigate to waypoint
- `rth(altitude)`: Return to home
- `send_velocity(vx, vy, vz)`: Send velocity commands
- `send_yaw_command(yaw, speed, relative)`: Control yaw

#### Flight Mode Control
- `set_flight_mode(mode)`: Set flight mode (GUIDED, RTL, LAND, etc.)
- `get_flight_mode()`: Get current flight mode
- `arm_disarm_drone(value)`: Arm (1) or disarm (0) drone

#### Advanced Features
- `precision_landing()`: Execute precision landing with ArUco markers
- `avoid_obstacle()`: Execute obstacle avoidance maneuver
- `connect_camera()`: Connect to camera for vision
- `detect_aruco_tags(frame)`: Detect ArUco markers

#### Data Access
- `get_yaw()`: Get current yaw angle
- `get_home_gps()`: Get home GPS coordinates

## Tower Class

The `Tower` class manages multiple drones and provides coordination services.

### Basic Usage

```python
# Create tower instance
tower = Tower(
    host='0.0.0.0',
    ports=[14551, 14552],
    server_port=6542,
    comm_port=6553,
    debug_gps=False,
    debug_avoidance=True
)

# Start the tower server
if tower.start_server():
    print("Tower server started")
    
    # Send commands to drones
    tower.send_command_to_drone("1", "TAKEOFF 5")
    tower.send_command_to_drone("2", "WAYPOINT 10 10 5")
    
    # Control flaps
    tower.open_flaps()
    tower.close_flaps()
    
    # Stop the server
    tower.stop()
```

### Context Manager Usage

```python
with Tower() as tower:
    # Server is automatically started
    print(f"Connected drones: {tower.get_drone_count()}")
    
    # Get drone data
    drone_data = tower.get_all_drone_data()
    print(f"Drone data: {drone_data}")
    
    # Send commands
    tower.send_command_to_drone("1", "RTH")
```

### Key Methods

#### Server Management
- `start_server(port)`: Start tower server
- `stop()`: Stop server and clean up
- `get_drone_count()`: Get number of connected drones

#### Drone Communication
- `send_command_to_drone(drone_id, command)`: Send command to specific drone
- `is_drone_connected(drone_id)`: Check if drone is connected
- `get_drone_data(drone_id)`: Get GPS data for specific drone
- `get_all_drone_data()`: Get GPS data for all drones

#### Flap Control
- `open_flaps()`: Open landing platform flaps
- `close_flaps()`: Close landing platform flaps
- `move_stepper(steps, direction)`: Control stepper motor

#### Collision Avoidance
- `check_collision_avoidance(drone_id, lat, lon, alt)`: Check for collisions
- `trigger_avoidance(drone1, drone2)`: Trigger avoidance for two drones

#### Distance Calculations
- `haversine(coord1, coord2)`: Calculate distance between GPS coordinates

#### Interactive Interface
- `send_command()`: Start interactive command interface

## Command Reference

### Drone Commands

The tower can send these commands to drones:

- `TAKEOFF <altitude>`: Take off to specified altitude
- `WAYPOINT <x> <y> <z>`: Navigate to waypoint
- `ABSOLUTE_WAYPOINT <x> <y> <z>`: Navigate to absolute waypoint
- `RTH`: Return to home
- `LAND`: Land the drone
- `PRECISION_LAND`: Execute precision landing
- `AVOID`: Execute obstacle avoidance

### Tower Commands

The tower accepts these commands:

- `OPEN_FLAPS`: Open landing platform flaps
- `CLOSE_FLAPS`: Close landing platform flaps
- `ENABLE_AVOIDANCE`: Enable collision avoidance
- `DISABLE_AVOIDANCE`: Disable collision avoidance
- `SEND_COMMAND <drone_id> <command>`: Send command to specific drone

## Configuration

### Drone Configuration

```python
drone = Drone(
    mavlink_port=14551,      # MAVLink connection port
    hub_ip='10.203.121.89',  # Tower IP address
    hub_port=14551,          # Tower port
    use_pi_camera=True       # Use Raspberry Pi camera
)
```

### Tower Configuration

```python
tower = Tower(
    host='0.0.0.0',          # Bind to all interfaces
    ports=[14551, 14552],    # Ports for drone connections
    server_port=6542,        # Dashboard server port
    comm_port=6553,          # Command communication port
    debug_gps=False,         # Enable GPS debugging
    debug_avoidance=True     # Enable avoidance debugging
)
```

## GPIO Configuration

The Tower class uses the following GPIO pins for flap control:

- **Step Pin**: GPIO 17
- **Direction Pin**: GPIO 18
- **Enable Pin**: GPIO 27
- **Limit Switch Pin**: GPIO 22

## Error Handling

Both classes include comprehensive error handling:

```python
try:
    with Drone() as drone:
        if not drone.takeoff(5.0):
            print("Takeoff failed")
        # ... other operations
except Exception as e:
    print(f"Drone error: {e}")
```

## Threading

Both classes use threading for concurrent operations:

- **Drone**: GPS transmission and command reception threads
- **Tower**: Server loop, client handling, and periodic update threads

## Logging

The classes use Python's logging module:

```python
import logging
logging.basicConfig(level=logging.INFO)
```

## Examples

### Multi-Drone Mission

```python
from classes import Drone, Tower
import time

# Start tower
with Tower() as tower:
    # Wait for drones to connect
    time.sleep(5)
    
    # Send commands to multiple drones
    tower.send_command_to_drone("1", "TAKEOFF 5")
    tower.send_command_to_drone("2", "TAKEOFF 5")
    
    time.sleep(10)
    
    # Send waypoints
    tower.send_command_to_drone("1", "WAYPOINT 10 0 5")
    tower.send_command_to_drone("2", "WAYPOINT 0 10 5")
    
    time.sleep(20)
    
    # Return to home
    tower.send_command_to_drone("1", "RTH")
    tower.send_command_to_drone("2", "RTH")
```

### Precision Landing

```python
from classes import Drone

with Drone() as drone:
    # Take off
    drone.takeoff(10.0)
    
    # Search for landing target
    if drone.precision_landing():
        print("Precision landing successful")
    else:
        print("Precision landing failed")
```

### Obstacle Avoidance

```python
from classes import Tower

with Tower() as tower:
    # Enable collision avoidance
    tower.process_server_command("ENABLE_AVOIDANCE")
    
    # The tower will automatically detect when drones are too close
    # and send avoidance commands
```

## Dependencies

- `pymavlink`: MAVLink communication
- `opencv-python`: Computer vision and ArUco detection
- `numpy`: Numerical operations
- `RPi.GPIO`: GPIO control (for Tower class)

## Notes

- The classes are designed to be thread-safe
- All methods include proper error handling
- The classes can be used as context managers for automatic cleanup
- GPS data is automatically transmitted to the dashboard
- Collision avoidance is built into the Tower class
- Flap control requires proper GPIO setup on Raspberry Pi



