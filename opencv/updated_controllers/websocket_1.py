import asyncio
import json
import time
from pymavlink import mavutil
import websockets

# Connect to the drone using MAVLink
connection = mavutil.mavlink_connection('udpin:localhost:14552')
connection.wait_heartbeat()
print("✅ Connected to the vehicle")

# Set message interval for GLOBAL_POSITION_INT (GPS)
def set_message_interval(rate_hz, message_id):
    interval_us = int(1e6 / rate_hz)
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0, 0, 0, 0, 0
    )

set_message_interval(10, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT)

# Function to get GPS data
def get_drone_gps():
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        relative_alt = msg.relative_alt / 1000.0
        return {
            "lat": lat,
            "lon": lon,
            "relative_alt": relative_alt,
            "timestamp": time.time()
        }
    return None

# Set of connected clients
connected_clients = set()

# Handle each client connection
async def gps_stream_handler(websocket):
    print("🔌 Client connected")
    connected_clients.add(websocket)
    try:
        while True:
            gps_data = get_drone_gps()
            print(gps_data)
            if gps_data:
                data_str = json.dumps(gps_data)
                await websocket.send(data_str)
            await asyncio.sleep(0.1)
    except websockets.exceptions.ConnectionClosed:
        print("❌ Client disconnected")
    finally:
        connected_clients.remove(websocket)

# Start WebSocket server
async def main():
    server = await websockets.serve(gps_stream_handler, "0.0.0.0", 8765)
    print("🚀 WebSocket GPS server running on ws://0.0.0.0:8765")
    await server.wait_closed()

# Run the server
asyncio.run(main())
