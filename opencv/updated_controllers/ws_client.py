import asyncio
import websockets

async def receive_gps():
    uri = "ws://127.0.0.1:8765"
    async with websockets.connect(uri) as websocket:
        while True:
            data = await websocket.recv()
            print("📡 GPS Data:", data)

asyncio.run(receive_gps())
