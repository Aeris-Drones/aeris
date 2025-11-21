import asyncio
import json
import websockets
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('MockBridge')

connected_clients = set()

async def handler(websocket):
    logger.info(f"Client connected: {websocket.remote_address}")
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            data = json.loads(message)
            op = data.get('op')

            if op == 'subscribe':
                logger.info(f"Client subscribed to {data.get('topic')}")

            elif op == 'publish':
                topic = data.get('topic')
                msg = data.get('msg')

                broadcast_msg = {
                    'op': 'publish',
                    'topic': topic,
                    'msg': msg
                }
                broadcast_json = json.dumps(broadcast_msg)

                # Broadcast
                # Copy set to avoid size change during iteration if disconnects happen
                for client in list(connected_clients):
                    if client != websocket:
                        try:
                            await client.send(broadcast_json)
                        except Exception as e:
                            # logger.error(f"Failed to send to client: {e}")
                            pass # Client might be closed

    except Exception as e:
        logger.info(f"Client connection error/closed: {e}")
    finally:
        connected_clients.remove(websocket)
        logger.info("Client removed")

async def main():
    # Stop using 'serve' directly if we need to configure loop? No, serve is fine.
    # Note: websockets 14+ uses a different serve API sometimes, but this should work.
    async with websockets.serve(handler, "0.0.0.0", 9090):
        logger.info("Mock ROS Bridge running on ws://0.0.0.0:9090")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())
