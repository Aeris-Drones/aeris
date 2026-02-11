"""Mock ROS bridge server for testing WebSocket clients without a full ROS stack.

This module implements a lightweight WebSocket server that mimics the rosbridge
protocol for testing purposes. It handles subscribe/publish operations and
broadcasts messages to all connected clients.

Usage:
    python mock_rosbridge.py

The server runs on ws://0.0.0.0:9090 and supports the following operations:
    - subscribe: Client subscribes to a ROS topic
    - publish: Client publishes a message that gets broadcast to other clients

Example:
    Connect with a WebSocket client and send:
    {"op": "subscribe", "topic": "/test/topic"}
    {"op": "publish", "topic": "/test/topic", "msg": {"data": "hello"}}
"""

import asyncio
import json
import websockets
import logging

# Configure logging for connection events and debugging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('MockBridge')

# Global set tracking all connected WebSocket clients
connected_clients = set()


async def handler(websocket):
    """Handle a single WebSocket client connection.

    Args:
        websocket: The WebSocket connection object from websockets library.

    Handles:
        - Client connection/disconnection logging
        - Subscribe operations (logged for debugging)
        - Publish operations (broadcast to all other connected clients)

    Note:
        Uses a copy of the client set during broadcast to avoid issues if
        clients disconnect during iteration.
    """
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

                # Construct rosbridge-compatible broadcast message
                broadcast_msg = {
                    'op': 'publish',
                    'topic': topic,
                    'msg': msg
                }
                broadcast_json = json.dumps(broadcast_msg)

                # Broadcast to all clients except the sender
                # Copy set to avoid size change during iteration if disconnects happen
                for client in list(connected_clients):
                    if client != websocket:
                        try:
                            await client.send(broadcast_json)
                        except Exception:
                            # Silently ignore send failures (client may be closed)
                            pass

    except Exception as e:
        logger.info(f"Client connection error/closed: {e}")
    finally:
        connected_clients.remove(websocket)
        logger.info("Client removed")


async def main():
    """Run the mock ROS bridge WebSocket server.

    Starts a WebSocket server on 0.0.0.0:9090 that accepts client connections
    and handles rosbridge protocol messages indefinitely.
    """
    async with websockets.serve(handler, "0.0.0.0", 9090):
        logger.info("Mock ROS Bridge running on ws://0.0.0.0:9090")
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    asyncio.run(main())
