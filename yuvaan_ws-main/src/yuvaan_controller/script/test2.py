
# roslaunch rosbridge_server rosbridge_websocket.launch

# http://localhost:5001/

# 1. First, import eventlet and call monkey_patch before importing Flask or any other module
import eventlet
eventlet.monkey_patch()  # Ensure this is the first line

# Now import Flask and other necessary libraries
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import websocket
import json
import threading

# 2. Initialize Flask app and SocketIO
app = Flask(__name__)
# socketio = SocketIO(app, async_mode='eventlet')
socketio = SocketIO(app, async_mode='eventlet', cors_allowed_origins="*")


# 3. Global dictionary to store motor data
motor_data = {
    'vel_linear_x': 0,
    'vel_angular_z': 0,
    'mode': 0,
    'ra_1': 0,
    'ra_2': 0,
    'ra_3': 0,
    'ra_4': 0,
    'ra_5': 0,
    'ra_6': 0,
    'ba_1': 0,
    'ba_2': 0,
    'ba_3': 0,
    'ba_4': 0,
    'ba_5': 0,
    'ba_6': 0,
}

# 4. Function to handle WebSocket communication
def update_velocities_via_websocket():
    try:
        print("Attempting to connect to rosbridge WebSocket...")
        ws = websocket.create_connection("ws://localhost:9090", timeout=5)  # Update URL and add timeout
        print("Connected to rosbridge WebSocket.")

        subscribe_msg = {
            "op": "subscribe",
            "topic": "/motor_command",  # Make sure this topic is correct
            "type": "yuvaan_controller/yuvaan",
            "throttle_rate": 0,
            "queue_length": 1
        }

        ws.send(json.dumps(subscribe_msg))
        print("Subscription message sent to /motor_command topic.")

        while True:
            try:
                response = ws.recv()  # Non-blocking WebSocket receive
                if response:
                    data = json.loads(response)
                    print(f"Received data: {data}")
                    if "msg" in data:
                        motor_data.update({
                            "vel_linear_x": data['msg']['vel_linear_x'],
                            "vel_angular_z": data['msg']['vel_angular_z'],
                            "mode": data['msg']['mode'],
                            "ra_1": data['msg']['ra_1'],
                            "ra_2": data['msg']['ra_2'],
                            "ra_3": data['msg']['ra_3'],
                            "ra_4": data['msg']['ra_4'],
                            "ra_5": data['msg']['ra_5'],
                            "ra_6": data['msg']['ra_6'],
                            "ba_1": data['msg']['ba_1'],
                            "ba_2": data['msg']['ba_2'],
                            "ba_3": data['msg']['ba_3'],
                            "ba_4": data['msg']['ba_4'],
                            "ba_5": data['msg']['ba_5'],
                            "ba_6": data['msg']['ba_6'],
                        })
                        print(f"Motor data updated: {motor_data}")

                        # 5. Use Flask app context when emitting data
                        with app.app_context():  # Push the app context
                            socketio.emit('motor_data_update', motor_data)

            except websocket.WebSocketTimeoutException:
                # Handle timeout if no message is received
                pass
            except Exception as e:
                print(f"Error while receiving WebSocket data: {e}")
                break

    except Exception as e:
        print(f"Failed to connect to rosbridge WebSocket: {e}")

# Start WebSocket data thread
websocket_thread = threading.Thread(target=update_velocities_via_websocket, daemon=True)
websocket_thread.start()

# 6. Flask route to serve the frontend
@app.route('/')
def index():
    return render_template('index.html')

# 7. Run the Flask application
if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5001)
