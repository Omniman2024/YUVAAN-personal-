from flask import Flask, Response, jsonify, request
import cv2

app = Flask(__name__)

# Path to the video device (e.g., /dev/video4)
VIDEO_DEVICE_PATH1 = "/dev/video1"
VIDEO_DEVICE_PATH2 = "/dev/video2"
VIDEO_DEVICE_PATH3 = "/dev/video3"
VIDEO_DEVICE_PATH4 = "/dev/video4"
def generate_video(path):
    """Stream video from the specified device."""
    # Open video capture from the device
    cap = cv2.VideoCapture(path)
    
    if not cap.isOpened():
        raise RuntimeError(f"Unable to open video device {path}")
    
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            break
        
        # Encode the frame to JPEG
        _, buffer = cv2.imencode('.jpg', frame)
        frame_data = buffer.tobytes()
        
        # Yield the frame as part of the stream
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')
    
    # Release the capture after the streaming ends
    cap.release()

@app.route('/video1')
def stream_video():
    """Stream video from the specified video device."""
    return Response(generate_video(VIDEO_DEVICE_PATH1), content_type='multipart/x-mixed-replace; boundary=frame')

@app.route('/video2')
def stream_video():
    """Stream video from the specified video device."""
    return Response(generate_video(VIDEO_DEVICE_PATH2), content_type='multipart/x-mixed-replace; boundary=frame')

@app.route('/video3')
def stream_video():
    """Stream video from the specified video device."""
    return Response(generate_video(VIDEO_DEVICE_PATH3), content_type='multipart/x-mixed-replace; boundary=frame')

@app.route('/video4')
def stream_video():
    """Stream video from the specified video device."""
    return Response(generate_video(VIDEO_DEVICE_PATH4), content_type='multipart/x-mixed-replace; boundary=frame')

@app.route('/data', methods=['GET', 'POST'])
def data():
    """Return data in JSON format or process incoming data."""
    if request.method == 'GET':
        return jsonify({"message": "This is some data!"})
    elif request.method == 'POST':
        received_data = request.get_json()
        return jsonify({"received": received_data}), 201

@app.route('/cmd', methods=['GET'])
def cmd():
    """Receive and execute a command."""
    data = request.get_json()
    command = data.get("command")
    if not command:
        return jsonify({"error": "No command provided"}), 400

    # Example: Simulate command processing
    return jsonify({"message": f"Command '{command}' received and processed"}), 200

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)