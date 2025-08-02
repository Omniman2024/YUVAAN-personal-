from flask import Flask, Response, jsonify, request, render_template_string
import cv2

app = Flask(__name__)

# List of specific video device paths
VIDEO_DEVICE_PATHS = {
    "video0": "/dev/video0",
    "video1": "/dev/video2",
    "video2": "/dev/video4",
    "video3": "/dev/video6",
    "video4": "/dev/video8",
    "video5": "/dev/video10",
    "video6": "/dev/video12",
    "video7": "/dev/video14"
}

def generate_video(device_path):
    """Stream video from the specified device."""
    cap = cv2.VideoCapture(device_path)
    
    if not cap.isOpened():
        raise RuntimeError(f"Unable to open video device {device_path}")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        _, buffer = cv2.imencode('.jpg', frame)
        frame_data = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')
    
    cap.release()

@app.route('/video0')
def stream_video0():
    return Response(generate_video(VIDEO_DEVICE_PATHS["video0"]), content_type='multipart/x-mixed-replace; boundary=frame')

@app.route('/video1')
def stream_video1():
    return Response(generate_video(VIDEO_DEVICE_PATHS["video1"]), content_type='multipart/x-mixed-replace; boundary=frame')

@app.route('/video2')
def stream_video2():
    return Response(generate_video(VIDEO_DEVICE_PATHS["video2"]), content_type='multipart/x-mixed-replace; boundary=frame')

@app.route('/video3')
def stream_video3():
    return Response(generate_video(VIDEO_DEVICE_PATHS["video3"]), content_type='multipart/x-mixed-replace; boundary=frame')

@app.route('/video4')
def stream_video4():
    return Response(generate_video(VIDEO_DEVICE_PATHS["video4"]), content_type='multipart/x-mixed-replace; boundary=frame')

@app.route('/video5')
def stream_video5():
    return Response(generate_video(VIDEO_DEVICE_PATHS["video5"]), content_type='multipart/x-mixed-replace; boundary=frame')

@app.route('/video6')
def stream_video6():
    return Response(generate_video(VIDEO_DEVICE_PATHS["video6"]), content_type='multipart/x-mixed-replace; boundary=frame')

@app.route('/video7')
def stream_video7():
    return Response(generate_video(VIDEO_DEVICE_PATHS["video7"]), content_type='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    """Render an HTML page displaying all camera streams in a grid."""
    html_template = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Multi-Camera Stream</title>
        <style>
            .video2-container {
                text-align: center;
                width: 100vw;
                margin-bottom: 20px;
            }
            .video2-container img {
                width: 100vw;
                height: auto;
            }
            .grid-container {
                display: grid;
                grid-template-columns: 640px 640px 640px;
                gap: 20px;
                padding: 20px;
                width: 1960px;
                margin: 0 auto;
            }
            .grid-item {
                text-align: center;
            }
            img {
                width: 640px;
                height: 480px;
                cursor: pointer;
            }
        </style>
    </head>
    <body>
        <h1>Live Camera Streams</h1>
        <div class="video2-container">
            <h3>Video 2</h3>
            <a href="/video2" target="_blank"><img src="/video2"></a>
        </div>
        <div class="grid-container">
            <div class="grid-item"><h3>Video 0</h3><a href="/video0" target="_blank"><img src="/video0"></a></div>
            <div class="grid-item"><h3>Video 1</h3><a href="/video1" target="_blank"><img src="/video1"></a></div>
            <div class="grid-item"><h3>Video 3</h3><a href="/video3" target="_blank"><img src="/video3"></a></div>
            <div class="grid-item"><h3>Video 4</h3><a href="/video4" target="_blank"><img src="/video4"></a></div>
            <div class="grid-item"><h3>Video 5</h3><a href="/video5" target="_blank"><img src="/video5"></a></div>
            <div class="grid-item"><h3>Video 6</h3><a href="/video6" target="_blank"><img src="/video6"></a></div>
            <div class="grid-item"><h3>Video 7</h3><a href="/video7" target="_blank"><img src="/video7"></a></div>
        </div>
    </body>
    </html>
    """
    return render_template_string(html_template)

@app.route('/data', methods=['GET', 'POST'])
def data():
    """Return data in JSON format or process incoming data."""
    if request.method == 'GET':
        return jsonify({"message": "This is some data!"})
    elif request.method == 'POST':
        received_data = request.get_json()
        return jsonify({"received": received_data}), 201

@app.route('/cmd', methods=['POST'])
def cmd():
    """Receive and execute a command."""
    data = request.get_json()
    command = data.get("command")
    if not command:
        return jsonify({"error": "No command provided"}), 400

    return jsonify({"message": f"Command '{command}' received and processed"}), 200

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)