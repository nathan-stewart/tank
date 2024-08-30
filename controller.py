#!/usr/bin/env pythnon3
import zmq
import time
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtMultimedia import QCamera, QCameraViewfinder, QMediaRecorder
from PyQt5.QtCore import Qt
import sys

running = True

# Client (e.g., on your laptop)
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://tank:5555")
socket.setsockopt(zmq.RCVTIMEO, 1000)

def send_message(message):
    socket.send_string(message)
    try:
        reply = socket.recv_string()
        pass
    except zmq.error.Again:
        print("Timeout")

target = np.array([0, 0])
speed = np.array([0, 0])
current_time = time.time()
acceleration = 50.0 # Takes two seconds of keypress to reach full speed
controls = ['up', 'down', 'left', 'right', 'space', 'q']


from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtNetwork import QNetworkAccessManager, QNetworkRequest
from PyQt5.QtGui import QImage, QPixmap
import numpy as np
import time
import sys

class CameraControlWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Set up the camera feed URL (MJPEG stream URL)
        self.camera_feed_url = "http://tank:8554/mjpeg"

        # Set up the network manager for the camera feed
        self.network_manager = QNetworkAccessManager()
        self.network_manager.finished.connect(self.handle_response)

        # Set up the UI
        self.label = QLabel(self)
        self.setCentralWidget(self.label)
        self.setWindowTitle("Remote Camera Feed with Keyboard Control")
        self.setGeometry(100, 100, 800, 600)

        # Control variables
        self.speed = np.array([0, 0])
        self.target = np.array([0, 0])
        self.acceleration = 50  # Example acceleration value

        # Timer for control loop
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_control_loop)
        self.timer.start(100)  # Update every 100 ms

        # Buffer for MJPEG
        self.buffer = QByteArray()
        self.image = QImage()

        # Initialize time
        self.current_time = time.time()

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Up:
            self.target += [100, 100]
        elif key == Qt.Key_Down:
            self.target += [-100, -100]
        elif key == Qt.Key_Left:
            self.target += [-100, 100]
        elif key == Qt.Key_Right:
            self.target += [100, -100]
        elif key == Qt.Key_Space:
            self.speed = np.array([0, 0])
            self.target = np.array([0, 0])
        elif key == Qt.Key_Q:
            self.close()  # Close the window and exit

    def keyReleaseEvent(self, event):
        # Handle key release events if necessary
        event.accept()

    def update_control_loop(self):
        delta_time = time.time() - self.current_time
        self.current_time = time.time()

        # Normalize the target
        norm_target = np.linalg.norm(self.target)
        if norm_target > 0:
            self.target = (self.target / norm_target) * norm_target

        # Update speed based on acceleration
        self.speed = self.speed + self.acceleration * delta_time * (self.target - self.speed)
        print(f"Speed: {self.speed[0]}, {self.speed[1]}")
        print(f"Target: {self.target[0]}, {self.target[1]}")
        # Example: send_message(f"drive:{int(self.speed[0])},{int(self.speed[1])}")

        # Fetch the next frame from the camera
        self.fetch_frame()

    def fetch_frame(self):
        request = QNetworkRequest(self.camera_feed_url)
        self.network_manager.get(request)

    def handle_response(self, reply):
        data = reply.readAll()
        if data:
            self.buffer.append(data)
            # Check for end of frame (boundary detection for MJPEG)
            if b'\r\n\r\n' in self.buffer:
                parts = self.buffer.split(b'\r\n\r\n', 1)
                jpg_data = parts[1].split(b'\r\n', 1)[0]
                self.buffer = parts[1].split(b'\r\n', 1)[1]
                self.update_image(jpg_data)

    def update_image(self, jpg_data):
        self.image = QImage.fromData(jpg_data)
        if not self.image.isNull():
            self.label.setPixmap(QPixmap.fromImage(self.image))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CameraControlWindow()
    window.show()
    sys.exit(app.exec_())
