from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QImage
import av
import numpy as np

class VideoThread(QThread):
    update_frame_signal = pyqtSignal(QImage)

    def __init__(self, url):
        super().__init__()
        self.url = url
        self.container = None
        self.running = True
        self.setup()

    def setup(self):
        try:
            # Open the stream with av
            self.container = av.open(self.url)
            self.stream = self.container.streams.video[0]
        except av.AVError as e:
            print(f"Failed to open stream: {e}")
            self.running = False

    def run(self):
        last_frame = None
        for frame in self.container.decode(self.stream):
            if not self.running:
                break
            if frame:
                last_frame = frame

            # Convert the frame to RGB format
            frame_rgb = last_frame.to_ndarray(format='rgb24')

            h, w, ch = frame_rgb.shape
            bytes_per_line = ch * w
            qt_image = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.update_frame_signal.emit(qt_image)
            QThread.msleep(66)  # approximately 15fps

    def stop(self):
        self.running = False
        if self.container:
            self.container.close()
        self.quit()
