#!/usr/bin/env python3
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QImage
import av

class VideoThread(QThread):
    update_frame_signal = pyqtSignal(QImage)

    def __init__(self, url):
        super().__init__()
        self.url = url
        self.container = None
        self.stream = None
        self.running = True

        try:
            self.container = av.open(self.url)
            self.stream = self.container.streams.video[0]
        except av.AVError as e:
            print(f"Failed to open stream: {e}")
            self.running = False

    def run(self):
        while self.running:
            try:
                for frame in self.container.decode(self.stream):
                    if not self.running:
                        break

                    # Handle the frame if available
                    if frame:
                        frame_rgb = frame.to_ndarray(format='rgb24')
                        h, w, ch = frame_rgb.shape
                        bytes_per_line = ch * w
                        qt_image = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
                        self.update_frame_signal.emit(qt_image)
            except av.AVError as e:
                print(f"Error decoding frame: {e}")
                self.running = False
            except Exception as e:
                print(f"Unexpected error: {e}")
                self.running = False

    def stop(self):
        self.running = False
        if self.container:
            self.container.close()
        self.quit()
