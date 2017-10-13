from stream import Stream
import cv2

class WebcamStream(Stream):
    def __init__(self, resolution = (320,240)):
        self.resolution = resolution

    def __enter__(self):
        width, height = self.resolution
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cap.release()

    def frames(self):
        while self.cap.isOpened():
            ret, img = self.cap.read()
            if ret:
                yield img