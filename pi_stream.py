import cv2
import numpy as np
from threading import Thread, Event
from picamera.array import PiRGBArray
from picamera import PiCamera

class PiStream:
    def __init__(self, resolution=(1280, 720), framerate=30):
        try:
            self.camera = PiCamera()
            self.camera.resolution = resolution
            self.camera.framerate = framerate

            self.rgb_buffer = PiRGBArray(self.camera, size=resolution)
            self.frame_gen = self.camera.capture_continuous(self.rgb_buffer,
                                                         format="rgb",
                                                         use_video_port=True)
            self.frame_event = Event()
            self.frame = None
            self.stop_flag = False
        except Exception as e :
            self.release()
            raise e


    def release(self):
        try:
            pass
        finally:
            self.camera.close()
            self.rgb_buffer.close()
            self.frame_gen.close()

    def __enter__(self):
        self.frame_thread = Thread(target=self.capture_loop)
        self.frame_thread.start()

    def __exit__(self):
        self.stop_flag = True
        self.frame_thread.join()

    def capture_loop(self):
        for frame in self.frame_gen:
            self.frame_event.set()
            self.frame = self.rgb_buffer.array
            self.rgb_buffer.truncate(0)

            if self.stop_flag:
                self.release()
                return

    def get_frame(self):
        self.frame_event.wait()
        print("frameeee has been read")
        self.frame_event.clear()
        return self.frame

    def stop(self):
        self.__exit__()

    def start(self):
        self.__enter__()

