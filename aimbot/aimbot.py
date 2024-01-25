import math
import cv2
import numpy as np
from mss import mss
import config
import keyboard
import serial
import time
import traceback
import torch
from ultralytics import YOLO

offset_x = 0
offset_y = 0
offset_mul_x = 1.0
offset_mul_y = 1.0

class Aimbot:
    def __init__(self):
        self.cfg = config.Config("aimbot.cfg")
        self.scrw = self.cfg['width']
        self.scrh = self.cfg['height']
        self.procw = 128
        self.proch = 128
        self.load_model()
        self.setup_serial_connection()
        print("CUDA available: ", torch.cuda.is_available())
    
    def load_model(self):
        """Load the model based on configuration parameters"""
        self.model = YOLO(self.cfg['path'])
        self.model.fuse()

    def setup_serial_connection(self):
        """Setup the serial connection for arduino"""
        self.arduino = serial.Serial(self.cfg['arduino_port'], self.cfg['arduino_baudrate'], timeout=0.2)

    def distance_towards(self, head_center):
        distance = (head_center[0] - self.procw // 2, head_center[1] - self.proch // 2)
        # 5 is a step. This value must be the same in the Mouse.move function on the arduino.
        # for example: if the distance on the x-axis is 100, then we take a step of 5 units 20 times
        # distance = tuple(-(i//5) for i in distance) # reverse sign
        return distance
    
    def calculate_distance_sq(self, head_center):
        return (head_center[0] - (self.procw // 2)) ** 2 + (head_center[1] - (self.proch // 2)) ** 2

    def center_of(self, xmin, ymin, xmax, ymax):
        return xmin + (xmax-xmin) // 2, ymin + (ymax - ymin) // 2

    def send_message(self, distance):
        x = distance[0] * offset_mul_x + offset_x
        y = distance[1] * offset_mul_y + offset_y
        self.arduino.write(b'm' + int(x).to_bytes(2, byteorder='little', signed=True) + int(y).to_bytes(2, byteorder='little', signed=True))

    def extract_bounding_box(self, results):
        xmin = -1
        ymin = -1
        xmax = -1
        ymax = -1
        min_distance = 100000

        """Find the closest target to the crosshair"""
        for i in range(len(results)):
            if len(results[i]) <= 0:
                continue

            _confs = results[i].boxes.conf.cpu().numpy()
            _bboxes = results[i].boxes.xyxy.cpu().numpy()

            for j in range(_bboxes.shape[0]):
                if _confs[j] < .3: # skip if confidence is too low
                    continue

                _xmin = int(_bboxes[j][0])
                _ymin = int(_bboxes[j][1])
                _xmax = int(_bboxes[j][2])
                _ymax = int(_bboxes[j][3])
                cent = self.center_of(_xmin, _ymin, _xmax, _ymax)
                dist = self.calculate_distance_sq(cent)
                if dist < min_distance:
                    xmin = _xmin
                    ymin = _ymin
                    xmax = _xmax
                    ymax = _ymax
                    min_distance = dist


        return xmin, ymin, xmax, ymax

    def draw_on_image(self, screenshot, head_center, bounding_box):
        """Draw the detected object on the image"""
        xmin, ymin, xmax, ymax = bounding_box
        xmin = int(xmin * self.scrw / self.procw)
        xmax = int(xmax * self.scrw / self.procw)
        ymin = int(ymin * self.scrh / self.proch)
        ymax = int(ymax * self.scrh / self.proch)
        color = (255, 0, 0)
        head_center = (int(head_center[0] * self.scrw / self.procw), int(head_center[1] * self.scrh / self.proch))
        cv2.circle(screenshot, head_center, 5, (0, 255, 0), thickness = -1)
        cv2.rectangle(screenshot, (xmin, ymin), (xmax, ymax), color, 2)
        return screenshot

    def run(self):
        monitor = {
            "top": self.cfg['top'],
            "left": self.cfg['left'],
            "width": self.cfg['width'],
            "height": self.cfg['height'] 
        }
        with mss() as sct:
            while True:
                screenshot = np.array(sct.grab(monitor))
                screenshot_nn = cv2.resize(screenshot, (self.procw, self.proch)) # the model is trained with 256x256 image
                screenshot_nn = screenshot_nn[:,:,:3]
                result = self.model(screenshot_nn, verbose=False) # Drop alpha channel - https://stackoverflow.com/a/35902359
                try:
                    xmin, ymin, xmax, ymax = self.extract_bounding_box(result)
                    if xmin >= 0:
                        head_center = self.center_of(xmin, ymin, xmax, ymax)
                        screenshot = self.draw_on_image(screenshot, head_center, (xmin, ymin, xmax, ymax))
                        if keyboard.is_pressed('v'):
                            self.send_message(self.distance_towards(head_center))
               

                except Exception as e:
                    pass
                    traceback.print_exc()
                    print(e)

                cv2.imshow("frame", screenshot)
                if cv2.waitKey(1) == ord('q'):
                    cv2.destroyAllWindows()
                    break


if __name__ == "__main__":
    aimbot = Aimbot()
    aimbot.run()
