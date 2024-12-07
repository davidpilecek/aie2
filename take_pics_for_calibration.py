from picamera2 import Picamera2, Preview
import time
from config import *

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main = {"size": FRAME_DIMENSIONS}, transform = Transform(hflip=1, vflip=1)))
picam2.start_preview(Preview.QTGL)
picam2.start()
picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 5.0})


for i in range(0, 100):
	time.sleep(1)
	picam2.capture_file(f"calibration/calibration_pic{i}.jpg")
