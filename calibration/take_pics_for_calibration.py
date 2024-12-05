from picamera2 import Picamera2, Preview
import time

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration())

picam2.start_preview(Preview.QTGL)
picam2.start()


for i in range(0, 40):
	time.sleep(2)
	picam2.capture_file(f"patterns/pattern{i}.jpg")
