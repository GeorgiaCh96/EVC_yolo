import cv2
from PIL import Image

from ultralytics import YOLO

model = YOLO("C:/Users/user/Documents/TUe/Q4/5LIA0 Embedded visual control/Workshops/EVC/workshops/yolo_ws/yolo/src/yolo_package/src/runs/detect/yolov8n_custom6/weights/best.pt")

# WEBCAM
# accepts all formats - image/dir/Path/URL/video/PIL/ndarray. 0 for webcam
#results = model.predict(source="0")

# IMAGES FOLDER
test_folder = "C:/Users/user/Documents/TUe/Q4/5LIA0 Embedded visual control/Workshops/EVC/workshops/yolo_ws/yolo/src/yolo_package/src/archive/car/test/images"
#results = model.predict(source=test_folder, save=True)  # Display preds. Accepts all YOLO predict arguments

# video feed
results = model.predict(source="C:/Users/user/Documents/TUe/Q4/5LIA0 Embedded visual control/Workshops/EVC/workshops/yolo_ws/yolo/src/yolo_package/src/archive/video_inf.mp4", show=True)
#results = model.predict(source="video_inf.mp4", show=True)

# test on single image:
# from PIL
#im1 = Image.open("bus.jpg")
#results = model.predict(source=im1, save=True)  # save plotted images

# from ndarray
#im2 = cv2.imread("bus.jpg")
#results = model.predict(source=im2, save=True, save_txt=True)  # save predictions as labels

# from list of PIL/ndarray
#results = model.predict(source=[im1, im2])