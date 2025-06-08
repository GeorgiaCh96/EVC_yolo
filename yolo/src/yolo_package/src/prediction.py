import cv2
from PIL import Image

from ultralytics import YOLO

class_names = {
            0: "Green_light",
            1: "Red_light",
            2: "speed_limit_10",
            3: "speed_limit_100",
            4: "speed_limit_110" ,
            5: "speed_limit_120",
            6: "speed_limit_20",
            7: "speed_limit_30",
            8: "speed_limit_40",
            9: "speed_limit_50" ,
            10: "speed_limit_60",
            11: "speed_limit_70",
            12: "speed_limit_80",
            13: "speed_limit_90",
            14: "stop" 
}

model = YOLO("C:/Users/user/Documents/TUe/Q4/5LIA0 Embedded visual control/Workshops/EVC/workshops/yolo_ws/yolo/src/yolo_package/src/runs/detect/yolov8n_custom6/weights/best.pt")

# WEBCAM
# accepts all formats - image/dir/Path/URL/video/PIL/ndarray. 0 for webcam
#results = model.predict(source="0", show=True)

# IMAGES FOLDER
test_folder = "C:/Users/user/Documents/TUe/Q4/5LIA0 Embedded visual control/Workshops/EVC/workshops/yolo_ws/yolo/src/yolo_package/src/archive/car/test/images"
#results = model.predict(source=test_folder, save=True)  # Display preds. Accepts all YOLO predict arguments

# video feed
#results = model.predict(source="C:/Users/user/Documents/TUe/Q4/5LIA0 Embedded visual control/Workshops/EVC/workshops/yolo_ws/yolo/src/yolo_package/src/archive/video_inf.mp4", show=True)
#results = model.predict(source="video_inf.mp4", show=True)

# test on single image:
# from PIL
#im1 = Image.open("bus.jpg")
#results = model.predict(source=im1, save=True)  # save plotted images

# from ndarray
im2 = cv2.imread("speed30_02.png")
results = model.predict(source=im2, show=True)  # save predictions as labels
results = results[0]
print("results: ", results[0])
boxes   = results.boxes.xyxy.cpu().numpy() 
print("boxes: ", boxes)
scores  = results.boxes.conf.cpu().numpy()  
print("scores: ", scores)
classes = results.boxes.cls.cpu().numpy()    
print("classes: " , int(classes[0]))
print("class name: ", class_names[int(classes[0])])


# from list of PIL/ndarray
#results = model.predict(source=[im1, im2])