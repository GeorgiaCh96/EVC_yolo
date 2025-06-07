from ultralytics import YOLO



# Load a pretrained YOLO11n model
#model = YOLO("runs/detect/yolov8n_custom6/weights/best.py")
model = YOLO("C:/Users/user/Documents/TUe/Q4/5LIA0 Embedded visual control/Workshops/EVC/workshops/yolo_ws/yolo/src/yolo_package/src/runs/detect/yolov8n_custom6/weights/best.pt")

# Evaluate the model's performance on the validation set
metrics = model.val( data='traffic.yaml')

# Perform object detection on an image
#results = model("speed30.jpg")  # Predict on an image
#results[0].show()  # Display results

# Export the model to ONNX format for deployment
path = model.export(format="onnx")  # Returns the path to the exported model