from ultralytics import YOLO

# Load a pretrained YOLO11n model
model = YOLO("/Users/rededge/Documents/25_ws/dataset/runs/detect/train3/weights/best.pt")

# Run inference on 'bus.jpg' with arguments
model.predict("/Users/rededge/Downloads/25_ws-dbcb9d456d5b4b18d24bc9036f7670af39f17a51/traffic_frame.png", save=True, imgsz=320, conf=0.5)