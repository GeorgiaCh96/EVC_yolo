import os
import wandb
import random
import pandas as pd
from PIL import Image
import cv2
from ultralytics import YOLO
from IPython.display import Video
import numpy as np  
import matplotlib.pyplot as plt
import seaborn as sns
sns.set(style='darkgrid')
import pathlib
import glob
from tqdm.notebook import trange, tqdm
import warnings
warnings.filterwarnings('ignore')
import kagglehub


# ==================== Hyperparameters ====================
EPOCHS = 30
BATCH_SIZE = -1
LR = 1e-4
SHUFFLE_BUFFER_SIZE = 1000
#saved_model_path = os.path.join(os.getcwd(), "saved_models", f"apnea_det_1DCNN_TRIAL{TRIAL_NO}.h5")
TRIAL_N0=1
optimizer='auto'

# ==================== wandb ====================
# Use wandb-core, temporary for wandb's new backend
wandb.require("core")
wandb.login()

# Start a new run (TRIAL) to track this script
wandb.init(
    # Set the project where this run will be logged
    project="EVC",
    # We pass a run name (otherwise it’ll be randomly assigned, like sunshine-lollypop-10)
    name=f"experiment_{TRIAL_N0}",
    # Track hyperparameters and run metadata
    config={
        "learning_rate": LR,
        "architecture": "YOLOv8",
        "dataset": "kaggle data",
        "epochs": EPOCHS,
        "batch_size": BATCH_SIZE,
        "optimizer": optimizer
        })



config = wandb.config



# Download latest version
path = kagglehub.dataset_download("pkdarabi/cardetection")

print("Path to dataset files:", path)

image_dir = os.path.join(path, "car", "train", "images")
labels_dir = os.path.join(path, "car", "train", "labels")


# Configure the visual appearance of Seaborn plots
sns.set(rc={'axes.facecolor': '#eae8fa'}, style='darkgrid')


#Image_dir = '/kaggle/input/cardetection/car/train/images'

num_samples = 9
image_files = os.listdir(image_dir)
label_files = os.listdir(labels_dir)

# Randomly select num_samples images
random.seed(42)
rand_images = random.sample(image_files, num_samples)
rand_labels = rand_images #random.sample(label_files, num_samples)
print(rand_images)
print(rand_labels)

fig, axes = plt.subplots(3, 3, figsize=(11, 11))

for i in range(num_samples):
    image = rand_images[i]
    with open(os.path.join(labels_dir, rand_labels[i][:-4]+'.txt'), 'r') as file:
        for line in file:
            #line = file.readline()  # Reads only the first line
            label_value = line.strip().split()[0]
            print("assigned label: ", label_value)
        ax = axes[i // 3, i % 3]
        ax.imshow(plt.imread(os.path.join(image_dir, image)))
        ax.set_title(f'Image {i}, label {label_value}')
        ax.axis('off')

plt.tight_layout()
plt.show()


# Get the size of the image
image = cv2.imread(os.path.join(image_dir, rand_images[0]))
h, w, c = image.shape
print(f"The image has dimensions {w}x{h} and {c} channels.")

# Use a pretrained YOLOv8n model
model = YOLO("yolov8n.pt")

# Use the model to detect object
#image = "/kaggle/input/cardetection/car/train/images/FisheyeCamera_1_00228_png.rf.e7c43ee9b922f7b2327b8a00ccf46a4c.jpg"
result_predict = model.predict(source = image, imgsz=(640))

# show results
plot = result_predict[0].plot()
plot = cv2.cvtColor(plot, cv2.COLOR_BGR2RGB)
image_to_plot = Image.fromarray(plot)
image_to_plot.show()


# Build from YAML and transfer weights
Final_model = YOLO('yolov8n.pt')

# Training The Final Model
Result_Final_model = Final_model.train(data="C:/Users/user/Downloads/archive/car/data.yaml", epochs=EPOCHS, batch=BATCH_SIZE, optimizer='auto')
