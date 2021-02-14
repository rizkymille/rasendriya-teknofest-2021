# Rasendriya  
![RasendriyaUAV](https://github.com/rizkymille/rasendriya-auav-ui/blob/main/docs/rasendriya.jpg)
Rasendriya is a lightweight fixed wing UAV from AUAV UI for Tübitak International Unmanned Aerial Vehicle (UAV) Teknofest Competition. For Tubitak 2021, Rasendriya has two missions: Flying in oval track and loiter in one point with small radius, and flying in oval track for 3 laps while dropping two payloads in a designated dropzone. The designated dropzone must be found autonomously by using computer vision system.

## Workflow
![Full steps from prepare to deploy](https://github.com/rizkymille/rasendriya-auav-ui/blob/main/docs/fullsteps.png)

## Environment Setup
### TensorFlow 2
Please check: [TensorFlow 2 Local Setup](https://github.com/rizkymille/rasendriya-auav-ui/blob/main/tensorflow/TF2_LOCALSETUP.md)
### TensorFlow 2 Object Detection API
Please check: [TensorFlow 2 Object Detection API Local Setup](https://github.com/rizkymille/rasendriya-auav-ui/blob/main/tensorflow/TF2_OBJECT_DETECTION_API_LOCALSETUP.md)
### Arduino

## Training Model
![Inference video](https://github.com/rizkymille/rasendriya-auav-ui/blob/main/docs/inference.gif)  
This training instruction will guide you to generate new model which suits to your dataset, and converting it to TensorFlow Lite format and C array.
### Preparing Dataset
Please look at: [Dataset Workflow](https://github.com/rizkymille/rasendriya-auav-ui/blob/main/tensorflow/workspace/orange-dropzone-detection/datasets/DATASET_WORKFLOW.md)
### Selecting Pre Trained Model
Check the models at: [TensorFlow 2 Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md)

TensorFlow 2 has pre trained models from the repository with varying accuracy and speeds. You must select model according to your usage -- do you want to detect real time or not. In this guide we will use the SSD MobileNet V2 320x320, as it is the model with lowest speed (ms)/inference time.
### Managing File Directories
### Setting up pipeline.config
### Training
### Tensorboard
### Evaluate
### Export Model
### Test Inference
### Export TensorFlow 2 Model to TensorFlow Lite Graph Model
### Convert TensorFlow Lite Graph Model to TensorFlow Lite Format
### Convert TensorFlow Lite Format to C array

## Deploy to ESP32-CAM
### Work in Progress

## References
Vladimirov, Lyudmil (2020). *TensorFlow Object Detection API Tutorial*. https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/  
Tanner, Gilbert (2020). *TensorFlow Object Detection with TensorFlow 2.0*. https://github.com/TannerGilbert/Tensorflow-Object-Detection-with-Tensorflow-2.0  
