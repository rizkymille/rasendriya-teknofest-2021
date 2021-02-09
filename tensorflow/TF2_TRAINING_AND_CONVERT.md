# TensorFlow 2 Training and Convert
![Inference video](https://github.com/rizkymille/rasendriya-auav-ui/blob/main/tensorflow/inference.gif)  
This training instruction will guide you to generate new model which suits to your dataset, and converting it to TensorFlow Lite format and C array.
## Preparing Dataset
Please look at: [Dataset Workflow](https://github.com/rizkymille/rasendriya-auav-ui/blob/main/datasets/DATASET_WORKFLOW.md)
## Selecting Pre Trained Model
Check the models at: [TensorFlow 2 Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md)

TensorFlow 2 has pre trained models from the repository with varying accuracy and speeds. You must select model according to your usage -- do you want to detect real time or not. In this guide we will use the SSD MobileNet V2 320x320, as it is the model with lowest speed (ms)/inference time.
## Managing File Directories
## Setting up pipeline.config
## Training
## Tensorboard
## Evaluate
## Export Model
## Test Inference
## Export to TensorFlow Lite Graph Model
## Convert to TensorFlow Lite .tflite Format
## Convert .tflite to C array
