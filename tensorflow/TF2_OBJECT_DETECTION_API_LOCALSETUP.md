# TensorFlow 2 Object Detection API Installation
TensorFlow 2 Object Detection API is an API for implementing state-of-the-art (SOTA) models to custom object detection usage. We can use those published and proven model and customize it according to our usage. This method is called **Transfer Learning**.
## Some Folder Managements...
Generated files by tensorflow, your datasets, object detection API, pre trained models, etc can be quite messy to manage later. To make things tidier, I recommend you the folder structure I use. Here's my structure:
  ```
  Documents\  
  └─ TensorFlow\  
    └─ models\ (this will be used by TensorFlow Model Garden)  
    └─ workspace\  
      └─ your_project_name\  
         └─ Model Inference.ipynb  
         └─ export_tflite_graph_tf2.py  
         └─ exporter_main_v2  
         └─ model_main_tf2  
         └─ datasets\  
            └─ partition_dataset.py  
            └─ generate_tfrecord.py  
            └─ images\  
            └─ pascalvoc\  
            └─ record  
         └─ pre-trained-models\  
         └─ models\ (this will be used as model training folder)  
         └─ exported-models\ (this will be used as exported trained model folder)
   ```

Keep in mind that I will refer to this directory in my documentation later.
## Installing TensorFlow Model Garden
Since tensorflow model garden is continously updating, they dont upload this onto pip package. Instead, we must clone the TensorFlow Model Garden GitHub repository to our local system. There are two ways to do this.
### 1. Clone directly from GitHub
Open this link: **https://github.com/tensorflow/models**, then just download it to your directory.
![Clone GitHub](https://github.com/rizkymille/rasendriya-auav-ui/blob/main/docs/clonegithub.jpg)
Simple.
### 2. Using git
If you want to be fancy and some elite developer/hacker-like, you can use git to clone the repository. Open anaconda prompt, then install git first. **Don't forget to install this in your tensorflow environment!**
  ```
  pip install git
  ```
Then, after installation has completed, change your prompt directory to `Documents\TensorFlow`. If you don't know how to change directory in prompt, write this:
  ```
  cd C:\Users\Your_Username\Documents\TensorFlow
  ```
Clone the repository.
  ```
  git clone https://github.com/tensorflow/models.git
  ```
## Installing Protobuf
The Tensorflow Object Detection API uses Protobufs to configure model and training parameters. Before the framework can be used, the Protobuf libraries must be downloaded and compiled. To install Protobuf, use anaconda prompt and write:
  ```
  conda install protobuf
  ```
Make sure your directory is in `TensorFlow\models\research`. If not:
  ```
  cd C:\Users\Your_Username\Documents\TensorFlow\models\research
  ```
Inside above directory, install protobuf.
  ```
  protoc object_detection/protos/*.proto --python_out=.
  ```
## Installing COCO API Tools
COCO (Common Objects In Context) is widely used as image annotation dataset format. That's why we need to install the API tools so the Object Detection API can understand the images we've annotated. To install that, we need to install Cython first:
  ```
  conda install cython
  ```
Then you need to install Visual C++ 2015 as requirement to run COCO API Tools. You can download it from here: ![Visual C++ 2015](https://go.microsoft.com/fwlink/?LinkId=691126). **Don't forget to add the Visual C++ 2015 onto your computer Path environment variables**.

Finally, clone the repository:
  ```
  pip install git+https://github.com/philferriere/cocoapi.git#subdirectory=PythonAPI
  ```
## Installing TensorFlow 2 Object Detection API
Inside `TensorFlow/models/research/` directory, write in prompt:
  ```
  copy object_detection\packages\tf2\setup.py .
  ```
Then
  ```
  python -m pip install .
  ```
## Testing
To verify and ensure that our Object Detection API is working, we can test with the scripts provided with the Object Detection API. From within `TensorFlow/models/research/` directory:
  ```
  python object_detection/builders/model_builder_tf2_test.py
  ```
## Annotating images
Since we build our custom detection model, we must create our own dataset to make the model detect according to our dataset. This is why we must annotate images on our own. To annotate images, there's so many tools you can use. Some examples are labelImg, VOTT, CVAT, labelme, etc ranging from open source to paid service. I personally use labelImg because it's free and gives the satisfactory result. I already tried VOTT but the annotation result is in float format, which complicates the training and inference process.

To install labelImg, use the anaconda prompt in your tensorflow environment and write this:
  ```
  pip install labelImg==1.8.3
  ```
