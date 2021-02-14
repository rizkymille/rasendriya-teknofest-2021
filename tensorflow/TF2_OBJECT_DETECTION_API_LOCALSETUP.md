# TensorFlow 2 Object Detection API Installation
TensorFlow 2 Object Detection API is an API for implementing state-of-the-art (SOTA) models to custom object detection usage. We can use those published and proven model and customize it according to our usage. This method is called **Transfer Learning**.
## Some Folder Managements...
Generated files by tensorflow, your datasets, object detection API, pre trained models, etc can be quite messy to manage later. To make things tidier, I recommend you the folder structure I use. Here's my structure:

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
         
Keep in my that I will refer to this directory in my documentation later.
## Installing TensorFlow Model Garden
Since tensorflow model garden is continously updating, they dont upload this onto pip package. Instead, we must clone the TensorFlow Model Garden GitHub repository to our local system. There are two ways to do this.
### 1. Clone directly from GitHub
Open this link: **https://github.com/tensorflow/models**, then just download it to your directory. Simple.
### 2. Using git
If you want to be fancy and some elite developer/hacker-like, you can use git to clone the repository. Open anaconda prompt, then install git first by writing this:
  
  pip install git
  
Then, after installation has completed, write this:
  
  cd C:\Users\Your_Username\Documents\TensorFlow
  
  git clone https://github.com/tensorflow/models.git
  
## Installing Protobuf
The Tensorflow Object Detection API uses Protobufs to configure model and training parameters. Before the framework can be used, the Protobuf libraries must be downloaded and compiled.
