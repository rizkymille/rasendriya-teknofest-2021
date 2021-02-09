# TensorFlow 2 Set Up
TensorFlow is a free and open-source software library for machine learning. This manual is purposed for tensorflow installation reference because installing tensorflow is quite annoying and pain in the ass, especially for GPU version because you need to match version between cuDNN, CUDAtoolkit, Tensorflow-gpu, and even the Python itself. Regardless of the such difficulties, using Tensorflow-gpu is very recommended for performance because it really speeds up your training per-step. In my case it speeds up from 1.2 sec/step (tensorflow-cpu) to 0.3 sec/step (tensorflow-gpu).
## Installing Anaconda
To simplify things and managing packages you've installed, we will use Anaconda distribution. Anaconda is a distribution of the Python and R programming languages for scientific computing (data science, machine learning applications, large-scale data processing, predictive analytics, etc.), that aims to simplify package management and deployment. You don't need to install python from python.org because Anaconda comes with python itself.

Download Anaconda from https://www.anaconda.com/products/individual

Follow the installer steps like how you install other applications. **When installing,  check the "add to PATH" option.**
## Setting up TensorFlow in Anaconda
Anaconda comes with something called "environment". Environment is the "scope" or "region" of your programming, which contains the python, packages, libraries, and IDEs you need to manage. You can have multiple environments in your computer. This time we will create a new tensorflow environment because tensorflow packages is quite complex and we don't want to mix it with 'base (root)' environment.

To create a new environment called tensorflow, just open anaconda prompt and write:

    conda create -n tensorflow
To get into our new tensorflow environment, you must activate it. Write in prompt:

    conda activate tensorflow
We will use Python 3.8.5, Tensorflow 2.3.0, Tensorflow-gpu 2.3.0, CUDAtoolkit 10.1, and cuDNN 7.6.5 because those version has tested by me and working as of January 2021. If you really want to use latest version, do it at your own risk!

Our new tensorflow environment has only default scientific packages from anaconda. It didn't even have python. Install the python 3.8.5 version:

    conda install python=3.8
We finally have the python. Now to install tensorflow, we must specify which option to install. Tensorflow has 2 option, CPU and GPU. Tensorflow GPU only works in NVIDIA so we need to install CUDA and cuDNN.
### 1. Installing TensorFlow CPU
This will install tensorflow 2.3.0 CPU version and it's dependencies automatically:

    conda install tensorflow==2.3
### 2. Installing TensorFlow GPU
Tensorflow GPU uses CUDA, that's why we need to install cudatoolkit. Simply write:

    conda install cudatoolkit=10.1
Now install the cuDNN:

    conda install cudnn=7.6.5

Finally, install the tensorflow-gpu:

    conda install tensorflow-gpu==2.3
## Testing TensorFlow
Now, to verify if our tensorflow is working, we need to test it. Open your environment with IDE like VSCode, Spyder, Jupyter, or PyCharm.

Then write this:

    import tensorflow as tf
    tf.config.list_physical_devices(device_type=None)
If the output message has "physical_device:CPU:0" and "physical_device:XLA_CPU:0", it means tensorflow is working with CPU version.

If the output message also has "physical_device:GPU:0" and "physical_device:XLA_GPU:0", it means tensorflow is working with GPU version.

To test it further (for tensorflow GPU version only), let's see number of GPU available:

    import tensorflow as tf
    print("Num GPUs Available: ", len(tf.config.experimental.list_physical_devices('GPU')))
    tf.debugging.set_log_device_placement(True)
If the output "Num GPUs Available: 0" it means tensorflow didn't recognize your gpu.

# Troubleshooting
Installing the tensorflow GPU version is quite problematic. Sometimes you binge the output message so you can understand what's wrong in your tensorflow or cudatoolkit. That's why I post the problem I faced (and solved) here.
## 1. "cudart64_XXX.dll not found" even using TensorFlow GPU
If you open anaconda prompt, write 'python', then write 'import tensorflow as tf', but the output message has "cudart64_XXX.dll not found", it means the tensorflow version is not matched with cudatoolkit version, and constantly searching for cudatoolkit with XXX version.

In tensorflow 2.3.0, usually "cudart64_101.dll not found" will appear. It means the TensorFlow 2.3.0 still scanning for cudatoolkit 10.1 files, and ignore the newer cudatoolkit versions.


