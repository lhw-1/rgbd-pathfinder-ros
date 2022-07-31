# RGBD-Pathfinder-ROS

RGBD-Pathfinder-ROS is a ROS Node that takes in an RGB / RGB-D Image, as well as a Goal Destination (represented by a single set of pixel co-ordinates), and finds a traversable path and direction using image segmentation and obstacle detection.

Currently, only image segmentation is used to find a traversable path; obstacle detection has not been implemented.

The ROS Node is also available as a standalone version [here](https://github.com/lhw-1/rgbd-pathfinder). This will allow the script to be used on a local machine without reliance on ROS.

---

## Installation

Before the installation, it is recommended to install the following essential libraries (the code should work with Python >= 3.7):

```
pip install torch==1.9.0 torchvision==0.10.0 cudatoolkit=10.2
pip install -U opencv-python
```

Make sure that you are installing the correct CUDA versions for your system GPU. The versions for&nbsp;`torch`&nbsp;and&nbsp;`torchvision`&nbsp;that needs to be installed may also differ depending on your CUDA version.

RGBD-Pathfinder relies on the [Mask2Former Segmentation tool](https://github.com/facebookresearch/Mask2Former), which in turn relies on the [Detectron2](https://github.com/facebookresearch/detectron2) module, to perform Image Segmentation and Visualisation.

### Installation: Mask2Former and Detectron2

Install the&nbsp;`detectron2`&nbsp;module using the following commands. From your project root directory:

```
git clone https://github.com/facebookresearch/detectron2.git
cd detectron2
pip install -e .
pip install git+https://github.com/cocodataset/panopticapi.git
pip install git+https://github.com/mcordts/cityscapesScripts.git
```

You may check [here](https://detectron2.readthedocs.io/en/latest/tutorials/install.html) for more information regarding&nbsp;`detectron2`&nbsp;installation. If using Python 3.6 or lower, you may need to install an older version of&nbsp;`detectron2`&nbsp;from source instead.

### Installation: RGBD-Pathfinder

Once&nbsp;`detectron2`&nbsp;has been installed, we can install and set up RGBD-Pathfinder.

Use the commands below to clone this repository and download the ADE20K Panoptic Segmentation model:

```
cd ..
git clone https://github.com/lhw-1/rgbd-pathfinder-ros.git
cd rgbd-pathfinder-ros
pip install -r requirements.txt
sh init.sh
```

Alternatively, you may download your model of choice from the [Mask2Former Model Zoo](https://github.com/facebookresearch/Mask2Former/blob/main/MODEL_ZOO.md), and the corresponding configuration files from the [Mask2Former Configs](https://github.com/facebookresearch/Mask2Former/tree/main/configs). Refer to [this guide](https://github.com/facebookresearch/Mask2Former/blob/main/GETTING_STARTED.md) for more information on the available models and corresponding configuration files.

If using a model other than ADE20K, you may also need to modify the list of categories that should be classified as traversable. This list can be found [here](https://github.com/lhw-1/rgbd-pathfinder/blob/main/src/standalone/traversable.py), with instructions on how to do so.

Finally, change&nbsp;`init.sh`&nbsp;to run with the correct model and the configuration file.

### Known Problems

In some cases,&nbsp;`detectron2`&nbsp;may need to be rebuilt if PyTorch was re-installed during the installation process.

To rebuild&nbsp;`detectron2`&nbsp;from your working directory:

```
cd detectron2/
rm -rf build/ **/*.so
python -m pip install -e .
```

---

## What it currently does:

The current script handles the following:

-   Conversion from RGB Image to Image Segmentation using the ADE20K model (default) of the Mask2Former Segmentation Tool.
-   Calculation of Traversable Paths inferred from the Image Segmentation, and Plotting the Paths accordingly onto the original RGB Image.
-   Instructions for the agent to follow according to the Traversable Paths, depending on the location of the Goal Destination relative to current position.

Currently, additional pruning of possible traversable paths based on Depths / Obstacle Detection has not been implemented.

---