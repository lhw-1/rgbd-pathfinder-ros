# RGBD-Pathfinder-ROS

RGBD-Pathfinder-ROS is a ROS Node that takes in an RGB / RGB-D Image, as well as a Goal Destination (represented by a single set of pixel co-ordinates), and finds a traversable path and direction using image segmentation and obstacle detection.

Currently, only image segmentation is used to find a traversable path; obstacle detection has not been implemented.

The ROS Node is also available as a standalone version [here](https://github.com/lhw-1/rgbd-pathfinder). This allows the script to be used on a local machine without reliance on ROS.

---

## Installation

Before the installation, it is recommended to install the following essential libraries (the code should work with Python >= 3.7):

```
pip install torch==1.9.0 torchvision==0.10.0 cudatoolkit=10.2
pip install -U opencv-python
```

Make sure that you are installing the correct CUDA versions for your system GPU. The versions for `torch` and `torchvision` that needs to be installed may also differ depending on your CUDA version.

RGBD-Pathfinder relies on the [Mask2Former Segmentation tool](https://github.com/facebookresearch/Mask2Former), which in turn relies on the [Detectron2](https://github.com/facebookresearch/detectron2) module, to perform Image Segmentation and Visualisation. 

### Installation: Detectron2

Install the `detectron2` module using the following commands from your project root directory.

You may check [here](https://detectron2.readthedocs.io/en/latest/tutorials/install.html) for more information regarding `detectron2` installation. If using Python 3.6 or lower, you may need to install an older version of `detectron2` from source instead.

```
python -m pip install detectron2 -f https://dl.fbaipublicfiles.com/detectron2/wheels/cu102/torch1.9/index.html
pip install git+https://github.com/cocodataset/panopticapi.git
pip install git+https://github.com/mcordts/cityscapesScripts.git
```

### Installation: RGBD-Pathfinder-ROS

Once `detectron2` has been installed, we can install and set up RGBD-Pathfinder-ROS. Follow [this guide](http://wiki.ros.org/noetic/Installation/) to install ROS Noetic for your distribution.

Once ROS Noetic has been installed for your system, set up the Catkin Workspace as per the [guide here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment), or by following the commands below. From your root directory:

```
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make install
source devel/setup.bash
```

Use the commands below to clone this repository as a ROS Node and download the ADE20K Panoptic Segmentation model:

```
cd src/
git clone https://github.com/lhw-1/rgbd-pathfinder-ros.git
mv rgbd-pathfinder-ros/ pathfinder/
cd rgbd-pathfinder-ros
git submodule init
git submodule update
pip install -r requirements.txt
sh init.sh
```

Alternatively, you may download your model of choice from the [Mask2Former Model Zoo](https://github.com/facebookresearch/Mask2Former/blob/main/MODEL_ZOO.md), and the corresponding configuration files from the [Mask2Former Configs](https://github.com/facebookresearch/Mask2Former/tree/main/configs). Refer to [this guide](https://github.com/facebookresearch/Mask2Former/blob/main/GETTING_STARTED.md) for more information on the available models and corresponding configuration files. 

If using a model other than ADE20K, you may also need to modify the list of categories that should be classified as traversable. This list can be found [here](https://github.com/lhw-1/rgbd-pathfinder/blob/main/src/standalone/traversable.py), with instructions on how to do so.

Finally, change `init.sh` to run with the correct model and the configuration file if you are using any model other than the default.

### Known Problems

Some Python files in RGBD-Pathfinder-ROS may not be recognised. In this case, add the pathfinder src to the PYTHONPATH:

```
export PYTHONPATH=$PYTHONPATH:~/catkin_ws/src/pathfinder/src/
export PYTHONPATH=$PYTHONPATH:~/catkin_ws/src/pathfinder/src/Mask2Former/
```

This will allow Python to find the necessary scripts.

In some cases, `detectron2` may need to be rebuilt if PyTorch was re-installed during the installation process.

To rebuild `detectron2` from your working directory:

```
cd detectron2/
rm -rf build/ **/*.so
python -m pip install -e .
```

---

## What it currently does:

The current script handles the following:
- Conversion from RGB Image to Image Segmentation using the ADE20K model (default) of the Mask2Former Segmentation Tool.
- Calculation of Traversable Paths inferred from the Image Segmentation, and Plotting the Paths accordingly onto the original RGB Image.
- Instructions for the agent to follow according to the Traversable Paths, depending on the location of the Goal Destination relative to current position.

Currently, additional pruning of possible traversable paths based on Depths / Obstacle Detection has not been implemented.

---
