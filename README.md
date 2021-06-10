# course21

Here you can find the source files for the "Embodiment of AI" course @ Unical 2021

### setup the tutorial
The tutorial is based on the virtual simulator https://github.com/WPI-AIM/ambf. Libraries and codes have been tested on Ubuntu 18.04 LTS with python2.7 and python3.6 and ROS melodic. The environment is already set up on the virtual machine provided at https://tinyurl.com/VMEmbAI (suggested)

* download and install ROS melodic http://wiki.ros.org/melodic/Installation/Ubuntu
* download and install ambf (follow the instruction at https://github.com/WPI-AIM/ambf)
* clone this repository
* replace the content of the ambf/embf_models folder with the content of the "models" folder provided in this repository

## run the tutorial
* open a new linux terminal (we will call it A) and run the roscore
* open a new linux terminal (we will call it B), change the current directory to `ambf/lin-x86_64/ folder`, and run `./ambf_simulator -l 0`
* now you are able to run the pyhton script controlling your robot (check below)


### folder organization

The files are organized as follows:
````
course
|__models
  |__description
  |__meshes
|__script
  |__tutorial1
  |__tutorial2
  |__tutorial3
````

* The "models" folder contains all the 3d models and description used in the ambf simulator
* The "script" folder contains all the python files used to simulate the robot behavior
