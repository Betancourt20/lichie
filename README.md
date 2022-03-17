# LicHie Project
This repository correspond to the development of diferent control laws (beta version) for using two different robot manipulators: Jaco2 and puma560.
## Install
You will need Python >= 3.6
### Get the repositories 
```
$ mkdir name_ws
$ cd name_ws
$ git clone git@github.com:Betancourt20/lichie.git
```
This folder contains all the different test simulations with different robot manipulators as well as different control laws. 
Furthermore, we will need vcstool for installing dependences as well other repositories, thus
```
$ git clone git@gitlab.com:AugustinManecy/vcstool.git
$ cd cd vcstool/
$ vcs-recursive
```
Once all dependences installed, this will pull four repositories ``robotics-toolbox-python``, ``spatialgeometry``, ``spatialmath-python`` and ``swift``. Then,
```
$ cd robotics-toolbox-python
$ pip3 install -e .

$ cd spatialgeometry
$ pip3 install -e .

$ cd spatialmath-python
$ pip3 install -e .

$ cd swift
$ pip3 install -e .

```
This will completely install all the repositories in the python path in order to use the moduls. We ready to play!
