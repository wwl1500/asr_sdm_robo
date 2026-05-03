# asr_sdm_description

Mesh and URDF Model for the amphibious snake-like robot with screw-drive mechanism.

## Development Tools
### FreeCAD Installation
```sh
# Download AppIamge file from https://github.com/FreeCAD/FreeCAD

# Git clone RobotCAD
git clone https://github.com/drfenixion/freecad.robotcad.git

# Place FreeCAD 1.1 AppImage to docker/freecad/freecad_custom_appimage_dir
cd freecad.robotcad/docker

# Use following command for the first time
bash run.bash -cif

# Regular RobotCAD run
bash run.bash -c

# In case of start issue (no RobotCAD in workbench options), recreate container by
bash run.bash -f
```

## Model
* .stp files are exported by Autodesk Inventor. They are used in FreeCAD-RobotCAD to generate urdf and xml file.

## Issues

* Inertia matrix not correct yet
* No collision model yet

## Deprecated
```sh
* The model is based on https://github.com/bitcraze/bitcraze-mechanics/blob/master/models/cf_model_s_revA.skp
* Use SketchUp (Tested with 14.1) to update/change the model
* Export using the following options: ![Export settings](export.png)

change xacro into urdf:
rosrun xacro xacro --inorder -o underwater_snakerobot.urdf underwater_snakerobot.urdf.xacro

check urdf model visually:
cd ~/catkin_ws/src
catkin_create_pkg my_urdf_test_pkg urdf
roscd my_urdf_test_pkg
mkdir urdf
gedit urdf/my_simple_model.urdf
sudo apt install liburdfdom-tools
roslaunch urdf_tutorial display.launch model:=urdf/my_simple_model.urdf
```