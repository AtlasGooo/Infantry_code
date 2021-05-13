# Introduction to InfantryRTS

InfantryRTS is a Real-Time Strategy system for infantry robot.

## Structure

There will be 5 parts in the system
```bash
infantry_camera     # to capture the image from the camera
infantry_detection  # to recognize and locate the armor in the image
infantry_msgs       # to define the message format in the pacakge
infantry_serial     # to communicate with the MCUs
infantry_bringup    # to launch the system
```


## TODO
- [ ] infantry_camera     
- [ ] infantry_detection 
- [x] infantry_msgs      
- [x] infantry_serial     
- [ ] infantry_bringup

## documents

1. [ARTINX步兵上位机串口通信手册](./infantry_serial/Readme.md)


## usage

```
cd {your catkin workspace}/src
git clone https://github.com/Artinx-Algorithm-Group/InfantryRTS.git
cd ..
catkin_make
```