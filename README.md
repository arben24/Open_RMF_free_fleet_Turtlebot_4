# Integration of a Turtlebot 4 into Open-RMF using the Free Fleet Adapter

This repository demonstrates how to integrate a Turtlebot 4 into [Open-RMF](https://www.open-rmf.org) using the [Free Fleet Adapter](https://github.com/open-rmf/free_fleet). The aim is to showcase its practical application in a realistic environment. For this purpose, an affordable demonstrator was developed that enables testing of robots, doors, and elevators.

---

## Demo Video

[![Demonstration](https://img.youtube.com/vi/EfvdhYBY6P0/0.jpg)](https://www.youtube.com/watch?v=EfvdhYBY6P0)

---

## Overview

1. [Building the Demonstrator](#1-building-the-demonstrator)
2. [Programming the Door and Elevator Control](#2-programming-the-door-and-elevator-control)
3. [Setting Up Open-RMF and the Free Fleet Adapter](#3-setting-up-open-rmf-and-the-free_fleet_adapter)
4. [Demonstration Results](#4-demonstration-results)

---

## 1. Building the Demonstrator

The demonstrator is modular to ensure flexibility and ease of customization.

### Materials and Construction:
- **Base Material**: Chipboard (Dimensions: 1690 x 634 x 12 mm, other sizes possible).
- **Connections**: 3D-printed connectors allow quick and flexible assembly.
- **Doors**: 
  - Cutouts from chipboard attached with affordable hinges.
  - Movement is achieved using standard RC servos mounted in 3D-printed housings.
  - The door and servo are connected via a servo horn and a 3D-printed "U"-shaped element, tolerating misalignments.

---

## 2. Programming the Door and Elevator Control

An **ESP32** is used to control the doors and elevator. [Visual Studio Code](https://code.visualstudio.com) with the [PlatformIO](https://platformio.org) extension is recommended as the **development environment**.

Additionally, [Micro-ROS](https://micro.ros.org) must be set up. This allows the ESP32 to interact directly with the ROS network, reading and writing the states of doors and elevators. The relevant topics are `/door_states` and `/door_requests`.
You can Find example Code [here](/Platformio/).

---

## 3. Setting Up Open-RMF and the Free_Fleet_Adapter

### Creating the Map for Open-RMF

A floor plan in PNG format is required for visualization in RViz. 

![example floor plan](/images/map_L1.png)

Before setting up, the robot must map its environment, following instructions similar to [THIS](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html). This process generates two files: a `.yaml` file (metadata) and a `.pgm` file (visual representation).

![example map](/images/lidar_map.png)

#### Steps to Create a Map with the Traffic Editor:

A more detailed guide can be found [HERE](https://docs.google.com/presentation/d/1Lt79xlM_XkITmURSbI5hkAAgnjSX8dHKBkgvz3x3Uzw/edit?pli=1#slide=id.g117b0289c78_0_0l).

1. **Launch the Traffic Editor**: Run `traffic-editor` in the sourced Open-RMF workspace.
2. **Create Project and Levels**: Define names, height, and floor plan.
3. **Scale the Map and Add Elements**:
   - Draw walls, floors, doors, and elevators.
4. **Define Robot Paths**: Add nodes and edges.
5. **Adjust LiDAR Scans**:
   - Necessary for later coordinate transformation.

Save the `.building.yaml` file in the `rmf_demos_maps` directory.

---

### Setting Up the Free Fleet Adapter for Turtlebot 4

The Turtlebot 4 is integrated into Open-RMF via the Free Fleet Adapter. A dedicated ROS workspace was created based on the instructions from the [Free Fleet](https://github.com/open-rmf/free_fleet/tree/main) Adapter. The following changes were made:

#### Server Side:
1. Create a launch file ([example](/server.launch)) in the `ff_examples_ros2/launch` directory.
2. Configure:
   - Names.
   - Coordinate transformation parameters such as `translation_x`, `translation_y`, `rotation`, and `scale` (from the Traffic Editor).

#### Client Side:
1. Create a launch file ([example](/turtlebot4_world_ff.launch.xml)) in the `ff_examples_ros2/launch` directory.
2. Use the mapped environment (`.yaml` and `.pgm` files).
3. Adjust the file paths in the launch file.
4. If display issues occur, set the `origin` parameter in the `.yaml` file to `0`.

---

### Setting Up the Fleet Adapter for Open-RMF

To ensure Open-RMF recognizes the robot:
1. Create a launch file ([example](/fleet_adapter.launch.xml)) for the fleet adapter (based on [THIS](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/launch/fleet_adapter.launch.xml)).
2. Place it in the `rmf_demos_fleet_adapter/launch` directory.
3. Adapt the configuration to the robot's parameters (example launch file provided in this repository).

---

## Starting the System

### Steps:
1. **Build and Source**:
   - Run `colcon build` in each workspace and source them.
2. **Start the Systems**:
   1. Free Fleet Server:  
      `ros2 launch ff_examples_ros2 turtlebot4_world_ff_server.launch.xml`
   2. Free Fleet Client:  
      `ros2 launch ff_examples_ros2 turtlebot4_world_ff.launch.xml`
   3. Open-RMF:  
      `ros2 launch rmf_demos test.launch.xml`

After step 3, RViz should open and display the created map.

---

## Common Issues and Solutions

| Problem                                   | Solution                                                                |
|-------------------------------------------|-------------------------------------------------------------------------|
| Robot is displayed in the wrong position  | Check the `origin` parameter in the `.yaml` file.                      |
|                                           | Verify coordinate transformation in the server launch file.            |
|                                           | Rebuild and source the workspaces.                                     |
| Fleet adapter not recognized by Open-RMF  | Ensure Free Fleet Client and Server are running correctly.             |
|                                           | Check the fleet adapter launch file for errors.                        |


## To-Do-List
- [ ] improve README
- [ ] complete examples
- [ ] add more Solutions for Issues


