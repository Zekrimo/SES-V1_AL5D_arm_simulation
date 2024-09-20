# ROS Simulation

This repository contains a ROS simulation for [project name]. It allows you to [describe what the simulation does].

## Prerequisites

Before running the simulation, make sure you have the following installed:

- ROS (Robot Operating System) [HUMBLE ]
- Linux (Ubuntu) [22.04.4 LTS Jammy Jellyfish]

## Installation

To install the simulation, follow these steps:

1. Clone this repository to your ROS workspace:
    ```bash
    git clone https://github.com/Zekrimo/SES-V1_AL5D_arm_simulation.git
    ```

2. Build the workspace:
    ```bash
    cd [path to your ROS workspace]
    colcon build
    ```

## Usage

To run the simulation, execute the following command:
    ```bash
    ros2 launch lynxarmsimulation lynxarmsimulation_launch.py 
    ```

Open a new terminal and rum the command to publish the demo sequence serial messages for the arm position
    ```bash
    ros2 run commandpublisher commandpublisher
    ```    
To send a custom serial message use the command below
    ```bash
    ros2 run commandpublisher commandpublisher "#0P500T1000"
    ```   
To enable emergency brake
    ```bash
    ros2 run commandpublisher commandpublisher "STOP"
    ```   
To disable emergency brake
    ```bash
    ros2 run commandpublisher commandpublisher "START"
    ```  

# Serial Message Format

To see the format of the serial messages please refer to the documnetation PDF file in this repository.

## Debug
Debug flag van be turned to true to print debug messages
