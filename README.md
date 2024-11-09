# UAV NetSim

UAV NetSim is an open-source project that integrates Unreal Engine 4 (UE4), Network Simulator 3 (ns-3), and the Robot Operating System 2 (ROS 2) to create a dynamic simulation environment for UAV (Unmanned Aerial Vehicle) networks. This project uses ROS 2 to provide a flexible framework for UAV control and communication, while ns-3 simulates the underlying communication infrastructure (Radio, Wi-Fi, LTE, etc.), accurately modeling network conditions and limitations across various scenarios. UE4, enhanced with the Microsoft AirSim plugin, is selected for its robust capabilities in creating a highly realistic simulation environment. AirSim enables high-fidelity UAV simulations with support for software-in-the-loop and hardware-in-the-loop setups, compatible with popular flight controllers like PX4 and ArduPilot.

UAV NetSim aims to support researchers in testing communication protocols and network performance under varied environmental conditions, enabling the development and experimentation of UAV control algorithms and network communication—all within a simulated indoor environment.

![Build Status](https://img.shields.io/github/workflow/status/username/repo-name/build) 
![License](https://img.shields.io/github/license/username/repo-name)

## Table of Contents  

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Features  

- Realistic UAV Network Simulation ✔️
- Support ROS 2 ✔️
- High-fidelity Visualization with Unreal Engine ✔️
- Multi-platform (Windows WSL2, Ubuntu Linux) ✔️
- Support UAV Swarm Simulation

## Installation  

### 1. Intall Unreal Engine  

The current version of Unreal Engine supported is UE4.27. There will be a migration onto UE5 version in the future if needed.  

#### 1.1. Windows  

- Download Epic Games Launcher at https://store.epicgames.com/en-US/download
- Create/Login with an account and go to Unreal Engine tab, click the Install button and choose the right version (UE4.27).

#### 1.2. Ubuntu  

This project is developed and tested on Ubuntu 22.04 Jammy. Other Ubuntu OS versions (Focal, Noble) can be used but might have unexpected problems.  

*First you may have to link the GitHub account to the Epic Games account at https://www.epicgames.com/account/connections* 

*Also create and add SSH key of your computer to GitHub in the Setting since GitHub remove the password method*

- Go to the Unreal Engine repository https://github.com/EpicGames/UnrealEngine and clone the repository.
  ```bash
  git clone -b 4.27 git@github.com:EpicGames/UnrealEngine.git
  
- Go to the source directory and run the Setup.sh script to set up the install environment.
  
  ```bash
  cd /UnrealEngine
  . /Setup.sh

- Run the `GenerateProjectFiles.sh` to create the default project file for Unreal Editor.
  
  ```bash
  . /GenerateProjectFiles.sh

- Run the Unreal Editor inside terminal.

  ```bash
  Engine/Binaries/Linux/UnrealEditor

### 2. Install AirSim plugin for Unreal Engine  

Follow the instruction from Microsoft to learn how to install AirSim and build on different OS at https://microsoft.github.io/AirSim/  
After built, copy the `settings.json` file into `~/Documents/AirSim` directory.

### 3. Install UAV NetSim

- Clone the repository.
  
  ```bash
  git clone https://github.com/dukeb1212/uav-netsim.git
  cd uav-netsim/

#### 3.1. Set up the environment variables

- Create `.env` file inside the `uav-netsim/` directory.
  
  ```env
  AIRSIM_FOLDER_PATH=/path/to/AirSim
  WSL_HOST_IP=<WSL-IP-Address> # For WSL user

*To find the WSL IP address, open command prompt in Windows and use `ipconfig`, find the IPv4 address of the WSL interface.*

 - **(For WSL user only)** Set up network route in Windows.
   Open `Command Prompt` in Administrator mode and run the command *(change the WSL_IP_ADDR to real WSL IP address)*.
   
   ```bash
   route add 172.30.0.0 MASK 255.255.255.0 <WSL_IP_ADDR> METRIC 1 -p

