# UAV NetSim

UAV NetSim is an open-source project that integrates Unreal Engine 4 (UE4), Network Simulator 3 (ns-3), and the Robot Operating System 2 (ROS 2) to create a dynamic simulation environment for UAV (Unmanned Aerial Vehicle) networks. This project uses ROS 2 to provide a flexible framework for UAV control and communication, while ns-3 simulates the underlying communication infrastructure (Radio, Wi-Fi, LTE, etc.), accurately modeling network conditions and limitations across various scenarios. UE4, enhanced with the Microsoft AirSim plugin, is selected for its robust capabilities in creating a highly realistic simulation environment. AirSim enables high-fidelity UAV simulations with support for software-in-the-loop and hardware-in-the-loop setups, compatible with popular flight controllers like PX4 and ArduPilot.

UAV NetSim aims to support researchers in testing communication protocols and network performance under varied environmental conditions, enabling the development and experimentation of UAV control algorithms and network communication—all within a simulated indoor environment.



## Table of Contents  

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)

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
  cd UnrealEngine/
  . ./Setup.sh

- Run the `GenerateProjectFiles.sh` to create the default project file for Unreal Editor.
  
  ```bash
  . ./GenerateProjectFiles.sh

- Run the Unreal Editor inside terminal.

  ```bash
  Engine/Binaries/Linux/UnrealEditor

### 2. Install AirSim plugin for Unreal Engine  

Follow the instruction from Microsoft to learn how to install AirSim and build on different OS at https://microsoft.github.io/AirSim/  
After built, copy the `settings.json` file into `~/Documents/AirSim` directory.

#### 2.1. Install PX4 SITL
PX4-Autopilot provides a software to perform Sofware in the Loop (SITL) vehicles (car, UAV, etc) simulation.

- Clone the repository (current tested version is v1.13).
  ```bash
  git clone -b release/1.13 --recurse-submodules https://github.com/PX4/PX4-Autopilot.git

### 3. Install UAV NetSim

- Clone the repository.
  
  ```bash
  git clone https://github.com/dukeb1212/uav-netsim.git
  cd uav-netsim/

#### 3.1. Set up the environment variables

- Create `.env` file inside the `uav-netsim/` directory.
  
  ```env
  AIRSIM_DIR=/path/to/AirSim
  PX4_DIR=/path/to/PX4-Autopilot
  WSL_HOST_IP=<WSL-IP-Address> # For WSL user

*To find the WSL IP address, open command prompt in Windows and use `ipconfig`, find the IPv4 address of the WSL interface.*

 - **(For WSL user only)** Set up network route in Windows.
   Open `Command Prompt` in Administrator mode and run the command *(change the WSL_IP_ADDR to real WSL IP address)*.
   
   ```bash
   route add 172.30.0.0 MASK 255.255.255.0 <WSL_IP_ADDR> METRIC 1 -p

#### 3.2. Set up AirSim Setting

The default settings file of AirSim is `settings.json`. For more information about every parameter inside AirSim settings please visit the official page at https://microsoft.github.io/AirSim/settings/  

- **(WSL User)** For the basic scenario, add these two values into `settings.json` at the vehicle's settings after `TcpPort`. Change the values to real IP addresses.

   ```json
   "LocalHostIp": "wsl-ip-addr",
   "ControlIp": "linux-eth0-addr",

## Usage

- Create a new Unreal Engine project and add the `AirSim/Unreal/Plugins` directory into the project's directory. (Or use the `Blocks` project under the directory `AirSim/Unreal/Environments/Blocks/`)
- Build and run the project.
- Inside Unreal Editor UI, go to `Windows -> World Settings` and change the game mode to AirSimGameMode.
- Click the `Play` button.

### 1. With Docker
- Inside `uav-netsim` directory, run the command:

	```bash
	sudo ./docker_setup.sh

- The command above will run the `docker-compose.yml` setup and set up the network inside each container.  
  **_You may have to wait about 20-30 seconds for the setup process to complete._**
- To vefiry the network is working, try to ping between 2 UAV containers. The default name is `uav1`, `uav2`.
- First exec inside the container, then run the ping command. For example, in `uav1` container:
	```bash
	docker exec -it uav1 /bin/bash
	ip a
	ping 10.0.0.4
- The IP addresses of UAV containers will start at `10.0.0.1` for `uav1`, then `10.0.0.4` for `uav2` and so on.

- To shutdown UAV NetSim, run this command from the directory `uav-netsim` in terminal.
    ```bash
    docker compose down

- For more specific test cases, checkout the [Wiki](https://github.com/dukeb1212/uav-netsim/wiki) page.

### 2. Build from source
If you want to build the ROS 2 packages manually, follow the tutorial from Wiki: [Build from source](https://github.com/dukeb1212/uav-netsim/wiki/Build-from-source)

** **Prerequisites**: ROS 2 Humble, ns-3-dev

Make sure to install all the necessary packages above.

## Contributing
Contributions are welcome! Please follow these guidelines:
1. Fork the repository.
2. Create a new branch (`git checkout -b feature-name`).
3. Commit your changes (`git commit -m 'Add feature'`).
4. Push to the branch (`git push origin feature-name`).
5. Open a pull request.