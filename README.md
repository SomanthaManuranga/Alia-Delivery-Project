# ALIA Delivery Project

## Alia-Delivery Project

## Description

A webots project to simulate the digital twin of a hospital building. We are focusing on a single floor of the building for now.

## Roadmap

-> Get started with learning the webots software, learn how to add simulation, implement MQTT and path finding algorithm to the software
-> Implement basic working demo of a software on order going to the ordered room in the floor plan
-> After setting the basic, functionalities like delivering medicine can be realized.

## Contributing

make branch with the name format "feature_username_featurename" . After implementing the necessary task put an Merge Request to the main branch, which will be reviewed and merged by the owner.
Peer reviews are expected when someone is making a merge request.

# Project Setup Instructions

## Requirements

Install below tools to run the project:

-   Cyberbotics Webots: "https://cyberbotics.com/"
-   Any IDE preferable VS Code: "https://code.visualstudio.com/download"
-   Mosquitto broker: "https://mosquitto.org/download/"
-   Python 3: "https://www.python.org/downloads/"

## Setup and Run

### 1. Run the Hospital Simulation

-   Open the "Alia_Delivery.wbt" simulation file in Webots.
-   The file exist on path "webots/worlds/Alia_Delivery"

### 2. Run the Web Application

-   Run VS Code IDE and open the web_app folder in it.

### 3. Install Dependencies

-   Open bash or command prompt and execute the command: `pip install -r requirements.txt`

### 4. Run the Web Project

-   Run the project by executing the command: `python app.py`

### 5. Interact With Web Interface

-   Use the dropdown to select a room and send a command.
-   The robot should listen to the command and start moving to the selected room.
