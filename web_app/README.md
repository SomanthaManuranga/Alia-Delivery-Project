# Project Setup Instructions

## Requirements

Install below tools to run the project:

-   Any IDE preferable VS Code: "https://code.visualstudio.com/download"
-   Mosquitto broker: "https://mosquitto.org/download/"
-   Python 3: "https://www.python.org/downloads/"

## Setup and Run

### 1. Run the Web Application

-   Run VS Code IDE and open the web_app folder in it.

### 2. Install Dependencies

-   Open bash or command prompt and execute the command: `pip install -r requirements.txt`

### 3. Run the Web Project

-   Run the project by executing the command: `python app.py`

### 4. Open your browser

-   Visit [http://127.0.0.1:5050](http://127.0.0.1:5050)

### 5. Interact With Web Interface

-   Use the dropdown to select a room and send a command.
-   You can verify messages using:

```bash
mosquitto_sub -t robot/commands -v
```
