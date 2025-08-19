from flask import Flask, render_template, request, jsonify
import paho.mqtt.client as mqtt
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

broker = "localhost"
port = 1883

def send_mqtt_command(command):
    try:
        print(f"Publishing to MQTT: {command}")
        client = mqtt.Client()
        client.connect("127.0.0.1", 1883, 60)

        # Start the MQTT network loop
        client.loop_start()  
        client.publish("robot/commands", command)
        client.loop_stop()  
        client.disconnect()
        print("Publish successful.")
    except Exception as e:
        print("MQTT Publish failed:", e)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/send", methods=["POST"])
def send():
    data = request.json
    room_number = data.get("room")
    command = f"go_to_room_{room_number}"
    send_mqtt_command(command)
    return jsonify({"status": "Command sent", "command": command})

if __name__ == '__main__':
    app.run(debug=True,port=5050)