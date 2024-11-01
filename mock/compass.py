import paho.mqtt.publish as publish
import time

# MQTT settings
MQTT_TOPIC = "pico_w/car"
MQTT_HOST = "localhost"

# Initialize pitch and roll values
pitch = 0
roll = 0


# Function to publish pitch and roll values
def publish_values():
    payload = f"{{p:{pitch},r:{roll},y:0}}"
    publish.single(MQTT_TOPIC, payload, hostname=MQTT_HOST)
    print(f"Published: {payload}")


# Function to update pitch and roll based on key press
def update_values(key):
    global pitch, roll
    if key == "w":
        pitch = 30
        roll = 0
    elif key == "s":
        pitch = -30
        roll = 0
    elif key == "a":
        roll = -30
        pitch = 0
    elif key == "d":
        roll = 30
        pitch = 0
    elif key == "q":
        roll = 0
        pitch = 0
    elif key == "":
        return False
    return True


# Main loop
try:
    while True:
        key = (
            input("Press 'w', 's', 'a', 'd' to control, 'q' to quit: ").strip().lower()
        )
        if not update_values(key):
            break
        publish_values()
        time.sleep(1)  # Publish every second
        # payload = f"{{p:{pitch},r:{roll},y:0}}"
        # publish.single(MQTT_TOPIC, payload, hostname=MQTT_HOST)
except KeyboardInterrupt:
    print("Script terminated by user")
