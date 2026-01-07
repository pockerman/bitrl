import time
import datetime
import json
import paho.mqtt.client as mqtt

BROKER = "localhost"
PORT = 1883
ULTRASOUND_TOPIC = "ultrasound"

def main():
    # Initialize MQTT
    client = mqtt.Client()
    client.connect(BROKER, PORT, 60)

    distance = 0.0
    try:
        while True:
            z_str = json.dumps({"distance": distance, "unit":"cm",
                                "timestamp": str(datetime.datetime.now(datetime.UTC))}
                              )
            distance += 0.01
            client.publish(topic=ULTRASOUND_TOPIC, payload=z_str)
            time.sleep(2.0)

    except Exception:
        print("Stopping...")


if __name__ == "__main__":
    main()
