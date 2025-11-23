import cv2
import base64
import time
import paho.mqtt.client as mqtt

BROKER = "localhost"
PORT = 1883
TOPIC = "camera"

def main():
    # Initialize MQTT
    client = mqtt.Client()
    client.connect(BROKER, PORT, 60)

    # Open camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ Could not open camera")
        return

    print("📷 Sending frames... Press Ctrl+C to stop.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            # Encode as JPEG
            success, jpeg = cv2.imencode(".jpg", frame)
            if not success:
                continue

            # Base64 encode
            jpeg_bytes = jpeg.tobytes()
            b64 = base64.b64encode(jpeg_bytes).decode("ascii")

            # Publish
            client.publish(TOPIC, b64)
            print("Frame send...")
            time.sleep(2.0)

    except KeyboardInterrupt:
        print("Stopping...")

    cap.release()
    client.disconnect()

if __name__ == "__main__":
    main()
