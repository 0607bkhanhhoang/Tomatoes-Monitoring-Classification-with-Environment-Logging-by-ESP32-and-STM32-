import threading
import logging
import sys
import cv2
from ultralytics import YOLO
import serial
import json
import time

port = 'COM10' 
baudrate = 115200

logging.basicConfig(
    filename="capture.log",  
    level=logging.INFO,      
    format="%(asctime)s - %(message)s"
)

class StreamToLogger:
    """
    Captures the output from terminal and sends it to the logger.
    """
    def __init__(self, logger, log_level):
        self.logger = logger
        self.log_level = log_level

    def write(self, message):
        """
        Captures and logs the message.
        """
        if message.strip():  
            self.logger.log(self.log_level, message.strip())

    def flush(self):
        """
        Flush the stream (not needed for this implementation).
        """
        pass

sys.stdout = StreamToLogger(logging.getLogger(), logging.INFO)  
sys.stderr = StreamToLogger(logging.getLogger(), logging.ERROR)

def send_to_esp32(data):
    try:
        message = f"{data}\n"
        ser.write(message.encode())
        print(f"Sent detection data to ESP32: {message}")
    except Exception as e:
        print(f"Error sending data to ESP32: {e}")

def run_yolo_inference():

    model = YOLO("best.pt")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot access to webcam")
        return

    print("Webcam opened. Starting YOLO inference... Press 'q' to stop.")

    while True:
        try:
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame from webcam. Exiting...")
                break

            # Perform YOLO inference
            results = model(frame)

            # Check if there are any detected boxes
            if results and len(results) > 0:
                detection = results[0]  

                if detection.boxes:

                    for box in detection.boxes:
                        xyxy = box.xyxy.numpy()  # Bounding box coordinates (x1, y1, x2, y2)
                        conf = box.conf.numpy()   # Confidence score
                        cls = box.cls.numpy()     # Class ID

                        # Log detection data
                        logging.info(f"Detected object with confidence: {conf}")
                        logging.info(f"Bounding Box: {xyxy}")
                        logging.info(f"Class ID: {cls}")

                # Annotate frame with detection boxes
                annotated_frame = detection.plot() if hasattr(detection, 'plot') else frame
                cv2.imshow("YOLO Webcam", annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Exiting webcam inference...")
                break

        except Exception as e:
            print(f"Error during YOLO inference: {e}")
            break

    # Release resources and close windows
    cap.release()
    cv2.destroyAllWindows()
    print("Webcam closed.")

    cap.release()
    cv2.destroyAllWindows()
    print("Webcam closed.")


def process_and_send_counts():
    """
    Processes the log file to count detections and sends the counts over USB to ESP32.
    """
    ser = serial.Serial(port, baudrate, timeout=1)  # Connect to ESP32
    log_file = "capture.log"

    while True:  
        try:
            blight = 0  # early_blight counting value
            mold = 0    # mold_leaf counting value
            tomato = 0  # tomato_healthy counting value

            # Read the log file and count detections
            with open(log_file, "r") as file:
                for line in file:
                    if "Class ID: [2.]" in line:
                        blight += 1
                    if "Class ID: [1.]" in line:
                        mold += 1
                    if "Class ID: [3.]" in line:
                        tomato += 1

            detection_data = f"early_blight:{blight},mold_leaf:{mold},tomato_healthy:{tomato}"

            send_to_esp32(detection_data)

            if ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                print(f"Received from ESP32: {response}")

            time.sleep(1)

        except Exception as e:
            print(f"Error in USB communication: {e}")
            break

    ser.close()
    print("Serial communication ended.")


def main():
    """
    Main function to start YOLO inference and log monitoring in parallel.
    """
    yolo_thread = threading.Thread(target=run_yolo_inference)
    usb_thread = threading.Thread(target=process_and_send_counts)

    yolo_thread.start()
    usb_thread.start()

    yolo_thread.join()
    usb_thread.join()

if __name__ == "__main__":
    main()
