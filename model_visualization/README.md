This script is designed to perform real-time object detection using YOLO (You Only Look Once) and communicate detection results to an ESP32 via a serial connection. The overall goal is to detect specific tomato deseases using a webcam, log the detection results, and periodically send these results to an ESP32 device. Here is a breakdown of the script’s key components:

# Key Components:
## Logging Setup:

The script uses Python's logging module to log detection events into a file called capture.log. This allows for tracking YOLO inference results and error messages.

## StreamToLogger Class:

Captures print statements (stdout) and error messages (stderr) and sends them to the logger instead of printing to the terminal. This ensures that all important messages are logged for review.
send_to_esp32 Function:

Sends detection data (like the count of specific detected plant diseases) to the ESP32 via a serial connection. The data is formatted as a string and sent through the serial interface.
YOLO Inference (run_yolo_inference Function):

Initializes the YOLO model (best.pt) and captures frames from the webcam.
Each frame is processed by YOLO for object detection, and bounding boxes, confidence scores, and class IDs are logged.
Detected objects are annotated in the webcam feed (bounding boxes displayed), and the feed is shown on the screen.
Processing and Sending Detection Counts (process_and_send_counts Function):

Reads the capture.log file where YOLO detections are logged.
Counts the occurrences of specific class IDs corresponding to plant diseases (early_blight, mold_leaf, and tomato_healthy).
Sends these counts to the ESP32 device over the serial connection.
If any data is received from the ESP32, it is logged.
## Multithreading:

The script uses Python's threading module to run two tasks concurrently:
YOLO inference and webcam processing.
Processing log data, counting detections, and sending this information to the ESP32.
## Main Function:

Initializes and starts two threads: one for running YOLO inference (run_yolo_inference) and one for processing log data and sending counts to the ESP32 (process_and_send_counts).
Serial Communication:

The ESP32 device is connected via serial port (COM10 with baud rate 115200).
The script sends detection counts over the serial link and waits for responses from the ESP32.
## Execution Flow:
YOLO Detection: The webcam feed is continuously captured and processed by the YOLO model for object detection. The results are logged.
Counting and Sending: Detection counts (based on specific class IDs) are periodically computed from the log file and sent to the ESP32 via serial communication.
Real-time Interaction: Detection data is updated regularly, and communication with the ESP32 allows for real-time feedback.

### Function to send to ESP32
```bash
def send_to_esp32(data):
    try:
        message = f"{data}\n"
        ser.write(message.encode())
        print(f"Sent detection data to ESP32: {message}")
    except Exception as e:
        print(f"Error sending data to ESP32: {e}")
```

### Setting log output
```bash
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
```

### Process Log File 
**Processing log file captured from terminal**
```bash
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
```