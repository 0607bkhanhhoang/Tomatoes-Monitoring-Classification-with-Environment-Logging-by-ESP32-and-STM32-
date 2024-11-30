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
