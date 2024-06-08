from picamera2 import Picamera2
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

# GPIO pin setup for accident detection signal
ACCIDENT_PIN = 26  # Choose an available GPIO pin on Raspberry Pi

GPIO.setmode(GPIO.BCM)
GPIO.setup(ACCIDENT_PIN, GPIO.IN)

# GPIO pin setup
EnableL = 19
HighL = 27       # LEFT SIDE MOTOR
LowL = 22
EnableR = 18
HighR = 23      # RIGHT SIDE MOTOR
LowR = 24

# Initialize GPIO for motors
GPIO.setmode(GPIO.BCM)
motor_pins = [EnableL, HighL, LowL, EnableR, HighR, LowR]
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

# Focal Length (you need to calculate this based on your setup)
FOCAL_LENGTH = 200  # Example value, adjust this based on your calibration
OBJECT_REAL_WIDTH = 20  # Real width of the object in cm (adjust this to your object)
STOP_REAL_WIDTH = 8

# Set up PWM
pwm_frequency = 1000  # Adjust as needed
pwmL = GPIO.PWM(EnableL, pwm_frequency)
pwmR = GPIO.PWM(EnableR, pwm_frequency)
pwmL.start(0)  # Start with motors off
pwmR.start(0)

LeftLanePos, RightLanePos, frameCenter, laneCenter, Result = 0, 0, 188, 0, 0
Source = np.float32([[55,160], [345,160], [55,210], [345,210]])
Destination = np.float32([[100,0], [280,0], [100,240], [280,240]])

# Motor control functions
def set_motor_speed(left_speed, right_speed):
    pwmL.ChangeDutyCycle(left_speed)
    pwmR.ChangeDutyCycle(right_speed)

def forward(speed=50):
    GPIO.output(HighL, GPIO.HIGH)
    GPIO.output(LowL, GPIO.LOW)
    GPIO.output(HighR, GPIO.HIGH)
    GPIO.output(LowR, GPIO.LOW)
    set_motor_speed(speed, speed)

def stop():
    set_motor_speed(0, 0)

def turn_left(speed=50, intensity=1):
    # Intensity can be 1 (mild turn), 2 (medium turn), 3 (sharp turn)
    left_speed = max(0, speed - intensity * 15)
    set_motor_speed(left_speed, speed)

def turn_right(speed=50, intensity=1):
    # Intensity can be 1 (mild turn), 2 (medium turn), 3 (sharp turn)
    right_speed = max(0, speed - intensity *15)
    set_motor_speed(speed, right_speed)
    
def detect_accident():
    """
    Detect accident based on the signal from Arduino.
    Returns True if an accident is detected, else False.
    """
    accident_detected = GPIO.input(ACCIDENT_PIN)
    return accident_detected

# Load cascades for detection
stop_cascade = cv2.CascadeClassifier('/home/pi/Desktop/Stop_cascade.xml')
object_cascade = cv2.CascadeClassifier('/home/pi/Desktop/Object_cascade.xml')
traffic_sign_cascade = cv2.CascadeClassifier('/home/pi/Desktop/traffic_cascade.xml')

picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(main={"size":(400, 240)})
picam2.configure(camera_config)

def Perspective(frame):
    src_int = [(int(x[0]), int(x[1])) for x in Source]
    cv2.line(frame, src_int[0], src_int[1], (0, 0, 255), 2)
    cv2.line(frame, src_int[1], src_int[3], (0, 0, 255), 2)
    cv2.line(frame, src_int[3], src_int[2], (0, 0, 255), 2)
    cv2.line(frame, src_int[2], src_int[0], (0, 0, 255), 2)
    Matrix = cv2.getPerspectiveTransform(Source, Destination)
    framePers = cv2.warpPerspective(frame,Matrix, (400,240))
    return framePers

def Threshold(framePers):
    frameGray = cv2.cvtColor(framePers, cv2.COLOR_RGB2GRAY)
    _, frameThresh = cv2.threshold(frameGray, 200, 230, cv2.THRESH_BINARY)
    frameEdge = cv2.Canny(frameGray, 900, 900)
    frameFinal = cv2.add(frameThresh, frameEdge)
    frameFinal = cv2.cvtColor(frameFinal, cv2.COLOR_GRAY2RGB)
    return frameFinal

def Histrogram(frameFinal):
    histrogramLane = []
    for i in range(400):
        ROILane = frameFinal[140:240, i:i+1]
        histrogramLane.append(np.sum(ROILane / 255))
    return histrogramLane

def LaneFinder(histrogramLane):
    global LeftLanePos, RightLanePos
    LeftLanePos = np.argmax(histrogramLane[:150])
    RightLanePos = np.argmax(histrogramLane[250:]) + 250
    
def LaneCenter():
    global laneCenter, Result
    laneCenter = (RightLanePos - LeftLanePos) // 2 + LeftLanePos
    Result = laneCenter - frameCenter

def LaneEndDetection(histrogramLane):
    # Detects if a lane end is reached
    lane_end_threshold = 11000  # Adjust as needed
    lane_end_value = np.sum(histrogramLane)
    return lane_end_value > lane_end_threshold

def estimate_distance_object(object_width_in_pixels):
    """
    Estimate distance to an object based on its width in pixels.
    """
    return (OBJECT_REAL_WIDTH * FOCAL_LENGTH) / object_width_in_pixels

def estimate_distance_stop(stop_width_in_pixels):
    """
    Estimate distance to an object based on its width in pixels.
    """
    return (STOP_REAL_WIDTH * FOCAL_LENGTH) / stop_width_in_pixels


def detect_stop_sign(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    stop_signs = stop_cascade.detectMultiScale(gray, 1.1, 5)
    for (x, y, w, h) in stop_signs:
        distance = estimate_distance_stop(w)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
        cv2.putText(frame, f"{distance:.2f} cm", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        if distance < 25:  # If the estimated distance is less than 10 cm
                 # Stop the vehicle
            return True  # Object detected and vehicle stopped
    return False  # No object detected within the stopping distance

def detect_objects_and_control_vehicle(frame):
    """
    Detect objects, estimate distance, and control the vehicle accordingly.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    objects = object_cascade.detectMultiScale(gray, 1.3, 5)
    for (x, y, w, h) in objects:
        distance = estimate_distance_object(w)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
        cv2.putText(frame, f"{distance:.2f} cm", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        if distance < 27:  # If the estimated distance is less than 10 cm
                 # Stop the vehicle
            return True  # Object detected and vehicle stopped
    return False  # No object detected within the stopping distance

def detect_traffic_signs(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    signs = traffic_sign_cascade.detectMultiScale(gray, 1.3, 5)
    for (x,y,w,h) in signs:
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
    return len(signs) > 0

def main():
        picam2.start()
        try:
            while True:
                start_time = time.time()
                frame = picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                framePers = Perspective(frame)
                frameFinal = Threshold(framePers)
                histrogramLane = Histrogram(frameFinal)
                LaneFinder(histrogramLane)
                LaneCenter()
                detect_accident()
                
                if detect_stop_sign(frame):
                    print("Stop sign detected")
                    stop()  # Stop the motors
                    time.sleep(2)  # Wait for 5 seconds
                    
                if detect_objects_and_control_vehicle(frame):
                    print("Object detected close. Vehicle stopped.")
                    stop()
                    time.sleep(4)  # Wait for a moment before next actions    
                
                if detect_traffic_signs(frame):
                    print("Traffic sign detected.")
                    stop()
                    time.sleep(10)  # Wait for a moment before next actions    
                    
                if LaneEndDetection(histrogramLane):
                   print("Lane end")
                   stop()
                   time.sleep(5)
                    
                if detect_accident():
                    print("Accident detected. Stopping the vehicle.")
                    stop()
                    break# Wait for a moment before next actions

                cv2.line(frameFinal, (LeftLanePos, 0), (LeftLanePos, 240), (0, 255, 0), 2)
                cv2.line(frameFinal, (RightLanePos, 0), (RightLanePos, 240), (0, 255, 0), 2)
                cv2.line(frameFinal, (laneCenter, 0), (laneCenter, 240), (0, 255, 0), 3)
                cv2.line(frameFinal, (frameCenter, 0), (frameCenter, 240), (255, 0, 0), 3)

                if Result > 20:
                    turn_right(70, 5)
                elif Result > 10:
                    turn_right(70, 4)
                elif Result < -20:
                    turn_left(70, 5)
                elif Result < -10:
                    turn_left(70, 4)
                else:
                    forward(50)
                    
                

            # Display and FPS calculation 
                display_text = f"Result = {Result}"
                cv2.putText(frame, display_text, (1, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow("Frame", frame)
                cv2.imshow("Perspective", framePers)
                cv2.imshow("Final", frameFinal)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                 break
                
                end_time = time.time()
                fps = 1 / (end_time - start_time)
                print(f"FPS = {fps:.2f}")
            
        finally:
                picam2.stop()
                cv2.destroyAllWindows()
                pwmL.stop()
                pwmR.stop()
                GPIO.cleanup()            

if __name__ =="__main__":
    main()

