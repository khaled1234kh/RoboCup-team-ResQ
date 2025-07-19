from controller import Robot, Motor, Camera, Lidar, Emitter, Receiver
import cv2 as cv
import numpy as np
import math
import struct

# Constants
timestep = 32
max_velocity = 6.28

# Global Variables
map_position = [0, 0]
robot_position = [0, 0]
visited_tiles = dict()
initial_map_position = [0, 0]
rounded_position = None
compass_value = 0
number_of_visits = 0
last_position = [0, 0]
position_timer = 0
stuck_threshold = 0.05  
stuck_time_threshold = 5000  
position_checkpoints = []  
checkpoint_radius = 0.2  
min_checkpoints_for_detection = 3  
checkpoint_sample_interval = 5000 
checkpoint_timer = 0
last_checkpoint_time = 0
min_time_between_checkpoints = 10000  

# Permanent start checkpoint variable
start_checkpoint = None  # Will store initial position permanently

# Initialize Robot
robot = Robot()

# Wheels configuration
wheel1 = robot.getDevice('wheel1 motor')
wheel2 = robot.getDevice('wheel2 motor')
wheel1.setPosition(float('inf'))
wheel2.setPosition(float('inf'))

# Sensors
gps = robot.getDevice('gps')
compass = robot.getDevice('compass')
camera_right = robot.getDevice('right_camera')
camera_left = robot.getDevice('left_camera')
color_sensor = robot.getDevice('colour_sensor')
lidar = robot.getDevice('lidar')
emitter = robot.getDevice('emitter')
receiver = robot.getDevice('receiver')

# Enable Sensors
gps.enable(timestep)
compass.enable(timestep)
camera_right.enable(timestep)
camera_left.enable(timestep)
color_sensor.enable(timestep)
lidar.enable(timestep)
lidar.enablePointCloud()
receiver.enable(timestep)

# Sensor Values initialization
lidar_values = []
gps_readings = [0, 0, 0]
color_sensor_values = [0, 0, 0]
two_d_array = []

# Supporting functions
def delay(ms):
    init_time = robot.getTime()
    while robot.step(timestep) != -1:
        if (robot.getTime() - init_time) * 1000 > ms:
            break

def round_to_12(value):
    return round(value / 12) * 12

# Main Robot Functions
def move_robot():
    wheel1.setVelocity(max_velocity)
    wheel2.setVelocity(max_velocity)

def turn_right():
    wheel1.setVelocity(max_velocity)
    wheel2.setVelocity(-max_velocity)

def turn_left():
    wheel1.setVelocity(max_velocity / 4)
    wheel2.setVelocity(max_velocity)

def move_backwards():
    wheel1.setVelocity(-max_velocity)
    wheel2.setVelocity(-(max_velocity / 4))
    delay(100)

def stop(time):
    wheel1.setVelocity(0)
    wheel2.setVelocity(0)
    delay(time)

def spin():
    wheel1.setVelocity(-max_velocity)
    wheel2.setVelocity(max_velocity)

# Lidar Functions
def rays_array(start, end):
    for col in range(start, end):
        return two_d_array[2][col]

def convert_to_2d_array(original_array, num_layers, values_per_layer):
    return [
        original_array[i * values_per_layer: (i + 1) * values_per_layer]
        for i in range(num_layers)
    ]

# Color Sensor 
def colour_value(image):
    r = color_sensor.imageGetRed(image, 1, 0, 0)
    g = color_sensor.imageGetGreen(image, 1, 0, 0)
    b = color_sensor.imageGetBlue(image, 1, 0, 0)
    return r, g, b

# Reporting
def report(victimType):
    print("Victim Type:", victimType)
    victimtype = bytes(victimType, 'utf-8')
    position_X = int(gps.getValues()[0] * 100)
    position_Y = int(gps.getValues()[2] * 100)

    stop(2000)
    message = struct.pack('i i c', position_X, position_Y, victimtype)
    emitter.send(message)
    robot.step(timestep)

# GPS & Map Logic
def get_gps(initial_map_position, robot_position, visited_tiles, imgL, camera):
    if not initial_map_position[0] and not initial_map_position[1]:
        initial_map_position[0] = gps.getValues()[0] * 100
        initial_map_position[1] = gps.getValues()[2] * 100
        robot_position = [0, 0]
    else:
        robot_position[0] = gps.getValues()[0] * 100 - initial_map_position[0]
        robot_position[1] = gps.getValues()[2] * 100 - initial_map_position[1]

    rounded_position = round_to_12(robot_position[0]), round_to_12(robot_position[1])
    detected, result = full_detection(imgL, camera)

    if rounded_position not in visited_tiles:
        visited_tiles[rounded_position] = False

    if detected and not visited_tiles[rounded_position]:
        if result in ['C', 'O', 'P', 'F']:
            report(result)
        elif result in ['H', 'S', 'U']:
            report(result)
            print("Victim Found")
        visited_tiles[rounded_position] = True

    return rounded_position

# Vision Processing
def get_mean_color(img):
    img = np.array(img)
    return np.mean(img, axis=(0, 1))[:3]

def sign_is_mostly_blue(img):
    lower = np.array([120, 100, 20])
    upper = np.array([160, 145, 80])
    mean_color = get_mean_color(img)
    return np.all(mean_color >= lower) and np.all(mean_color <= upper) and mean_color[0] > mean_color[1] > mean_color[2]

def contour_without_blue(img):
    h, w = img.shape[:2]
    quadrants = [
        img[:h//2, :w//2], img[:h//2, w//2:],
        img[h//2:, :w//2], img[h//2:, w//2:]
    ]
    return not any(sign_is_mostly_blue(q) for q in quadrants)

def get_image_contours(image, camera):
    img = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    mask1 = cv.inRange(hsv, np.array([0, 0, 110]), np.array([180, 40, 255]))
    mask2 = cv.inRange(hsv, np.array([0, 50, 50]), np.array([50, 255, 255]))
    mask3 = cv.inRange(hsv, np.array([125, 0, 0]), np.array([100, 255, 255]))
    mask = cv.bitwise_or(mask1, cv.bitwise_or(mask2, mask3))

    cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    return cnts, mask

def sign_detection(img, camera):
    img = np.frombuffer(img, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    cnts, binary = get_image_contours(img, camera)

    if cnts:
        for c in cnts:
            x, y, w, h = cv.boundingRect(c)
            if w > 18 and h > 18 and (5 < x < 20) and y < 25:
                region = img[y:y+h, x:x+w]
                if contour_without_blue(region):
                    return binary[y:y+h, x:x+w], region, True
    return None, None, False

def detect_hazards(sign):
    hsv = cv.cvtColor(sign[sign.shape[0] // 2:, :], cv.COLOR_BGR2HSV)
    if cv.countNonZero(cv.inRange(hsv, (10, 100, 100), (25, 255, 255))) > 10:
        return 'O'
    red1 = cv.inRange(hsv, (0, 100, 100), (10, 255, 255))
    red2 = cv.inRange(hsv, (160, 100, 100), (180, 255, 255))
    if cv.countNonZero(cv.bitwise_or(red1, red2)) > 25:
        return 'F'
    return 'N'

def letter_detection(sign):
    if sign is None:
        return 'N', None
        
    # Convert to grayscale
    if len(sign.shape) > 2:
        gray = cv.cvtColor(sign, cv.COLOR_BGR2GRAY)
    else:
        gray = sign
    
    # binary image
    _, binary = cv.threshold(gray, 127, 255, cv.THRESH_BINARY)
    letter = cv.bitwise_not(binary)
    h, w = letter.shape

    for x in range(h):
        for y in range(w):
            if letter[x, y] < 20:
                break
            letter[x, y] = 0
        for y in range(w - 1, 0, -1):
            if letter[x, y] < 20:
                break
            letter[x, y] = 0

    cnts = cv.findContours(letter, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0]
    for c in cnts:
        x, y, w, h = cv.boundingRect(c)
        letter = letter[y:y+h, x:x+w]

    thirds = np.array_split(letter, 3, axis=0)
    cnts_top = len(cv.findContours(thirds[0], cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0])
    cnts_mid = len(cv.findContours(thirds[1], cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0])
    cnts_bot = len(cv.findContours(thirds[2], cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0])

    if cnts_top == 1 and cnts_bot == 1:
        return 'S', thirds[2]
    elif cnts_top == 2 and cnts_mid == 1 and cnts_bot == 2:
        return 'H', thirds[2]
    elif cnts_top == 2 and cnts_mid == 2 and cnts_bot == 1:
        return 'U', thirds[2]
    return 'N', thirds[2]

def hazard_sign_detection(sign):
    if sign is None:
        return 'N'
        
    # grayscale
    if len(sign.shape) > 2:
        gray = cv.cvtColor(sign, cv.COLOR_BGR2GRAY)
    else:
        gray = sign
    
    # binary image
    _, binary = cv.threshold(gray, 127, 255, cv.THRESH_BINARY)
    
    # bottom part
    bottom = binary[int(binary.shape[0] * 0.5):, 
                   int(binary.shape[1] * 0.3):int(binary.shape[1] * 0.85)]
    
    white_pixel = cv.countNonZero(bottom)
    black_pixel = bottom.size - white_pixel
    return 'C' if black_pixel > white_pixel else 'P'

def full_detection(img, camera):
    sign, colored, detected = sign_detection(img, camera)
    if detected:
        # Convert sign to grayscale
        if sign is not None and len(sign.shape) > 2:
            sign_gray = cv.cvtColor(sign, cv.COLOR_BGR2GRAY)
        else:
            sign_gray = sign
            
        checks = [
            (detect_hazards, colored),
            (letter_detection, sign_gray),
            (hazard_sign_detection, sign_gray)
        ]
        
        for check_func, check_img in checks:
            if check_img is None:
                continue
            result = check_func(check_img)
            if isinstance(result, tuple): 
                detected, result = True, result[0]
            if result != 'N':
                return True, result
    return False, 'N'

# Main Loop
while robot.step(timestep) != -1:
    try:
        current_time = robot.getTime() * 1000 
        current_pos = [gps.getValues()[0], gps.getValues()[2]]
        
        # Set the start checkpoint only once at the beginning
        if start_checkpoint is None:
            start_checkpoint = current_pos.copy()  # Store a copy of the initial position
            print(f"Start checkpoint set at: {start_checkpoint}")
        
        # Floating wall detection checkpoints
        checkpoint_timer += timestep
        if checkpoint_timer >= checkpoint_sample_interval:
            checkpoint_timer = 0
            
            if current_time - last_checkpoint_time > min_time_between_checkpoints:
                
                near_existing = False
                for checkpoint in position_checkpoints:
                    distance = math.sqrt(
                        (current_pos[0] - checkpoint[0])**2 + 
                        (current_pos[1] - checkpoint[1])**2
                    )
                    if distance < checkpoint_radius:
                        checkpoint[2] += 1  # Increment pass counter
                        near_existing = True
                        print("Revisited checkpoint (total passes: {})".format(checkpoint[2]))
                        last_checkpoint_time = current_time
                        break
                
                # Add new checkpoint
                if not near_existing and current_time - last_checkpoint_time > min_time_between_checkpoints:
                    position_checkpoints.append([current_pos[0], current_pos[1], 1])
                    print("New checkpoint added at {} (total checkpoints: {})".format(current_pos, len(position_checkpoints)))
                    last_checkpoint_time = current_time
                
                # Check for floating wall pattern
                for checkpoint in position_checkpoints:
                    if checkpoint[2] >= min_checkpoints_for_detection:
                        print("Floating wall confirmed - executing escape")
                        move_backwards()
                        delay(1500)  
                        turn_right()
                        move_robot()
                        
                        position_checkpoints = []
                        last_checkpoint_time = current_time
                        continue 
                
                if len(position_checkpoints) > 5:
                    position_checkpoints.pop(0)
       
        # Stuck detection
        position_changed = (abs(current_pos[0] - last_position[0]) > stuck_threshold or 
                          abs(current_pos[1] - last_position[1]) > stuck_threshold)
        
        if position_changed:
            position_timer = 0
            last_position = current_pos
        else:
            position_timer += timestep

        # Get sensor readings
        c_image = color_sensor.getImage()
        r, g, b = colour_value(c_image)
        imgL = camera_left.getImage()
        imgR = camera_right.getImage()
        my_list = list(lidar.getRangeImage())
        two_d_array = convert_to_2d_array(my_list, 4, 512)

        # Check if stuck
        if position_timer >= stuck_time_threshold and not (120 > r > 40 and 120 > g > 40 and 120 > b > 40):
            print("Robot appears stuck - executing recovery maneuver")
            move_backwards()
            delay(500)
            turn_right()
            delay(1000)
            position_timer = 0
            last_position = [gps.getValues()[0], gps.getValues()[2]]
            position_checkpoints = [] 
            checkpoint_timer = 0
            continue 

        # Wall following logic
        left_wall = rays_array(384, 444) < 0.07
        corner_case = rays_array(445, 475) < 0.07
        front_wall = rays_array(476, 512) < 0.055 or rays_array(0, 40) < 0.055
        front_wallRL = rays_array(476, 512) < 0.055 or rays_array(0, 128) < 0.055

        visited_key = get_gps(initial_map_position, robot_position, visited_tiles, imgL, camera_left)
        visited_key = get_gps(initial_map_position, robot_position, visited_tiles, imgR, camera_left)

        if 120 > r > 40 and 120 > g > 40 and 120 > b > 40:
         turn_right()
         delay(500)
         move_robot()
        else:

         if corner_case:
            
            turn_right()
         elif front_wall:
            
            turn_right()
         elif left_wall:
            
            move_robot()
         else:
            
            turn_left()


    except Exception as e:
        print("Error in main loop: {}".format(str(e)))
        stop(1000)
        continue
