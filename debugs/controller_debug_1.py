from controller import Robot , Motor, Camera, Lidar, Emitter, Receiver
import cv2 as cv
import numpy as np
import math
import struct

timestep =32
max_velocity = 6.28
map_position = [0,0]
robot_position = [0,0]
visited_tiles = dict()
counter = 0
initial_map_position = [0,0]
rounded_position = None
compass_value = 0

number_of_visits = 0

robot = Robot()


# Wheels
wheel1 = robot.getDevice('wheel1 motor')
wheel2 = robot.getDevice('wheel2 motor')
wheel1.setPosition(float('inf'))
wheel2.setPosition(float('inf'))

# Defining
gps = robot.getDevice('gps')
compass = robot.getDevice('compass')
camera_right = robot.getDevice('right_camera')
camera_left = robot.getDevice('left_camera')
color_sensor = robot.getDevice('colour_sensor')
lidar = robot.getDevice('lidar')
emitter = robot.getDevice('emitter')
receiver = robot.getDevice('receiver')

#Enabling sensors
gps.enable(timestep)
compass.enable(timestep)
camera_right.enable(timestep)
camera_left.enable(timestep)
color_sensor.enable(timestep)
lidar.enable(timestep)
lidar.enablePointCloud()
lidar_values = []
gps_readings = [0, 0, 0]
compass_value = 0
color_sensor_values = [0, 0, 0]
receiver.enable(timestep)

two_d_array = []
def get_gps(initial_map_position, robot_position, visited_tiles, imgL, camera):
    if not initial_map_position[0] and not initial_map_position[1]: 
        initial_map_position[0] = gps.getValues()[0] * 100  # Fixed = instead of -
        initial_map_position[1] = gps.getValues()[2] * 100  # Fixed = instead of -
        robot_position = [0, 0]
    else:
        robot_position[0] = gps.getValues()[0] * 100 - initial_map_position[0]
        robot_position[1] = gps.getValues()[2] * 100 - initial_map_position[1]
    
    rounded_position = round_to_12(robot_position[0]), round_to_12(robot_position[1])
    _, result = full_detection(imgL, camera)
        
    if rounded_position not in visited_tiles:
        visited_tiles[rounded_position] = False  # Initialize with False
        
    if _ == True:
        if visited_tiles[rounded_position] == False:
            if result in ['C', 'O', 'P', 'F']:
                report(result)
            elif result in ['H', 'S', 'U']:
                report(result)
                print("Victim Found")
            visited_tiles[rounded_position] = True  # Fixed [] instead of ()
    return rounded_position

def colour_value(image):
    r = color_sensor.imageGetRed(image,1, 0, 0)
    g = color_sensor.imageGetGreen(image,1, 0, 0)
    b = color_sensor.imageGetBlue(image,1, 0, 0)

def round_to_12(value):
    """Round to the nearest multiple of 12 (for grid alignment)"""
    return round(value / 12) * 12

def report(victimType):
    print("Victim Type: ", victimType)
    victimtype = bytes(victimType, 'utf-8')
    position_X = int(gps.getValues()[0]*100)
    position_Y = int(gps.getValues()[2]*100)
    
    stop(2000)
    message = struct.pack('i i c', position_X, position_Y, victimtype)
    emitter.send(message)
    robot.step(timestep)
    

def stop(time):
    wheel1.setVelocity(0)
    wheel2.setVelocity(0)
    delay(time)
    

def rays_array(start,end):
    for col in range(start, end):
        element = two_d_array[2][col]
        return element
    

def convert_to_2d_array(original_array, num_layers, values_per_layer):
    two_d_array = []
    for layer_index in range(num_layers):
        start_index = layer_index * values_per_layer
        end_index = start_index + values_per_layer
        layer_values = original_array[start_index:end_index]
        two_d_array.append(layer_values)
    return two_d_array

def move_robot():
    wheel1.setVelocity(max_velocity)
    wheel2.setVelocity(max_velocity)
    
def turn_right():
    wheel1.setVelocity(max_velocity)
    wheel2.setVelocity(-max_velocity)
    
def turn_left():
    wheel1.setVelocity(max_velocity/4)
    wheel2.setVelocity(max_velocity)
    
def move_backwards():
    wheel1.setVelocity(-max_velocity)
    wheel2.setVelocity(-(max_velocity / 4))
    delay(100)
    
def spin():
    wheel1.setVelocity(-max_velocity)
    wheel2.setVelocity(max_velocity)
    
def delay(ms):
    initTime = robot.getTime()
    while robot.step(timestep) != -1:
        if (robot.getTime() - initTime) * 1000 > ms:
            break
        
def get_mean_color(img):
    img = np.array(img)
    mean_color = np.mean(img, axis=(0, 1))
    return mean_color[:3]

def sign_is_mostly_blue(img):
    lower = np.array([120,100,20])
    upper = np.array([160,145,80])
    mean_color = get_mean_color(img)
    if np.all(mean_color >= lower) and np.all(mean_color <= upper) and mean_color[0] > mean_color[1] and mean_color[1] > mean_color[2]:
        return True
    

def contour_without_blue(img):
    image_dimensions = img.shape
    
    upper_left = img[0:int(image_dimensions[0]/2), 0:int(image_dimensions[1]/2)]
    upper_right = img[0:int(image_dimensions[0]/2), int(image_dimensions[1]/2):image_dimensions[1]]
    lower_left = img[int(image_dimensions[0]/2):image_dimensions[0], 0:int(image_dimensions[1]/2)]
    lower_right = img[int(image_dimensions[0]/2):image_dimensions[0], int(image_dimensions[1]/2):image_dimensions[1]]
    
    quadrants = [upper_left, upper_right, lower_left, lower_right]
    
    for quadrant in quadrants:
        if sign_is_mostly_blue(quadrant):
            return False
    
    return True

def get_image_contours(image,camera):
    img = np.array(np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
    hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    
    lower1 = np.array([0, 0, 110])
    upper1 = np.array([180,40,255])
    
    mask1 = cv.inRange(hsv_image, lower1, upper1)
    
    lower2 = np.array([0, 50, 50])
    upper2 = np.array([50,255,255])
    
    mask2 = cv.inRange(hsv_image, lower2, upper2)
    
    lower3 = np.array([125, 0, 0])
    upper3 = np.array([100, 255, 255])
    
    mask3 = cv.inRange(hsv_image, lower3, upper3)
    
    mask_red = cv.bitwise_or(mask3, mask2)
    mask = cv.bitwise_or(mask1, mask_red)
    
    cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    return cnts, mask

def sign_detection(img, camera):
    img = np.array(np.frombuffer(img, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
    cnts, binary_image = get_image_contours(img, camera)
    
    if len(cnts) > 0:
        sign = None
        sign_colored = None
        image_detected = False
        
        image_detected = False
        for c in cnts:
            [x, y, w, h] = cv.boundingRect(c)
            sign_colored = img[y:y+h, x:x+w]
            if w > 18 and h > 18 and (5 < x < 20) and y < 25 and contour_without_blue(sign_colored):
                img_copy = img.copy()
                cv.rectangle(img_copy, (x, y), (x + w, y + h), (36, 255, 12), 2)
                sign = binary_image[y:y+h, x:x+w]
                image_detected = True
                break
        return sign, sign_colored, image_detected
    return None, None, False

def detect_hazards(sign_colored):
    sign_colored = cv.cvtColor(sign_colored, cv.COLOR_BGR2GRAY)
    sign_type = 'N'
    h, w, = sign_colored.shape
    half = h // 2
    bottom = sign_colored[half:, :]
    
    lower_orange = np.array([192,142,0])
    upper_orange = np.array([204,190,20])
    orange_mask = cv.inRange(bottom, lower_orange, upper_orange)
    pixels = cv.countNonZero(orange_mask)
    if pixels > 10:
        sign_type = 'O'
    lower_red = np.array([185,0,0])
    upper_red = np.array([255,100,110])
    red_mask = cv.inRange(bottom, lower_red, upper_red)
    pixels = cv.countNonZero(red_mask)
    if pixels > 25:
        sign_type = 'F'
        
    return sign_type

def letter_detection(sign):
    letter = cv.bitwise_not(sign)
    h, w = letter.shape
    for x in range(0, h):
        for y in range(0, w):
            pixel = letter[x,y]
            if pixel <20:
                break
            else:
                letter[x,y] = (0)
        for y_inversed in range(w - 1, 0, -1):
            pixel = letter[x, y_inversed]
            if pixel < 20:
                break
            else:
                letter[x, y_inversed] = (0)
    cnts = cv.findContours(letter, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) ==2 else cnts[1]
    
    for c in cnts:
        (x, y, w, h) = cv.boundingRect(c)
        letter = letter[y:y + h, x:x + w]
        
    h, w = letter.shape
    letter_type = 'N'
    third = h // 3
    top = letter[:third, :]
    middle = letter[third:third*2, :]
    bottom = letter[third*2:, :]
    
    cnts = cv.findContours(top, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) ==2 else cnts[1]
    c1 = (len(cnts))
    
    cnts = cv.findContours(middle, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) ==2 else cnts[1]
    c2 = (len(cnts))
    
    cnts = cv.findContours(bottom, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) ==2 else cnts[1]
    c3 = (len(cnts))
    if c1 == 1 and c3 == 1:
        letter_type = 'S'
    elif c1 == 2 and c2 == 1 and c3 == 2:
        letter_type = 'H'
    elif c1 == 2 and c2 == 2 and c3 == 1:
        letter_type = 'U'
        
    return letter_type, bottom

def hazard_sign_detection(sign):
    sign_type = 'N'
    h, w = sign.shape
    bottom = sign[int(h * 0.5):, int(w * 0.3) : int(w * 0.85)]
    white_pixel = cv.countNonZero(bottom)
    black_pixel = bottom.size - white_pixel
    
    if black_pixel > white_pixel:
        sign_type = 'C'
    else:
        sign_type = 'P'
    return sign_type

def full_detection(img, camera):
    sign, sign_colored, image_detected = sign_detection(img, camera)
    
    if image_detected:
        sign_type = detect_hazards(sign_colored)
        if sign_type != 'N':
            return True, sign_type
        letter_type, bottom = letter_detection(sign)
        if letter_type != 'N':
            return True, letter_type
        sign_type_white = hazard_sign_detection(sign)
        if sign_type_white != 'N':
            return True, sign_type_white
    return False, 'N'

while robot.step(timestep) != -1:
    c_image = color_sensor.getImage()
    colour_value(c_image)
    
    r = color_sensor.imageGetRed(c_image, 1, 0, 0)
    g = color_sensor.imageGetGreen(c_image, 1, 0, 0)
    b = color_sensor.imageGetBlue(c_image, 1, 0, 0)
    
    imgL = camera_left.getImage()
    
    my_list = lidar.getRangeImage()
    my_list = list(my_list)
    two_d_array = convert_to_2d_array(my_list, 4, 512)
    
    left_ray = rays_array(384,444)
    top_left = rays_array(445,475)
    front_rayL = rays_array(476,512)
    front_rayR = rays_array(0,40)
    front_rayRL = rays_array(0, 128)
    
    left_wall = left_ray < 0.07
    corner_case = top_left < 0.07
    front_wall = front_rayL < 0.055 or front_rayR < 0.055
    front_wallRL = front_rayL < 0.055 or front_rayRL < 0.055
    
    stuck = left_wall or corner_case and front_wallRL
    visited_tiles_key = get_gps(initial_map_position, robot_position, visited_tiles, imgL, camera_left)
    
    if 120>r>40 and 120>g>40 and 120>b>40:
        stop(1000)
        turn_right()
        delay(500)
        move_robot()
    else:
        if corner_case:
            #print("came too close to a corner, turn right")
            turn_right()
        elif front_wall:
            #print("front wall detected, turn right")
            turn_right()
        elif left_wall:
            #print("left wall detected, turn left")
            move_robot()
        else:
            turn_left()
