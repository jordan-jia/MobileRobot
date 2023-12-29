import rospy
import threading
from std_msgs.msg import Int16MultiArray
import time
import RPi.GPIO as GPIO
import random

####### Define Parameter #######
state = "init"
status_duration = 0
walk_arround_time_limit = 3 # <-set it yourself
go_time_limit = 15 # <-set it yourself

# Touch sensor
sensor = [False, False, False]

# Speed info
ARotate = 8800 * 0.9 # <-set it yourself
find_ir_rotate_count = int(ARotate * 0.35) # <-set it yourself
find_light_back_count = 3000 # <-set it yourself
find_ir_back_count = 300 # <-set it yourself
find_ir_forward_count = 3000 # <-set it yourself
back_count = find_light_back_count 
rotate_speed = 10 # <-set it yourself
rotate_direction = 1
respeed = 4 # <-set it yourself
speed = 15 # <-set it yourself

# Light info
target_light = 0
target_threshold = 30 # <-set it yourself
max_light = 1024
max_degree = 0

# Subscribe data = [light, motorA, motorB]
subscribe_data_queue = []
subscribe_data = [0, 0, 0]

# Publish data
publish_queue = []
publish_queue2 = []

# IR data
beacon_select = 0
beacon_lower_bound = [0.18, 0.4]
beacon_upper_bound = [0.28, 0.5]
read_ir_times = 1200 # <-set it yourself
received_ir_data = 0.0
find_ir = False
not_find_ir_threshold = 2 # <-set it yourself

# Servo data
is_locked = False
not_catch_time = 0
not_catch_threshold = 7 # <-set it yourself

lock = threading.Lock()
find_lock = False


####### ROS function #######
# Subscriber callback function
def callback(dataset):
    global subscribe_data_queue
    subscribe_data_queue.append([int(dataset.data[0]), int(dataset.data[1]), int(dataset.data[2])])

# Subscribe data handeling loop
def subscribe_data_handeling_loop():
    print("=start subscribe_data_handeling_loop")
    global subscribe_data_queue, subscribe_data, max_light, max_degree
    while not rospy.is_shutdown():
        try:
            with lock:
                if subscribe_data_queue:
                    subscribe_data = min(subscribe_data_queue, key = lambda x: x[0])
                    subscribe_data_queue.clear()
                    if subscribe_data[0] < max_light:
                        max_light = subscribe_data[0]
                        max_degree = subscribe_data[1] + subscribe_data[2]
        except:
            print("subscribe_data_handeling_loop error")
        time.sleep(0.01)

# Publish data handeling_loop
def publish_data_handeling_loop():
    print("=start publish_data_handeling_loop")
    global publish_queue, publish_queue2
    pub = rospy.Publisher('control', Int16MultiArray, queue_size = 10)
    pub2 = rospy.Publisher('servo', Int16MultiArray, queue_size = 10)
    rate = rospy.Rate(100)  # 100Hz

    while not rospy.is_shutdown():
        if publish_queue:
            msg = Int16MultiArray()
            data = publish_queue.pop(0)
            msg.data.append(data['left'])
            msg.data.append(data['right'])
            pub.publish(msg)
            rate.sleep()
        elif publish_queue2:
            msg = Int16MultiArray()
            data = publish_queue2.pop(0)
            msg.data.append(data['servo'])
            pub2.publish(msg)
            rate.sleep()
        else:
            time.sleep(0.01)


####### State function #######
# Init state
def init():
    print("=start init")
    global state, status_duration, publish_queue, publish_queue2, is_locked, not_catch_time, target_light, max_light, max_degree, rotate_direction

    state = "init"
    status_duration = 0

    publish_queue = [{'left': 0, 'right': 0}]
    time.sleep(1)

    publish_queue2 = [{'servo': 0}]
    is_locked = False
    not_catch_time = 0

    target_light = 0
    max_light = 1024
    max_degree = 0
    rotate_direction = 1

# Rotate state
def rotate():
    print("=start rotate")
    global state, status_duration, publish_queue, max_light, rotate_direction
    publish_queue = [{'left': 0, 'right': 0}]
    time.sleep(0.5)
    if is_locked:
        publish_queue = [{'left': int(rotate_speed * rotate_direction * 0.7), 'right': - int(rotate_speed * rotate_direction * 0.7)}]
        rotate_direction *= -1
    else:
        publish_queue = [{'left': rotate_speed, 'right': - rotate_speed}]
    time.sleep(0.2)
    max_light = 1024
    state = "rotate"
    status_duration = 0

# Go state    
def go():
    print("=start go")
    global state, status_duration, publish_queue
    publish_queue = [{'left': speed, 'right': speed}]
    time.sleep(0.2)
    state = "go"
    status_duration = 0

# Walk arround state    
def walk_arround():
    print("=start walk_arround")
    global state, status_duration, publish_queue
    if random.randint(1, 100) % 2:
        publish_queue = [{'left': int(speed * 0.7), 'right': speed}]
    else:
        publish_queue = [{'left': speed, 'right': int(speed * 0.7)}]
    time.sleep(0.2)
    state = "walk_arround"
    status_duration = 0

# Back state
def back():
    print("=start back")
    global state, status_duration, publish_queue, back_count
    if is_locked:
        sensor_tmp = sensor
        back_count = find_ir_forward_count

        # Backward
        publish_queue = [{'left': - speed, 'right': - speed}]
        time.sleep(0.2)
        while (subscribe_data[1] + subscribe_data[2]) <= find_ir_back_count:
            time.sleep(0.01)
        print(sensor_tmp)
        # Rotate
        if sensor_tmp[2]: # Left side
            publish_queue = [{'left': speed, 'right': - speed}]
        else:#if sensor_tmp[1]: # Right side
            publish_queue = [{'left': -speed, 'right': speed}]
        time.sleep(1)
        while (subscribe_data[1] + subscribe_data[2]) <= find_ir_rotate_count:
            time.sleep(0.01)

        # Forward
        publish_queue = [{'left': speed, 'right': speed}]
        time.sleep(0.2)
    else:
        back_count = find_light_back_count
        publish_queue = [{'left': - speed, 'right': - speed}]
        time.sleep(0.2)

    state = "back"
    status_duration = 0

# Found state
def found():
    print("=start found")
    global state, status_duration, publish_queue
    publish_queue = [{'left': - respeed, 'right': respeed}]
    time.sleep(0.5)
    publish_queue = [{'left': 0, 'right': 0}]
    time.sleep(0.2)
    state = "found"
    status_duration = 0

# Lock servo state
def lock_servo():
    print("=start lock_servo") 
    global state, status_duration, publish_queue, publish_queue2, is_locked
    publish_queue = [{'left': 0, 'right': 0}]
    time.sleep(1)
    # publish_queue2 = [{'servo': 1}]
    # time.sleep(2)
    is_locked = True
    state = "lock_servo"
    status_duration = 0

# Unlock servo state
def unlock_servo():
    print("=start unlock_servo") 
    global state, status_duration, publish_queue, publish_queue2, is_locked
    publish_queue = [{'left': 0, 'right': 0}]
    time.sleep(1)
    # publish_queue2 = [{'servo': 0}]
    # time.sleep(2)
    is_locked = False
    state = "unlock_servo"
    status_duration = 0

# Finish state
def finish():
    print("=start finish")
    global state, status_duration, publish_queue, publish_queue2, is_locked
    publish_queue = [{'left': 0, 'right': 0}]
    time.sleep(1)
    publish_queue2 = [{'servo': 0}]
    time.sleep(2)
    publish_queue = [{'left': - speed, 'right': - speed}]
    while (subscribe_data[1] + subscribe_data[2]) < back_count:
        time.sleep(0.01)
    publish_queue = [{'left': 0, 'right': 0}]
    is_locked = False
    state = "finish"
    status_duration = 0


####### Loop function#######
# Read GPIO
def read_gpio_loop():
    print("=start read_gpio_loop")
    global sensor, received_ir_data, find_ir
    # Init GPIO
    GPIO.setmode(GPIO.BOARD)
    # Set GPIO pin mode
    pins = [11, 12, 13, 15]
    for pin in pins:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    ir_count0 = 0.0
    ir_count1 = 0.0
    not_find_ir_counter = 0
    while True:
        # Save GPIO0 ~ GPIO2 to sensor[], GPIO3 to ir_count
        for idx, pin in enumerate(pins):
            if idx < 3:
                sensor[idx] = (GPIO.input(pin) == GPIO.LOW)
            else:
                if GPIO.input(pin) == GPIO.LOW:
                    ir_count0 += 1
                else:
                    ir_count1 += 1
        # Calculate ir ratio
        if (ir_count0 + ir_count1) >= read_ir_times:
            received_ir_data = ir_count0 / (ir_count0 + ir_count1)
            if (received_ir_data >= beacon_lower_bound[beacon_select]) and (received_ir_data <= beacon_upper_bound[beacon_select]):
                not_find_ir_counter = 0
                find_ir = True
            else:
                not_find_ir_counter += 1
                if not_find_ir_counter >= not_find_ir_threshold:
                    find_ir = False
            ir_count0 = ir_count1 = 0.0
        time.sleep(0.0001) 

# Auto loop
def auto_loop():
    print("=start auto_loop")
    global target_light, not_catch_time, status_duration
    while not rospy.is_shutdown():
        time.sleep(0.01)
        if state not in ["init", "finish"]:
            status_duration += 0.01
            if is_locked: # Locked, need to find IR
                # Record sensor not catch time
                if sensor[0]:
                    not_catch_time = 0
                else:
                    not_catch_time += 0.01

                if not_catch_time >= not_catch_threshold: # If long time no catch
                    not_catch_time = 0
                    unlock_servo()
                elif state == "lock_servo":
                    rotate()
                elif state == "go":
                    if sensor[1] or sensor[2]:
                        if not first_touch:
                            first_touch = time.time()
                        if sensor[1] and sensor[2]:
                            if find_ir:
                                finish()
                            else:
                                back()
                        else:
                            if find_ir :
                                if time.time() - first_touch > 5:
                                    finish()
                            else:
                                back()
                    elif not find_ir:
                        first_touch = 0
                        rotate()
                    else:
                        first_touch = 0
                elif state == "walk_arround":
                    if find_ir:
                        go()
                    elif sensor[1] or sensor[2]: # Touch the wall
                        back()
                    elif status_duration > walk_arround_time_limit:
                        if random.randint(1, 100) % 2:
                            walk_arround()
                        else:
                            rotate()
                elif state == "rotate":
                    if find_ir:
                        go()
                    # elif sensor[1] or sensor[2]: # Touch the wall
                    #     back()
                    elif (ARotate < subscribe_data[1] + subscribe_data[2]): # Can't find IR
                        walk_arround()
                elif state == "back":
                    if (subscribe_data[1] + subscribe_data[2]) >= back_count: # Complete backward
                        rotate()
            else: # Not locked, need to find light
                if sensor[0]: # Catch light ball
                    target_light = 0
                    lock_servo()
                elif state == "unlock_servo":
                    rotate()
                elif state == "go":
                    if sensor[1] or sensor[2]: # Touch the wall
                        back()
                    elif status_duration > go_time_limit:
                        rotate()
                elif state == "rotate":
                    if sensor[1] or sensor[2]: # Touch the wall
                        target_light = 0
                        back()
                    elif target_light and (abs(subscribe_data[0] - target_light) < target_threshold): # Find target light
                        print("Find:{}/{}".format(target_light, subscribe_data[0]))
                        target_light = 0
                        found()
                    elif (ARotate < subscribe_data[1] + subscribe_data[2]): # Complete 1 cycle rotating
                        print("Max_light:", max_light, " max_degree:", max_degree, " rotate_degree:", max_degree / ARotate * 360)
                        target_light = max_light
                        rotate()
                elif state == "back":
                    if (subscribe_data[1] + subscribe_data[2]) >= back_count: # Complete backward
                        rotate()
                elif state == "found": # Facing light ball
                    go()


####### Main function#######
def main():
    global beacon_select
    rospy.init_node('cp4', anonymous=True)
    
    # Start loop function as a thread
    threading.Thread(target = publish_data_handeling_loop).start()
    threading.Thread(target = subscribe_data_handeling_loop).start()
    threading.Thread(target = read_gpio_loop).start()
    threading.Thread(target = auto_loop).start()

    # Subscriber
    rospy.Subscriber("output", Int16MultiArray, callback)

    # Get user input
    while not rospy.is_shutdown():
        try:
            print("1. Go")
            print("2. Back")
            print("3. Rotate")
            print("4. Print sensor state")
            print("5. Set target becon")
            print("Else. Initialize")
            count = int(input("Action:"))
            if count == 1:
                go()
            elif count == 2:
                back()
            elif count == 3:
                rotate()
            elif count == 4:
                print(state)
                print(subscribe_data)
                print(sensor)
                print(beacon_select + 1)
                print(received_ir_data)
            elif count == 5:
                user_input = input("Enter your target beacon(1 or 2):")
                if user_input not in ['1', '2']:
                    print("Invalid beacon")
                else:
                    beacon_select = int(user_input) - 1
            else:
                init()
        except ValueError:
            print("Invalid input. Please enter a number.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass