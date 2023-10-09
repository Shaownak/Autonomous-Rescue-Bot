import datetime
from pymavlink import mavutil
import time
import math
import os
import serial
os.environ['MAVLINK20'] = ''


# Create the connection
master = mavutil.mavlink_connection('COM5')
arduino = serial.Serial('COM6', 9600)


# Wait for the heartbeat message to find the system ID
while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg.get_type() == 'HEARTBEAT':
        # print(f"System ID: {msg.get_srcSystem()}")
        print("ID: {}".format(msg.get_srcSystem()))

        break
    
    
    
# ================ Constant Values =====================
distance_threshold = 2

    
    
# ================ Get Current Heading =================

def heading():
    msg = master.recv_match(type='AHRS2', blocking=True)
    if msg:
        yaw = msg.yaw
        heading = math.degrees(yaw)
        # Adjust heading to range 0-360 degrees
        if heading < 0:
            heading += 360

        # Convert heading to integer degrees
        heading = int(heading)
        # print("Heading: {}".format(heading))
        
        return heading
        
        
        
# ============== Get Current Location ================

def position():
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        lat = msg.lat
        lon = msg.lon
        # print(f"lat: {lat} lon: {lon}")       
        
        return lat, lon
        
        

# ======= Calculate the distance between the current location and the destination =======

def distance(current_location, destination):
    dlat = destination[0] - current_location[0]
    dlong = destination[1] - current_location[1]
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

# Check if the UAV is within the distance threshold of the destination
def is_at_destination(current_location, destination):
    return distance(current_location, destination) <= distance_threshold



# ======== Calculate the IMU yaw heading from the current location to the destination =====

# Calculate the bearing from the current location to a destination point
def get_bearing(current_location, destination_location):
    off_x = destination_location[1] - current_location[1]
    off_y = destination_location[0] - current_location[0]
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing




# =========================== Control Functions ===================================================

tout = 0.3
an_threshold = 6


# Function for running the rover without headlock
def run_wo_headlock(heading_error, is_at_destination):

    if heading_error>-an_threshold and heading_error<an_threshold and not is_at_destination:
        # arduino.write(forward.encode("utf-8"))
        arduino.write(b'w')
        print("Going Forward")
        # time.sleep(tout)
        # arduino.write(b's')
    elif heading_error>=an_threshold and not is_at_destination:
        # arduino.write("d".encode("utf-8"))
        arduino.write(b'd')
        print("Going right")
        time.sleep(tout)
        arduino.write(b's')
    elif heading_error<=-an_threshold and not is_at_destination:
        arduino.write(b'a')
        print("Going left")
        time.sleep(tout)
        arduino.write(b's')
    elif is_at_destination:
        arduino.write(b's')
        print("Stop")
        # time.sleep(tout)
        # arduino.write(b's')

        time.sleep(1)

        # find_aruco()
        arduino.write(b'a')
        print("Going left")
        time.sleep(1)
        arduino.write(b's')
        
        
        
        
# =============================== Destination Location ===================================

# Define the destination location
destination_list = [(23.775035858154297, 90.40911865234375), (23.77506446838379, 90.40901947021484)]

# Set the initial destination as the first item in the list
current_destination_index = 0
destination_location = destination_list[current_destination_index]


        
        
        
# ============================== Main control loop ==========================================================

while True:

    # Get the current location and heading of the rover
    current_location = position()
    current_heading = heading()

    # Calculate the bearing to the destination
    desired_heading = get_bearing(current_location, destination_location)

    # Calculate the heading error and adjust the steering with the PID controller
    heading_error = desired_heading - current_heading 

    dis = distance(current_location, destination_location)

    isAtDestination = is_at_destination(current_location, destination_location)

    print(f"DH: {desired_heading} HR: {heading_error} CH: {current_heading} DIS: {dis}")
    print(f"LAT: {current_location[0]} LON: {current_location[1]}")
    print("------------------------------------------------------------------------------------------------------")



    if isAtDestination:
        print("Destination Reached")
        time.sleep(5)
        # Set the next destination as the current destination, if it exists
        if current_destination_index < len(destination_list) - 1:
            current_destination_index += 1
            destination_location = destination_list[current_destination_index]
        else:
            print("All destinations reached")
            break

    
    else:
        run_wo_headlock(heading_error, isAtDestination)



    print("======================================================================================================")
