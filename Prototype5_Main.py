#!/usr/bin/env python3 

#---------------------------------------------------------------------------------------------------------------

# Test: Open and close claws using Sound
# Test: Collect data
# Test: Follow IR Beacon
# Test: Remote Control

#----------------------------------------------------------------------------------------------------------------

from ev3dev2.sensor.lego import TouchSensor, ColorSensor, GyroSensor, UltrasonicSensor, InfraredSensor
from ev3dev2.port import LegoPort
from ev3dev2.sensor import Sensor
from ev3dev2.sound import Sound
from ev3dev2.sensor.lego import SoundSensor
from sys import stderr
from time import sleep
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.motor import MoveTank, MoveSteering, MediumMotor, OUTPUT_A, OUTPUT_D, OUTPUT_B, OUTPUT_C
from random import randint
from ev3dev2.button import Button
from ev3dev2.led import Leds
from datetime import datetime
from threading import Thread
import csv
import random

# port 1 = Barometer sensor
# port 2 = Ultrasonic sensor, Infrared Sensor and Gyro Sensor via EV3 Port splitter
# port 3 = Thermometer
# port 4 = Sound Sensor

# port A = Medium Motor (Claw)
# port B = Large Motor
# port C = Large Motor
# port D = Medium Motor (Lift)

tank_pair = MoveTank(OUTPUT_B,OUTPUT_C)
steer_pair = MoveSteering(OUTPUT_B,OUTPUT_C)
MediumMotor_Claw = MediumMotor(OUTPUT_A)
MediumMotor_lift = MediumMotor(OUTPUT_D)

sound = Sound()
leds = Leds()
btn = Button()

# Mindstorms EV3-Sensor-Mux connected to EV3 Ultrasonic Sensor, Infrared Sensor and Gyro Sensor --------------------

us_port = LegoPort(address='ev3-ports:in2:i2c80:mux1') 
ir_port = LegoPort(address='ev3-ports:in2:i2c81:mux2') 
gyro_port = LegoPort(address='ev3-ports:in2:i2c82:mux3')

us_port.mode = 'uart'
ir_port.mode = 'uart'
gyro_port.mode = 'uart'

us_port.set_device = 'lego-ev3-us' 
ir_port.set_device = 'lego-ev3-ir'
gyro_port.set_device = 'lego-ev3-gyro'

sleep(2)

us = UltrasonicSensor('ev3-ports:in2:i2c80:mux1')
ir = InfraredSensor('ev3-ports:in2:i2c81:mux2')
gyro = GyroSensor('ev3-ports:in2:i2c82:mux3')

us_value = us.value(0)
us_dist = us.distance_centimeters # takes multiple measurements
us_numvalues = us.num_values
us_address = us.address
us_units = us.units

ir_value = ir.value(0)
ir_proximity = ir.proximity # an estimate of the distance between the sensor and the objects in front of it
ir_beacon= ir.distance(channel=1) # returns distance (0,100) to the beacon on the given channel
ir_heading = ir.heading(channel=1) # returns heading (-25,25) to the beacon on the given channel
ir_address = ir.address

gyro_value = gyro.value(0)
gyro_angle = gyro.angle
gyro_address = gyro.address
gyro_units = gyro.units

# Sensors connected to NXT-Sensor-Split MUX - Barometer---------------------------------------------------------

temp1 = Sensor(INPUT_1)
temp1.mode = 'TEMP'
Temp_NXT = (temp1.value(0)/1000)

pressure = Sensor(INPUT_1)
pressure.mode = 'PRESS'
Pressure_NXT = ((pressure.value(0)/10)*25.4)

# Sensors connected to either INPUT3 or INPUT 4 -------------------------------------------------------------------

a = LegoPort(INPUT_4)
a.set_device = 'lego-nxt-sound'
sleep(1)
ss = SoundSensor()
spl = ss.sound_pressure_low
sound_value = ss.value(0)
    
temp2 = Sensor(INPUT_3)
temp2.mode = 'NXT-TEMP-C'
Temp_EV3 = (temp2.value(0)/10)

#=================================================================================================================
# S-E-N-S-O-R   T-E-S-T
#=================================================================================================================

def Detect_Sensors():
    print("Test initiated...",file=stderr)
    sleep(2)
    print("testing sensors...",file=stderr)

    # Ultrasonic Sensor info
    print("",file=stderr)
    print("Ultrasonic Sensor info", file=stderr)
    print("Ultrasonic value: ",us_value, file=stderr)
    print("Distance CM: ",us_dist,file=stderr)
    print("Distance number values: ",us_numvalues,file=stderr)
    print("us address: ",us_address,file=stderr)
    print("Distance units: ",us_units,file=stderr)
    print("testing Infrared Sensor...",file=stderr)
    sleep(1)

    # IR Sensor Info
    print("",file=stderr)
    print("Infrared Sensor Info", file=stderr)
    print("Infrared value: ",ir_value,file=stderr)
    print("Distance proximity: ",ir_proximity,file=stderr)
    print("Distance from beacon ",ir_beacon,file=stderr)
    print("Heading or direction: ",ir_heading,file=stderr)
    print("IR address: ",us_address,file=stderr)
    print("testing Gyro Sensor...",file=stderr)
    sleep(1)

    # Gyro Sensor Info
    print("",file=stderr)
    print("Gyro Sensor Info",file=stderr)
    print("Gyro value: ",gyro_value,file=stderr)
    print("Gyro angle: ",gyro_angle,file=stderr)
    print("Gyro address: ",gyro_address,file=stderr)
    print("Gyro units: ",gyro_units,file=stderr)
    sleep(2)
    print("",file=stderr)
    print("EV3 Sensor MUX functioning",file=stderr)

    # Barometer Sensor Info
    print("",file=stderr)
    print("Barometer",file=stderr)
    print("Temperature : ... ",Temp_NXT,file=stderr)
    print("units = ",temp1.units,file=stderr)
    print("Pressure mmHg: ... ",Temp_NXT,file=stderr)
    print("units = ",pressure.units,file=stderr)
    print("",file=stderr)
    sleep(1)
    print("NXT-V2-Split functioning",file=stderr)
 

    # Sound Sensor Info
    print("",file=stderr)
    print("Sound Sensor Info", ss.value(0),file=stderr)
    print("DBA: ",ss.sound_pressure_low,file=stderr)
    print("Units", ss.units,file=stderr)
    sleep(1)

    print("ALL SENSORS DETECTED AND FUNCTIONING",file=stderr)


# Module Actions ------------------------------------------------------------------------------------------------------


def random_direction():

  number = random.randint(1,2) # select a random number 1 or 2

  if number == 1:
    steer_pair.on_for_rotations(steering = random.randint(35,50), speed=50, rotations=random.randint(3,5)) # move 35 to 50 degrees forward to the right
  elif number == 2:       
    steer_pair.on_for_rotations(steering = random.randint(-50,-35), speed=50, rotations=random.randint(3,5)) # move 35 to 50 degrees forward to the left
  

#-----------------------------------------------------------------------------------------------------------------------------

def close_claw(): # the robot closes its claw and lifts up the object

    MediumMotor_Claw.on_for_rotations(speed=38, rotations=1) # Claws close (claws by default open)
    sleep(1)
    MediumMotor_lift.on_for_seconds(speed=20, seconds=6) # lift goes up

#------------------------------------------------------------------------------------------------------------------------------

def open_claw(): # the robot puts down the object and opens claw
    MediumMotor_lift.on_for_seconds(speed=-20, seconds=6) # lift goes down
    MediumMotor_Claw.on_for_rotations(speed=-38, rotations=2) # claw opens

#---------------------------------------------------------------------------------------------------------------------------

def reverse(): # robot goes backwards
    steer_pair.on_for_rotations(0,speed=30,rotations=3)

#------------------------------------------------------------------------------------------------------------------------------

def complete_stop():
    steer_pair.off

#------------------------------------------------------------------------------------------------------------------------------

def red(): # red lights turn on and off
    leds.set_color('LEFT', 'RED')
    leds.set_color('RIGHT','RED')

#------------------------------------------------------------------------------------------------------------------------------

def green(): # green lights turn on and off
    leds.set_color('LEFT', 'GREEN')
    leds.set_color('RIGHT','GREEN')

#----------------------------------------------------------------------------------------------------------------------

def amber(): # amber lights turn on and off
    leds.set_color('LEFT','AMBER')
    leds.set_color('RIGHT','AMBER')

#----------------------------------------------------------------------------------------------------------------------

def red_green():
    leds.set_color('LEFT','RED')
    leds.set_color('RIGHT','GREEN')

#-----------------------------------------------------------------------------------------------------------------------

def amber_red():
    leds.set_color('LEFT','AMBER')
    leds.set_color('RIGHT','RED')

#---------------------------------------------------------------------------------------------------------------------

def amber_green():
    leds.set_color('LEFT','AMBER')
    leds.set_color('RIGHT','GREEN')  

#---------------------------------------------------------------------------------------------------------------------

def error():
    steer_pair.off
    red()
    sleep(0.1)
    green()
    sleep(0.1)
    red()
    sleep(0.1)
    green
    sleep(0.1)

#-------------------------------------------------------------------------------------------------------------------

def close_action():
    red()
    sleep(2)
    close_claw()
       
#--------------------------------------------------------------------------------------------------------------------

def open_action():
    red()
    open_claw()

#---------------------------------------------------------------------------------------------------------------------

def gyro_Angle():
    if 10 >= gyro.wait_until_angle_changed_by < 15:
        sound.speak("10 degrees reached")
        print("10 degrees reached", file = stderr)
        red()

    elif 15 >= gyro.wait_until_angle_changed_by < 20:
        sound.speak("15 degrees reached")
        print("15 degrees reached", file = stderr)
        red()

    elif gyro.wait_until_angle_changed_by >= 20:
        sound.speak("20 degrees or more exceeded")
        print("20 degrees or more exceeded", file = stderr)
        red()
        complete_stop()


#--------------------------------------------------------------------------------------------------------------------
# R-E-M-O-T-E    M-O-D-E     BRICK LEFT BUTTON
#---------------------------------------------------------------------------------------------------------------------

def top_left_channel_1_action(state):
    if state: # if state == True (pressed)
        steer_pair.on(steering=0, speed=40)
    else:
        steer_pair.off()

def bottom_left_channel_1_action(state):
    if state:
        steer_pair.on(steering=0, speed=-40)
    else:
        steer_pair.off()

def top_right_channel_1_action(state):
    if state:
        steer_pair.on(steering=100, speed=30)
    else:
        steer_pair.off()

def bottom_right_channel_1_action(state):
    if state:
        steer_pair.on(steering=-100, speed=30)
    else:
        steer_pair.off()


def remote_control_event():

    # Events associated with IR remote
    ir.on_channel1_top_left = top_left_channel_1_action
    ir.on_channel1_bottom_left = bottom_left_channel_1_action
    ir.on_channel1_top_right = top_right_channel_1_action
    ir.on_channel1_bottom_right = bottom_right_channel_1_action

    while True:
        ir.process()
        sleep(0.01)

#--------------------------------------------------------------------------------------------------------------------
# B-E-A-C-O-N    M-O-D-E       BRICK UP BUTTON
#--------------------------------------------------------------------------------------------------------------------

def beacon():
    while True:    # forever
        distance = ir.distance(channel=1) #(0 to 100)
        ir.heading(channel=1) #(-25 to 25)
        ir.beacon(channel=1)

        if distance:  # If distance is not None
            error = distance - 10
            steer_pair.on(steering=2*ir.heading(), speed=min(-90, 3*error))
        else:
            print('beacon not detected',end=' ') # prints to EV3 LCD screen
            print('beacon not detected',end=' ', file=stderr) # print to VS Code output panel
            sound.play_tone(1800-10*distance,0.4)
            steer_pair.off()
            sleep(0.1)
    print(' ') # start new line on EV3 screen
    print(' ', file=stderr) # start new line in VS code


#----------------------------------------------------------------------------------------------------------------------
# A-U-T-O-N-O-M-O-U-S     M-O-D-E      BRICK RIGHT BUTTON
#---------------------------------------------------------------------------------------------------------------------

def auto_thread():
    count = 0
    while True:
        while us.distance_centimeters >= 30:
            steer_pair.on(steering = 0, speed=-40) # Drive forward
    
        else:

            complete_stop()
            sound.speak("object detected")
            red()
            sound.speak("waiting for sound")
            
            sleep(2)

            if ss.sound_pressure_low < 75:
                print("sound value: ",ss.sound_pressure_low, file=stderr)
                reverse()
                random_direction()

            else: 
                print("sound value: ",ss.sound_pressure_low, file=stderr)
                sound.speak("Sound detected")
                close_action()
                sleep(9)
                random_direction()
                while us.distance_centimeters >= 30:
                    steer_pair.on(steering = 0, speed=-40) # Drive forward
                else:
                    print("sound value: ",ss.sound_pressure_low, file=stderr)
                    open_action()
                    sleep(9)
                    random_direction()
       
        count = count + 1
        if count == 5:
            break
 
def auto_mode():

    sound.speak("Autonomous mode")
    print("Autonomous mode", file=stderr)
    sleep(1)

    auto_thread()
         

#-----------------------------------------------------------------------------------------------------------------
# D-A-T-A     C-O-L-L-E-C-T-I-O-N     BRICK DOWN BUTTON
#------------------------------------------------------------------------------------------------------------------

def data_collection():

    index = 0
    count = 0
    
    start_time = datetime.now()

    with open('data.csv','a',newline='') as csvfile:
        data_file = csv.writer(csvfile,delimiter = ',')
        data_file.writerow([])
        data_file.writerow(['index','Temp1','Temp2','Pressure','Decibals','Angle','Time Point'])
     
        time_elapsed = datetime.now() - start_time

        while True:
           
            print(time_elapsed, file=stderr)
            
            sound.speak('Sample Taken')
            print("Sample Taken ...",file = stderr)
            green()          
            index = index + 1 
            count = count + 1
            print('Temp:',Temp_NXT,'or',Temp_EV3,'Degrees Celcius','---','Barometric Pressure:',Pressure_NXT,
            'mmHg','---', end=' ',file = stderr)
            #print('sound',spl,sound_value,'Degrees',gyro_value, end=stderr)
            print('Time: (hh:mm:ss) {}'.format(time_elapsed),file=stderr)
        
            data_file.writerow([index,Temp_NXT, Temp_EV3,Pressure_NXT,sound_value,gyro_value,time_elapsed])
            sleep(2)

            if count == 2:
                break

def random_drive():
    count = 0
    while True:
        while us.distance_centimeters >= 30:
            steer_pair.on(steering = 0, speed=-40) # Drive forward
        else:
            random_direction()
        count = count + 1
        if count == 2:
            break
     

def data_collection_mode():

    thread1 = Thread(name = "data collection", target = data_collection)
    thread2 = Thread(name = "random drive",target = random_drive)
    
    count = 0
    while True:
        thread1.start()
        thread2.start()
        count = count + 1
        if count == 10:
            break
            
        
#===================================================================================================================     
# Brick Buttons
#===================================================================================================================

def brick_up(state):
    if state:
        sound.speak("Beacon mode activated")
        red_green()
        sleep(2)
        beacon()

def brick_left(state):
    if state:
        sound.speak("Remote control mode activated")
        amber_red()
        sleep(2)
        remote_control_event()
    
def brick_right(state):
    if state:
        sound.speak("Automonmous mode activated")
        red()
        sleep(2)
        auto_mode()

        count = 0
        while True:
            count = count + 1
            auto_mode()
            if count == 5:
                break

def brick_down(state):
    if state:
        sound.speak("Data collection mode activated")
        amber_green()
        sleep(2)
        data_collection_mode()
        

def brick_button_event():
    # Events associated with Brick Buttons
    btn.on_left = brick_left
    btn.on_right = brick_right
    btn.on_up = brick_up
    btn.on_down = brick_down

    while True:
        btn.process()
        sleep(0.01)

#=====================================================================================================================
#----------------------------------------------------------------------------------------------------------------------
# M-A-I-N   P-R-O-G-R-A-M
#----------------------------------------------------------------------------------------------------------------------
#====================================================================================================================

# Brick Down = Data Collection
# Brick Right = Autonomous Mode
# Brick Up = Beacon Mode
# Brick Left = Remote Mode



red()
green()
amber()
sound.speak("Detecting all sensors")
Detect_Sensors() # Detect whether all sensors are detected and functioning
sound.speak("All sensors have been detected")
sleep(1)
sound.speak("Choose mode")
brick_button_event() # Begin main program