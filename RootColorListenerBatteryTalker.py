#!/usr/bin/env python3
# https://github.com/zlite/PyRoot/blob/master/drive-root.py

import gatt
import threading
import time,termios,tty,sys
import datetime
import rospy
from std_msgs.msg import String
import requests


instructions = """---------------------
 KEY         COMMAND
---------------------
 w           Forward
 s           Backward
 a           Left
 d           Right
 space       Stop
 t           Turn
 o           Pen Up
 l           Pen Down
 q           Quit
 c           Colour"""
 

# BLE UUID's
root_identifier_uuid = '48c5d828-ac2a-442d-97a3-0c9822b04979'
uart_service_uuid = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
tx_characteristic_uuid = '6e400002-b5a3-f393-e0a9-e50e24dcca9e' # Write
rx_characteristic_uuid = '6e400003-b5a3-f393-e0a9-e50e24dcca9e' # Notify
sensorReading = {
    'batteryLevel' : None
}
class BluetoothDeviceManager(gatt.DeviceManager):
    robot = None # root robot device

    def device_discovered(self, device):
        print("[%s] Discovered: %s" % (device.mac_address, device.alias()))
        self.stop_discovery() # Stop searching
        self.robot = RootDevice(mac_address=device.mac_address, manager=self)
        self.robot.connect()

class RootDevice(gatt.Device):
    def connect_succeeded(self):
        super().connect_succeeded()
        print("[%s] Connected" % (self.mac_address))

    def connect_failed(self, error):
        super().connect_failed(error)
        print("[%s] Connection failed: %s" % (self.mac_address, str(error)))

    def disconnect_succeeded(self):
        super().disconnect_succeeded()
        print("[%s] Disconnected" % (self.mac_address))

    def services_resolved(self):
        super().services_resolved()
        print("[%s] Resolved services" % (self.mac_address))

        self.uart_service = next(
            s for s in self.services
            if s.uuid == uart_service_uuid)

        self.tx_characteristic = next(
            c for c in self.uart_service.characteristics
            if c.uuid == tx_characteristic_uuid)

        self.rx_characteristic = next(
            c for c in self.uart_service.characteristics
            if c.uuid == rx_characteristic_uuid)

        self.rx_characteristic.enable_notifications() # listen to RX messages

    def characteristic_value_updated(self, characteristic, value):
        global batteryLevel

        message = []
        mtype = ""
        for byte in value:
            message.append(byte)
        print ("Messages from Root:")
        if message[0] == 4:  mtype = "Color Sensor"
        if message[0] == 12: mtype = "Bumper"
        if message[0] == 13: mtype = "Light Sensor"
        if message[0] == 17: mtype = "Touch Sensor"
        if message[0] == 20: mtype = "Cliff Sensor"
        if message[0] == 14: 
            mtype = "Battery Level"
            sensorReading['batteryLevel'] = message[9]

        print(mtype, message)

    def drive_forward(self):
        self.tx_characteristic.write_value([0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD1])
        time.sleep(2)

    def drive_left(self):
        self.tx_characteristic.write_value([0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8A])
        time.sleep(2)

    def drive_right(self):
        self.tx_characteristic.write_value([0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25])
        time.sleep(2)

    def stop(self):
        self.tx_characteristic.write_value([0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E])
        time.sleep(2)

    def drive_backwards(self):
        self.tx_characteristic.write_value([0x01, 0x04, 0x00, 0xFF, 0xFF, 0xFF, 0x9C, 0xFF, 0xFF, 0xFF, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x71])
        time.sleep(2)

    def pen_up(self):
        self.tx_characteristic.write_value([0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        time.sleep(2)

    def pen_down(self):
        self.tx_characteristic.write_value([0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        time.sleep(2)

    def rotate_right(self, angle):
        angle = angle*10
        angleBytes = angle.to_bytes(4,byteorder='big',signed=True)
        self.tx_characteristic.write_value([0x01, 0x0C, 0x00, angleBytes[0], angleBytes[1], angleBytes[2], angleBytes[3],0x00, 0x00, 
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        time.sleep(2)

    def set_colour(self, mode, red, green, blue):
        modeBytes = mode.to_bytes(1,byteorder='big',signed=False)
        redBytes = red.to_bytes(1,byteorder='big',signed=False)
        greenBytes = green.to_bytes(1,byteorder='big',signed=False)
        blueBytes = blue.to_bytes(1,byteorder='big',signed=False)
        self.tx_characteristic.write_value([0x03, 0x02, 0x00, modeBytes[0], redBytes[0], greenBytes[0], blueBytes[0],0x00, 0x00, 
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def drive_distance(self, dist):
        distBytes = dist.to_bytes(4,byteorder='big',signed=True)
        self.tx_characteristic.write_value([0x01, 0x08, 0x00, distBytes[0], distBytes[1], distBytes[2], distBytes[3],0x00, 0x00, 
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        time.sleep(2)

    def turn_rate(self, rate):
        left = 0
        right = 0
        if rate >= 0:
            left = rate
        if rate < 0:
            right = -1*rate
        leftbytes = left.to_bytes(4,byteorder='big',signed=True)  # need to convert to byte string
        rightbytes = right.to_bytes(4,byteorder='big',signed=True)
        # note that we're not dynamically calculating the CRC at the end, so just leaving it 0 (unchecked)
        self.tx_characteristic.write_value([0x01, 0x04, 0x00, leftbytes[0], leftbytes[1], leftbytes[2], leftbytes[3], rightbytes[0], rightbytes[1], rightbytes[2], rightbytes[3], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0])
        time.sleep(2)

    def steer(self, left, right):
        leftbytes = left.to_bytes(4,byteorder='big',signed=True)  # need to convert to byte string
        rightbytes = right.to_bytes(4,byteorder='big',signed=True)
        # note that we're not dynamically calculating the CRC at the end, so just leaving it 0 (unchecked)
        self.tx_characteristic.write_value([0x01, 0x04, 0x00, leftbytes[0], leftbytes[1], leftbytes[2], leftbytes[3], rightbytes[0], rightbytes[1], rightbytes[2], rightbytes[3], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0])
        time.sleep(2)

    def getBatteryLevel():
        print ("Getting battery level")   
        self.tx_characteristic.write_value([0x0E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0])
        

def secondsSince(startTime: datetime.datetime):
    return (datetime.datetime.now() - startTime).total_seconds()


def drive_root(command):
    angle = 0
    if command == "4":
        print ("Draw a square")
        # prompt user to enter seconds until a proper number is entered...
        duration = None
        while duration is None:
            duration = input('How big do you want your square to be? Tell me in mm :)')
            try:
                duration = int(duration)
            except:
                # user input invalid number. clear duration and ask again
                duration = None
                print('Invalid number please try again')

        #print(f'Driving forward for {duration} seconds...')
        print('Duration: ' + str(duration))
        startTime = datetime.datetime.now()
        print(startTime)
        manager.robot.pen_down()
       	manager.robot.drive_distance(duration)
        manager.robot.rotate_right(90)
        manager.robot.drive_distance(duration)
        manager.robot.rotate_right(90)
        manager.robot.drive_distance(duration)
        manager.robot.rotate_right(90)
        manager.robot.drive_distance(duration)
        manager.robot.rotate_right(90)
        manager.robot.stop()
       
        print('Stop Complete')

    if command == "s":
        print ("Drive backwards")
        # prompt user to enter seconds until a proper number is entered...
        duration = None
        while duration is None:
            duration = input('For how many seconds?')
            try:
                duration = float(duration)
            except:
                # user input invalid number. clear duration and ask again
                duration = None
                print('Invalid number please try again')

        #print(f'Driving backward for {duration} seconds...')
        startTime = datetime.datetime.now()
        while secondsSince(startTime) < duration:
            manager.robot.drive_backwards()

    if command == "d":
        print ("Drive right")
        # prompt user to enter seconds until a proper number is entered...
        duration = None
        while duration is None:
            duration = input('For how many seconds?')
            try:
                duration = float(duration)
            except:
                # user input invalid number. clear duration and ask again
                duration = None
                print('Invalid number please try again')

        #print(f'Driving right for {duration} seconds...')
        startTime = datetime.datetime.now()
        while secondsSince(startTime) < duration:
            manager.robot.drive_right()

    if command == "a":
        print ("Drive left")
        # prompt user to enter seconds until a proper number is entered...
        duration = None
        while duration is None:
            duration = input('For how many seconds?')
            try:
                duration = float(duration)
            except:
                # user input invalid number. clear duration and ask again
                duration = None
                print('Invalid number please try again')

        #print(f'Driving left for {duration} seconds...')
        startTime = datetime.datetime.now()
        while secondsSince(startTime) < duration:
            manager.robot.drive_left()

    if command == " ":
        print ("Stop")
        manager.robot.stop()

    if command == "o":
        print ("Pen up")
        manager.robot.pen_up()
    if command == "l":
        print ("Pen down")
        manager.robot.pen_down()
    if command == "t":
        print ("Enter turn rate (up to +-90):")
        char = input()
        rate = int(char)
        print ("Turning ", rate)
        manager.robot.turn_rate(rate)
    if command == "p":
        print ("Steer")
        manager.robot.steer(30,40)
    if command == "c":
        print ("What colour? Red, green or blue") 

# keybaord inputs
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def root_callback(msg):
    global manager
    """Callback for pub subscriber"""
    rospy.loginfo(' I heard %s', msg.data)
    
    if msg.data == "red":
        manager.robot.set_colour(1,255,0,0)
    if msg.data == "green":
        manager.robot.set_colour(1,0,255,0)
    if msg.data == "blue":
        manager.robot.set_colour(1,0,0,255)


def main():
    global manager
    #manager = BluetoothDeviceManager(adapter_name = 'hci0')
    manager.start_discovery(service_uuids=[root_identifier_uuid])
    thread = threading.Thread(target = manager.run)
    thread.start()
    #rospy.init_node('rootPi', anonymous=True) 
    #rospy.Subscriber('toRoot', String, root_callback)
    #pub = rospy.Publisher('fromRoot', String, queue_size=10)

    print("Getting battery level - main") 
    manager.robot.getBatteryLevel()
    msg = String()
    msg.data = "Battery Level: {}".format(sensorReading['batteryLevel'])
    print msg 
    #pub.publish(msg)

    
    #rate = rospy.Rate(0.1)
    #msg = String()
    
    #rospy.spin()    # Run until stopped 



if __name__ == '__main__':
    try:
        manager = BluetoothDeviceManager(adapter_name = 'hci0')
        main()
    
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
