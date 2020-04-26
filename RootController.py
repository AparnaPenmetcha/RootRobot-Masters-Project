# !/usr/bin/env python3
# https://github.com/zlite/PyRoot/blob/master/drive-root.py

import gatt
import threading
import time, termios, tty, sys
import datetime
import rospy
from std_msgs.msg import String
import numpy as np
import random

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
tx_characteristic_uuid = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'  # Write
rx_characteristic_uuid = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'  # Notify
sensorReading = {
    'batteryLevel': None
}

robotTouched = False

class BluetoothDeviceManager(gatt.DeviceManager):
    robot = None  # root robot device

    def device_discovered(self, device):
        print("[%s] Discovered: %s" % (device.mac_address, device.alias()))
        self.stop_discovery()  # Stop searching
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

        self.rx_characteristic.enable_notifications()  # listen to RX messages

    def characteristic_value_updated(self, characteristic, value):
        global robotTouched

        message = []
        mtype = ""
        for byte in value:
            message.append(byte)
        print("Messages from Root:")
        # if message[0] == 4:  mtype = "Color Sensor"
        # if message[0] == 12: mtype = "Bumper"
        # if message[0] == 13: mtype = "Light Sensor"
        if message[0] == 17:
            mtype = "Touch Sensor"
            robotTouched = True
        # if message[0] == 20: mtype = "Cliff Sensor"
        # if message[0] == 14:
        #     mtype = "Battery Level"
        #     sensorReading['batteryLevel'] = message[9]

        print(mtype, message)

    def drive_forward(self):
        self.tx_characteristic.write_value(
            [0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0xD1])
        time.sleep(2)

    def drive_left(self):
        self.tx_characteristic.write_value(
            [0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x8A])
        time.sleep(2)

    def drive_right(self):
        self.tx_characteristic.write_value(
            [0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x25])
        time.sleep(2)

    def stop(self):
        self.tx_characteristic.write_value(
            [0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x7E])
        time.sleep(2)

    def drive_backwards(self):
        self.tx_characteristic.write_value(
            [0x01, 0x04, 0x00, 0xFF, 0xFF, 0xFF, 0x9C, 0xFF, 0xFF, 0xFF, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x71])
        time.sleep(2)

    def pen_up(self):
        self.tx_characteristic.write_value(
            [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00])
        time.sleep(2)

    def pen_down(self):
        self.tx_characteristic.write_value(
            [0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00])
        time.sleep(2)

    def speak(self, string):
        packet = [0x05, 0x04, 0x00]
        stringUTF = [c.encode(encoding='UTF-8',errors='replace') for c in string]
        packet = packet + stringUTF
        for i in range(20-len(packet)):
            packet.append(0x00)

        self.tx_characteristic.write_value(packet)
        time.sleep(2)

    def rotate_right(self, angle):
        angle = angle * 10
        angleBytes = angle.to_bytes(4, byteorder='big', signed=True)
        self.tx_characteristic.write_value(
            [0x01, 0x0C, 0x00, angleBytes[0], angleBytes[1], angleBytes[2], angleBytes[3], 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        time.sleep(5)

    def set_colour(self, mode, red, green, blue):
        modeBytes = mode.to_bytes(1, byteorder='big', signed=False)
        redBytes = red.to_bytes(1, byteorder='big', signed=False)
        greenBytes = green.to_bytes(1, byteorder='big', signed=False)
        blueBytes = blue.to_bytes(1, byteorder='big', signed=False)
        self.tx_characteristic.write_value(
            [0x03, 0x02, 0x00, modeBytes[0], redBytes[0], greenBytes[0], blueBytes[0], 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def drive_distance(self, dist):
        distBytes = dist.to_bytes(4, byteorder='big', signed=True)
        self.tx_characteristic.write_value(
            [0x01, 0x08, 0x00, distBytes[0], distBytes[1], distBytes[2], distBytes[3], 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        time.sleep(5)

    def turn_rate(self, rate):
        left = 0
        right = 0
        if rate >= 0:
            left = rate
        if rate < 0:
            right = -1 * rate
        leftbytes = left.to_bytes(4, byteorder='big', signed=True)  # need to convert to byte string
        rightbytes = right.to_bytes(4, byteorder='big', signed=True)
        # note that we're not dynamically calculating the CRC at the end, so just leaving it 0 (unchecked)
        self.tx_characteristic.write_value(
            [0x01, 0x04, 0x00, leftbytes[0], leftbytes[1], leftbytes[2], leftbytes[3], rightbytes[0], rightbytes[1],
             rightbytes[2], rightbytes[3], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0])
        time.sleep(2)

    def steer(self, left, right):
        leftbytes = left.to_bytes(4, byteorder='big', signed=True)  # need to convert to byte string
        rightbytes = right.to_bytes(4, byteorder='big', signed=True)
        # note that we're not dynamically calculating the CRC at the end, so just leaving it 0 (unchecked)
        self.tx_characteristic.write_value(
            [0x01, 0x04, 0x00, leftbytes[0], leftbytes[1], leftbytes[2], leftbytes[3], rightbytes[0], rightbytes[1],
             rightbytes[2], rightbytes[3], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0])
        time.sleep(2)

    def getBatteryLevel(self):
        print("Getting battery level")
        self.tx_characteristic.write_value(
            [0x0E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x0])

    def goToSquare(self, x, y, currentX, currentY):
        if x > 2 or x < 0 or y > 2 or y < 0:
            return

        if currentX != -1:
            self.goHome()
        else:
            if x == 0:
                self.rotate_right(90)
                self.drive_distance(200)
                self.rotate_right(-90)
            if x == 2:
                self.rotate_right(-90)
                self.drive_distance(200)
                self.rotate_right(90)

            self.drive_distance((y+1)*200)
            currentX = x
            currentY = y

        return currentX, currentY

    def goHome(self, currentX, currentY):

        if currentX == -1:
            pass
        else:
            if currentX == 0:
                self.rotate_right(-90)
                self.drive_distance(200)
                self.rotate_right(-90)
            if currentX == 2:
                self.rotate_right(90)
                self.drive_distance(200)
                self.rotate_right(90)
            if currentX == 1:
                self.rotate_right(180)

            self.drive_distance((currentY + 1) * 200)
            currentX = -1
            currentY = -1
            time.sleep(5)

            self.rotate_right(180)

        return currentX, currentY

    def drawX(self):

        self.rotate_right(45)
        self.drive_distance(75)
        self.rotate_right(180)
        self.pen_down()
        self.drive_distance(150)
        self.pen_up()
        self.rotate_right(180)
        self.drive_distance(75)
        self.rotate_right(90)
        self.drive_distance(75)
        self.rotate_right(180)
        self.pen_down()
        self.drive_distance(150)
        self.pen_up()
        self.rotate_right(180)
        self.drive_distance(75)
        self.rotate_right(225)

    def celebrate(self):
        self.set_colour(2,255,255,255)
        self.rotate_right(360)
        # self.set_colour(2,255,255,255)
        self.rotate_right(360)
        # self.set_colour(2,255,255,255)
        self.rotate_right(360)
        # self.set_colour(2,255,255,255)
        self.rotate_right(360)

        self.speak('I won loser')

    def congratulate(self):
        self.set_colour(2, 255, 255, 255)
        self.rotate_right(360)

        self.speak('Congratulations!')

class RootController:

    def __init__(self):
        rospy.init_node('rootPi', anonymous=True)
        rospy.Subscriber('toRoot', String, self.root_callback)
        self.pub = rospy.Publisher('fromRoot', String, queue_size=10)
        self.received = False
        self.lastOLocation = -1

    def sendRequest(self, message):

        # rospy.loginfo('Sending to Camera: %s', messageString.data)
        msg = String()
        rate = rospy.Rate(1)  # update rate in Hz
        if message == 'received':
            n = 0
            print("Sending: " + message)
            while not rospy.is_shutdown():
                # print(message)
                msg.data = message
                self.pub.publish(msg)
                rate.sleep()
                n = n + 1
                if n > 100:
                    return
        else:
            print("Sending: " + message)
            while not rospy.is_shutdown():
                msg.data = message
                self.pub.publish(msg)
                rate.sleep()
                if self.received:
                    self.received = False
                    return

    def root_callback(self, msg):
        if msg.data == 'received':
            self.received = True
        else:
            try:
                self.lastOLocation = int(msg.data)
                print("Received Last O Location: " + msg.data)
                self.sendRequest('received')
            except:
                print("Received: " + msg.data)
                self.sendRequest('received')



class Board:
    def __init__(self):
        self.board = ['N', 'N', 'N', 'N', 'N', 'N', 'N', 'N', 'N']
        self.turns = 0

    def addX(self, position):
        self.board[position] = 'X'
        self.turns = self.turns + 1
    def addO(self, position):
        self.board[position] = 'O'
        self.turns = self.turns + 1

    def checkWin(self):
        normalizedBoard = []
        for i in range(len(self.board)):
            if self.board[i] == 'N':
                normalizedBoard.append(0)
            if self.board[i] == 'O':
                normalizedBoard.append(1)
            if self.board[i] == 'X':
                normalizedBoard.append(-1)

        topRow    =    [0, 1, 2]
        midRow    =    [3, 4, 5]
        botRow    =    [6, 7, 8]
        leftCol   =    [0, 3, 6]
        midCol    =    [1, 4, 7]
        rightCol  =    [2, 5, 8]
        majorDiag =    [0, 4, 8]
        minorDiag =    [6, 4, 2]

        groups = [
            topRow,
            midRow,
            botRow,
            leftCol,
            midCol,
            rightCol,
            majorDiag,
            minorDiag
        ]

        for group in groups:
            sum = 0
            for cell in group:
                sum = sum + normalizedBoard[cell]

            if sum == 3:
                return 'O'
            if sum == -3:
                return 'X'

        return 'N'

    def checkNextTurnWinforX(self):
        normalizedBoard = []
        for i in range(len(self.board)):
            if self.board[i] == 'N':
                normalizedBoard.append(0)
            if self.board[i] == 'O':
                normalizedBoard.append(1)
            if self.board[i] == 'X':
                normalizedBoard.append(-1)

        topRow    =    [0, 1, 2]
        midRow    =    [3, 4, 5]
        botRow    =    [6, 7, 8]
        leftCol   =    [0, 3, 6]
        midCol    =    [1, 4, 7]
        rightCol  =    [2, 5, 8]
        majorDiag =    [0, 4, 8]
        minorDiag =    [6, 4, 2]

        groups = [
            topRow,
            midRow,
            botRow,
            leftCol,
            midCol,
            rightCol,
            majorDiag,
            minorDiag
        ]

        for group in groups:
            sum = 0
            for cell in group:
                sum = sum + normalizedBoard[cell]

            if sum == -2:
                for cell in group:
                    if normalizedBoard[cell] == 0:
                        return cell

        return -1

    def checkNextTurnWinforO(self):
        normalizedBoard = []
        for i in range(len(self.board)):
            if self.board[i] == 'N':
                normalizedBoard.append(0)
            if self.board[i] == 'O':
                normalizedBoard.append(1)
            if self.board[i] == 'X':
                normalizedBoard.append(-1)

        topRow    =    [0, 1, 2]
        midRow    =    [3, 4, 5]
        botRow    =    [6, 7, 8]
        leftCol   =    [0, 3, 6]
        midCol    =    [1, 4, 7]
        rightCol  =    [2, 5, 8]
        majorDiag =    [0, 4, 8]
        minorDiag =    [6, 4, 2]

        groups = [
            topRow,
            midRow,
            botRow,
            leftCol,
            midCol,
            rightCol,
            majorDiag,
            minorDiag
        ]

        for group in groups:
            sum = 0
            for cell in group:
                sum = sum + normalizedBoard[cell]

            if sum == 2:
                for cell in group:
                    if normalizedBoard[cell] == 0:
                        return cell

        return -1

    def getRandomEmpty(self):
        empties = []
        for i, cell in enumerate(self.board):
            if cell == 'N':
                empties.append(i)

        seed = random.randint(0, len(empties)-1)
        return empties[seed]



    def printBoard(self):
        print()
        print(self.board[0]+'|'+self.board[1]+'|'+self.board[2])
        print('- - -')
        print(self.board[3]+'|'+self.board[4]+'|'+self.board[5])
        print('- - -')
        print(self.board[6]+'|'+self.board[7]+'|'+self.board[8])
        print()

if __name__ == '__main__':

    rootController = RootController()
    manager = BluetoothDeviceManager(adapter_name = 'hci0')
    manager.start_discovery(service_uuids=[root_identifier_uuid])
    thread = threading.Thread(target=manager.run)
    thread.start()

    currentX = -1
    currentY = -1

    board = Board()


    while manager.robot is None:
        print('Robot not assigned. Waiting to complete connection.')
        time.sleep(1)

    print('Robot assigned and connected')
    time.sleep(10)

    # x = int(input('Type a x-coor.'))
    # y = int(input('Type a y-coor.'))
    # currentX, currentY = manager.robot.goToSquare(x,y,currentX, currentY)
    # time.sleep(2)
    # manager.robot.drawX()
    # time.sleep(10)
    # currentX, currentY = manager.robot.goHome(currentX, currentY)
    # rospy.spin()

    rootController.sendRequest('newGame')
    while board.turns < 9 and board.checkWin() == 'N':

        while not robotTouched:
            time.sleep(1)

        robotTouched = False
        print("User touched robot sensor. Requesting O location from camera...")
        rootController.sendRequest('getOs')

        lastOLocation = rootController.lastOLocation

        while lastOLocation == rootController.lastOLocation:
            time.sleep(1)

        board.addO(rootController.lastOLocation)

        if board.checkWin() != 'N':
            break
        elif board.checkNextTurnWinforX() > -1:
            winningSquare = board.checkNextTurnWinforX()
            winningX = winningSquare % 3
            winningY = winningSquare // 3

            currentX, currentY = manager.robot.goToSquare(winningX, winningY, currentX, currentY)
            time.sleep(2)
            manager.robot.drawX()
            board.addX(winningSquare)
            time.sleep(2)
            currentX, currentY = manager.robot.goHome(currentX, currentY)


        elif board.checkNextTurnWinforO() > -1:
            blockingSquare = board.checkNextTurnWinforO()
            blockingX = blockingSquare % 3
            blockingY = blockingSquare // 3

            currentX, currentY = manager.robot.goToSquare(blockingX, blockingY, currentX, currentY)
            time.sleep(2)
            manager.robot.drawX()
            board.addX(blockingSquare)
            time.sleep(2)
            currentX, currentY = manager.robot.goHome(currentX, currentY)

        else:
            randomSquare = board.getRandomEmpty()
            randomX = randomSquare % 3
            randomY = randomSquare // 3

            currentX, currentY = manager.robot.goToSquare(randomX, randomY, currentX, currentY)
            time.sleep(2)
            manager.robot.drawX()
            board.addX(randomSquare)
            time.sleep(2)
            currentX, currentY = manager.robot.goHome(currentX, currentY)

    if board.checkWin() == 'O':
        manager.robot.congratulate()
    elif board.checkWin() == 'X':
        manager.robot.celebrate()

