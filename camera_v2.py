import rospy
import json
from std_msgs.msg import String
import random
from picamera import PiCamera
import time
import cv2
import threading

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

    def toJSON(self):

        x = {
            'board': self.board,
            'turns': self.turns
        }

        return json.dumps(x)

    @staticmethod
    def fromJSON(string):
        x = json.loads(string)

        board = Board()
        board.board = x['board']
        board.turns = x['turns']

        return board

# initiate variables
rospy.init_node('camera', anonymous=True)
pub = rospy.Publisher('toRoot', String, queue_size=10)
board = Board()
camera = PiCamera()
time.sleep(1)


def getOLocation(im):

    locations = []

    imgwidth=im.shape[0]
    imgheight=im.shape[1]


    y1 = 0
    M = imgwidth//3
    N = imgheight//3


    n = 0
    for x in range(0,imgwidth,M):
        for y in range(0, imgheight, N):
            x1 = x + M
            y1 = y + N
            tiles = im[x:x+M,y:y+N]

            cv2.rectangle(im, (x, y), (x1, y1), (0, 255, 0))
            # cv2.imwrite(r'/Users/shaivpatel/Desktop/Apoopoo/tictactoe/after/' +str(x)+ str(y)+"SAMPLE.png",tiles)

            # Read image.

            gray = cv2.cvtColor(tiles, cv2.COLOR_BGR2GRAY)

            gray = cv2.medianBlur(gray, 5)

            rows = gray.shape[0]
            detected_circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                      param1=100, param2=30,
                                      minRadius=1, maxRadius=3000)

            # Draw circles that are detected.
            if detected_circles is not None:
                locations.append(n)

            n = n + 1
    print(locations)
    return locations

def callback(data):
    global board
    print('Received data from Root.')

    if 'type' in data.data:
        dic = json.loads(data.data)
        messageType = dic['type']
        messageData = dic['data']
        if messageType == 'newGame':
            board = Board.fromJSON(messageData)
            print('Starting New Game')
            board.printBoard()
        elif messageType == 'update':
            board = Board.fromJSON(messageData)
            print('Received update request')
            board.printBoard()


            x = 85
            y = 0
            h = 1000
            w = 1000



            oldLocations = []
            for i, cell in enumerate(board.board):
                if cell == 'O':
                    oldLocations.append(i)


            newLocation = -1
            newLoc = input('Type new O.')
            if newLoc != '-1':
                newLocation = int(newLoc)
                board.addO(newLocation)


            while newLocation == -1:
                camera.capture('new.jpg')
                im2 = cv2.imread(r'new.jpg')
                im2 = im2[y:y + h, x:x + w]
                im2 = cv2.resize(im2, (600, 600))
                newLocations = getOLocation(im2)

                for loc in newLocations:
                    if loc not in oldLocations:
                        print('{} is a new location'.format(loc))
                        newLocation = loc
                        board.addO(loc)

                if newLocation == -1:
                    print('No new Os found. will try again in 10 seconds')
                    time.sleep(10)

            print('Sending updated board')
            board.printBoard()

            x = {
                'type':'board',
                'data':board.toJSON()
            }
            message = json.dumps(x)
            pub.publish(message)



def listener():
    rospy.Subscriber("fromRoot", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()