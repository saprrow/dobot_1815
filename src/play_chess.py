import sys
sys.path.append("../lib")

import DobotDllType as dType
import time
import cv2
import numpy as np
from camera import YoloChessDetect

class Dobot:
    def __init__(self):
        api = dType.load()
        self.get_com()
        self.connect_to_Dobot()
    
    def get_com(self):
    #TODO: choose right COMs automatically
        self.com_port = "COM3"

    def connect_to_Dobot(self):
    """
    connect to the Dobot platform
    :param com_port: The port of the DOBOT you want to connect to
    :return:
    state: The connect state of the Dobot, contain the dobotId
    api: The Handle of the Dobot
    """
        CON_STR = {
            dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
            dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
            dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}
        #Connect Dobot
        state = dType.ConnectDobot(api, self.com_port, 115200)
        print("Connect status:", CON_STR[state[0]])
        if state[0] == dType.DobotConnect.DobotConnect_NoError:
            return state
        else:
            print("connection to Dobot failure")
            exit(0)

    def set_init_position(self):
    """
    reset the Dobot
    :param api:  Handle of the Dobot
    :return:
    """
        dType.SetQueuedCmdClear(api) #clear cmd queue
        dType.SetHOMEParams(api, 200, 200, 200, 200, isQueued=1)
        lastIndex = dType.SetHOMECmd(api, temp=0, isQueued=1)[0]
        dType.SetQueuedCmdStartExec(api)  #start to Excute command
        
        #Wait for Excuting Last Command
        while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
            dType.dSleep(100)
        #stop excuting
        dType.SetQueuedCmdStopExec(api)

    def execute_arm_cmd(self, point_x=180, point_y=0, point_z=15, point_r=0):
    """
    control the position of arm
    """

        # result = api.SetPTPCmd(dobotId, byref(cmd), isQueued, byref(queuedCmdIndex))
        lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, point_x, point_y, point_z, point_r, isQueued=1)[0]
        dType.SetQueuedCmdStartExec(api)
        while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
            dType.dSleep(100)
        # Stop to Execute Command Queued
        dType.SetQueuedCmdStopExec(api)

    def execute_endeffector_cmd(self, suck=0, enable=0, isQueued=0):
    # enable = 1  # 0 -> no, 1 -> yes
    # suck = 1  # 0 -> no, 1 -> yes
        lastIndex1 = dType.SetEndEffectorSuctionCup(api, enable, suck, isQueued=1)
        dType.SetQueuedCmdStartExec(api)
        dType.SetQueuedCmdStopExec(api)

    def pick_chess(self, xy):
        '''
        picks red things to one position and stacks them.
        param:
        x,y,z: the position of red things
        dx, dy: position where robot put down red things
        index_h: index to adjust droping height
        '''
        dType.SetPTPJointParams(api, 30, 0, 20, 0, 20, 0, 0, 0, isQueued=1)
        dType.SetPTPCommonParams(api, 30, 20, isQueued=1)
        x, y= xy[0], xy[1]
        print("actual position:", x, y)
        self.execute_arm_cmd(api, self.cx, self.cy, -53-self.index_h*3.75, 0)
        time.sleep(0.2)
        self.execute_endeffector_cmd(api, suck=1, enable=1)
        time.sleep(0.8)
        self.execute_arm_cmd(api, x, y, 30, 0)
        time.sleep(0.2)
        self.execute_arm_cmd(api, x, y, -63, 0)
        time.sleep(0.2)
        self.execute_endeffector_cmd(api, suck=0, enable=0)
        self.execute_arm_cmd(api, x, y, 20, 0)
        time.sleep(0.2)
        self.execute_arm_cmd(api, self.cx, self.cy, 30, 0)
        time.sleep(0.2)

class Chess(Dobot):
    def __init__(self):
        super.__init__()
        self.board_grids = []
        self.board = [" "]*9
        self.offset = 46        #the distance between each grid
        self.h_index = 0  #index to adjust the height to pick chess

    def init_chess(self, xyc):
        self.cx = xyc["chess_board"][0]
        self.cy = xyc["chess_board"][1]
        del xyc["chess_board"]
        
        for key, value in xyc.items(): 
            if value != []:
                self.turn = key
                self.cx = value[0]
                self.cy = value[1]

        if self.turn = "x":
            self.computer = 'x'
            self.computer_move()
        else:
            self.computer = 'o'
            self.update_board()

    def xy_to_ij(self, xy):
        a = round((xy[1] - cby) / self.offset) + 1
        b = round((xy[0] - cbx) / self.offset) + 1
        return a, b

    def board_state(self, xyc):
        i, j = 0, 0
        del xyc["chess_board"]
        for key, lv in xyc.items():
            for xy in lv:
                i, j = xy_to_ij(xy, cbx, cby)
                if i >= 2 or j >=2:
                    continue 
                board2[3*i+j] = key

    def update_board(self, xyc):
        i, j = 0, 0
        board2 = [" "] * 9
        del xyc["chess_board"]
        for key, lv in xyc.items():
            for xy in lv:
                i, j = xy_to_ij(xy, cbx, cby)
                if i >= 2 or j >=2:
                    continue 
                board2[3*i+j] = key
        if board2 == self.board:
            return False
        else:
            self.board = board2
            return True

    def transform_xy(self, x, y):
        nx = -0.1 * x -0.1 * y
        ny = -0.1 * x -0.1 * y 
        return [nx, ny]

    def get_board_grids(self):
        x, y = 0, 0
        for i in range(-1, 2):
            for j in range(-1, 2)：
                x = self.cx + i * self.offset
                y = self.cy + j * self.offset
                self.board_grids.append(self.transform_xy(x, y))

    def legal_moves(self):
    #return empty position
        moves=[]
        for i in range(9):
            if self.board[i]==" ":
                moves.append(i)
        return moves
    
    def display_board(self):
        board2=self.board[:]
        for i in range(len(self.board)):
            if self.board[i]==" ":
                board2[i]=i
        print("\t",board2[0],"|",board2[1],"|",board2[2])
        print("\t","----------")
        print("\t",board2[3],"|",board2[4],"|",board2[5])
        print("\t","----------")
        print("\t",board2[6],"|",board2[7],"|",board2[8])

    def winner(self):
        boards = self.board
        waystowin=((0,1,2),(3,4,5),(6,7,8),(0,3,6),(1,4,7),(2,5,8),(0,4,8),(2,4,6))#几种赢的方式
        for row in waystowin:
            if  boards[row[0]]==boards[row[1]]==boards[row[2]]!=" ":
                winner=boards[row[0]]
                return winner
        if " " not in boards:
            return "平局"
        return False
    
    def computer_move(self):
        board2=self.board[:]
        bestmove=(4,0,2,6,8,1,3,5,7)#最佳下棋位置顺序表
        legalmove=self.legal_moves()#获取空白位置
        for move in legalmove:
        #if dobot wins the go here
            if self.winner(board2)==self.computer:
                print("Go here:", move)
                self.put_chess(board_grids[move])
                self.h_index += 1
                self.board[move] = self.computer
 
            elif self.winner(board2)==self.human:
                print("Go here:", move)
                self.put_chess(board_grids[move])
                self.h_index += 1
                self.board[move] = self.computer

            else move in legalmove:
                print("Go here:", move)
                self.put_chess(board_grids[move])
                self.h_index += 1
                self.board[move] = self.computer
    
    def next_turn(self,turn):
        if turn=="o":
            return "x"
        else:
            return "o"


if __name__ == '__main__':

    LABELS = ['o', 'x', 'chess_board']
    configPath = os.path.join("yolov3-tiny.cfg")
    weightsPath = os.path.join("yolov3-tiny_final.weights")
    
    chess_detect = YoloChessDetect(img_path, configPath, weightsPath)
    xyc = chess_detect.detect_chess()

    chess=Chess()
    chess.init_chess(xyc)
    chess.display_board()

    while not chess.winner(chess.board):
        if turn==chess.human:
            chess.update_board()
        else:
            move=chess.computer_move()

        chess.display_board()
        turn=chess.next_turn(turn)
        the_winner=chess.winner(chess.board)
        if the_winner == chess.computer:
            print("dobot win ！\n")
        elif the_winner == chess.human:
            print("player win ！\n")
        elif the_winner == "TIE":
            print("tie, game_over\n")


