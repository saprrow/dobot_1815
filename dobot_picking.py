import DobotDllType as dType
import time
import cv2
import numpy as np
from camera import Detect

def connect_to_Dobot(com_port):
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
    state = dType.ConnectDobot(api, com_port, 115200)
    print("Connect status:", CON_STR[state[0]])
    if state[0] == dType.DobotConnect.DobotConnect_NoError:
        return state
    else:
        print("connection to Dobot failure")
        exit(0)

def set_init_position(api):
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

def execute_arm_cmd(api, point_x=180, point_y=0, point_z=15, point_r=0):
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

def execute_endeffector_cmd(api, suck=0, enable=0, isQueued=0):
    # enable = 1  # 0 -> no, 1 -> yes
    # suck = 1  # 0 -> no, 1 -> yes
    lastIndex1 = dType.SetEndEffectorSuctionCup(api, enable, suck, isQueued=1)
    dType.SetQueuedCmdStartExec(api)
    dType.SetQueuedCmdStopExec(api)

def read_frame_from_camera():
    """
    read one frame from the video camera
    :return:
    retval: indicate the success of image reading
    frame: the image
    """
    cap = cv2.VideoCapture(1)
    if cap.isOpened():  # 判断是否正常打开
        retval, frame = cap.read()
        print("read image ok")
    else:
        retval = False
        frame = None
        print("read image failure")
        exit(0)
    return retval, frame

def get_com():
    #TODO: choose right COMs automatically
    return "COM5"

if __name__ == "__main__":
    print("action!")
    com = get_com()

    api = dType.load()

    connect_result = connect_to_Dobot(com)
    set_init_position(api)
    i = 1

    while i:

        cap = cv2.VideoCapture(1)
        for index in range(5):
            if cap.isOpened():
                retval, frame = cap.read()
                cv2.imwrite("detect.jpg".format(index), frame)
                time.sleep(0.2)
        
        #it seem no distortion exists and distortion elimination seems work wrongly.
        #so distortion elimination is laid aside
        """ 
        mtx= np.matrix([[1389, 0, 0], [0, 1346, 0], [689, 353, 1]])
        dist=np.matrix([-0.5273, 0.1840, 0, 0, 0])
        h, w= frame.shape[:2]
        newcameramtx, roi= cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0, (w, h))
        re_frame= cv2.undistort(frame,mtx,dist,None,newcameramtx)
        """

        #cv2.imshow("image1",frame)
        #cv2.waitKey() #make imshow work 
        d = Detect("detect.jpg")
        d.img_process_main()

        #resure
        if(d.dx == 0 and d.dy == 0):
            print("victory!")
            break
            
        d.img_process_main()
        print("camera position:", d.dx, d.dy)
        
        dx = 0.479 * d.dx - 0.263 * d.dy
        dy = -0.172 * d.dx - 0.566 * d.dy
        
        #set parameters of dobot
        dType.SetPTPJointParams(api, 30, 0, 20, 0, 20, 0, 0, 0, isQueued=1)
        dType.SetPTPCommonParams(api, 30, 20, isQueued=1)
        print("move to fish")
        print("actual position:",dx, dx)
        execute_arm_cmd(api, dx, dy, -20, 0)
        time.sleep(0.2)
        execute_arm_cmd(api, dx, dy, -43, 0)
        time.sleep(0.2)
        execute_endeffector_cmd(api, suck=1, enable=1)
        time.sleep(1)
        execute_arm_cmd(api, dx, dy, 40, 0)
        time.sleep(0.2)
        execute_arm_cmd(api, 298, 8, 40, 0)
        time.sleep(0.2)
        execute_arm_cmd(api, 298, 8, -29 + (i-1) * 44, 0)
        execute_endeffector_cmd(api, suck=0, enable=0)
        time.sleep(0.2)
        execute_arm_cmd(api, 298, 8, (i-1) * 44, 0)
        time.sleep(0.2)
        i += 1
