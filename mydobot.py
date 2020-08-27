import DobotDllType as dType
import time

api = dType.load()

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
com = "COM5"
connect_result = connect_to_Dobot(com)
dobotId = connect_result[3]

def execute_endeffector_cmd(api, suck=0, enable=0, isQueued=0):
    # enable = 1  # 0 -> no, 1 -> yes
    # suck = 1  # 0 -> no, 1 -> yes
    lastIndex1 = dType.SetEndEffectorSuctionCup(api, enable, suck, isQueued=1)
    dType.SetQueuedCmdStartExec(api)
    dType.SetQueuedCmdStopExec(api)

set_init_position(api)
'''
execute_arm_cmd(api, 80, 20, 40)
execute_endeffector_cmd(api, 1, 1)

time.sleep(5)
execute_arm_cmd(api, 200, -50, -50)
execute_endeffector_cmd(api, 1, 0)

'''