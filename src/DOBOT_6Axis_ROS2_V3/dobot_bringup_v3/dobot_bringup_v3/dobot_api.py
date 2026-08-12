#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import re
import socket

CONNECT_TIMEOUT_SEC = 3.0

REGISTER_VALUE_PATTERN = re.compile(
    r'^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$'
)
REGISTER_VALUE_TYPES = {'U16', 'U32', 'F32', 'F64'}


class DobotParameterError(ValueError):
    """Raised when a command cannot be safely serialized."""


def _validate_modbus_target(index, addr, count, max_count):
    if not 0 <= index <= 4:
        raise DobotParameterError(
            f'Modbus index must be in range 0..4, got {index}'
        )
    if not 0 <= addr <= 0xFFFF:
        raise DobotParameterError(
            f'Modbus register address must be in range 0..65535, got {addr}'
        )
    if not 1 <= count <= max_count:
        raise DobotParameterError(
            f'Modbus register count must be in range 1..{max_count}, '
            f'got {count}'
        )


def _normalize_register_type(value_type):
    if value_type is None or not str(value_type).strip():
        return None
    normalized = str(value_type).strip().upper()
    if normalized not in REGISTER_VALUE_TYPES:
        raise DobotParameterError(
            f'Unsupported Modbus value type {value_type!r}; expected one of '
            f'{sorted(REGISTER_VALUE_TYPES)}'
        )
    return normalized


def _format_register_values(values, count):
    if isinstance(values, (list, tuple)):
        tokens = [str(value).strip() for value in values]
    else:
        text = str(values).strip()
        if text.startswith('{') and text.endswith('}'):
            text = text[1:-1].strip()
        elif text.startswith('{') or text.endswith('}'):
            raise DobotParameterError(
                f'Malformed Modbus value table {values!r}'
            )
        tokens = [token.strip() for token in text.split(',') if token.strip()]

    if len(tokens) != count:
        raise DobotParameterError(
            f'Modbus value count mismatch: count={count}, values={tokens}'
        )
    if not all(REGISTER_VALUE_PATTERN.fullmatch(token) for token in tokens):
        raise DobotParameterError(
            f'Modbus values must be numeric, got {tokens}'
        )
    return '{' + ','.join(tokens) + '}'


class DobotApi:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.socket_dobot = None

        if self.port == 29999 or self.port == 30003:
            socket_dobot = socket.socket()
            socket_dobot.settimeout(CONNECT_TIMEOUT_SEC)
            try:
                socket_dobot.connect((self.ip, self.port))
            except OSError:
                socket_dobot.close()
                raise
            socket_dobot.settimeout(None)
            self.socket_dobot = socket_dobot

        else:
            raise ValueError(
                f"Dashboard/motion server requires port 29999 or 30003, "
                f"got {self.port}"
            )

    def send_data(self, string):
        print(string)
        if self.socket_dobot is None:
            raise ConnectionError("Dobot socket is not connected")
        self.socket_dobot.sendall(str.encode(string, 'utf-8'))

    def wait_reply(self):
        if self.socket_dobot is None:
            raise ConnectionError("Dobot socket is not connected")
        data = self.socket_dobot.recv(1024)
        if not data:
            self.close()
            raise ConnectionError("Dobot connection closed by robot")
        data_str = str(data, encoding="utf-8")
        return data_str

    def close(self):
        """
        Close the port
        """
        socket_dobot = self.socket_dobot
        self.socket_dobot = None
        if socket_dobot is not None:
            try:
                socket_dobot.close()
            except OSError:
                pass

    def sendRecvMsg(self, string):
        try:
            self.send_data(string)
            return self.wait_reply()
        except OSError:
            self.close()
            raise



# 控制及运动指令接口类
# Control and motion command interface


class DobotApiDashboard(DobotApi):

    def EnableRobot(self,*dynParams):
        """
        Enable the robot
        """
        string = "EnableRobot("+str(dynParams[0][0])+")"
        return self.sendRecvMsg(string)

    def DisableRobot(self):
        """
        Disabled the robot
        """
        string = "DisableRobot()"
        return self.sendRecvMsg(string)

    def ClearError(self):
        """
        Clear controller alarm information
        """
        string = "ClearError()"
        return self.sendRecvMsg(string)

    def ResetRobot(self):
        """
        Robot stop
        """
        string = "ResetRobot()"
        return self.sendRecvMsg(string)

    def SpeedFactor(self, speed):
        """
        Setting the Global rate   
        speed:Rate value(Value range:1~100)
        """
        string = "SpeedFactor({:d})".format(speed)
        return self.sendRecvMsg(string)

    def User(self, index):
        """
        Select the calibrated user coordinate system
        index : Calibrated index of user coordinates
        """
        string = "User({:d})".format(index)
        return self.sendRecvMsg(string)

    def Tool(self, index):
        """
        Select the calibrated tool coordinate system
        index : Calibrated index of tool coordinates
        """
        string = "Tool({:d})".format(index)
        return self.sendRecvMsg(string)

    def RobotMode(self):
        """
        View the robot status
        """
        string = "RobotMode()"
        return self.sendRecvMsg(string)

    def PayLoad(self, weight, inertia):
        """
        Setting robot load
        weight : The load weight
        inertia: The load moment of inertia
        """
        string = "PayLoad({:f},{:f})".format(weight, inertia)
        return self.sendRecvMsg(string)

    def DO(self, index, status):
        """
        Set digital signal output (Queue instruction)
        index : Digital output index (Value range:1~24)
        status : Status of digital signal output port(0:Low level，1:High level
        """
        string = "DO({:d},{:d})".format(index, status)
        return self.sendRecvMsg(string)

    def AccJ(self, speed):
        """
        Set joint acceleration ratio (Only for MovJ, MovJIO, MovJR, JointMovJ commands)
        speed : Joint acceleration ratio (Value range:1~100)
        """
        string = "AccJ({:d})".format(speed)
        return self.sendRecvMsg(string)

    def AccL(self, speed):
        """
        Set the coordinate system acceleration ratio (Only for MovL, MovLIO, MovLR, Jump, Arc, Circle commands)
        speed : Cartesian acceleration ratio (Value range:1~100)
        """
        string = "AccL({:d})".format(speed)
        return self.sendRecvMsg(string)

    def SpeedJ(self, speed):
        """
        Set joint speed ratio (Only for MovJ, MovJIO, MovJR, JointMovJ commands)
        speed : Joint velocity ratio (Value range:1~100)
        """
        string = "SpeedJ({:d})".format(speed)
        return self.sendRecvMsg(string)

    def SpeedL(self, speed):
        """
        Set the cartesian acceleration ratio (Only for MovL, MovLIO, MovLR, Jump, Arc, Circle commands)
        speed : Cartesian acceleration ratio (Value range:1~100)
        """
        string = "SpeedL({:d})".format(speed)
        return self.sendRecvMsg(string)

    def Arch(self, index):
        """
        Set the Jump gate parameter index (This index contains: start point lift height, maximum lift height, end point drop height)
        index : Parameter index (Value range:0~9)
        """
        string = "Arch({:d})".format(index)
        return self.sendRecvMsg(string)

    def CP(self, ratio):
        """
        Set smooth transition ratio
        ratio : Smooth transition ratio (Value range:1~100)
        """
        string = "CP({:d})".format(ratio)
        return self.sendRecvMsg(string)

    def LimZ(self, value):
        """
        Set the maximum lifting height of door type parameters
        value : Maximum lifting height (Highly restricted:Do not exceed the limit position of the z-axis of the manipulator)
        """
        string = "LimZ({:d})".format(value)
        return self.sendRecvMsg(string)

    def RunScript(self, project_name):
        """
        Run the script file
        project_name ：Script file name
        """
        string = "RunScript({:s})".format(project_name)
        return self.sendRecvMsg(string)

    def StopScript(self):
        """
        Stop scripts
        """
        string = "StopScript()"
        return self.sendRecvMsg(string)

    def PauseScript(self):
        """
        Pause the script
        """
        string = "PauseScript()"
        return self.sendRecvMsg(string)

    def ContinueScript(self):
        """
        Continue running the script
        """
        string = "ContinueScript()"
        return self.sendRecvMsg(string)

    def GetHoldRegs(self, id, addr, count, type=None):
        _validate_modbus_target(id, addr, count, max_count=16)
        value_type = _normalize_register_type(type)
        if value_type is not None:
            string = "GetHoldRegs({:d},{:d},{:d},{:s})".format(
                id, addr, count, value_type)
        else:
            string = "GetHoldRegs({:d},{:d},{:d})".format(
                id, addr, count)
        return self.sendRecvMsg(string)

    def SetHoldRegs(self, id, addr, count, table, type=None):
        _validate_modbus_target(id, addr, count, max_count=4)
        value_table = _format_register_values(table, count)
        value_type = _normalize_register_type(type)
        if value_type is not None:
            string = "SetHoldRegs({:d},{:d},{:d},{:s},{:s})".format(
                id, addr, count, value_table, value_type)
        else:
            string = "SetHoldRegs({:d},{:d},{:d},{:s})".format(
                id, addr, count, value_table)
        return self.sendRecvMsg(string)

    def GetErrorID(self):
        """
        Get robot error code
        """
        string = "GetErrorID()"
        return self.sendRecvMsg(string)
    
    
    def DOExecute(self,offset1,offset2):
        string = "DOExecute({:d},{:d}".format(offset1,offset2)+")"
        return self.sendRecvMsg(string)
      
    def ToolDO(self,offset1,offset2):
        string = "ToolDO({:d},{:d}".format(offset1,offset2)+")"
        return self.sendRecvMsg(string)

    def ToolDOExecute(self,offset1,offset2):
        string = "ToolDOExecute({:d},{:d}".format(offset1,offset2)+")"
        return self.sendRecvMsg(string)

    def  SetArmOrientation(self,offset1):
        string = "SetArmOrientation({:d}".format(offset1)+")"
        return self.sendRecvMsg(string)

    def SetPayload(self, weight, inertia):
        string = "SetPayLoad({:f},{:f})".format(weight, inertia)
        return self.sendRecvMsg(string)

    def PositiveSolution(self,offset1,offset2,offset3,offset4,user,tool):   
        string = "PositiveSolution({:f},{:f},{:f},{:f},{:d},{:d}".format(offset1,offset2,offset3,offset4,user,tool)+")"
        return self.sendRecvMsg(string)

    def InverseSolution(self,offset1,offset2,offset3,offset4,user,tool,*dynParams):       
        string = "InverseSolution({:f},{:f},{:f},{:f},{:d},{:d}".format(offset1,offset2,offset3,offset4,user,tool)
        for params in dynParams:
            print(type(params), params)
            string = string + repr(params)
        string = string + ")"
        return self.sendRecvMsg(string)     

    def SetCollisionLevel(self,offset1):
        string = "SetCollisionLevel({:d}".format(offset1)+")"
        return self.sendRecvMsg(string)

    def SetSafeSkin(self, status):
        """Enable (1) or disable (0) the legacy V3 SafeSkin function."""
        if status not in (0, 1):
            raise DobotParameterError(
                f'SafeSkin status must be 0 or 1, got {status}'
            )
        string = "SetSafeSkin({:d})".format(status)
        return self.sendRecvMsg(string)

    def  GetAngle(self):
        string = "GetAngle()"
        return self.sendRecvMsg(string)

    def  GetPose(self,User=0,Tool=0):
        string = "GetPose(User={:d},Tool={:d})".format(User,Tool)
        return self.sendRecvMsg(string)
    
    def EmergencyStop(self):
        string = "EmergencyStop()"
        return self.sendRecvMsg(string)


    def ModbusCreate(self,ip,port,slave_id,isRTU):
        string ="ModbusCreate({:s},{:d},{:d},{:d}".format(ip,port,slave_id,isRTU)+")"
        return self.sendRecvMsg(string)
    
    def ModbusClose(self,offset1):
        string = "ModbusClose({:d}".format(offset1)+")"
        return self.sendRecvMsg(string)

    def GetInBits(self,offset1,offset2,offset3):
        string = "GetInBits({:d},{:d},{:d}".format(offset1,offset2,offset3)+")"
        return self.sendRecvMsg(string)        

    def GetInRegs(self,offset1,offset2,offset3,*dynParams):
        string = "GetInRegs({:d},{:d},{:d}".format(offset1,offset2,offset3)
        for params in dynParams:
            print(type(params), params)
            string = string + params[0]
        string = string + ")"
        return self.sendRecvMsg(string)  

    def GetCoils(self,offset1,offset2,offset3):
        string = "GetCoils({:d},{:d},{:d}".format(offset1,offset2,offset3)+")"
        return self.sendRecvMsg(string)          

    def SetCoils(self,offset1,offset2,offset3,offset4):
        string = "SetCoils({:d},{:d},{:d}".format(offset1,offset2,offset3)+","+ repr(offset4)+")"
        print(str(offset4))
        return self.sendRecvMsg(string)              

    def DI(self,offset1):
        string = "DI({:d}".format(offset1)+")"
        return self.sendRecvMsg(string)        

    def ToolDI(self,offset1):
        string = "ToolDI({:d}".format(offset1)+")"
        return self.sendRecvMsg(string)   

    def DOGroup(self,*dynParams):
        string = "DOGroup("
        for params in dynParams[0]:
            string = string + str(params)+","
        string =string+ ")"   
        return self.wait_reply()  

    def BrakeControl(self,offset1,offset2): 
        string = "BrakeControl({:d},{:d}".format(offset1,offset2)+")"
        return self.sendRecvMsg(string)             

    def StartDrag(self):
        string = "StartDrag()"
        return self.sendRecvMsg(string)      

    def StopDrag(self):
        string = "StopDrag()"
        return self.sendRecvMsg(string)           

    def LoadSwitch(self,offset1):    
        string = "LoadSwitch({:d}".format(offset1)+")"
        return self.sendRecvMsg(string)                                                       

    def wait(self):
        string = "wait()"
        return self.sendRecvMsg(string)

    def pause(self):
        string = "pause()"
        return self.sendRecvMsg(string)

    def Continue(self):
        string = "continue()"
        return self.sendRecvMsg(string)
    
class DobotApiMove(DobotApi):
    """
    Define class dobot_api_move to establish a connection to Dobot
    """

    def MovJ(self, x, y, z, rx,ry,rz,*dynParams):
        """
        Joint motion interface (point-to-point motion mode)
        x: A number in the Cartesian coordinate system x
        y: A number in the Cartesian coordinate system y
        z: A number in the Cartesian coordinate system z
        r: A number in the Cartesian coordinate system R
        """
        string = "MovJ({:f},{:f},{:f},{:f},{:f},{:f}".format(
            x, y, z, rx,ry,rz)
        for params in dynParams[0]:
             string =string+ ","+ str(params)
        string =string+ ")" 
        print(string)  
        return self.sendRecvMsg(string)

    def MovL(self, x, y, z, rx,ry,rz,*dynParams):
        """
        Coordinate system motion interface (linear motion mode)
        x: A number in the Cartesian coordinate system x
        y: A number in the Cartesian coordinate system y
        z: A number in the Cartesian coordinate system z
        r: A number in the Cartesian coordinate system R
        """
        string = "MovL({:f},{:f},{:f},{:f},{:f},{:f}".format(
            x, y, z, rx,ry,rz)
        for params in dynParams[0]:
             string =string+ ","+ str(params)
        string =string+ ")" 
        print(string) 
        return self.sendRecvMsg(string)

    def JointMovJ(self, j1, j2, j3, j4,j5,j6,*dynParams):
        """
        Joint motion interface (linear motion mode)
        j1~j6:Point position values on each joint
        """
        string = "JointMovJ({:f},{:f},{:f},{:f},{:f},{:f}".format(
            j1, j2, j3, j4,j5,j6)
        for params in dynParams[0]:
            string =string+ ","+ str(params)
        string =string+ ")" 
        print(string)
        return self.sendRecvMsg(string)
    
    def ServoJ(self, j1, j2, j3, j4,j5,j6,t,*dynParams):
        string = "ServoJ({:f},{:f},{:f},{:f},{:f},{:f},t={:f}".format(
            j1,j2,j3,j4,j5,j6,t)
        for params in dynParams[0]:
             string =string+ ","+ str(params)
        string =string+ ")" 
        print(string) 
        return self.sendRecvMsg(string)

    def ServoP(self, x, y, z, rx,ry,rz):
        string = "ServoP({:f},{:f},{:f},{:f},{:f},{:f})".format(
            x, y, z, rx,ry,rz)
        print(string) 
        return self.sendRecvMsg(string)

    def Jump(self):
        print("待定")

    def RelMovJ(self, offset1, offset2, offset3,offset4, offset5,offset6,*dynParams):
        """
        Offset motion interface (point-to-point motion mode)
        j1~j6:Point position values on each joint
        """
        string = "RelMovJ({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset1, offset2, offset3,offset4, offset5,offset6)
        for params in dynParams[0]:
            string =string+ ","+ str(params)
        string =string+ ")" 
        return self.sendRecvMsg(string)

    def RelMovL(self, offset1, offset2, offset3,offset4, offset5,offset6,*dynParams):
        """
        Offset motion interface (point-to-point motion mode)
        x: Offset in the Cartesian coordinate system x
        y: offset in the Cartesian coordinate system y
        z: Offset in the Cartesian coordinate system Z
        r: Offset in the Cartesian coordinate system R
        """
        string = "RelMovL({:f},{:f},{:f},{:f},{:f},{:f}".format(offset1, offset2, offset3,offset4, offset5,offset6)
        for params in dynParams[0]:
            string =string+ ","+ str(params)
        string =string+ ")" 
        return self.sendRecvMsg(string)

    def MovLIO(self, x, y, z, rx,ry,rz, *dynParams):
        # example： MovLIO(0,50,0,0,0,0,(0,50,1,0),(1,1,2,1))
        string = "MovLIO({:f},{:f},{:f},{:f},{:f},{:f}".format(
            x, y, z, rx,ry,rz)
        for params in dynParams[0]:
            string =string+ ","+ str(params)
        string =string+ ")" 
        return self.sendRecvMsg(string)

    def MovJIO(self, x, y, z, rx,ry,rz, *dynParams):
        # example： MovJIO(0,50,0,0,0,0,(0,50,1,0),(1,1,2,1))
        string = "MovJIO({:f},{:f},{:f},{:f},{:f},{:f}".format(
            x, y, z, rx,ry,rz)
        for params in dynParams[0]:
            string =string+ ","+ str(params)
        string =string+ ")" 
        print(string)
        return self.sendRecvMsg(string)

    def Arc(self, x1, y1, z1, rx1,ry1,rz1,x2, y2, z2, rx2,ry2,rz2,*dynParams):
        """
        Circular motion instruction
        x1, y1, z1, r1 :Is the point value of intermediate point coordinates
        x2, y2, z2, r2 :Is the value of the end point coordinates
        Note: This instruction should be used together with other movement instructions
        """
        string = "Arc({:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f}".format(
            x1, y1, z1, rx1,ry1,rz1,x2, y2, z2, rx2,ry2,rz2)
        for params in dynParams[0]:
            string =string+ ","+ str(params)
        string =string+ ")" 
        print(string)
        return self.sendRecvMsg(string)

    def Circle(self, x1, y1, z1, rx1,ry1,rz1,x2, y2, z2, rx2,ry2,rz2,count,*dynParams):
        """
        Full circle motion command
        count：Run laps
        x1, y1, z1, r1 :Is the point value of intermediate point coordinates
        x2, y2, z2, r2 :Is the value of the end point coordinates
        Note: This instruction should be used together with other movement instructions
        """
        string = "Circle({:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:d}".format(
             x1, y1, z1, rx1,ry1,rz1,x2, y2, z2, rx2,ry2,rz2, count)
        for params in dynParams:
            string = string + ","+ str(params)
        string = string + ")" 
        return self.sendRecvMsg(string)

    def MoveJog(self, axis_id=None, *dynParams):
        """
        Joint motion
        axis_id: Joint motion axis, optional string value:
            J1+ J2+ J3+ J4+ J5+ J6+
            J1- J2- J3- J4- J5- J6- 
            X+ Y+ Z+ Rx+ Ry+ Rz+ 
            X- Y- Z- Rx- Ry- Rz-
        *dynParams: Parameter Settings（coord_type, user_index, tool_index）
                    coord_type: 1: User coordinate 2: tool coordinate (default value is 1)
                    user_index: user index is 0 ~ 9 (default value is 0)
                    tool_index: tool index is 0 ~ 9 (default value is 0)
        """
        if axis_id is not None:
          string = "MoveJog({:s}".format(axis_id)
        else:
          string = "MoveJog("
        for params in dynParams[0]:
            string = string + ","+ str(params)
        string = string + ")" 
        return self.sendRecvMsg(string)


    def Sync(self):
        """
        The blocking program executes the queue instruction and returns after all the queue instructions are executed
        """
        string = "Sync()"
        return self.sendRecvMsg(string)

    def RelMovJUser(self, offset_1, offset_2, offset_3, offset_4, offset_5, offset_6, user, *dynParams):
        string = "RelMovJUser({:f},{:f},{:f},{:f},{:f},{:f}, {:d}".format(
            offset_1, offset_2, offset_3, offset_4, offset_5, offset_6, user)
        for params in dynParams[0]:
            string = string + ","+ str(params)
        string = string + ")"
        return self.sendRecvMsg(string)
    
    def RelMovJTool(self, offset_1, offset_2, offset_3, offset_4, offset_5, offset_6, tool, *dynParams):
        string = "RelMovJTool({:f},{:f},{:f},{:f},{:f},{:f}, {:d}".format(
            offset_1, offset_2, offset_3, offset_4, offset_5, offset_6, tool)
        for params in dynParams[0]:
            string = string + ","+ str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def RelMovLUser(self, offset_1, offset_2, offset_3, offset_4, offset_5, offset_6, user, *dynParams):
        string = "RelMovLUser({:f},{:f},{:f},{:f},{:f},{:f}, {:d}".format(
            offset_1, offset_2, offset_3, offset_4, offset_5, offset_6, user)
        for params in dynParams[0]:
            string = string + ","+ str(params)
        string = string + ")"
        return self.sendRecvMsg(string)
    
    def RelMovLTool(self, offset_1, offset_2, offset_3, offset_4, offset_5, offset_6, tool, *dynParams):
        string = "RelMovLTool({:f},{:f},{:f},{:f},{:f},{:f}, {:d}".format(
            offset_1, offset_2, offset_3, offset_4, offset_5, offset_6, tool)
        for params in dynParams[0]:
            string = string + ","+ str(params)
        string = string + ")"
        return self.sendRecvMsg(string)
    

    def RelJointMovJ(self, offset1, offset2, offset3, offset4,offset5, offset6, *dynParams):
        """
        The relative motion command is carried out along the joint coordinate system of each axis, and the end motion mode is joint motion
        Offset motion interface (point-to-point motion mode)
        j1~j6:Point position values on each joint
        *dynParams: parameter Settings（speed_j, acc_j, user）
                    speed_j: Set Cartesian speed scale, value range: 1 ~ 100
                    acc_j: Set acceleration scale value, value range: 1 ~ 100
        """
        string = "RelJointMovJ({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset1, offset2, offset3, offset4,offset5, offset6)
        for params in dynParams:
           string = string + ","+ str(params)
        string = string + ")"
        return self.sendRecvMsg(string)
    
    def MovJExt(self, offset1, *dynParams):
        string = "MovJExt({:f}".format(
            offset1)
        for params in dynParams[0]:
           string = string + ","+ str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def SyncAll(self):
        string = "SyncAll()"
        return self.sendRecvMsg(string)

    def StartTrace(self, traceName):
        """
        Track fitting motion: Fit a motion path using the recorded points (at least 4 points) from the specified trajectory file,
        then the robot moves along that path.
        Before calling this command, use other motion commands to move the robot to the first point of the trajectory.
        
        Track file location: /dobot/userdata/project/process/trajectory/
        
        traceName: Trajectory file name (including suffix), must not be empty
        """
        if not traceName or not traceName.strip():
            raise ValueError("traceName must not be empty")
        string = "StartTrace({:s})".format(traceName)
        return self.sendRecvMsg(string)

    def StartPath(self, traceName, const_val, cart):
        """
        Track playback motion: Reproduce the recorded motion trajectory based on the specified trajectory file (at least 4 points).
        Before calling this command, use other motion commands to move the robot to the first point of the trajectory.
        
        Track file location: /dobot/userdata/project/process/trajectory/
        
        traceName: Trajectory file name (including suffix), must not be empty
        const_val: Whether to playback at constant speed.
                   1 - Constant speed playback (removes pauses in the trajectory)
                   0 - Playback at original speed
        cart: Playback path type.
              1 - Cartesian path playback
              0 - Joint path playback
        """
        if not traceName or not traceName.strip():
            raise ValueError("traceName must not be empty")
        if const_val not in (0, 1):
            raise ValueError("const_val must be 0 or 1")
        if cart not in (0, 1):
            raise ValueError("cart must be 0 or 1")
        string = "StartPath({:s},{:d},{:d})".format(traceName, const_val, cart)
        return self.sendRecvMsg(string)
