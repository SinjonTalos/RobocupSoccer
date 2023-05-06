from math import pi, sin, cos, sqrt
import time
import math, random

class Device:
    def __init__(self,robot):
        self.robot = robot

class Motor(Device):
    def __init__(self,robot,id):
        Device.__init__(self, robot)
        self.id = id # the id for the robot
        self.speed = 0

    def set_default_speed(self,percent):
        self.speed = 0.05*percent/100.0  #5N.cm @ 100%

    def start(self,speed = -1000):
        self.robot.hub.sleep()
        if speed > -1000:
            self.set_default_speed(speed) 
        self.robot.moveMotor(self.id,self.speed)  #5N.cm @ 100%

    def stop(self):
        self.robot.hub.sleep()
        self.robot.moveMotor(self.id,0)

# MotionSensor class for Spike Prime robot 
class MotionSensor:
    def __init__(self,robot):
        self.robot = robot
        self.Yaw = 0.0

    # yaw angle in degrees
    def getYaw(self) -> float:
        self.robot.hub.sleep()
        return  -(self.robot.APz / pi * 180 ) #% 360.0)
    
    # reset yaw angle to zero
    def reset_yaw_angle(self):
        self.robot.hub.sleep()
        self.Yaw = self.getYaw()

    # yaw angle in degrees since last reset of yaw angle
    def get_yaw_angle(self) -> float:
        return self.getYaw() - self.Yaw

#class to emulate disctance sensor for Spike Prime robot
class DistanceSensor(Device):
    def __init__(self,robot):
        Device.__init__(self, robot)
        self.distance = 0.0

    def getDistance(self) -> float:
        self.robot.hub.sleep()
        return self.distance

# IRSeeker class for Spike Prime robot
class IRSeeker(Device):
    def __init__(self,robot):
        Device.__init__(self, robot)
        self.ball = None  # the ball the seeker is tracking

    # seek the ball
    def seekBall(self,ball):
        self.ball = ball       

    def mode(self,a,b):
        return

    # return the direction to the ball    
    def getSector(self) -> int:
        self.robot.hub.sleep()
        dx = self.ball.Px - self.robot.Px
        dy = self.ball.Py  - self.robot.Py
        angle = math.degrees(math.atan2(float(dy),float(dx))) + 90.0 + self.robot.APz*180.0/pi #+ random.randrange(-100, 100)*30/100
        angle += random.randrange(-50, 50)/100*50.0
        if angle < 0:
            angle += 360
        return int(angle/30+0.5)

    # return the strength of the signal
    def getStrength(self) -> float:
        self.robot.hub.sleep()
        dx = self.ball.Px - self.robot.Px
        dy = self.ball.Py  - self.robot.Py
        r0 = self.ball.radius + self.robot.radius
        d = sqrt(dx*dx+dy*dy)
        d += random.randrange(-50, 50)/100*0.05  # 5cm randomness
        # r0+0.3 => str = 50   (stremp 50% at 30cm)
        # r0     => str = 100
        str30cm = 50.0
        hd = 0.3
        maxStr = 100.0
        if d < r0+hd:
            str = min ( (d-r0-hd)*maxStr/(-hd) + (d-r0)*str30cm/hd , maxStr )
        else:
            str = ( hd / (d-r0) ) * str30cm
        return str
    
    # return the sector and strength of the signal
    def get(self):
        return (self.getSector(),0,self.getStrength())

class Port:
    def __init__(self,hub,portName):
        self.hub = hub
        self.portName = portName
        self.device = None

    # connect the device to the port
    def connect(self, device):
        self.device = device

class Ports:
    def __init__(self,hub):
        self.hub = hub
        self.A = Port(hub,"A")
        self.B = Port(hub,"B")
        self.C = Port(hub,"C")
        self.D = Port(hub,"D")
        self.E = Port(hub,"E")
        self.F = Port(hub,"F")

class DummyDevice:
    def is_pressed(self):
        return False
    def on(self,str):
        return
    def beep(self):
        return
    def off(self):
        return
    
class Hub:
    def __init__(self,robot):
        self.port = Ports(self)

        #   A3/---\2B           \x+ (wheel)
        #     |   |  --> x     --\  
        #   C1\---/4D            |\

        self.port.A.connect(Motor(robot,3))
        self.port.B.connect(Motor(robot,2))
        self.port.C.connect(Motor(robot,1))
        self.port.D.connect(Motor(robot,4))

        self.port.E.connect(IRSeeker(robot))
        self.port.F.connect(DistanceSensor(robot))

        self.motion_sensor = MotionSensor(robot)

        self.speaker = DummyDevice()
        self.light_matrix = DummyDevice()
        self.left_button = DummyDevice()
        self.status_light = DummyDevice()

        self.clock =  500 # specify clock on Hz

    # sleep for 1/clock seconds
    def sleep(self):
        time.sleep(1/self.clock)

    def getIRSeeker(self):
        return self.port.E.device

    # get motor connected to port str
    def Motor(self, str) -> Motor:
        if str=="A":
            return self.port.A.device
        if str=="B":
            return self.port.B.device
        if str=="C":
            return self.port.C.device
        if str=="D":
            return self.port.D.device
        if str=="E":
            return self.port.C.device
        if str=="F":
            return self.port.D.device

