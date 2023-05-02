from math import pi, sin, cos, sqrt
from panda3d.core import *

# my external files..
from shader import *
from stdfunction import *
from SpikePrime import *

class Wall:
    def __init__(self, x0, y0, dx, dy):
        self.x0 = x0 
        self.y0 = y0 
        self.l = sqrt(dx*dx+dy*dy)
        self.dx = dx / self.l
        self.dy = dy / self.l
        self.K = 1000.0

    def collideWith(self,intoObject):
        Lx =  intoObject.Px - self.x0
        Ly =  intoObject.Py - self.y0
        d =  self.dx * Ly - self.dy*Lx # othogonal distance

        K = (intoObject.K+self.K)/2.0
        r0 = intoObject.radius
        if (d<r0):
            AbsFx = - K*(r0-d)*self.dy
            AbsFy = K*(r0-d)*self.dx
            intoObject.addForce(AbsFx,AbsFy,0.0)     

class WallSegment(Wall):
    def __init__(self, x0, y0, dx, dy):
        Wall.__init__(self, x0, y0, dx , dy)
        self.K = 1000.0
        self.x1 = x0+dx
        self.y1 = y0+dy

    def collideWith(self,intoObject):
        Lx =  intoObject.Px - self.x0
        Ly =  intoObject.Py - self.y0
        d =  self.dx * Ly - self.dy*Lx # othogonal distance
        l = Lx*self.dx+Ly*self.dy

        Ax, Ay = self.x0, self.y0
        Lx0 =  intoObject.Px - Ax
        Ly0 =  intoObject.Py - Ay
        dX0 =  sqrt(Ly0 * Ly0 + Lx0*Lx0) # distance from x0
        Bx, By = self.x1, self.y1 
        Lx1 =  intoObject.Px - Bx
        Ly1 =  intoObject.Py - By
        dX1 =  sqrt(Ly1 * Ly1 + Lx1*Lx1) # distance from x1

        K = (intoObject.K+self.K)/2.0
        r0 = intoObject.radius
        if (l>0) and (l< self.l):
            if (abs(d)<r0):
                AbsFx = - K*(r0-abs(d))*self.dy * sign(d)
                AbsFy = K*(r0-abs(d))*self.dx * sign(d)
                intoObject.addForce(AbsFx,AbsFy,0.0)     
        if (l<=0): # bump on edge 
            if (dX0<r0):
                AbsFx = K*(r0-dX0)*Lx0/dX0
                AbsFy = K*(r0-dX0)*Ly0/dX0
                intoObject.addForce(AbsFx,AbsFy,0.0)     
        if (l>=self.l): # bump on edge 
            if (dX1<r0):
                AbsFx = K*(r0-dX1)*Lx1/dX1
                AbsFy = K*(r0-dX1)*Ly1/dX1
                intoObject.addForce(AbsFx,AbsFy,0.0)

class Goal:
    def __init__(self, x0, y0, x1, y1):
        goaldir = sign(x1-x0)
        self.wall1 = WallSegment(x0, y0, 0 , y1-y0)
        self.wall2 = WallSegment(x0-0.25*goaldir, y0, x1-x0+0.25*goaldir, 0 )
        self.wall3 = WallSegment(x0-0.25*goaldir, y1, x1-x0+0.25*goaldir, 0 )
        self.K = 1000.0
        self.x0, self.y0 = x0, y0
        self.x1, self.y1 = x1, y1

    def collideWith(self,intoObject):
        self.wall1.collideWith(intoObject)
        self.wall2.collideWith(intoObject)
        self.wall3.collideWith(intoObject)

    def isInside(self, object):
        if ((object.Px >= min(self.x0,self.x1)) and
            (object.Px <= max(self.x0,self.x1)) and
            (object.Py >= min(self.y0,self.y1)) and
            (object.Py <= max(self.y0,self.y1))) :
            return True
        return False

class KineticObject:
    def __init__(self, imass, isize):

        # dimensions (MKS)
        self.mass = imass
        self.diametre = isize

        self.Ic = 0.5 * self.mass * ( self.diametre / 2.0 ) * ( self.diametre / 2.0 )

        # kinematic
        self.APz = 0.0 # orientation
        self.AVz = 0.0 # angular velocit
        self.AAz = 0.0 # angular acceleration

        self.Px, self.Py, self.Pz = 0.0, 0.0, 0.0
        self.Vx ,self.Vy ,self.Vz = 0.0, 0.0, 0.0
        self.Ax ,self.Ay ,self.Az = 0.0, 0.0, 0.0

        # external Force & Torque
        self.Fx, self.Fy , self.Fz = 0.0, 0.0, 0.0

        self.Tz = 0.0

    def applyTorque(self, torque):
        self.Tz = torque

    def addTorque(self, torque):
        self.Tz += torque

    def resetTorque(self):
        self.Tz = 0.0

    def addForce(self, fx, fy, fz):
        self.Fx += fx
        self.Fy += fy
        self.Fz += fz

    def resetForce(self):
        self.Fx = 0.0
        self.Fy = 0.0
        self.Fz = 0.0

    # update velocities and position given an acceleration
    def update(self,dt):
        V0x = self.Vx
        V0y = self.Vy
        V0z = self.Vz
        AV0z = self.AVz

        # angular velocity 
        self.AVz = AV0z + dt * self.AAz
        # rotation
        self.APz = self.APz + dt * AV0z + 0.5 * self.AAz * dt * dt

        # velocity 
        self.Vx = V0x + dt * self.Ax
        self.Vy = V0y + dt * self.Ay
        self.Vz = V0z + dt * self.Az
        # position
        self.Px = self.Px + dt * V0x + 0.5 * self.Ax * dt * dt
        self.Py = self.Py + dt * V0y + 0.5 * self.Ay * dt * dt
        self.Pz = self.Pz + dt * V0z + 0.5 * self.Az * dt * dt

    def reset(self):
        self.Px = 0.0
        self.Py = 0.0
        self.Ax = 0.0
        self.Ay = 0.0
        self.Vx = 0.0
        self.Vy = 0.0
        self.Fx = 0.0
        self.Fy = 0.0

class KineticCylinder(KineticObject):
    def __init__(self, imass, isize):
        KineticObject.__init__(self, imass, isize)
        # dimensions (MKS)
        self.mass = imass
        self.diametre = isize
        self.Ic = 0.5 * self.mass * ( self.diametre / 2.0 ) * ( self.diametre / 2.0 )

class Ball(KineticObject):
    def __init__(self, x0, y0):
        self.radius = 3.5/100.0
        KineticObject.__init__(self, 0.125, self.radius*2.0)
        self.Px = x0 
        self.Py = y0 
        self.Pz = 0.0
        self.drag = 0.2 
        self.K = 100.0
        self.n = 1
        self.AvgPx = 0.0
        self.AvgPy = 0.0
        self.AvgPx2 = 0.5
        self.AvgPy2 = 0.5

    def collideWith(self,intoRobot):
        Dx = self.Px - intoRobot.Px
        Dy = self.Py - intoRobot.Py
        Dz = self.Pz - intoRobot.Pz
        K = (self.K+intoRobot.K)/2.0
        r0 = (self.radius + intoRobot.radius)
        d = sqrt(Dx*Dx+Dy*Dy+Dz*Dz)
        if (d<r0):
            dx = Dx/d 
            dy = Dy/d 
            dz = Dz/d 
            AbsFx = K*(r0-d)*dx
            AbsFy = K*(r0-d)*dy
            AbsFz = K*(r0-d)*dz
            self.addForce(AbsFx/2.0,AbsFy/2.0,AbsFz/2.0)
            intoRobot.addForce(-AbsFx/2.0,-AbsFy/2.0,-AbsFz/2.0)

    def setPos(self, x, y, z=0.0):
        self.Px = x/100
        self.Py = y/100
        self.Pz = z/100
        self.node.setPos(x,y,z+5.0)

    #set initial position in cm
    def updatePos(self):
        self.setPos(self.Px * 100.0,self.Py * 100.0, -0.25*self.radius*100.0) 

    def update(self,dt):
        # F = mA
        Fdragx = - self.drag * self.Vx
        Fdragy = - self.drag * self.Vy
        Fdragz = - self.drag * self.Vz
        self.addForce(Fdragx,Fdragy,Fdragz)

        self.Ax = self.Fx / self.mass
        self.Ay = self.Fy / self.mass
        self.Az = self.Fz / self.mass
        KineticObject.update(self,dt)
        self.autoReset()

    def autoReset(self):
        reset = False
        ep=0.005
        weight = 100.0

        self.AvgPx = (self.AvgPx*(weight-1.0) + self.Px)/weight
        self.AvgPy = (self.AvgPy*(weight-1.0) + self.Py)/weight

        weight2 = 5000.0

        self.AvgPx2 = (self.AvgPx2*(weight2-1.0) + self.Px)/weight2
        self.AvgPy2 = (self.AvgPy2*(weight2-1.0) + self.Py)/weight2

        # lack of progress
        if abs(self.AvgPx2 - self.Px) < ep and abs(self.AvgPy2 - self.Py) < ep:
            reset = True

        # on edge 
        if self.AvgPx - self.radius < -2.43/2.+ep:
            reset = True
        if self.AvgPx + self.radius > 2.43/2.-ep:
            reset = True
        if self.AvgPy - self.radius < -1.82/2.+ep:
            reset = True
        if self.AvgPy + self.radius > 1.82/2.-ep:
            reset = True
        if reset:
            self.reset(sign(self.Py))

    def reset(self,loc):
        KineticObject.reset(self)
        self.Py = loc*2.43/8

class Wheel(KineticCylinder):
    def __init__(self,radius):
        KineticCylinder.__init__(self, 0.05, radius*2.0)
        self.friction_s = 0.5 # + random.randrange(-50, 50)/100*0.1
        self.friction_d = self.friction_s / 2.0 # once slip lost traction
        self.drag = 0.005 
        
    def update(self,dt):
        r = self.diametre/2.0 
        m = ( - self.Fz ) / 9.81
        P = self.Fx # pull force from robot
        AV0 = self.AVz

        Vc = r * self.AVz  +  self.Vx  # slipping velocity

        #print("rpm=",self.AVz/3.14/2.0*60.0)

        Ffmax = abs(self.friction_s * self.Fz)
        Ffmax_d = abs(self.friction_d * self.Fz)

        # should use mid-step velocity (AV1+AV0)/2
        Tdrag = - self.drag * AV0
        
        slipping =  abs(self.Tz/r + Tdrag/r - P) > abs(Ffmax)  

        # sign if Ffmax? Ffmax should opose P (friction is taking power awway)
        Ffmax_d = Ffmax_d * sign(self.Tz/r + Tdrag/r - P)
        Ffmax = Ffmax * sign(self.Tz/r + Tdrag/r - P)


        if not(slipping):
            d = r*m + self.Ic/r 
            Ax = ( self.Tz + Tdrag) / d  # => will be converted to wheel rotation if no slip

            self.AAz = Ax / r
            AV1 = AV0 + self.AAz * dt
            Axmax = (Ffmax + P ) / m   
            Axmax_d = (Ffmax_d + P ) / m   # => Axmax_d is what is transimitted to robot if AX > Axmax (slipping)

            if abs(Ax)>=abs(Axmax):   # slipping @ end of this time step
                slipping = True

        if slipping:
            Ax = (  Ffmax_d ) / m
            self.AAz =  (self.Tz + Tdrag) / self.Ic    

            Axmax_d = Ax
        
        KineticCylinder.update(self,dt)

        # set force transmitted to robot
        if slipping or (abs(Ax)>=abs(Axmax)):   # slipping
            self.FXx = m * Axmax_d
        else:
            self.FXx = m * Ax    
            self.AVz = self.Vx / r # no slip condition

class Robot(KineticCylinder):
    def __init__(self,name):
        self.wheelbase = 10.0 / 100.0 # (8cm + thickness wheel)
        KineticCylinder.__init__(self, 0.9, self.wheelbase) 
        # dimensions (MKS)
        self.wheelRadiun = 5.0 / 100.0  # (5cm)

        self.wheels = [ Wheel(self.wheelRadiun),Wheel(self.wheelRadiun),Wheel(self.wheelRadiun),Wheel(self.wheelRadiun)]
        self.robot_dist = 0.0
        self.wheel_dist = 0.0
        self.radius = 10.0 / 100.0
        self.name = name
        self.K = 1000.0
        self.hub = Hub(self)

    def moveMotor(self, id, torque):
        self.wheels[id-1].applyTorque(torque)

    def update(self,dt):
        F = [0.0,0.0,0.0,0.0]

        #      /|\
        #       |y
        #
        #    3/---\2            \x+ (wheel)
        #     |   |  --> x     --\  
        #    1\---/4             |\
        #
        # wheel orientation relative to robot (eg. wheel 2= -45 degrees rotated etc...)
        c = [+1/sqrt(2.0),-1/sqrt(2.0),-1/sqrt(2.0),+1/sqrt(2.0)]
        s = [-1/sqrt(2.0),+1/sqrt(2.0),-1/sqrt(2.0),+1/sqrt(2.0)]
        # relative velicity and accelation, transposed "back" to robot orientation
        RAx = self.Ax * cos(-self.APz) + self.Ay * sin(-self.APz)
        RAy = self.Ay * cos(-self.APz) - self.Ax * sin(-self.APz)  
        RVx = self.Vx * cos(-self.APz) + self.Vy * sin(-self.APz)
        RVy = self.Vy * cos(-self.APz) - self.Vx * sin(-self.APz)  
        R = self.wheelbase / 2.0

        for i in range(0,4):
            self.wheels[i].resetForce()    
            # Apply force on wheel ( mass is 1/4d)
            self.wheels[i].addForce( - self.Ic*self.AAz/R/4.0 + self.mass/4.0 * (RAx * c[i] + RAy *s[i] ),0,-self.mass*9.81/4.0) # apply weight of robot on wheel
            # initial velocity of wheel
            self.wheels[i].Vx = -R*self.AVz + (RVx * c[i] + RVy *s[i] )
            self.wheels[i].update(dt)    
            F[i] = self.wheels[i].FXx   # driving force from wheels
            self.Tz -= R * F[i]  
        RFx = (F[0]-F[1]-F[2]+F[3]) / sqrt(2.0) * 2.0 
        RFy = (-F[0]+F[1]-F[2]+F[3]) / sqrt(2.0) * 2.0         
        AbsFx = RFx * cos(self.APz) + RFy * sin(self.APz)
        AbsFy = RFy * cos(self.APz) - RFx * sin(self.APz)
        self.addForce(AbsFx,AbsFy,-self.mass*9.81)

        self.Ax = self.Fx / self.mass
        self.Ay = self.Fy / self.mass
        self.Az = self.Fz / self.mass
        
        self.AAz = self.Tz / self.Ic 
        KineticCylinder.update(self,dt)

    #set initial position in cm
    def setPos(self, x, y, z=0.0):
        self.Px = x/100
        self.Py = y/100
        self.Pz = z/100
        self.node.setPos(x,y,z+5.0)

    def setAzimuth(self, a=0.0):
        self.APz = a*pi/180.0
        self.updatePos()

    def collideWith(self,intoRobot):
        Dx = self.Px - intoRobot.Px
        Dy = self.Py - intoRobot.Py
        Dz = self.Pz - intoRobot.Pz
        K = (self.K+intoRobot.K)/2.0
        r0 = (self.radius + intoRobot.radius)
        d = sqrt(Dx*Dx+Dy*Dy+Dz*Dz)
        if (d<r0):
            dx = Dx/d 
            dy = Dy/d 
            dz = Dz/d 
            AbsFx = K*(r0-d)*dx
            AbsFy = K*(r0-d)*dy
            AbsFz = K*(r0-d)*dz
            self.addForce(AbsFx/2.0,AbsFy/2.0,AbsFz/2.0)
            intoRobot.addForce(-AbsFx/2.0,-AbsFy/2.0,-AbsFz/2.0)

            vx = intoRobot.Ax*intoRobot.mass-self.Ax*self.mass
            vy = intoRobot.Ay*intoRobot.mass-self.Ay*self.mass
            v = sqrt(vx*vx+vy*vy)
            if (v>0.0):
                Fz = (vy*dx - vx*dy)*0.25
                self.addTorque(-Fz/2*self.radius)
                intoRobot.addTorque(Fz/2*intoRobot.radius)

    #set initial position in cm
    def updatePos(self):
        self.node.setR(self.APz / pi * 180.0) 
        self.setPos(self.Px * 100.0,self.Py * 100.0) 
        for i in range(0,4):
            self.wheels[i].node.setR(self.wheels[i].APz / pi * 180.0) 
