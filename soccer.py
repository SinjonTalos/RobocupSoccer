#!/usr/bin/env python

from direct.task import Task
from panda3d.core import *
from direct.showbase.DirectObject import DirectObject

# my external files..
from shader import *
from stdfunction import *
from RobotProgram import ThreadRobot
from RobotKinetic import *
from SpikePrime import *
import random
import signal
import os

load_prc_file_data("", """
framebuffer-srgb #t
default-fov 45
gl-version 3 2
""")

from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenText import OnscreenText

# can be 2 or 4
NUMBER_ROBOT=4
REALTIME=True
DT=0.01  #ignored if real time

class World(DirectObject, myShader):
    def __init__(self,base):
        myShader.__init__(self)
        self.title = OnscreenText(  # Create the title
            text="Soccer Sim",
            parent=base.a2dBottomRight, align=TextNode.A_right,
            style=1, fg=(0, 0, 0, 1), pos=(-0.1, 0.1), scale=.07)

        # Position the camera.  Set a saner far distance.
        base.trackball.node().set_pos(0, 300, 20)
        base.trackball.node().set_hpr(0, +45, 0)
        base.camLens.set_far(1000)

        #the shaders...
        shader = Shader.make(Shader.SL_GLSL,self.v_shader, self.f_shader)
        render.set_shader(shader)

        # create objects
        self.Robot1TeamA = Robot("TheRobot1TeamA")
        self.Robot1TeamB = Robot("TheRobot1TeamB")
        self.robots = (self.Robot1TeamA,self.Robot1TeamB)
        self.robotsA = (self.Robot1TeamA,)
        self.robotsB = (self.Robot1TeamB,)

        if (NUMBER_ROBOT>2):
            self.Robot2TeamA = Robot("TheRobot2TeamA")
            self.Robot2TeamB = Robot("TheRobot2TeamB")        
            self.robots = (self.Robot1TeamA,self.Robot2TeamA,self.Robot1TeamB,self.Robot2TeamB)
            self.robotsA = (self.Robot1TeamA,self.Robot2TeamA)
            self.robotsB = (self.Robot1TeamB,self.Robot2TeamB)

        
        x0 = -243/2.
        y0 = -182/2.
        self.wallB = Wall(x0/100.0,y0/100.0,2.43,0)
        self.wallT = Wall(x0/100.0+2.43,y0/100.0+1.82,-2.43,0)
        self.wallL = Wall(x0/100.0,y0/100.0+1.82,0,-1.82)
        self.wallR = Wall(x0/100.0+2.43,y0/100.0,0,+1.82)
        self.goalA = Goal((x0+243-25-5.1+7.4)/100.0,
                          (y0+182/2-45/2.)/100.0,
                          (x0+243-25-5.1)/100.0,
                          (y0+182/2+45/2.)/100.0)
        self.goalB = Goal((x0+25-7.4+5)/100.0,
                          (y0+182/2-45/2.)/100.0,
                          (x0+25+5.1)/100.0,
                          (y0+182/2+45/2.)/100.0)

        self.walls = (self.wallB,self.wallT,self.wallL,self.wallR, self.goalA , self.goalB)

        self.ball = Ball(0.05,0.0)

        self.loadObjects(x0,y0)

        self.resetRobots()

        #light
        # Also add an ambient light and set sky color.
        skycol = VBase3(135 / 255.0, 206 / 255.0, 235 / 255.0)
        base.set_background_color(skycol)

        self.my_light = render.attach_new_node(Spotlight("Spot"))
        self.my_light.node().set_shadow_caster(True, 8192, 8192)
        self.my_light.node().set_color((0.9, 0.9, 0.8, 1.0))
        self.my_light.node().get_lens().set_fov(150)
        self.my_light.node().get_lens().set_near_far(0.1, 300)
        self.my_light.set_pos(50, -50, 100)
        self.my_light.look_at(0, -10, 0)

        render.set_shader_input('my_light',self.my_light)

        #seek the ball...
        for rb in self.robots:
            rb.hub.getIRSeeker().seekBall(self.ball)

        self.lasttime = 0.0
        self.it=0
        self.Bscore = 0
        self.Ascore = 0
        taskMgr.add(self.runSim, "RunSimTask")

        # not thread safe, but as long as no Panda3D function is called in the Robot thread...
        # The only variable changed in thread is the torque applied to wheels of robots
        self.Robot1TeamA.thread = ThreadRobot(1,self.Robot1TeamA)
        self.Robot1TeamB.thread = ThreadRobot(3,self.Robot1TeamB)
        self.Robot1TeamA.thread.start()
        self.Robot1TeamB.thread.start()
        if (NUMBER_ROBOT>2):
            self.Robot2TeamA.thread = ThreadRobot(2,self.Robot2TeamA)
            self.Robot2TeamB.thread = ThreadRobot(4,self.Robot2TeamB)
            self.Robot2TeamA.thread.start()
            self.Robot2TeamB.thread.start()

        base.win.setCloseRequestEvent('exit_stage_right')
        self.accept('exit_stage_right',TurnOffTheLightFunction)

        
    def resetRobots(self, str="random"):
        d1 = random.randrange(-50, 50)/100*1.0
        d2 = random.randrange(-50, 50)/100*1.0
        d3 = random.randrange(-50, 50)/100*1.0
        d4 = random.randrange(-50, 50)/100*1.0
        start = str
        if (str=="random"):
            start="B Starts" # not so random...

        if (start=="A Starts"):
            self.Robot1TeamA.setPos(-self.Robot1TeamA.radius*200-self.ball.radius*200,d1)
            self.Robot1TeamA.setAzimuth(-90-d3)

            if (NUMBER_ROBOT>2):
                self.Robot2TeamA.setPos(-243/2.0+60,0.0)
                self.Robot2TeamA.setAzimuth(-90+d3)

                self.Robot1TeamB.setPos(+91.5-30,+10.0)
                self.Robot1TeamB.setAzimuth(90+d4)
                self.Robot2TeamB.setPos(+91.5-30,-10.0)
                self.Robot2TeamB.setAzimuth(90+d2)
            else:                
                self.Robot1TeamB.setPos(+91.5-30,d2)
                self.Robot1TeamB.setAzimuth(90+d1)

        elif (start=="B Starts"):
            self.Robot1TeamB.setPos(self.Robot1TeamB.radius*200+self.ball.radius*200,d1)
            self.Robot1TeamB.setAzimuth(90+d3)

            if (NUMBER_ROBOT>2):
                self.Robot2TeamB.setPos(+243/2.0-60,0.0)
                self.Robot2TeamB.setAzimuth(90+d3)
                self.Robot1TeamA.setPos(-91.5+30,+10.0)
                self.Robot1TeamA.setAzimuth(-90+d4)
                self.Robot2TeamA.setPos(-91.5+30,-10.0)
                self.Robot2TeamA.setAzimuth(-90+d2)
            else:                
                self.Robot1TeamA.setPos(-91.5+30,d2)
                self.Robot1TeamA.setAzimuth(-90+d1)


    def runSim(self, task):

        n = 10
        if REALTIME: # Realtime should be rdt = ~ 1/60s (refresh rate) 
            rdt = max(min(task.time-self.lasttime,0.1),1.0e-6)
        else:
            rdt = DT
            
        self.lasttime = task.time
        self.it = self.it + 1 
        
        #self.Robot1TeamB.moveMotor(2,-0.05)  #5N.cm

        mindt = 0.001 # for stability, mainly the "drag" is unstable is dt > 0.001
        n = max(math.ceil(rdt/mindt),1)
        dt = rdt/n
        for i in range(0,n):
            self.ball.resetForce()
            for rb in self.robots:
                rb.resetForce()
                rb.resetTorque()

            for rb in self.robotsA:
                for rb2 in self.robotsB:
                        rb.collideWith(rb2)
            if (NUMBER_ROBOT>2):
                self.Robot1TeamA.collideWith(self.Robot2TeamA)
                self.Robot1TeamB.collideWith(self.Robot2TeamB)

            for rb in self.robots:
                self.ball.collideWith(rb)
                for wl in self.walls:
                    wl.collideWith(rb)

            for wl in self.walls:
                wl.collideWith(self.ball)

            for rb in self.robots:
                rb.update(dt)
            self.ball.update(dt)

            if self.goalA.isInside(self.ball):
                self.ball.reset(0.0)
                for rb in self.robots:
                    rb.reset()
                self.resetRobots("B Starts")
                self.Bscore = self.Bscore + 1
            if self.goalB.isInside(self.ball):
                self.ball.reset(0.0)
                for rb in self.robots:
                    rb.reset()
                self.resetRobots("A Starts")
                self.Ascore = self.Ascore + 1

            self.title.text = "Score " + str(self.Bscore) + ":" + str(self.Ascore)
            self.updatePositions()

        base.trackball.node().set_pos(0-self.ball.Px*50.0, 180 - self.ball.Py*50.0 , 20 - self.ball.Py*25.0)
        return Task.cont

    def createRobot(self, robot, cNode):

        robot.node.setP(-90)
        body = loader.loadModel("models/Robot")
        body.reparentTo(robot.node)
        if robot.name == "TheRobot1TeamA":
            robot_tex = loader.loadTexture("models/ironB.jpg")
        elif robot.name == "TheRobot2TeamA":
            robot_tex = loader.loadTexture("models/ironB.jpg")
        elif robot.name == "TheRobot1TeamB":
            robot_tex = loader.loadTexture("models/ironR.jpg")
        elif robot.name == "TheRobot2TeamB":
            robot_tex = loader.loadTexture("models/ironR.jpg")
        else:
            robot_tex = loader.loadTexture("models/iron05.jpg")
        body.setTexture(robot_tex, 1)
        body.setPos(0.0,-4.0,0.0)
        body.setScale(1.5,1,2.0)
        body.setColorScale(1.0,0.0,0.0,1.0)
        body.setP(0)

        wheel_tex = loader.loadTexture("models/iron05.jpg")

        for w in robot.wheels:
            w.node = loader.loadModel("models/cylinder")
            w.node.reparentTo(robot.node)
            w.node.setTexture(wheel_tex, 1)
            w.node.setScale(1.0)
            w.node.setH(-90.0)
            w.node.setP(45.0)

        robot.wheels[0].node.setPos(-4.0,0.0,-4.0)
        robot.wheels[0].node.setH(-90.0)
        robot.wheels[0].node.setP(45.0)
        robot.wheels[1].node.setPos(4.0,0.0,4.0)
        robot.wheels[1].node.setH(90.0)
        robot.wheels[1].node.setP(-45.0)
        robot.wheels[2].node.setPos(-4.0,0.0,4.0)
        robot.wheels[2].node.setH(90.0)
        robot.wheels[2].node.setP(45.0+180)
        robot.wheels[3].node.setPos(4.0,0.0,-4.0)
        robot.wheels[3].node.setH(-90.0)
        robot.wheels[3].node.setP(-45.0+180)

    def getRobot(self,robotStr):
        if (self.Robot1TeamA.node.getName() == robotStr):
            return self.Robot1TeamA
        if (self.Robot1TeamB.node.getName() == robotStr):
            return self.Robot1TeamB
        if (self.Robot2TeamA.node.getName() == robotStr):
            return self.Robot2TeamA
        if (self.Robot2TeamB.node.getName() == robotStr):
            return self.Robot2TeamB
        
    def loadObjects(self, x0, y0):
        self.scene = render.attach_new_node("scene")

        cm = CardMaker("plane")
        cm.setFrame(x0, x0+243, y0, y0+182)
        cm.setUvRange((0, 0), (8, 8))
        self.field = render.attachNewNode(cm.generate())
        self.field.setP(-90.)
        self.field.reparentTo(self.scene)
        self.field_tex = loader.loadTexture("models/green.jpg")
        self.field_tex.setWrapU(Texture.WM_repeat)
        self.field_tex.setWrapV(Texture.WM_repeat)
        ts = TextureStage('ts')
        self.field.setTexture(ts, self.field_tex)

        cm = CardMaker("line1")
        cm.setFrame(x0+25,x0+243-25,y0+25,y0+25+5)
        line = render.attachNewNode(cm.generate())
        line.setP(-90.)        
        line.setPos(0,0,0.1)
        line.setColor(1.0,1.0,1.0)        
        line.reparentTo(self.scene)

        cm = CardMaker("line2")
        cm.setFrame(x0+25,x0+243-25,y0+182-5-25,y0+182-25)
        line = render.attachNewNode(cm.generate())
        line.setP(-90.)        
        line.setPos(0,0,0.1)
        line.setColor(1.0,1.0,1.0)        
        line.reparentTo(self.scene)

        cm = CardMaker("line3")
        cm.setFrame(x0+25,x0+25+5,y0+25,y0+182-25)
        line = render.attachNewNode(cm.generate())
        line.setP(-90.)        
        line.setPos(0,0,0.1)
        line.setColor(1.0,1.0,1.0)        
        line.reparentTo(self.scene)

        cm = CardMaker("line4")
        cm.setFrame(x0+243-5-25,x0+243-25,y0+25,y0+182-25)
        line = render.attachNewNode(cm.generate())
        line.setP(-90.)        
        line.setPos(0,0,0.1)
        line.setColor(1.0,1.0,1.0)        
        line.reparentTo(self.scene)

        black_tex = loader.loadTexture("models/black.png")
        black_tex.setWrapU(Texture.WM_repeat)
        black_tex.setWrapV(Texture.WM_repeat)
        tsb = TextureStage('ts')

        cm = CardMaker("goal1")
        cm.setFrame(x0+243-25-5.1,x0+243-25-5.1+7.4,y0+182/2-45/2.,y0+182/2+45/2.)
        line = render.attachNewNode(cm.generate())
        line.setP(-90.)        
        line.setPos(0,0,0.2)
        #line.setTexture(tsb, black_tex)
        line.reparentTo(self.scene)

        cm = CardMaker("goal2")
        cm.setFrame(x0+25-7.4+5,x0+25+5.1,y0+182/2-45/2.,y0+182/2+45/2.)
        line = render.attachNewNode(cm.generate())
        line.setP(-90.)        
        line.setPos(0,0,0.2)
        #line.setTexture(tsb, black_tex)
        line.reparentTo(self.scene)

        for rb in self.robots:
            rb.node = NodePath(rb.name)
            rb.node.reparentTo(self.scene)
            cn = CollisionNode(rb.name)
            self.createRobot(rb,cn)

        self.ball.node = loader.loadModel("models/Sphere")
        self.ball.node.setTexture(tsb, black_tex)
        self.ball.node.reparentTo(self.scene)
        self.ball.node.setScale(0.75)
        #self.ball.node.setPos(0, 0, 7.5/2.0 - 2000.0)

        self.goalA.node = loader.loadModel("models/Goal")
        self.goalA.node.setTexture(tsb, black_tex)
        self.goalA.node.reparentTo(self.scene)
        self.goalA.node.setScale(0.1)
        self.goalA.node.setP(-90.0)
        self.goalA.node.setH(-90.0)
        self.goalA.node.setPos((self.goalA.x0+self.goalA.x1)*50.0, (self.goalA.y0+self.goalA.y1)*50.0, 0)

        self.goalB.node = loader.loadModel("models/Goal")
        self.goalB.node.setTexture(tsb, black_tex)
        self.goalB.node.reparentTo(self.scene)
        self.goalB.node.setScale(0.1)
        self.goalB.node.setP(-90.0)
        self.goalB.node.setH(90.0)
        self.goalB.node.setPos((self.goalB.x0+self.goalB.x1)*50.0, (self.goalB.y0+self.goalB.y1)*50.0, 0)

    def updatePositions(self):
        for rb in self.robots:
            rb.updatePos()
        self.ball.updatePos()

# end class world

def TurnOffTheLightFunction():
    p1_pid = os.getpid()
    os.kill(p1_pid, signal.SIGTERM)

# instantiate the class
if __name__ == '__main__':
    base = ShowBase()

    w = World(base)
    base.run()
