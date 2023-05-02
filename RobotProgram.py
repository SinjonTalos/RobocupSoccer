import robotPrg1A
import robotPrg1B
import robotPrg2A
import robotPrg2B
import threading
from stdfunction import *

class ThreadRobot (threading.Thread):
    def __init__(self, threadID, robot):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.robot = robot

    def run(self):
        wait_for_seconds(3.0)
        if self.threadID == 1:
            robotPrg1A.main(self.robot.hub)
        elif self.threadID == 3:
            robotPrg1B.main(self.robot.hub)
        elif self.threadID == 2:
            robotPrg2A.main(self.robot.hub)
        elif self.threadID == 4:
            robotPrg2B.main(self.robot.hub)
