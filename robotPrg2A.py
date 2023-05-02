from stdfunction import * 

class SoccerPlayer:
    def __init__(self,hub):
        self.hub = hub
        self.ir_seeker = hub.port.E.device

        self.motor_a = hub.Motor("A")
        self.motor_b = hub.Motor("B")
        self.motor_c = hub.Motor("C")
        self.motor_d = hub.Motor("D")

        self.speed = 100

    #calculate motor speed
    def move_motor(self,direction):    
        d12 = [-1., 1.,-1., 1.]
        d9  = [-1.,-1., 1., 1.]
        d6  = [ 1.,-1., 1.,-1.]
        d3  = [ 1., 1.,-1.,-1.]
        motor = [0,0,0,0]

        for i in range(0,4):
            if direction >= 9 :
                f = (direction - 9.0)/3.0
                motor[i] = d12[i]*f + d9[i]*(1-f) 
            elif direction >= 6 :
                f = (direction - 6.0)/3.0
                motor[i] = d9[i]*f + d6[i]*(1-f) 
            elif direction >= 3 :
                f = (direction - 3.0)/3.0
                motor[i] = d6[i]*f + d3[i]*(1-f) 
            else:
                f = direction / 3.0
                motor[i] = d3[i]*f + d12[i]*(1-f) 

        speed = self.speed
        return (motor[0]*speed,motor[1]*speed,motor[2]*speed,motor[3]*speed)
    
    def calcRobotDirection(self,ball_direction):
        if ball_direction >= 1 and ball_direction < 6:
            robot_direction = ball_direction + 3
        elif ball_direction >= 6 and ball_direction < 11:
            robot_direction = ball_direction - 3
        else:
            robot_direction = 12
        return robot_direction

    def start(self,a,b,c,d):
        self.motor_a.start(-a)
        self.motor_b.start(-b)
        self.motor_c.start(-c)
        self.motor_d.start(-d)

    def run(self): 

        self.hub.motion_sensor.reset_yaw_angle()
        while True:
            data = self.ir_seeker.get()
            direction = data[0]
            strength = data[2]

            #orbit if close                
            if strength >= 70:
                direction = self.calcRobotDirection(direction)

            a,b,c,d = self.move_motor(direction)
            self.start(a, b, c, d)

def main(hub):
    s = SoccerPlayer(hub)
    s.run()    

if __name__ == '__main__':
    hub = PrimeHub()
    main(hub)