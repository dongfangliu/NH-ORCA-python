
from __future__ import print_function
from turtlebot import *
from pyrvo2 import *
class MultiTurtleSim:
    def __init__(self,
                 timestep=0.25,
                 botRaidus=0.175, 
                 effective_distance = 0.175,
                 wheelDist=0.23,
                 neighborDist=15,
                 maxNeighbors=10,
                 timeHorizon=10,
                 timeHorizonObst=10,
                 maxSpeed=2,
                 maxLinearSpeed = 0.65,
                 maxRotSpeed = 3.14,
                 botStartVel=np.array([0, 0])):
        self.D = effective_distance
        self.L = wheelDist
        self.r = botRaidus

        self.neighborDist = neighborDist
        self.maxNeighbors = maxNeighbors
        self.timeHorizon = timeHorizon
        self.timeHorizonObst = timeHorizonObst
        self.maxSpeed = maxSpeed
        self.startVel = botStartVel
        self.maxRotSpeed = maxRotSpeed
        self.maxLinearSpeed = maxLinearSpeed
        self.timeStep = timestep

        self.bots = []
        self.simulator = RVOSimulator()
        self.simulator.setTimeStep(self.timeStep)
        self.simulator.setAgentDefaults(self.neighborDist, self.maxNeighbors, self.timeHorizon,
                                        self.timeHorizonObst, self.r+self.D, self.maxSpeed, self.startVel)
        
        rospy.init_node("multi_turtle", anonymous=True)
                                        
    def addBot(self, botId, botPosition,botOri, botGoal):
        bot = TurtleBot(botId, self.r,botPosition,botOri,botGoal,self.L, self.D,maxRotSpeed=self.maxRotSpeed,maxLinearSpeed=self.maxLinearSpeed)
        while not bot.initialized:
            print("\r waiting for bot "+ str(botId)+" to initialize its pose  ....", end="")
        
        print("Add bot "+ str(botId)+" with pos ",bot.getEffectivePos(), end="")
        self.simulator.addAgent(bot.getEffectivePos())
        self.bots.append(bot)
        
    def done(self):
        # all_reachgoals
        for i in range(len(self.bots)):
            if self.bots[i].done():
                return False
        return True

    def step(self):
        if self.done():
            print("all reach goals")
            return
        for i in range(len(self.bots)):
            self.simulator.setAgentPosition(i, self.bots[i].getEffectivePos())
            self.simulator.setAgentVelocity(i, self.bots[i].getEffectiveVel())
            goalVector = self.bots[i].botGoal- self.simulator.getAgentPosition(i)
            if  np.linalg.norm(goalVector) > 1.0:
                goalVector = goalVector/np.linalg.norm(goalVector)
            self.simulator.setAgentPrefVelocity(i, self.maxSpeed*goalVector)
            self.simulator.step()
        velocities = []
        for i in range(len(self.bots)):
    #     get simulated velocity for each robot
            velocity =  self.simulator.getAgentVelocity(i)
            velocities.append(velocity)
        # print("\r velocities "+str(velocities),end="")
        for i in range(len(self.bots)):
            # if self.bots[i].done():
            #     continue
            vel_msg = self.bots[i].cal_effective_cmd(velocities[i])
            self.bots[i].cmd_vel(vel_msg)



