from multiTurtleSim import *
sim  = MultiTurtleSim(
                timestep= 0.25,
                 botRaidus=0.35/2, 
                 effective_distance=0.35/2,
                wheelDist=0.23,
                 neighborDist=15,
                 maxNeighbors=10,
                 timeHorizon= 0.25,
                 timeHorizonObst=10,
                 maxSpeed=0.65,
                 maxLinearSpeed = 9999,
                 maxRotSpeed =99999,
                botStartVel = np.array([0,0])
)
#rospy.set_param("use_sim_time",True)
sim.addBot(1,np.array([0,0]),0.0,np.array([5,4.8]))
sim.addBot(2,np.array([0,0]),0.0,np.array([1,4.8]))


while not (sim.done() or rospy.is_shutdown()):
    sim.step()
    rospy.sleep(0.25)
