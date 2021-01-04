from multiTurtleSim import *
sim  = MultiTurtleSim(
                timestep= 0.25,
                 botRaidus=0.35/2, 
                 effective_distance=0.35/2,
                wheelDist=0.23,
                 neighborDist=15,
                 maxNeighbors=10,
                 timeHorizon= 1,
                 timeHorizonObst=10,
                 maxSpeed=0.2,
                 maxLinearSpeed = 9999,
                 maxRotSpeed =99999,
                botStartVel = np.array([0,0])
)
# rospy.set_param("use_sim_time",True)

turtle_positions = [
    None,
        np.array([2.9,1.0]),
        np.array([4.6,2.8]),
        np.array([2.8,4.8]),
        np.array([1.1,2.8])
]
GoBack = False

if GoBack:
    sim.addBot(1,np.array([0,0]),0.0,turtle_positions[1])
    sim.addBot(2,np.array([0,0]),0.0,turtle_positions[2])
    sim.addBot(3,np.array([0,0]),0.0,turtle_positions[3])
    sim.addBot(4,np.array([0,0]),0.0,turtle_positions[4])
else:
    sim.addBot(1,np.array([0,0]),0.0,turtle_positions[3])
    sim.addBot(2,np.array([0,0]),0.0,turtle_positions[4])
    sim.addBot(3,np.array([0,0]),0.0,turtle_positions[1])
    sim.addBot(4,np.array([0,0]),0.0,turtle_positions[2])

while not (sim.done() or rospy.is_shutdown()):
    sim.step()
    rospy.sleep(0.25)
