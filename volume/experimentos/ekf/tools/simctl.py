import sys,time
sys.path.insert(0,"/opt/CoppeliaSim_Edu_V4_7_0_rev2_Ubuntu20_04/programming/zmqRemoteApi/clients/python/src")
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
sim=RemoteAPIClient().require("sim")
cmd=sys.argv[1]
if cmd=="state": print("state",sim.getSimulationState())
elif cmd=="restart":
    sim.stopSimulation()
    while sim.getSimulationState()!=sim.simulation_stopped: time.sleep(0.1)
    sim.startSimulation(); print("reiniciada")
elif cmd=="stop":
    sim.stopSimulation()
    while sim.getSimulationState()!=sim.simulation_stopped: time.sleep(0.1)
    print("detenida")
elif cmd=="start":
    sim.startSimulation(); print("iniciada")
