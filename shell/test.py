import sys
sys.path.append("/usr/local/share/irsl_shm_controller")

import irsl_shm
import numpy as np
import time

ss = irsl_shm.ShmSettings()
ss.hash = 8888
ss.shm_key = 8888
ss.numJoints = 5
ss.numForceSensors = 0
ss.numImuSensors = 0
ss.jointType = irsl_shm.JointType.PositionCommand | irsl_shm.JointType.PositionGains
sm = irsl_shm.ShmManager(ss)

res = sm.openSharedMemory(False)
print(res)
res = sm.checkHeader()
print(res)

res = sm.isOpen()
print(res)

delta = 0.05

for i in range(100):
    pos = sm.readPositionCurrent()
    print(sm.getFrame(), pos)
    for j in range(len(pos)):
        if abs(pos[j]) <= delta:
            pos[j] = 0.0
        elif pos[j] > 0:
            pos[j] -= delta
        else:
            pos[j] += delta
    print(pos)
    ret = sm.writePositionCommand(pos)
    print(ret)
    time.sleep(0.01)
