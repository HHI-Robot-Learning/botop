"""Python version of botop() from main.cpp, step 1: connect and read.

Two things the C++ version does not do, both needed here:
  * seed C with the arm's real pose before constructing BotOp, because the
    constructor takes its initial reference from C.getJointState() and step()
    starts commanding the arm towards it at 500 Hz;
  * negate joints 2/3/4 — the model's axes were flipped during URDF conversion
    (PhysX cannot build a generic hinge), so driver and model disagree in sign.
"""
import sys
import numpy as np
import robotic as ry

sys.path.insert(0, '/home/yelchaninova/LOCAL_26-robot-models/26-robot-models')
from wxai import prepare_model as pm, config

URDF = config.DESCRIPTION / 'urdf' / 'generated' / 'wxai' / 'wxai_follower.urdf'

C = ry.Config()
C.addFile('scene.yml')

q_driver = np.loadtxt('direct.dat')[-1, 1:8]
q_model = pm.flip_driver_signs(q_driver, C.getJointNames(), URDF)
C.setJointState(q_model)
print('driver:', np.round(q_driver, 4))
print('model :', np.round(q_model, 4))

bot = ry.BotOp(C, False, False)   # no sim, no franka
bot.launch_trossen()
print('launched')
print('state methods:', [m for m in dir(bot) if 'tate' in m])
