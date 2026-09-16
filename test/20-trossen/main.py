"""Python version of moveToTarget(), with both gripper carriages linked.

    ~/venvs/rai-src/bin/python main.py

Same behaviour as the C++ moveToTarget: drive to a pose, stop on contact, hold,
return home. The one thing this buys over x.exe is the carriage mimic — scene.yml
freezes the right carriage so the DOF count matches the driver, which leaves the
model asymmetric. A mimic cannot live in a .g file or in PhysX, so it has to be
created here.

Needs the source build: ~/venvs/rai-src has BotOp.launch_trossen, ~/venvs/rai does not.
"""
import sys
import time
import numpy as np
import robotic as ry

sys.path.insert(0, '/home/yelchaninova/LOCAL_26-robot-models/26-robot-models')
from wxai import config

C = ry.Config()
C.addFile('scene.yml')

left = C.getFrame("left_carriage_joint")
right = C.getFrame("right_carriage_joint")
lo, hi = config.CARRIAGE_RANGE
right.setJoint(left.getJointType(), limits=[lo, hi], mimic=left,
               scale=config.CARRIAGE_MIMIC_SCALE)
print('DOFs:', C.getJointDimension(), C.getJointNames())

bot = ry.BotOp(C, False, False)      # no sim, no franka — needs our binding fix
bot.launch_trossen()

q_now = bot.get_q()
print('starting from:', np.round(q_now, 4))

# All poses measured by hand-guiding the arm in direct() and read off direct.dat
# (driver convention there, negated on 2/3/4 for the model), except q_carry, which is
# the midpoint of grasp and place with the shoulder and elbow set by eye in the viewer.
q_grasp = np.array([ 0.0147, 1.8252, -0.6952, -0.5762, -0.0753,  0.0635, 0.0252])
q_carry = np.array([-0.4107, 0.9882, -0.9432, -0.5095, -0.1112, -0.0715, 0.02518])
q_place = np.array([-1.0458, 1.9514, -0.8936, -0.4877,  0.2691, -0.1413, 0.0252])

OPEN = 0.040

def opened(q):
    p = q.copy(); p[6] = OPEN; return p

print('approach, jaws open')
bot.moveTo(opened(q_grasp), 1.); bot.wait(C)

print('close on the cube')
bot.moveTo(q_grasp, 1.); bot.wait(C)
print('  tau[6] = %.2f   carriage %.4f (commanded %.4f)'
      % (bot.get_tauExternal()[6], bot.get_q()[6], q_grasp[6]))

print('carry')
bot.moveTo(q_carry, 1.); bot.wait(C)

print('place')
bot.moveTo(q_place, 1.); bot.wait(C)

print('release')
bot.moveTo(opened(q_place), 1.); bot.wait(C)

print('back off')
bot.moveTo(opened(q_carry), 1.); bot.wait(C)

q_home = np.array([-0.04, 0.05, -0.3, 0.02, 0., -0.05, 0.0002])
bot.moveTo(q_home, 1.)
bot.wait(C)
print('returned home')
