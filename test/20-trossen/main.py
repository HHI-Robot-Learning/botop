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

q_target = q_now.copy()
q_target[0] = 1.5
q_target[1] = 0.5
q_target[2] = -0.6
q_target[3] = 0.4
q_target[6] = 0.03

TOUCH = 2.2      # free motion peaks at 1.9, contact reaches 2.3+ and plateaus near 4
TICKS = 4        # 40 ms at the 100 Hz sync rate

bot.moveTo(q_target, 2.)
n = 0
touched = False
while bot.getTimeToEnd() > 0.:
    bot.sync(C, .01)
    tau = np.linalg.norm(bot.get_tauExternal()[:6])
    n = n + 1 if tau > TOUCH else 0
    if n >= TICKS:
        print('contact, |tau| = %.2f' % tau)
        touched = True
        break

if touched:
    q_stop = bot.get_q()
    bot.moveTo(q_stop, 3., True)     # overwrite=True: blend down from current velocity
    bot.wait(C)
    time.sleep(1.5)

q_home = np.array([-0.04, 0.05, -0.3, 0.02, 0., -0.05, 0.0002])
bot.moveTo(q_home, 1.)
bot.wait(C)
print('returned home')
