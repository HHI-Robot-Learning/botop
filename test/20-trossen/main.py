"""Python version of botop() from main.cpp — Marc's third test.

    ~/venvs/rai-src/bin/python main.py

A literal translation: same start pose, same T=10 waypoints of 0.3*randn(6), same
timing. The only change is the sign convention — joints 2/3/4 are negated because our
URDF conversion flips those axes for PhysX, so the pose that is valid in the driver's
convention needs negating for the model.

Needs the source build: ~/venvs/rai-src has BotOp.launch_trossen, ~/venvs/rai does not.
"""
import numpy as np
import robotic as ry

C = ry.Config()
C.addFile('scene.yml')

bot = ry.BotOp(C, False)
bot.launch_trossen()
bot.wait(C, True, False)

q0 = np.array([0.124552, 0.630388, -0.830282, 0.140574, 0.621233, 0.422866, 0.02])
print('moving to the start pose')
bot.moveTo(q0, 1.)
bot.wait(C)

T = 10
path = np.tile(q0, (T, 1))
for t in range(T):
    path[t, :6] += 0.3 * np.random.randn(6)
path[-1] = q0
bot.move(path, [.5 * T])
bot.wait(C)