import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Prepare plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-0.5, 0.5); ax.set_ylim(-0.5, 0.5); ax.set_zlim(0, 1.0)
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.view_init(elev=20, azim=30)  # set an initial view angle

# Plot elements: points for joints and a line for arm
(sh_point,) = ax.plot([0], [0], [0], 'o', markersize=5, color='black')             # shoulder (origin)
(el_point,) = ax.plot([0], [0], [0], 'o', markersize=5, color='blue')              # elbow
(wr_point,) = ax.plot([0], [0], [0], 'o', markersize=5, color='blue')              # wrist
arm_line, = ax.plot([0, 0, 0], [0, 0, 0], [0, 0, 0], '-', lw=3, color='orange')    # line from shoulder->elbow->wrist

# In the main loop, after computing positions:
elbow_x, elbow_y, elbow_z = elbow_pos
wrist_x, wrist_y, wrist_z = wrist_pos
# Update data
el_point.set_data(elbow_x, elbow_y); el_point.set_3d_properties(elbow_z)
wr_point.set_data(wrist_x, wrist_y); wr_point.set_3d_properties(wrist_z)
# Line: connect shoulder (0,0,0) to elbow to wrist
arm_line.set_data([0, elbow_x, wrist_x], [0, elbow_y, wrist_y])
arm_line.set_3d_properties([0, elbow_z, wrist_z])
plt.draw()
plt.pause(0.001)  # small pause to allow redraw
