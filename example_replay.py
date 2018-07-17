import robot
import time
from sharedmem import *
from math import *

# Real time drawing
import Tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
from matplotlib import style

#3D
from mpl_toolkits.mplot3d.axes3d import Axes3D

robot.launch() # launches the robot C++ script
robot.init()   # initialises shared memory


def update_graph(dt):
    if (sqrt(pow(rshm('vel_x'),2)+ pow(rshm('vel_y'),2)+ pow(rshm('vel_z'),2))>0.01):
        traj = robot.stop_capture(False)
    else :
        traj = robot.stop_capture(True)
    t=[i*0.001 for i in range (0, len(traj))]
    x=[traj[i][0] for i in range (0, len(traj))]
    y=[traj[i][1] for i in range (0, len(traj))]
    z=[traj[i][2] for i in range (0, len(traj))]

    ax1.clear()
    ax2.clear()
    ax3.clear()
    ax4.clear()

    ax1.set_ylim(-0.15, 0.15, auto=False)
    ax2.set_ylim(-0.15, 0.15, auto=False)
    ax3.set_ylim(-0.15, 0.15, auto=False)
    ax4.set_xlim(-0.15, 0.15, auto=False)
    ax4.set_ylim(-0.15, 0.15, auto=False)
    ax4.set_zlim(-0.15, 0.15, auto=False)

    ax2.set_xlabel('Temps')
    ax3.set_xlabel('Temps')
    ax4.set_xlabel('x_axis')

    ax1.set_ylabel('y_axis', color='g')
    ax2.set_ylabel('z-axis', color='r')
    ax3.set_ylabel('x_axis', color='g')

    ax4.set_ylabel('y-axis', color='r')
    ax4.set_zlabel('z-axis', color='r')

    ax1.plot(t, y, 'g')
    ax2.plot(t, z, 'r')
    ax3.plot(t, x, 'g')
    ax4.plot3D(x,y,z, 'r')
    Axes3D.mouse_init(ax4,rotate_btn=1, zoom_btn=2)


print("Move your arm to start capture")

while (sqrt(pow(rshm('vel_x'),2)+ pow(rshm('vel_y'),2)+ pow(rshm('vel_z'),2))<0.1):
    pass

robot.start_capture()


app = tk.Tk()
app.wm_title("Graphe Matplotlib dans Tkinter")

style.use("ggplot")
fig = Figure(figsize=(17, 9), dpi=112)

graph = FigureCanvasTkAgg(fig, master=app)
canvas = graph.get_tk_widget()
canvas.grid(row=0, column=0)

ax1 = fig.add_subplot(221)
ax2 = fig.add_subplot(223, sharex=ax1)
ax3 = fig.add_subplot(222)
ax4 = fig.add_subplot(224,projection='3d')

ax2.set_xlabel('Temps')
ax3.set_xlabel('Temps')
ax4.set_xlabel('x_axis' , color='g')

ax1.set_ylabel('y_axis', color='g')
ax2.set_ylabel('z_axis', color='r')
ax3.set_ylabel('x_axis', color='g')
ax4.set_ylabel('y_axis', color='r')

ax4.set_zlabel('z_axis', color='r')
fig.tight_layout()


ani = animation.FuncAnimation(fig, update_graph, interval=1)
app.mainloop()



traj = robot.stop_capture(True)
x,y,z = zip(*traj)
#x=[traj[i][0] for i in range (0, len(traj))]
#y=[traj[i][1] for i in range (0, len(traj))]
#z=[traj[i][2] for i in range (0, len(traj))]

robot.load_a_trajectory(x,y,z)

robot.move_to(x[0],y[0],z[0], 2)
while (robot.move_is_done() == False):
    pass  #wait until the end of the movement

robot.play_movement ()
while (robot.move_is_done() == False):
    pass

robot.unload() #Stop the C++ script
