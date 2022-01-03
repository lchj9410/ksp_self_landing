import krpc
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import proj3d
from math import *
import numpy as np
conn = krpc.connect(name='landing1')
vessel = conn.space_center.active_vessel
ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
    position=vessel.orbit.body.reference_frame,
    rotation=vessel.surface_reference_frame)

R=320000
altitude =vessel.flight().bedrock_altitude
latitude =  vessel.flight().latitude 
longitude =  vessel.flight().longitude
xdata=np.array([latitude])
ydata=np.array([longitude])
zdata=np.array([altitude])

fig = plt.figure(figsize=(10,10))
ax = fig.add_subplot(111,projection='3d') 
def animate(i):
    global xdata, ydata, zdata
    ax.cla()
    ax.set_zlim3d(0, 1300)
    ax.set_xlim3d(-0.1, 0.1)
    ax.set_ylim3d(-15.8,-15.5)
    altitude =vessel.flight().bedrock_altitude
    latitude =  vessel.flight().latitude 
    longitude =  vessel.flight().longitude
    xdata=np.append(xdata,latitude)
    ydata=np.append(ydata,longitude)
    zdata=np.append(zdata,altitude)
    plt.plot( xdata, ydata, zdata,'-',color='b',linewidth=2.5)
    plt.plot( -0.01001, -15.64088, 0, "x" , color='g',markersize=20)
    velocity = vessel.flight(ref_frame).velocity 
    plt.plot( [latitude,latitude+velocity[1]/1500], [longitude,longitude+velocity[1]/1500], [altitude,altitude+velocity[0]*2], "-" , color='r')


ani = FuncAnimation(fig, animate, interval=200)
plt.show()