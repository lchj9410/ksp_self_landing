import math
import time
import krpc
import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
conn = krpc.connect(name='landing')
vessel = conn.space_center.active_vessel
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
g=vessel.orbit.body.surface_gravity
obt_frame = vessel.orbit.body.non_rotating_reference_frame
srf_frame = vessel.orbit.body.reference_frame
ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
    position=vessel.orbit.body.reference_frame,
    rotation=vessel.surface_reference_frame)
vessel.auto_pilot.engage()


############### deorbit  #########################################
# body_r=vessel.orbit.body.equatorial_radius
# atmosphere_depth=vessel.orbit.body.atmosphere_depth
# mu = vessel.orbit.body.gravitational_parameter
# r = vessel.orbit.apoapsis
# a1 = vessel.orbit.semi_major_axis
# a2 = (r+body_r+atmosphere_depth/2)/2
# v1 = math.sqrt(mu*((2./r)-(1./a1)))
# v2 = math.sqrt(mu*((2./r)-(1./a2)))
# delta_v = abs(v2 - v1)
# node = vessel.control.add_node(conn.space_center.ut + vessel.orbit.time_to_apoapsis, prograde=-delta_v)
# vessel.auto_pilot.reference_frame = node.reference_frame
# vessel.auto_pilot.target_direction =  (0, 1, 0)
# time.sleep(1)
# vessel.auto_pilot.wait()

# #### plan node ##########
# m0 = vessel.mass
# m1 = m0 / math.exp(delta_v/Isp)
# flow_rate = F / Isp
# burn_time = (m0 - m1) / flow_rate
# burn_ut = conn.space_center.ut + vessel.orbit.time_to_apoapsis - (burn_time/2.)
# lead_time = 10
# conn.space_center.warp_to(burn_ut - lead_time)
# time_to_apoapsis= vessel.orbit.time_to_apoapsis 
# while time_to_apoapsis - (burn_time/2.) > 0:
#     time_to_apoapsis= vessel.orbit.time_to_apoapsis
#     pass
# vessel.control.throttle = 1.0
# time.sleep(burn_time-0.2)
# vessel.control.throttle = 0.05
# while node.remaining_delta_v > 0.1:
#     pass
# vessel.control.throttle = 0.0
# node.remove()
# time.sleep(1)
# vessel.control.activate_next_stage()
# time.sleep(1)
# vessel.control.activate_next_stage()
# ####  in to the atmosphere ########################
# vessel.auto_pilot.reference_frame = vessel.orbit.body.reference_frame
# retrograde = vessel.flight(vessel.orbit.body.reference_frame).retrograde
# deploy_parachute=False
# altitude =vessel.flight().bedrock_altitude
# while altitude>1500:
#     altitude =vessel.flight().bedrock_altitude
#     if deploy_parachute:
#         velocity = vessel.flight(ref_frame).velocity 
#         vessel.auto_pilot.target_direction = tuple(velocity/(-np.linalg.norm(velocity)))
#     else:
#         vessel.auto_pilot.target_direction = vessel.flight(vessel.orbit.body.reference_frame).retrograde

#     srf_speed = vessel.flight(srf_frame).speed
#     if altitude<10000 and srf_speed<500 and not deploy_parachute:
#         vessel.control.activate_next_stage()
#         deploy_parachute=True

#########  seperation  ################



vessel.control.activate_next_stage()
print('... Fairing separation ... ')
vessel.control.sas = False
vessel.control.rcs = False
velocity = vessel.flight(ref_frame).velocity 
v_dir=velocity/np.linalg.norm(velocity)
vessel.auto_pilot.reference_frame = ref_frame# vessel.orbit.body.reference_frame
for i in range(100):  #  delay for Fairing separation 
    velocity = vessel.flight(ref_frame).velocity 
    vessel.auto_pilot.target_direction = tuple(velocity/(-np.linalg.norm(velocity)))
vessel.control.activate_next_stage()
print('... Heatshield separation ... ')
vessel.control.throttle = 1.0
for i in range(100):  #  delay for  heatsield separation
    velocity = vessel.flight(ref_frame).velocity 
    vessel.auto_pilot.target_direction = tuple(velocity/(-np.linalg.norm(velocity)))
vessel.control.throttle = 0.0
##### suicide burn ##################  
F = vessel.available_thrust #update F
Isp = vessel.specific_impulse * 9.82 #update Isp 


def vel_int(t,*args):
    delta_v,m0=args
    return Isp*(np.log(m0)-np.log(m0-F/Isp*t))-g*t-delta_v

def calculate_decel_distance():
    delta_v=vessel.flight(srf_frame).speed
    m0 = vessel.mass
    T0=delta_v/(F/m0-g)
    T=fsolve(vel_int, T0,args=(delta_v,m0))
    L=delta_v*T+g*T**2/2+Isp*T+m0*Isp**2/F*(np.log(m0*Isp-F*T)-np.log(m0*Isp))
    return L

decel_dis=calculate_decel_distance()
altitude =vessel.flight().bedrock_altitude

margin=300
while altitude-decel_dis > margin:
    decel_dis=calculate_decel_distance()
    altitude =vessel.flight().bedrock_altitude
    velocity = vessel.flight(ref_frame).velocity 
    vessel.auto_pilot.target_direction =  tuple(velocity/(-np.linalg.norm(velocity)))

target_latitude=-0.01001  ######### flat ground coordinate
target_longitude=-15.64029 ###########
vessel.control.throttle = 1
kkp=-1000
kkv=-1.5
target_decending_speed=-10
velocity = vessel.flight(ref_frame).velocity 
while altitude > 20: #and velocity[0]<-5:
    vessel.control.throttle= max(min(vessel.mass*vessel.orbit.body.surface_gravity/F+(target_decending_speed-velocity[0])*0.5, 1) , 0.2)
    latitude =  vessel.flight().latitude 
    longitude =  vessel.flight().longitude
    altitude =vessel.flight().bedrock_altitude
    velocity = vessel.flight(ref_frame).velocity 
    latitude =  vessel.flight().latitude 
    longitude =  vessel.flight().longitude
    lati_e=latitude-target_latitude
    longi_e=longitude-target_longitude
    acc_lati=lati_e*kkp+velocity[1]*kkv
    acc_longi=longi_e*kkp+velocity[2]*kkv
    modified_v= np.array([velocity[0]-10,-acc_lati,-acc_longi]) # changing vel vec direction to create a correction vel towards the targets 
    v_norm=np.linalg.norm(velocity)
    modified_v_norm=np.linalg.norm(modified_v)
    print('altitude',altitude,'velocity',velocity,'e',lati_e,longi_e)
    # if v_norm>2:
    vessel.auto_pilot.target_direction = tuple(modified_v/(-modified_v_norm))
    # else:
        # v_norm_=np.linalg.norm(np.array([*velocity])+[-3,0,0])
        # vessel.auto_pilot.target_direction = tuple((np.array([*velocity])+[-3,0,0])/(-v_norm_))
vessel.control.gear = True
while altitude>1.0:
    altitude =vessel.flight().bedrock_altitude
    target_speed=min(-altitude/5.0,-0.5)
    velocity = vessel.flight(ref_frame).velocity 
    print('altitude',altitude,'velocity',velocity,'e',lati_e,longi_e)
    v_norm=np.linalg.norm(velocity)
    if v_norm>3:
        vessel.auto_pilot.target_direction = tuple(velocity/(-v_norm))
    else:
        v_norm_=np.linalg.norm(np.array([*velocity])+[-3,0,0])
        vessel.auto_pilot.target_direction = tuple((np.array([*velocity])+[-3,0,0])/(-v_norm_))
    vessel.control.throttle= max(min(vessel.mass*vessel.orbit.body.surface_gravity/F+(target_speed-velocity[0])*0.5, 1) , 0.2)
vessel.control.throttle=0



# def min_deacceleration_time(delta_v):  #  wrong
#     m0 = vessel.mass
#     m1 = m0 / math.exp(delta_v/Isp)
#     F = vessel.max_thrust-m0*vessel.orbit.body.surface_gravity
#     flow_rate = F / Isp
#     burn_time = (m0 - m1) / flow_rate
#     return burn_time

# def constant_vel_impact_time():
#     srf_speed = vessel.flight(srf_frame).speed
#     altitude =vessel.flight().bedrock_altitude
#     return altitude/srf_speed

# while min_deacceleration_time(vessel.flight(srf_frame).speed)<constant_vel_impact_time():
#     velocity = vessel.flight(ref_frame).velocity 
#     vessel.auto_pilot.target_direction =  tuple(velocity/(-np.linalg.norm(velocity)))

# kp=1
# kv=10
# hover_height=2
# hover_vertical_vel=-0.5
# deploy_legs=False
# while vessel.flight().bedrock_altitude>1.5:
#     if vessel.flight().bedrock_altitude< 100 and not deploy_legs:
#         vessel.control.gear = True
#         deploy_legs=True
#     velocity = vessel.flight(ref_frame).velocity 
#     v_norm=np.linalg.norm(velocity)
#     if v_norm>1.5:
#         vessel.auto_pilot.target_direction = tuple(velocity/(-v_norm))
#     else:
#         vessel.auto_pilot.target_direction = (1,0,0)
#     altitude =vessel.flight().bedrock_altitude
#     gravity_counter_thrust=vessel.mass*vessel.orbit.body.surface_gravity/vessel.max_thrust
#     srf_speed = vessel.flight(srf_frame).speed
#     acc=(hover_height-altitude)*kp+(srf_speed+hover_vertical_vel)*kv #second order system,  hover over 5 meter
#     F=(acc+vessel.orbit.body.surface_gravity)*vessel.mass
#     thro=F/vessel.max_thrust
#     thro=max(min(thro,1),0)
#     vessel.control.throttle = thro

# vessel.control.throttle = 0






 
