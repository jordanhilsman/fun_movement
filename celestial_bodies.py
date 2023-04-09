import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import animation
import math

G = 3
height = 1500
width = 1500
#borders


body_one_pos = np.array([[0,0]])
body_two_pos = np.array([[2,0]])
body_three_pos = np.array([[-2,0]])

body_one_velo = np.array([[0, -.1]])
body_two_velo = np.array([[1, -.1]])
body_three_velo = np.array([[0, -.1]])

body_one_accel = np.array([[0,0]])
body_two_accel = np.array([[0,0]])
body_three_accel = np.array([[0, 0]])

body_one_mass = 3
body_two_mass = 12
body_three_mass = 12

delta_t = 1

t = 0

while t < 100:

    distance1 = body_one_pos - body_two_pos
    distance22 = body_two_pos - body_one_pos
    distance2 = body_one_pos - body_three_pos
    distance3 = body_two_pos - body_three_pos
    
    distance1_mag = np.linalg.norm(distance1)
    distance2_mag = np.linalg.norm(distance2)
    distance3_mag = np.linalg.norm(distance3)
    distance22_mag = np.linalg.norm(distance22)
    
    force1_mag = (G * body_one_mass * body_two_mass) / (distance1_mag**2)
    force2_mag = (G * body_one_mass * body_three_mass) / (distance2_mag**2)
    force3_mag = (G * body_two_mass * body_three_mass) / (distance3_mag**2)
    force22_mag = (G * body_one_mass * body_two_mass) / (distance22_mag**2)
    
    r_hat22 = distance22/distance22_mag
    r_hat1 = distance1/distance1_mag
    r_hat2 = distance2/distance2_mag
    r_hat3 = distance2/distance2_mag
    
    force1_net = -(force1_mag * r_hat1)
    force2_net = -(force2_mag * r_hat2)
    force3_net = -(force3_mag * r_hat3)
    force22_net = -(force22_mag * r_hat22)
    
    body_one_accel = body_one_accel + ((force1_net + force2_net)/body_one_mass)
    
    body_one_velo = body_one_velo + (body_one_accel * delta_t)
    

    
    body_two_accel = body_two_accel + ((force22_net + force3_net)/body_two_mass)
    
    body_two_velo = body_two_velo + (body_two_accel * delta_t)
    


    body_three_accel = body_three_accel + (-(force2_net + force3_net)/body_three_mass)
    body_three_velo = body_three_velo + (body_three_accel * delta_t)
    

    
    if (body_one_pos[0][0] > width or body_one_pos[0][0] < -(width)):
        body_one_velo[0][0] = -(body_one_velo[0][0])
        
    if (body_one_pos[0][1] > height or body_one_pos[0][1] < -(height)):
        body_one_velo[0][1] = -(body_one_velo[0][1])
        
    if (body_two_pos[0][0] > width or body_two_pos[0][0] < -(width)):
        body_two_velo[0][0] = -(body_two_velo[0][0])
        
    if (body_two_pos[0][1] > height or body_two_pos[0][1] < -(height)):
        body_two_velo[0][1] = -(body_two_velo[0][1])       
        
    if (body_three_pos[0][0] > width or body_three_pos[0][0] < -(width)):
        body_three_velo[0][0] = -(body_three_velo[0][0])
        
    if (body_three_pos[0][1] > height or body_three_pos[0][1] < -(height)):
        body_three_velo[0][1] = -(body_three_velo[0][1])        
        
    body_one_pos = body_one_pos + (body_one_velo * delta_t)    
    body_two_pos = body_two_pos + (body_two_velo * delta_t)
    body_three_pos = body_three_pos + (body_three_velo * delta_t)
        
    plt.plot(body_one_pos[:,0], body_one_pos[:,1],'ob:')
    plt.plot(body_two_pos[:,0], body_two_pos[:,1],'or:')
    plt.plot(body_three_pos[:,0], body_three_pos[:,1],'og:')
    t += delta_t
    plt.pause(0.00001)
plt.show()
