import numpy as np
import math
import tf
from scipy.spatial.transform import Rotation as R

#for the quaternion , you need to get this package
#http://kieranwynn.github.io/pyquaternion/

# Point relative to left camera frame (input of algorithm)
Vl=np.array([-9,-10,-5])

#Constants (SI units) 
b = 0.12 #Baseline or distance between two camera centers
th = 0.006  #Housing Thickness (acrylic)
dh = 0.022843  #Distance between pinhole and housing (assumed till we get info. from stereolabs)
n = np.array([0,0,1]) #normal unit vector of housing plane (pointing out)
eta_air = 1.000277 #refractive index of air
eta_water = 1.333 #refractive index of water
eta_acrylic = 1.4917 #refractive index of acrylic


#Step 1
#Get point location relative to right frame

#result of first operation
res_1 = np.dot(np.array( [ [1 ,0, 0, -b],
                   [0 , 1, 0, 0],
                   [0 ,0 ,1, 0],
                   [0 ,0 ,0, 1] ]
                )  , np.append(Vl,1))

#Point relative to right camera frame
Vr = res_1[:3]        

#Step 2 (will be repeated twice for each camera (a & b))

#Left Camera 
# a : get the angle of incidence and refraction

tht_a_l = math.acos( np.dot(n,Vl) / np.linalg.norm(Vl) )
tht_h_l = math.asin( (eta_air*math.sin(tht_a_l))/eta_acrylic )

#b : compute the normal vector of the air-housing interface
n_ha_l = np.cross(n,Vl) / ( np.linalg.norm(Vl)*math.sin(tht_a_l) ) 

#c : compute the point of incidence at the air-housing interface
lambda_h_l = dh/math.cos(tht_a_l)
Vl_i = lambda_h_l * (Vl/np.linalg.norm(Vl)); #point of incidence

#d : use quaternions to get the ray vector in housing
alpha = (tht_a_l - tht_h_l)/2

q =            [
                math.cos(alpha),
                math.sin(alpha)*n_ha_l[0],
                math.sin(alpha)*n_ha_l[1],
                math.sin(alpha)*n_ha_l[2]
                
               ]
                                       #why the negative?
quad = R.from_quat( [q[1], q[2], q[3], -q[0]] )
Vl_dash = quad.apply(np.transpose(Vl))
Vl_dash = np.transpose(Vl_dash)

#e : get the angle of incidence and refraction (for second interface)
tht_h2_l = math.acos( np.dot(n,Vl_dash)/np.linalg.norm(Vl_dash))
tht_w_l = math.asin((eta_acrylic*math.sin(tht_h2_l))/eta_water)

#f : compute the normal vector of the housing-water interface
n_wh_l = np.cross(n,Vl_dash) / (np.linalg.norm(Vl_dash)*math.sin(tht_h2_l)); 



#g : compute the point of incidence at the housing-water interface
lambda_w_l = (dh+th- np.dot(n,Vl_i))/ math.cos(tht_h_l)
Vl_o = Vl_i + lambda_w_l * (Vl_dash/np.linalg.norm(Vl_dash))#% point of incidence

#h : use quaternions to get the ray vector in water
alpha = (tht_h2_l - tht_w_l)/2

q = [ math.cos(alpha) , math.sin(alpha)*n_wh_l[0] , math.sin(alpha)*n_wh_l[1] ,math.sin(alpha)*n_wh_l[2]]

                                       #why the negative?
quad = R.from_quat( [q[1], q[2], q[3], -q[0]] )
Vl_ddash = quad.apply(np.transpose(Vl_dash))
Vl_ddash = np.transpose(Vl_ddash)

#RightCamera

#a : get the angle of incidence and refraction
tht_a_r = math.acos(np.dot(n,Vr)/np.linalg.norm(Vr))
tht_h_r = math.asin((eta_air*math.sin(tht_a_r))/eta_acrylic)

#b : compute the normal vector of the air-housing interface
n_ha_r = np.cross(n,Vr) / (np.linalg.norm(Vr)* math.sin(tht_a_r)); 

#c : compute the point of incidence at the air-housing interface
lambda_h_r = dh/math.cos(tht_a_r)
Vr_i = lambda_h_r * (Vr/np.linalg.norm(Vr)) # point of incidence

#d : use quaternions to get the ray vector in housing
alpha = (tht_a_r - tht_h_r)/2
q = [math.cos(alpha),math.sin(alpha)*n_ha_r[0],math.sin(alpha)*n_ha_r[1], math.sin(alpha)*n_ha_r[2]]

                                       #why the negative?
quad = R.from_quat( [q[1], q[2], q[3], -q[0]] )
Vr_dash = quad.apply(np.transpose(Vr))
Vr_dash = np.transpose(Vr_dash)

#e : get the angle of incidence and refraction (for second interface)
tht_h2_r = math.acos(np.dot(n,Vr_dash)/np.linalg.norm(Vr_dash))
tht_w_r = math.asin((eta_acrylic*math.sin(tht_h2_r))/eta_water)

#f : compute the normal vector of the housing-water interface
n_wh_r = np.cross(n,Vr_dash) / (np.linalg.norm(Vr_dash)*math.sin(tht_h2_r)) 

#g : compute the point of incidence at the housing-water interface
lambda_w_r = (dh+th-np.dot(n,Vr_i))/math.cos(tht_h_r)
Vr_o = Vr_i + lambda_w_r * (Vr_dash/np.linalg.norm(Vr_dash)) # point of incidence

#h : use quaternions to get the ray vector in water
alpha = (tht_h2_r - tht_w_r)/2
q = [math.cos(alpha) ,math.sin(alpha)*n_wh_r[0] ,math.sin(alpha)*n_wh_r[1] ,math.sin(alpha)*n_wh_r[2] ]
                                       #why the negative?
quad = R.from_quat( [q[1], q[2], q[3], -q[0]] )

Vr_ddash = quad.apply(np.transpose(Vr_dash))
Vr_ddash = np.transpose(Vr_ddash)

#Step 3 : Find the best fit point of intersection
Vr_o = Vr_o + np.array([b,0,0]) #transfom point of incidence to left frame 
#Using parametric representation of lines
A = ((Vr_o[0]-Vl_o[0])*Vl_ddash[2])/Vl_ddash[0] - Vr_o[2] + Vl_o[2]
B = Vr_ddash[2] - ((Vr_ddash[0]*Vl_ddash[2])/Vl_ddash[0])
T = A/B

Point = np.array([  Vr_o[0] + T*Vr_ddash[0],
                    Vr_o[1] + T*Vr_ddash[1],
                    Vr_o[2] + T*Vr_ddash[2]
                 
                ])
S = (-Vr_o[2] + Vl_o[2] - T*Vr_ddash[2])/-Vl_ddash[2]

error = Point[1] - Vl_o[1] - S*(Vl_ddash[1])

print(error)

quit()

