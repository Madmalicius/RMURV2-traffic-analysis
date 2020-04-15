import numpy as np
import rospy
import rosbag
from filterpy.kalman import KalmanFilter
import matplotlib.pyplot as plt

R_x_std = 2 # a gues
Q_x_std = 1000 # big number since we assume process noise is significant.

dt=1.0 #time difference
prev_x=0. #The previus position for x - to calculate the speed
prev_y=0. #The previus position for y - to calculate the speed


# Initialize the Kalman filter.
# We have one state variable (x) and one measurement variable (z).
kalm_filt = KalmanFilter(dim_x=4, dim_z=2)
# Define the state transition matrix - Here is it called F but can also be called A
kalm_filt.F = np.array([
                        [1.,dt,0.,0.],          #X
                        [0.,1./dt,0.,0.],       #X_dot
                        [0.,0.,1.,dt],          #Y            
                        [0.,0.,0.,1./dt]        #Y_dot    
                        ])

# Define the measurment function - 
kalm_filt.H = np.array([[1.,0.,0.,0.],    #X
                        [0.,0.,1.,0.]     #Y
                        ])
# Define the process uncertainty / noise.
kalm_filt.Q = np.array([[Q_x_std**2, 0,0,0],
                        [0,Q_x_std**2,0,0],
                        [0,0,Q_x_std**2,0],
                        [0,0,0,Q_x_std**2],
                        ])
# Define the measurment uncertainty / noise.
kalm_filt.R *= np.eye(2)*(R_x_std**2)



bag = rosbag.Bag('Kalman_bag.bag')

first_run = True
time_first = True
time_ref_zero=0.0 # For having the time start at zero
time_same=0 #To difference between the images
time_mesur_x = [] # Time values for measurments
M_x = [] # The measured value of the system

M_pos_x=[]
M_pos_y=[]

Pred_value_x = [] # The predicted value of x position
updateValue_x = [] # The updated value of x position

Pred_value_y = [] # The predicted value of y position
updateValue_y = [] # The updated value of y position


M_data_x=np.array([])
M_data_y=[]
M_data_prev_x=[]
M_data_prev_y=[]


for topic, msg, t in bag.read_messages(topics=['X', 'Y']):

    if(time_first):
        time_ref_zero = t.to_sec()
        time_first=False

        #data_x = msg.data
        #print(data_x)


    if(topic == 'X'):
        data_x = msg.data
        M_pos_x.append(data_x)
        time_mesur_x.append(update_time)
        #print(update_time)
    
    if(topic == 'Y'):
        data_y = msg.data
        M_pos_y.append(data_y)
        #print(data_y)
        #time_mesur_x.append(update_time)




        if(first_run):
            # Initialize the Kalman filter. Set the state to the
            # first measurement and the uncertainty to a big value.
            kalm_filt.x = np.array([[data_x],           #X
                                    [data_x-prev_x],    #X_dot
                                    [data_y],           #Y
                                    [data_x-prev_y],    #Y_dot
                                    ])
            kalm_filt.P *= 1000.
            prev_time = update_time

            
            kalm_filt.predict()
            Pred_value_x.append(kalm_filt.x[0])
            Pred_value_y.append(kalm_filt.x[2])
            kalm_filt.x = np.array([[data_x],           #X
                                    [data_x-prev_x],    #X_dot
                                    [data_y],           #Y
                                    [data_x-prev_y],    #Y_dot
                                    ])
            updateValue_x.append(kalm_filt.x[0])
            updateValue_y.append(kalm_filt.x[2])   
            first_run = False
        else:
            
            # Predict the next state and update the state estimate
            # using the next update.
            dt = update_time - prev_time
            prev_time= update_time

            kalm_filt.predict()
            Pred_value_x.append(kalm_filt.x[0])
            Pred_value_y.append(kalm_filt.x[2])
            kalm_filt.x = np.array([[data_x],           #X
                                    [data_x-prev_x],    #X_dot
                                    [data_y],           #Y
                                    [data_x-prev_y],    #Y_dot
                                    ])
            updateValue_x.append(kalm_filt.x[0])
            updateValue_y.append(kalm_filt.x[2])

            prev_x=data_x
            prev_y=data_y

bag.close()


# Print the standard deviation of the measurements.
print 'the std for the measurements: ', np.std(M_pos_x)

print("time_mesur_x length: ", len(time_mesur_x))
print("M_x length: ", len(M_pos_x))
print("updateValue_x length: ", len(updateValue_x))
print("Pred_value_x length: ", len(Pred_value_x))


# Plot the raw measurments and the updated state estimates.
plt.plot(updateValue_y,updateValue_x,'go-', label='updates')
plt.plot(M_pos_y, M_pos_x,'rx-', label='measured pos')
#plt.plot(Pred_value_y,Pred_value_x,'ko-', label='predictions')
plt.legend(loc='upper left')
plt.show()

plt.plot(time_mesur_x, M_pos_x,'rx-', label='measured pos x')
plt.plot(time_mesur_x, M_pos_y,'bx-', label='measured pos y')
plt.legend(loc='upper left')
plt.show()


print("Estimated state after the last update")
print(kalm_filt.x)
