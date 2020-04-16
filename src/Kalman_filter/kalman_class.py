import numpy as np
from filterpy.kalman import KalmanFilter



class kalman():

    def __init__(self,x0,y0):#,ID):

        self.R_x_std = 2    # a gues
        self.Q_x_std = 1000 # big number since we assume process noise is significant.
        self.dt=0.05        #Time difference - it's asummed to be 20fps so it will be 0.05 s
        self.x=x0           #Current pos
        self.y=y0           #Current pos
        self.x_prev=0.      #The previus position for x - to calculate the speed
        self.y_prev=0.      #The previus position for y - to calculate the speed
        self.first_run = False
        #self.id=ID          #The ID of the car - To be able to find it in a list

        self.initialize_kalman_filter() # init the kalman when the object is createated


    def initialize_kalman_filter(self):
    
        # Initialize the Kalman filter.
        # We have one state variable (x) and one measurement variable (z).
        self.kalm_filt = KalmanFilter(dim_x=4, dim_z=2)
        # Define the state transition matrix - Here is it called F but can also be called A
        self.kalm_filt.F = np.array([
                                [1.,self.dt,0.,0.],          #X
                                [0.,1./self.dt,0.,0.],       #X_dot
                                [0.,0.,1.,self.dt],          #Y            
                                [0.,0.,0.,1./self.dt]        #Y_dot    
                                ])

        # Define the measurment function - 
        self.kalm_filt.H = np.array([[1.,0.,0.,0.],    #X
                                [0.,0.,1.,0.]     #Y
                                ])
        # Define the process uncertainty / noise.
        self.kalm_filt.Q = np.array([[Q_x_std**2, 0,0,0],
                                [0,Q_x_std**2,0,0],
                                [0,0,Q_x_std**2,0],
                                [0,0,0,Q_x_std**2],
                                ])
        # Define the measurment uncertainty / noise.
        self.kalm_filt.R *= np.eye(2)*(R_x_std**2)

    def first_update(self):
        self.is_state_initialized = True
            # Initialize the Kalman filter. Set the state to the
            # first measurement and the uncertainty to a big value.
            self.kalm_filt.x = np.array([[self.x],                #X
                                    [self.x-self.x_prev],    #X_dot
                                    [self.y],                #Y
                                    [self.x-self.y_prev],    #Y_dot
                                    ])
            self.kalm_filt.P *= 1000.

    def run_filter(self,x_data,y_data):

        if self.is_state_initialized!=True:
            self.first_update()

        # Specify the state transition matrix given the 
        # time since the last update.
        self.kalm_filt.predict()
        self.x, self.y =self.kalm_filt.update([[x_data,y_data]])
        self.x_prev=x
        self.y_prev=y
