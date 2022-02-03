# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        self.dim_state = params.dim_state # process model dimension
        self.dt = params.dt # sample time
        self.q = params.q # process noise value for Q covariance

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        F = np.eye((self.dim_state))
        F = np.asmatrix(F) # convert from array to matrix instance
        F[0, 3] = self.dt
        F[1, 4] = self.dt
        F[2, 5] = self.dt
        return F
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        dt2 = self.dt**2
        dt3 = self.dt**3
        
        q_11 = dt3 * self.q / 3.0
        q_13 = dt2 * self.q / 2.0
        q_33 = self.dt * self.q

        return np.matrix([[q_11,  0.0,  0.0, q_13,  0.0,  0.0],
                          [ 0.0, q_11,  0.0,  0.0, q_13,  0.0],
                          [ 0.0,  0.0, q_11,  0.0,  0.0, q_13],
                          [q_13,  0.0,  0.0, q_33,  0.0,  0.0],
                          [ 0.0, q_13,  0.0,  0.0, q_33,  0.0],
                          [ 0.0,  0.0, q_13,  0.0,  0.0, q_33]])
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############        
        x = self.F() * track.x # state prediction
        P = self.F() * track.P * self.F().T + self.Q() # covariance prediction
        
        track.set_x(x)
        track.set_P(P)
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        H = meas.sensor.get_H(track.x) # measurement matrix
        gamma = self.gamma(track,meas) # residual
        S = self.S(track, meas, H)  # covariance of residual
        I = np.asmatrix(np.eye((self.dim_state)))
        K = track.P * H.T * S.I # Kalman gain
        x = track.x + K * gamma # state update
        P = (I - K*H) * track.P # covariance update
        
        track.set_x(x)
        track.set_P(P)
        
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############
        return meas.z - meas.sensor.get_hx( track.x )
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############
        return H * track.P * H.T + meas.R
        
        ############
        # END student code
        ############ 