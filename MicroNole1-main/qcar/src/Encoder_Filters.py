# Two filters to get counts/second from encoder counts
import numpy as np
#import pandas as pd
import matplotlib.pyplot as plt

from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter

class Encoder_filters:
    def __init__(self, d_max=8390000):
        self.d_max = d_max   # encoder maximum counts (for normalizing)
        
        # initialization for high gain filter
        self.t_previous_highgain = 0
        self.y_hat_low_k_1 = 0

        # initilization for MH filter with quandratic fit
        self.t_previous_MHQuau = 0
        self.Theta_hat0 = np.zeros([3,1])
        self.Theta_k_1  = self.Theta_hat0
        self.d_k_1      = 0
        self.dd_hat_low_k_1 = 0

        # initialization for Kalman flter
        self.f = KalmanFilter (dim_x=3, dim_z=1)
        self.f.x = np.array([[0.],    # position
                        [0.],    # velocity
                        [0.]])   # acceleration
        self.f.H = np.array([[1.,0.,0.]])    # measurement matrix
        self.f.R = 0.15
        self.tau = 0.01

    def high_gain_filter(self,t,d,Kf=1/5):
        ''' high gain filter with a low pass filter,
            Inputs:
                   - Kf: Low-pass filter time constant (cutt-off frequency = 1/tau)
                   - t: time
                   - d: encoder counts
            outputs:
                   - counts/second
        '''
        # signal input
        delta_k = t - self.t_previous_highgain

        # low-pass filter
        y_hat_low =(1 - delta_k * Kf) * self.y_hat_low_k_1 + delta_k * Kf * d
        # print(y_hat_low)
        
        # outputing results
        dd_hat = Kf * (d - y_hat_low)

        # update cache
        self.t_previous_high_gain = t
        self.y_hat_low_k_1 = y_hat_low

        return dd_hat * self.d_max


    def recursive_MH_Quau_filter(self,t,d,gamma=1e-4,tau=1e-1):
        '''A robust recursive moving-horizon least squares filter with quadratic fit.
            It estimates the signal, it’s derivative and it’s second derivative
            Inputs:
                   - gamma: forgetting factor
                   - tau: Low-pass filter time constant (cutt-off frequency = 1/tau)
                   - t: time
                   - d: encoder counts
            outputs:
                   - counts/second: MH quadratic filter
                   - counts/second: MH quadratic filter + low pass filter
        '''

        # singal input
        delta_k = t - self.t_previous_MHQuau
        d_k     = d 

        # calling filter
        Theta_k, d_k_hat, d_k_d_hat, d_k_dd_hat = self.MH_Quau_filter(self.Theta_k_1, delta_k, d_k, self.d_k_1, gamma=gamma)
        Theta_hat = Theta_k
        d_hat     = d_k_hat
        dd_hat    = d_k_d_hat
        ddd_hat   = d_k_dd_hat 

        # Low-pass filter
        dd_hat_low = (1-delta_k/tau) * self.dd_hat_low_k_1 + (delta_k/tau) * dd_hat

        # update cache
        self.Theta_k_1  = Theta_hat
        self.t_previous_MHQuau = t
        self.d_k_1      = d_k
        self.dd_hat_low_k_1 = dd_hat_low

        return d_hat * self.d_max, dd_hat * self.d_max, dd_hat_low * self.d_max


    def MH_Quau_filter(self,Theta_k_1, delta_k, d_k, d_k_1, gamma=1e-4):
        '''A least squares filter with quadratic fit.
            Inputs:
                   - Theta_k_1: [3-by-1] weights for [filtered measurement, filtered derivative, filtered second derivative]
                   - delta_k  : [scalar] time gap
                   - d_k      : [scalar] measurement
                   - d_k_1    : [scalar] past measurement
                   - gamma    : [scalar] forgetting factor
            outputs:
                   - Theta_k   : [3-by-1] updated weights
                   - d_k_hat   : [scalar] filtered measurement
                   - d_k_d_hat : [scalar] filtered derivative
                   - d_k_dd_hat: [scalar] filtered second derivative
        '''
        # 1. packing
        y_k_bar  = np.array([[1, 1], [0, delta_k], [0, delta_k**2]],dtype=object)
        y_k_bar = y_k_bar.reshape((3,2))
        d_k_bar = np.array([d_k_1,d_k],dtype=object).T
        d_k_bar = d_k_bar.reshape((2,1))

        # 2. compute gain matrix
        delta = (gamma + 1)*(gamma + 1 + delta_k**2 + delta_k**4) - 1
        D_k   = np.array([[1/gamma - (2*gamma+delta_k**2+delta_k**4)/(gamma*delta), -delta_k/delta, -delta_k**2/delta],
        [-delta_k/delta, 1/gamma - delta_k**2*(gamma+1)/(gamma*delta), -delta_k**3*(gamma+1)/(gamma*delta)],
        [-delta_k**2/delta, -delta_k**3*(gamma+1)/(gamma*delta), 1/gamma - delta_k**4*(gamma+1)/(gamma*delta)]])

        D_k = D_k.reshape((3,3))

        # 3. Update weights
        Theta_k = np.dot(D_k,(gamma * Theta_k_1 + np.dot(y_k_bar,d_k_bar)))

        # 4. Output
        d_k_hat    = np.dot(np.array([1, delta_k, delta_k**2],dtype=object), Theta_k)  # filtered measurement
        d_k_d_hat  = np.dot(np.array([0, 1, 2*delta_k],dtype=object),Theta_k)        # filtered derivative
        d_k_dd_hat = np.dot(np.array([0, 0, 2],dtype=object),Theta_k)                # filtered second derivative

        return Theta_k, d_k_hat, d_k_d_hat, d_k_dd_hat


    def KF(self,z,dt):
        '''
        Kalman Filter
        Inputs:
              - z: measurements
              - dt: time constant (time-varying)
        '''
        self.f.F = np.array([[1.,dt,0],
                    [0.,1.,dt],
                    [0.,0.,1.]])
        self.f.Q = Q_discrete_white_noise(dim=3, dt=dt, var=1e-2)
        
        self.f.predict()
        self.f.update(z)

        states = self.f.x.copy()

        xs = states[0]*self.d_max      # filtered counts
        vs = states[1]*self.d_max       # filtred counts/second

        # print(self.f.x, 'log-likelihood', self.f.log_likelihood)
        return xs, vs

  
        

## testing codes (example code to use Encoder_filter class)
# comment below codes after testing
'''
if __name__ == '__main__':
    # read data
    df = pd.read_csv('turn_0_dot_1.csv',sep=',',header=None)
    df = df.values
    time = df[:,0]
    counts = df[:,11]

    counts_max = 8390000
    d = counts/counts_max
    t = time

    # initialize filters
    Myfilters = Encoder_filters(d_max = counts_max)
    

    ##### 1. run high gain filte ########
    # (1) initialization
    counts_per_sec = np.zeros(d.shape)

    # (2) run function
    for iter in range(0,d.shape[0]):
        counts_per_sec[iter] = Myfilters.high_gain_filter(t[iter],d[iter])

    # (3) plotting
    plt.plot(time,counts)
    plt.plot(time,counts_per_sec)
    plt.show()
  

    ####### 1. calculated Counts/second #######
    cal_counts_per_sec = np.zeros(time.shape)
    dt = 0.1
    for iter in range(0,d.shape[0]):
        if iter>0:
            dt = t[iter]-t[iter-1]
            cal_counts_per_sec[iter] = counts_max*(d[iter]-d[iter-1])/dt


    ##### 2. run Kalman filter ########
    KF_counts = np.zeros(d.shape)
    KF_counts_per_sec = np.zeros(d.shape)
    dt = 0.1
    for iter in range(0,d.shape[0]):
        if iter > 0:
            dt = t[iter]-t[iter-1]
        KF_counts[iter],KF_counts_per_sec[iter] = Myfilters.KF(z=d[iter],dt=dt)
    

    ##### 3. run MH-quau-filter ########
    # (1) Initialization
    counts_estimate = np.zeros(d.shape)
    counts_per_sec1 = np.zeros(d.shape)
    counts_per_sec2 = np.zeros(d.shape)

    # (2) run function
    for iter in range(0,d.shape[0]):
        counts_estimate[iter], counts_per_sec1[iter], counts_per_sec2[iter] = Myfilters.recursive_MH_Quau_filter(t[iter],d[iter])

    # (3) Plotting
    #plt.plot(time,counts,label='raw_counts')
    #plt.plot(time,counts_estimate, label='filtered_counts')
    #plt.plot(time,counts_per_sec1, label='Counts_second_MH_quau_without_low_pass')
    plt.plot(time,cal_counts_per_sec,'ko', label='Calculated')
    plt.plot(time,KF_counts_per_sec, label='Kalman_filter')
    plt.plot(time,counts_per_sec2, label='MH_quau_with_low_pass')
    plt.legend()
    plt.title('counts/second')
    plt.show()
'''
