
import numpy as np
from pykalman import KalmanFilter
import scipy.linalg

class KLF(object):

    def __init__(self):
        self._trans_mat = np.eye(6)
        self._trans_conv = scipy.linalg.block_diag(np.eye(3)*0.05, np.eye(3)*0.2)
        self._trans_conv[2,2] = 0.0872665
        self._trans_conv[5,5] = 0.349066

        self._observation_mat = np.block([np.eye(3),np.eye(3)*0])
        self._observation_conv = np.eye(3) * 0.1
        self._observation_conv[2,2] = 0.0872665

        self._filter = None

        #self._initialize()

    def _initialize(self, cur_state):
        #reinitialize kalman filter
        self._filter = KalmanFilter(
            observation_matrices=self._observation_mat,
            transition_covariance=self._trans_conv,
            observation_covariance=self._observation_conv,
            transition_offsets=np.array([0,0,0,0,0,0]),
            observation_offsets=np.array([0,0,0]),
            n_dim_state=6,
            n_dim_obs=3
        )

        self._filtered_state = np.hstack([cur_state, [0,0,0]])
        self._filtered_covariance = np.eye(6)*0.1

    def filter(self, cur_state, dt):

        #first time, we just initialize stuff
        if(self._filter is None):
            self._initialize(cur_state)
            return cur_state

        #if dt is certain time, restart filter
        if(dt > 1.25):
            #reinitialize
            print("KLF timed out, dt{}".format(dt))
            self._initialize(cur_state)
            return cur_state
        
        #do kalman update
        self._trans_mat[:3,3:] = np.eye(3)*dt
        #update
        nxt_state, nxt_conv = self._filter.filter_update(
            self._filtered_state,
            self._filtered_covariance,
            cur_state,
            transition_matrix=self._trans_mat
        )
        self._filtered_state = nxt_state
        self._filtered_covariance = nxt_conv
        return nxt_state


def main():
    KLF()

if __name__ == '__main__':
    main()