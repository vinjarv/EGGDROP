import numpy as np

class KalmanFilter:
    def __init__(self, A, B, H, Q, R):
        self.A = A
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R
        self.x_est = np.zeros((A.shape[0], 1))
        self.Pk = np.zeros(A.shape)

    def update(self, z, u):
        # Shorter name for member fields
        A, B, H, Q, R = self.A, self.B, self.H, self.Q, self.R
        x_est, Kk, Pk = self.x_est, self.Kk, self.Pk

        # Kalman algorithm
        # Predict
        x_est = A@x_est + B@u
        Pk = A@Pk@A.T + Q

        # Correct
        Kk = Pk@H.T @ np.linalg.inv(H@Pk@H.T + R)
        x_est = x_est + Kk@(z - H@x_est)
        Pk = (np.eye(3) - Kk@H) @ Pk
        
        # Store state
        self.x_est, self.Kk, self.Pk = x_est, Kk, Pk
        return x_est

    # System matrices
    A, B, H = 0, 0, 0
    # Covariance matrices
    Q, R = 0, 0
    # Persistent states
    Kk = [0] # Kalman gain
    Pk = 0 # Estimated error covariance
    x_est = 0 # States
