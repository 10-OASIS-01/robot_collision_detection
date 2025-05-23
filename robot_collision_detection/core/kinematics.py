import numpy as np

def dh_transform(a, alpha, d, theta):
    """
    Calculate the homogeneous transformation matrix based on DH parameters.
    
    Args:
        a: Link length
        alpha: Link twist
        d: Link offset
        theta: Joint angle
    
    Returns:
        4x4 homogeneous transformation matrix
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])