import numpy as np 

def process_matrix(message):
    m = np.array([[[message.m00, message.m01, message.m02, message.m03],
                    [message.m10, message.m11, message.m12, message.m13],
                    [message.m20, message.m21, message.m22, message.m23],
                    [0, 0, 0, 1]]])
    return m 

def process_matrices(skeleton, matrix = np.eye(4)):
    return np.concatenate([matrix @ process_matrix(joint) for joint in skeleton], axis = 0)
