import math
import numpy as np

# generate cross product matrix for 3 x 1 vector
# expecting k as a 
def hat(k):
    khat = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return khat

# generate 3 x 3 rotation matrix for theta degrees about k
def rot(k, theta):
    I = np.identity(3)
    khat = hat(k)
    khat2 = khat.dot(khat)
    return I + math.sin(theta)*khat + (1.0 - math.cos(theta))*khat2

# generate screw matrix
def screw_matrix(r):
    G1 = np.concatenate((np.identity(3), hat(r)), axis = 1)
    G2 = np.concatenate((np.zeros((3,3)), np.identity(3)), axis = 1)
    return np.concatenate((G1, G2), axis = 0)

# convert quaternion to 3 x 3 rotation matrix
def q2R(q):
    I = np.identity(3)
    qhat = hat(q[1:4])
    qhat2 = qhat.dot(qhat)
    return I + 2*q[0]*qhat + 2*qhat2;

# convert 3 x 3 rotation matrix to quaternion
def R2q(R):
    tr = np.trace(R)
    if tr > 0:
        S = 2*math.sqrt(tr + 1)
        q = np.array([[0.25*S], \
                      [(R[2,1] - R[1,2]) / S], \
                      [(R[0,2] - R[2,0]) / S], \
                      [(R[1,0] - R[0,1]) / S]])
                      
    elif (R[0,0] > R[1,1] and R[0,0] > R[2,2]):
        S = 2*math.sqrt(1 + R[0,0] - R[1,1] - R[2,2])
        q = np.array([[(R[2,1] - R[1,2]) / S], \
                      [0.25*S], \
                      [(R[0,1] + R[1,0]) / S], \
                      [(R[0,2] + R[2,0]) / S]])
    elif (R[1,1] > R[2,2]):
        S = 2*math.sqrt(1 - R[0,0] + R[1,1] - R[2,2])
        q = np.array([[(R[0,2] - R[2,0]) / S], \
                      [(R[0,1] + R[1,0]) / S], \
                      [0.25*S], \
                      [(R[1,2] + R[2,1]) / S]])
    else:
        S = 2*math.sqrt(1 - R[0,0] - R[1,1] + R[2,2])
        q = np.array([[(R[1,0] - R[0,1]) / S], \
                      [(R[0,2] + R[2,0]) / S], \
                      [(R[1,2] + R[2,1]) / S], \
                      [0.25*S]])
    return q

# returns quaternion complement
def quatcomplement(q):
    return np.array([q[0],-1*q[1],-1*q[2],-1*q[3]])

# returns 4 x 4 quaternion matrix for left product
def quatproduct(q):
    I = np.identity(3)
    return np.concatenate((q,np.concatenate((-1*q[1:4].T, q[0]*I + hat(q[1:4])),axis = 0)), axis = 1)

# returns jacobian relating angular velocity to quaternion velocity    
def quatjacobian(q):
    I = np.identity(3)
    J = 0.5 * np.concatenate((q[1:4].T, q[0]*I - hat(q[1:4])), axis = 0)
    return J

# forward kinematics (R, p) for a kinematic chain
def fwdkin(H, P, joint_type, theta):
    p = P[:,[0]]
    R = np.identity(3)
    for i in xrange(0,joint_type.size):
        if (joint_type[i] == 0 or joint_type[i] == 2):
            R = R.dot(rot(H[:,[i]],theta[i]))
        elif (joint_type[i] == 1 or joint_type[i] == 3):
            p = p + theta[i] * R.dot(H[:,[i]])
        p = p + R.dot(P[:,[i+1]])
    return (R, p)

# jacobian of forward kinematics for a kinematic chain
def robotjacobian(H, P, joint_type, theta):
    hi = np.zeros(H.shape)
    pOi = np.zeros(P.shape)
    
    p = P[:,[0]]
    R = np.identity(3)
    
    pOi[:,[0]] = p
    
    for i in xrange(0, joint_type.size):
        if (joint_type[i] == 0 or joint_type[i] == 2):
            R = R.dot(rot(H[:,[i]],theta[i]))
        elif (joint_type[i] == 1 or joint_type[i] == 3):
            p = p + theta[i] * R.dot(H[:,[i]])
        p = p + R.dot(P[:,[i+1]])
        pOi[:,[i+1]] = p
        hi[:,[i]] = R.dot(H[:,[i]])
    
    pOT = pOi[:,[joint_type.size]]
    J = np.zeros([6,joint_type.size])
    i = 0
    j = 0
    while (i < joint_type.size):
        if (joint_type[i] == 0):
            J[0:3,[j]] = hi[:,[i]]
            J[3:6,[j]] = hat(hi[:,[i]]).dot(pOT - pOi[:,[i]])
        elif (joint_type[i] == 1):
            J[3:6,[j]] = hi[:,[i]]
        elif (joint_type[i] == 3):
            J[3:6,[j]] = rot(hi[:,[i+2]], theta[i+2]).dot(hi[:,[i]])
            J[0:3,[j+1]] = hi[:,[i+2]]
            J[3:6,[j+1]] = hat(hi[:,[i+2]]).dot(pOT - pOi[:,[i+2]])
            J = J[:,0:-1]
            i = i + 2
            j = j + 1
        
        i = i + 1
        j = j + 1
    return J

