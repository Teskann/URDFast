"""
The  code  in  this  file  has  been  generated  by  URDFast  Code Generator on
01/10/2021,  16:03:41.  Consider  testing  this  code before using it as errors
remain  possible.  For  more  details,  check out the github repository of this
project at https://github.com/Teskann/URDFast.
"""

from math import cos, sin
from numpy import array, cross, dot, zeros, eye

# -----------------------------------------------------------------------------
# |                        FORWARD TRANSITION MATRICES                        |
# -----------------------------------------------------------------------------

# Joint 0 _____________________________________________________________________

def T_joint1(theta_joint1):
    """
    Description
    -----------
    
    Transition Matrix to go from link link1 to link link2.
    This joint is continuous. The matrix is :
    
    [1          0                  0           0]
    [                                           ]
    [0  cos(theta_joint1)  -sin(theta_joint1)  0]
    [                                           ]
    [0  sin(theta_joint1)  cos(theta_joint1)   0]
    [                                           ]
    [0          0                  0           1]
    
    Parameters
    ----------
    
    theta_joint1 : float
        Rotation value (in radians) around the joint1 joint axis.
    
    """

    v_cos = cos(theta_joint1)
    v_sin = sin(theta_joint1)
    
    # Returned Matrix
    mat = array([[1,0,0,0],
          [0,v_cos,-v_sin,0],
          [0,v_sin,v_cos,0],
          [0,0,0,1]])

    return mat


# Joint 1 _____________________________________________________________________

def T_joint2(theta_joint2):
    """
    Description
    -----------
    
    Transition Matrix to go from link link1 to link link3.
    This joint is continuous. The matrix is :
    
    [1          0                  0           0]
    [                                           ]
    [0  cos(theta_joint2)  -sin(theta_joint2)  0]
    [                                           ]
    [0  sin(theta_joint2)  cos(theta_joint2)   0]
    [                                           ]
    [0          0                  0           1]
    
    Parameters
    ----------
    
    theta_joint2 : float
        Rotation value (in radians) around the joint2 joint axis.
    
    """

    v_cos = cos(theta_joint2)
    v_sin = sin(theta_joint2)
    
    # Returned Matrix
    mat = array([[1,0,0,0],
          [0,v_cos,-v_sin,0],
          [0,v_sin,v_cos,0],
          [0,0,0,1]])

    return mat


# Joint 2 _____________________________________________________________________

def T_joint3(theta_joint3):
    """
    Description
    -----------
    
    Transition Matrix to go from link link3 to link link4.
    This joint is continuous. The matrix is :
    
    [1          0                  0           0]
    [                                           ]
    [0  cos(theta_joint3)  -sin(theta_joint3)  0]
    [                                           ]
    [0  sin(theta_joint3)  cos(theta_joint3)   0]
    [                                           ]
    [0          0                  0           1]
    
    Parameters
    ----------
    
    theta_joint3 : float
        Rotation value (in radians) around the joint3 joint axis.
    
    """

    v_cos = cos(theta_joint3)
    v_sin = sin(theta_joint3)
    
    # Returned Matrix
    mat = array([[1,0,0,0],
          [0,v_cos,-v_sin,0],
          [0,v_sin,v_cos,0],
          [0,0,0,1]])

    return mat




# -----------------------------------------------------------------------------
# |                            FORWARD KINEMATICS                             |
# -----------------------------------------------------------------------------

def fk_link1_link2(q):
    """
    Description
    -----------
    
    Computes  the forward kinematics from the link link1 to the link link2. The
    result  is  returned  as  a  4x4  numpy.ndarray in homogeneous coordinates,
    giving the position and the orientation of link2 in the link1 frame.
    
    Parameters
    ----------
    
    q : numpy.ndarray
        Vector of variables where :
            - q[0] = theta_joint1 :
                  Rotation value (in radians) around the joint1 joint axis.
    
    """

    v_cos = cos(q[0])
    v_sin = sin(q[0])
    
    # Returned Matrix
    mat = array([[1.00000000000000,0,0,0],
          [0,v_cos,-v_sin,0],
          [0,v_sin,v_cos,0],
          [0,0,0,1.00000000000000]])

    return mat


def fk_link1_link4(q):
    """
    Description
    -----------
    
    Computes  the forward kinematics from the link link1 to the link link4. The
    result  is  returned  as  a  4x4  numpy.ndarray in homogeneous coordinates,
    giving the position and the orientation of link4 in the link1 frame.
    
    Parameters
    ----------
    
    q : numpy.ndarray
        Vector of variables where :
            - q[0] = theta_joint2 :
                  Rotation value (in radians) around the joint2 joint axis.
            - q[1] = theta_joint3 :
                  Rotation value (in radians) around the joint3 joint axis.
    
    """

    v_cos = cos(q[0])
    v_cos_0 = cos(q[1])
    v_sin = sin(q[0])
    v_sin_0 = sin(q[1])
    v_prod = v_cos*v_cos_0
    v_prod_0 = v_sin_0*v_cos
    v_negative = -v_sin
    v_prod_1 = v_negative*v_sin_0
    v_sum = v_prod_1+v_prod
    
    # Returned Matrix
    mat = array([[1.00000000000000,0,0,0],
          [0,v_sum,v_negative*v_cos_0-v_prod_0,0],
          [0,v_sin*v_cos_0+v_prod_0,v_sum,0],
          [0,0,0,1.00000000000000]])

    return mat


# -----------------------------------------------------------------------------
# |                                 JACOBIANS                                 |
# -----------------------------------------------------------------------------

# Jacobian of the link_1 position and orientation _____________________________

def jacobian_link1_to_link2(p0, q):
    """
    Description
    -----------
    
    Computes  the  Jacobian  Matrix of the link2 coordinates in the link1 frame
    from  the point p0. This matrix is returned as a (6 x 1) matrix where every
    column  is  the  derivative  of  the position/orientation with respect to a
    degree of freedom.
          -  The  line  1 is the derivative of X position of link2 in the link1
          frame,
          -  The  line  2 is the derivative of Y position of link2 in the link1
          frame,
          -  The  line  3 is the derivative of Z position of link2 in the link1
          frame,
          -  The  line  4 is the derivative of the roll orientation of link2 in
          the link1 frame,
          -  The  line 5 is the derivative of the pitch orientation of link2 in
          the link1 frame,
          - The line 6 is the derivative of the yaw orientation of link2 in the
          link1 frame,
    Here is the list of all the derivative variables :
        - Column 0 : theta_joint1
    
    Parameters
    ----------
    
    p0 : numpy.ndarray
        Point in the link1 frame where you want to compute the Jacobian Matrix.
        p0 is a (3 x 1) vector.
    
    q : numpy.ndarray
        Vector  of  length 1 containing all the degrees of freedom of the robot
        between link1 and link2 chain. This vector contains :
            - q[0] = theta_joint1 :
                  Rotation value (in radians) around the joint1 joint axis.
    
    """

    Jac = zeros((6, 1))
    T = T_joint1(q[0])
    L = p0-T[0:3,3]
    Z = T[0:3,2]
    Jac[0:3,0] = cross(Z,L)
    Jac[3:6,0] = Z
    
    return Jac


# Jacobian of the link_3 position and orientation _____________________________

def jacobian_link1_to_link4(p0, q):
    """
    Description
    -----------
    
    Computes  the  Jacobian  Matrix of the link4 coordinates in the link1 frame
    from  the point p0. This matrix is returned as a (6 x 2) matrix where every
    column  is  the  derivative  of  the position/orientation with respect to a
    degree of freedom.
          -  The  line  1 is the derivative of X position of link4 in the link1
          frame,
          -  The  line  2 is the derivative of Y position of link4 in the link1
          frame,
          -  The  line  3 is the derivative of Z position of link4 in the link1
          frame,
          -  The  line  4 is the derivative of the roll orientation of link4 in
          the link1 frame,
          -  The  line 5 is the derivative of the pitch orientation of link4 in
          the link1 frame,
          - The line 6 is the derivative of the yaw orientation of link4 in the
          link1 frame,
    Here is the list of all the derivative variables :
        - Column 0 : theta_joint2
        - Column 1 : theta_joint3
    
    Parameters
    ----------
    
    p0 : numpy.ndarray
        Point in the link1 frame where you want to compute the Jacobian Matrix.
        p0 is a (3 x 1) vector.
    
    q : numpy.ndarray
        Vector  of  length 2 containing all the degrees of freedom of the robot
        between link1 and link4 chain. This vector contains :
            - q[0] = theta_joint2 :
                  Rotation value (in radians) around the joint2 joint axis.
            - q[1] = theta_joint3 :
                  Rotation value (in radians) around the joint3 joint axis.
    
    """

    Jac = zeros((6, 2))
    T = T_joint2(q[0])
    L = p0-T[0:3,3]
    Z = T[0:3,2]
    Jac[0:3,0] = cross(Z,L)
    Jac[3:6,0] = Z
    T = dot(T,T_joint3(q[1]))
    L = p0-T[0:3,3]
    Z = T[0:3,2]
    Jac[0:3,1] = cross(Z,L)
    Jac[3:6,1] = Z
    
    return Jac


# -----------------------------------------------------------------------------
# |                          POLYNOMIAL TRAJECTORIES                          |
# -----------------------------------------------------------------------------

# Trajectory r ________________________________________________________________

def d0_r(t):
    """
    Description
    -----------
    
    Trajectory  polynomial. This function returns the position as a function of
    time.  The  polynomial  was created following some conditions. The followed
    conditions are :
        - r(0) = 0
        - r(1) = 1
        - d_r/dt(0) = 0
        - d_r/dt(1) = 0
        - d^2_r/dt^2(0) = 0
        - d^2_r/dt^2(1) = 0
    
    BE  CAREFUL  : This function does not take into account the physical limits
    of the robot (maximum velocities, accelerations, positions ...).
    
    Parameters
    ----------
    
    t : float
        Time variable
    
    """

    return 15.0*t**3*(0.4*t**2-t+0.666666666666667)


def d1_r(t):
    """
    Description
    -----------
    
    Trajectory  polynomial.  This  function  returns the speed as a function of
    time.  The  polynomial  was created following some conditions. The followed
    conditions are :
        - r(0) = 0
        - r(1) = 1
        - d_r/dt(0) = 0
        - d_r/dt(1) = 0
        - d^2_r/dt^2(0) = 0
        - d^2_r/dt^2(1) = 0
    
    BE  CAREFUL  : This function does not take into account the physical limits
    of the robot (maximum velocities, accelerations, positions ...).
    
    Parameters
    ----------
    
    t : float
        Time variable
    
    """

    return 30.0*t**2*(t-1.0)**2


def d2_r(t):
    """
    Description
    -----------
    
    Trajectory polynomial. This function returns the acceleration as a function
    of time. The polynomial was created following some conditions. The followed
    conditions are :
        - r(0) = 0
        - r(1) = 1
        - d_r/dt(0) = 0
        - d_r/dt(1) = 0
        - d^2_r/dt^2(0) = 0
        - d^2_r/dt^2(1) = 0
    
    BE  CAREFUL  : This function does not take into account the physical limits
    of the robot (maximum velocities, accelerations, positions ...).
    
    Parameters
    ----------
    
    t : float
        Time variable
    
    """

    return 120.0*t*(t-1.0)*(t-0.5)



