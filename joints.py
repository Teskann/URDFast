"""
Joint Elements
"""

import numpy as np
from sympy import nsimplify, Matrix, cos, sin, Symbol, Min, Max
from abc import ABC, abstractmethod


# ----------------------------------------------------------------------------
# | Joint Class                                                              |
# ----------------------------------------------------------------------------

class Joint(ABC):
    """
    Description
    -----------

    Joint class. Used to describe joint objects, either in URDF (see JointURDF
    class) or for Denavit–Hartenberg joints (see JointDH class).
    This class is not supposed to be instantiated.

    Data Structure
    --------------

    name : str
        Joint name. This field is required or an error will be thrown

    parent : int
        Parent  link id. Corresponds to the link_id of the link which name
        is  the  <parent>  element  of the <joint> element in the URDF XML
        representation.

    child : int
        Child  link id.  Corresponds to the link_id of the link which name
        is  the  <child>  element  of  the <joint> element in the URDF XML
        representation.

    T : sympy.matrices.immutable.ImmutableDenseMatrix
        Transition matrix of the joint

    Tinv : sympy.matrices.immutable.ImmutableDenseMatrix
        Inverse of the transition matrix of the joint

    """

    # Init ___________________________________________________________________

    @abstractmethod
    def __init__(self, name, parent, child):
        """
        Description
        -----------

        Joint  constructor.  This  is  the  super  constructor  that  MUST  be
        overridden in the child classes.

        Parameters
        ----------

        name : str
            Joint name. This field is required or an error will be thrown

        parent : int
            Parent  link id. Corresponds to the link_id of the link which name
            is  the  <parent>  element  of the <joint> element in the URDF XML
            representation.

        child : int
            Child  link id.  Corresponds to the link_id of the link which name
            is  the  <child>  element  of  the <joint> element in the URDF XML
            representation.

        Returns
        -------

        None.

        """

        self.name = name
        self.parent = parent
        self.child = child
        self.T = None
        self.Tinv = None
        self.update_T()

    # Update T _______________________________________________________________

    def update_T(self):
        """
        Description
        -----------

        Updates the T and Tinv attributes calling self.__T()
        """

        self.T = self.T_()
        self.Tinv = (self.T ** (-1)).simplify()
        self.Tinv = nsimplify(self.Tinv, tolerance=1e-10).evalf()

    # T ______________________________________________________________________

    @abstractmethod
    def T_(self, consider_limits=False, tolerance=1e-10):
        """
        Description
        -----------

        Returns  the  transformation  matrix from the parent link to the child
        link in homogeneous coordinates.
        This matrix is computed symbolically

        The degrees of freedom names are formatted like this :
            prefix_name

            where name is the joint name (self.joint_name) and prefix is :
                - 'd' for 1 DOF translation
                - 'dx' / 'dy' for 2 DOF translations
                - 'dx' / 'dy' / 'dz' for 3 DOF translations
                - 'theta' for 1 DOF rotations
                - 'roll' / 'pitch' / 'yaw' for 3 DOF rotations

        Parameters
        ----------

        consider_limits : bool
            If  True,  the  transition matrices ensure it's not possible to go
            beyond  self.limit_lower and self.limit_upper. This results adding
            max  and  min  functions in the transition matrix. For example, if
            the  degree  of  freedom is d and consider_limits is True, every d
            value will become :
                min(max(d, self.limit_lower), self.limit_upper)
            (if the limits are not None).

            This can cause derivation  problems so you can disable this option
            turning consider_limits to  False.  Note that some types of joints
            NEED limits  (revolute & prismatic)  and  some  just  ignore them.

            Be  careful  with  this option, as the default limit values are 0.
            So,  if  your  URDF  file doesn't specify limits, this option will
            allow  the  DOF  to  move  between 0 and 0. This will consequently
            "remove" the degree of freedom from the transition matrix.

            Defaults to False

        tolerance : float
            Tolerance for simplification of the expression.

            Defaults to 1e-10

        Returns
        -------
        sympy.matrices.immutable.ImmutableDenseMatrix
            Transformation  matrix  from  the parent link to the child link in
            homogeneous coordinates. The shape is (4, 4)

        """
        pass

    # Valid __________________________________________________________________

    @abstractmethod
    def valid(self):
        """
        Description
        -----------

        This function checks if the Joint object is valid, ie if it has :
            - A str name
            - A list of valid parent links
            - A list of valid child links
        This  function  raises  an exception if any of these statements is not
        true.

        Returns
        -------

        True if no exceptions are raised

        """
        # self.name ..........................................................

        if self.name is None:
            raise ValueError(
                "Joint name is None. You must give it a valid " +
                "name (str)")

        if type(self.name) != str:
            raise TypeError(
                "Joint name must be a str and is currently a " +
                f"{type(self.name)}")

        # self.parent ........................................................

        if type(self.parent) != int:
            raise TypeError("Joint parent must be an integer and is " +
                            f"currently a {type(self.parent)}")

        if self.parent < 0:
            raise ValueError("Joint parent must be a positive integer")

        # self.child .........................................................

        if type(self.child) != int:
            raise TypeError("Joint child must be an integer and is " +
                            f"currently a {type(self.child)}")

        if self.child < 0:
            raise ValueError("Joint child must be a positive integer")

        return True


# ----------------------------------------------------------------------------
# | JointURDF Class                                                          |
# ----------------------------------------------------------------------------

class JointURDF(Joint):
    """
    Description
    -----------

    Joint Class for URDF files.

    This class describes joint objects. These objects represent URDF joint
    elements. This class is used inside the Robot class (see below).
    The  joint  element describes the kinematics and dynamics of the joint
    and also specifies the safety limits of the joint.

    This class  doesn't deal with <calibration> and <dynamics> elements as
    they are not useful in this project.
    <safety_controller>   elements  and  <mimic>  elements  are  also  not
    implemented.

    For  more  details  about  this  object description, refer to the URDF
    Joint documentation : http://wiki.ros.org/urdf/XML/joint

    Inherrited from Joint
    ---------------------

    name : str
        Joint name. This field is required or an error will be thrown

    parent : int
        Parent  link id. Corresponds to the link_id of the link which name
        is  the  <parent>  element  of the <joint> element in the URDF XML
        representation.

    child : int
        Child  link id.  Corresponds to the link_id of the link which name
        is  the  <child>  element  of  the <joint> element in the URDF XML
        representation.

    T : sympy.matrices.immutable.ImmutableDenseMatrix
        Transition matrix of the joint

    Tinv : sympy.matrices.immutable.ImmutableDenseMatrix
        Inverse of the transition matrix of the joint

    Data Structure
    --------------

    joint_type : str
        Specifies  the  type  of  joint,  where  type  can  be  one of the
        following :
            - 'revolute' :
                A  hinge  joint  that  rotates  along  the  axis and has a
                limited range specified by the upper and lower limits.
            - 'continuous' :
                A  continuous hinge joint that rotates around the axis and
                has no upper and lower limits.
            - 'prismatic' :
                A  sliding  joint  that  slides  along the axis, and has a
                limited range specified by the upper and lower limits.
            - 'fixed' :
                This  is  not  really a joint  because it cannot move. All
                degrees of freedom are locked. This type of joint does not
                require  the   axis,   calibration,  dynamics,  limits  or
                safety_controller.
            - 'floating' :
                This joint allows motion for all 6 degrees of freedom.
            - 'planar' :
                This  joint  allows motion in a plane perpendicular to the
                axis.

    origin_xyz : 3 x 1 numpy.ndarray
        Translation from the parent link to the child link

        SI Unit : m

        Default : Null 3x1 Vector : numpy.zeros((3, 1))

    origin_rpy : 3 x 1 numpy.ndarray
        Represents  the  rotation around fixed axis : first roll around x,
        then pitch around y and finally yaw around z.

        SI Unit : radians

        Default : Null 3x1 Vector : numpy.zeros((3, 1))

    axis : 3 x 1 numpy.ndarray
        The  joint  axis specified in the joint frame. This is the axis of
        rotation  for  revolute  joints,   the  axis  of  translation  for
        prismatic  joints,  and  the surface normal for planar joints. The
        axis  is  specified  in  the  joint  frame of reference. Fixed and
        floating  joints  do  not  use  the  axis  field.  The  vector  is
        automatically normalized before being stored in the Joint object.

        SI Unit : (doesn't matter)

        Default : x axis vector : np.array([[1, 0, 0]]).T

    limit_lower : float / int
        An  attribute  specifying  the  lower  joint  limit  (radians  for
        revolute  joints,  meters  for prismatic joints). Omit if joint is
        continuous.

        SI Unit : radians for revolute joints, m for prismatic joints

        Default : 0 m for prismatic joints, 0 radians for revolute joints

    limit_upper : float / int
        An  attribute  specifying  the  higher joint  limit  (radians  for
        revolute  joints,  meters  for prismatic joints). Omit if joint is
        continuous.

        SI Unit : radians for revolute joints, m for prismatic joints

        Default : 0 m for prismatic joints, 0 radians for revolute joints

    limit_effort : float / int
        The effort limit is an attribute of the limit tag. In this case, a
        controller  cannot  command  an  effort of more than 30 N (N-m for
        revolute) nor less than -30 N (N-m for revolute) on the joint.  If
        the controller tries to command an effort beyond the effort limit,
        the magnitude of the effort is truncated.

        SI Unit : N for prismatic joints, N.m for revolute joints

    limit_velocity : float / int
        An attribute for enforcing the maximum joint velocity.

        SI Unit : m/s for prismatic joints, rad/s for revolute joints

    Constructor
    -----------

    Only one constructor is implemented.
    It creates a Joint from an URDF Object and a joint number

    Parameters :

        urdfObject : URDF
            URDF object from URDF library
        joint_number : int
            Joint  Number  in  the URDF file tree.  Must be fewer than the
            total number of joints in the URDF object.

    Examples :

        Example  URDF  files  are  located  in  ./Examples  where . is the
        directory containing robots.py

        - Running example_0.urdf URDF file (pretty simple <joint>) :
        >>> from URDF import URDF
        >>> urdf_obj = URDF("./Examples/example_0.urdf")
        >>> joint_obj = JointURDF(urdf_obj, 0)
        >>> print(joint_obj)
        Joint Name : joint1
        Joint Type : continuous
        Joint origin XYZ (m):
        [[0.]
         [0.]
         [0.]]
        Joint origin RPY (rad) :
        [[0.]
         [0.]
         [0.]]
        Parent Link ID : 0
        Child Link ID : 1
        Joint Axis :
        [[1]
         [0]
         [0]]
        Joint Limits :
            Lower : None
            Upper : None
            Effort : None
            Velocity : None

        - Running example_1.urdf URDF file (more complex <joint>) :
        >>> urdf_obj = URDF("./Examples/example_1.urdf")
        >>> joint_obj = JointURDF(urdf_obj, 0)
        >>> print(joint_obj)
        Joint Name : joint1
        Joint Type : prismatic
        Joint origin XYZ (m):
        [[0.]
         [0.]
         [1.]]
        Joint origin RPY (rad) :
        [[0.    ]
         [0.    ]
         [3.1416]]
        Parent Link ID : 0
        Child Link ID : 1
        Joint Axis :
        [[0.]
         [1.]
         [0.]]
        Joint Limits :
            Lower : -2.2
            Upper : 0.7
            Effort : 30.0
            Velocity : 1.0
    """
    # Data ===================================================================

    # Joint Name
    name = None

    # Joint Type
    joint_type = None

    # Origin Coordinates
    origin_xyz = np.zeros((3, 1))

    # Origin Rotation
    origin_rpy = np.zeros((3, 1))

    # Parent Link
    parent = None

    # Child Link
    child = None

    # Axis
    axis = np.array([[1, 0, 0]]).T

    # Position Lower Limit
    limit_lower = 0.0

    # Position Upper Limit
    limit_upper = 0.0

    # Effort Limit
    limit_effort = None

    # Max Velocity Limit
    limit_velocity = None

    # Constructor ============================================================

    def __init__(self, urdf_object, joint_number):
        """
        Description
        -----------

        Only one constructor is implemented.
        It creates a Joint from an URDF Object and a joint number

        Parameters
        ----------

        urdf_object : URDF.URDF
            URDF object from URDF library
        joint_number : int
            Joint  Number  in the URDF file tree. Must be fewer than the total
            number of joints in the URDF object.

        Examples
        --------

        Example URDF files are located in ./Examples  where . is the directory
        directory containing robots.py

        >>> from URDF import URDF
        >>> urdf_obj = URDF("./Examples/example_0.urdf")
        >>> joint_obj = JointURDF(urdf_obj, 0)
        """
        # Checking if the joint exists
        if joint_number > urdf_object.njoints():
            raise IndexError(f"joint_number ({joint_number}) out of range " +
                             f"({urdf_object.njoints()})")

        urdf_object_joint = urdf_object.joints[joint_number]

        # 1 - Joint Name .....................................................

        if 'name' in urdf_object_joint.keys():
            name = urdf_object_joint['name']
        else:
            raise KeyError("Joint must have a name")

        # 2 - Joint type .....................................................

        if 'type' in urdf_object_joint.keys():
            self.joint_type = urdf_object_joint['type']
        else:
            raise KeyError("Joint must have a type")

        # 3 - Origin XYZ & RPY ...............................................

        if 'origin' in urdf_object_joint.keys():

            # Simplifying notation
            origin_dict = urdf_object_joint['origin']

            # Origin XYZ . . . . . . . . . . . . . . . . . . . . . . . . . . .

            if 'xyz' in origin_dict.keys():
                # File Value
                self.origin_xyz = np.array(origin_dict['xyz']).reshape(3, 1)
            else:
                # Default Value
                self.origin_xyz = np.zeros((3, 1))

            # Origin RPY . . . . . . . . . . . . . . . . . . . . . . . . . . .

            if 'rpy' in origin_dict.keys():
                # File Value
                self.origin_rpy = np.array(origin_dict['rpy']).reshape(3, 1)
            else:
                # Default Value
                self.origin_rpy = np.zeros((3, 1))

        # 4 - Parent Link ....................................................

        parent = None
        if 'parent' in urdf_object_joint.keys():

            # Simplifying notation
            parent_dict = urdf_object_joint['parent']

            # Must have a 'link' element
            if 'link' in parent_dict.keys():

                # Searching the parent link in link list
                for i, linkURDF in enumerate(urdf_object.links):

                    # If found, get the index value
                    if parent_dict['link'] == linkURDF['name']:
                        parent = i
                        break
            else:
                raise KeyError("Joint Parent Link must have a Link property")
        else:
            raise KeyError("Joint must have a Parent Link")

        # 5 - Child Link .....................................................

        child = None
        if 'child' in urdf_object_joint.keys():

            # Simplifying notation
            child_dict = urdf_object_joint['child']

            # Must have a 'link' element
            if 'link' in child_dict.keys():

                # Searching the child link in link list
                for i, linkURDF in enumerate(urdf_object.links):

                    # If found, get the index value
                    if child_dict['link'] == linkURDF['name']:
                        child = i
                        break
            else:
                raise KeyError("Joint Child Link must have a Link property")
        else:
            raise KeyError("Joint must have a child Link")

        # 6 - Axis ...........................................................

        if 'axis' in urdf_object_joint.keys():
            # File Value
            if 'xyz' in urdf_object_joint['axis'].keys():
                self.axis = np.array(urdf_object_joint['axis']['xyz']) \
                    .reshape((3, 1))
                self.axis /= np.linalg.norm(self.axis)
            # Default Value
            else:
                self.axis = np.array([[1, 0, 0]]).T

        # 7 - Limits .........................................................

        if 'limit' in urdf_object_joint.keys():

            # Simplifying Notation
            limit_dict = urdf_object_joint['limit']

            # Lower Limit  . . . . . . . . . . . . . . . . . . . . . . . . . .

            if 'lower' in limit_dict:
                # File Value
                self.limit_lower = float(limit_dict['lower'][0])
            else:
                # Default Value
                self.limit_lower = 0.0

            # Upper Limit  . . . . . . . . . . . . . . . . . . . . . . . . . .

            if 'upper' in limit_dict:
                # File Value
                self.limit_upper = float(limit_dict['upper'][0])
            else:
                # Default Value
                self.limit_upper = 0.0

            # Effort limit . . . . . . . . . . . . . . . . . . . . . . . . . .

            if 'effort' in limit_dict:
                # File Value
                self.limit_effort = float(limit_dict['effort'][0])
            else:
                # NO DEFAULT VALUE : REQUIRED for revolute & prismatic
                if self.joint_type in ['revolute', 'prismatic']:
                    raise KeyError(f"{self.joint_type} joint must have " +
                                   "effort limit properties")
                # None for other types of joints (doesn't matter)
                else:
                    self.limit_effort = None

            # Velocity Limit . . . . . . . . . . . . . . . . . . . . . . . . .

            if 'velocity' in limit_dict:
                # File Value
                self.limit_velocity = float(limit_dict['velocity'][0])
            else:
                # NO DEFAULT VALUE : REQUIRED for revolute & prismatic
                if self.joint_type in ['revolute', 'prismatic']:
                    raise KeyError(f"{self.joint_type} joint must have " +
                                   "velocity limit property")
                # None for other types of joints (doesn't matter)
                else:
                    self.limit_velocity = None

        # Limits required for revolute & prismatic
        elif self.joint_type in ['revolute', 'prismatic']:
            raise KeyError(f"{self.joint_type} joint must have limit " +
                           "property")

        else:
            self.limit_lower = None
            self.limit_upper = None
            self.limit_effort = None
            self.limit_velocity = None

        # Super call .........................................................

        super().__init__(name, parent, child)

        # Updating T .........................................................

        super().update_T()

        # 8 - Checking if the joint is valid .................................

        self.valid()

    # Methods ================================================================

    # Getting the transition Matrix T ________________________________________

    def T_(self, consider_limits=False, tolerance=1e-10):
        """
        Description
        -----------

        Returns  the  transformation  matrix from the parent link to the child
        link in homogeneous coordinates.
        This matrix is computed symbolically

        The degrees of freedom names are formatted like this :
            prefix_name

            where name is the joint name (self.joint_name) and prefix is :
                - 'd' for 1 DOF translation
                - 'dx' / 'dy' for 2 DOF translations
                - 'dx' / 'dy' / 'dz' for 3 DOF translations
                - 'theta' for 1 DOF rotations
                - 'roll' / 'pitch' / 'yaw' for 3 DOF rotations

        Parameters
        ----------

        consider_limits : bool
            If  True,  the  transition matrices ensure it's not possible to go
            beyond  self.limit_lower and self.limit_upper. This results adding
            max  and  min  functions in the transition matrix. For example, if
            the  degree  of  freedom is d and consider_limits is True, every d
            value will become :
                min(max(d, self.limit_lower), self.limit_upper)
            (if the limits are not None).

            This can cause derivation  problems so you can disable this option
            turning consider_limits to  False.  Note that some types of joints
            NEED limits  (revolute & prismatic)  and  some  just  ignore them.

            Be  careful  with  this option, as the default limit values are 0.
            So,  if  your  URDF  file doesn't specify limits, this option will
            allow  the  DOF  to  move  between 0 and 0. This will consequently
            "remove" the degree of freedom from the transition matrix.

            Defaults to False

        tolerance : float
            Tolerance for simplification of the expression.

            Defaults to 1e-10

        Returns
        -------
        sympy.matrices.immutable.ImmutableDenseMatrix
            Transformation  matrix  from  the parent link to the child link in
            homogeneous coordinates. The shape is (4, 4)

        """
        # Initialisation
        T = Matrix([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

        # Translation ........................................................

        T[0:3, 3] = self.origin_xyz

        # Rotation ...........................................................

        # Yaw around Z axis  . . . . . . . . . . . . . . . . . . . . . . . . .

        angle = self.origin_rpy[2, 0]
        yaw = Matrix([[cos(angle), -sin(angle), 0],
                      [sin(angle), cos(angle), 0],
                      [0, 0, 1]])

        # Pitch around Y axis  . . . . . . . . . . . . . . . . . . . . . . . .

        angle = self.origin_rpy[1, 0]
        pitch = Matrix([[cos(angle), 0, sin(angle)],
                        [0, 1, 0],
                        [-sin(angle), 0, cos(angle)]])

        # Roll around X axis . . . . . . . . . . . . . . . . . . . . . . . . .

        angle = self.origin_rpy[0, 0]
        roll = Matrix([[1, 0, 0],
                       [0, cos(angle), -sin(angle)],
                       [0, sin(angle), cos(angle)]])

        # Yaw * Pitch * Roll
        T[0:3, 0:3] = (yaw * pitch * roll).evalf()

        # Rodrigues formula ..................................................

        def rodrigues(axis, angle_):
            """
            Description
            -----------

            Computes the  Rodrigues rotation matrix from a rotation around the
            axis 'axis' with the rotation 'angle'
            https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula

            Parameters
            ----------

            axis : numpy.ndarray
                Axis around the one the rotation is made.
                Has to be normalized and shape = (3, 1)

            angle_ : sympy.core.symbol.Symbol
                Symbol for angle rotation (in radians)

            Returns
            -------
            sympy.matrices.dense.MutableDenseMatrix
                Rotation matrix of the angle 'angle' around the axis 'axis'.
                Shape is 3x3

            """

            # Cross Product Matrix
            k = Matrix([[0, -axis[2, 0], axis[1, 0]],
                        [axis[2, 0], 0, -axis[0, 0]],
                        [-axis[1, 0], axis[0, 0], 0]])

            identity = Matrix([[1, 0, 0],
                               [0, 1, 0],
                               [0, 0, 1]])

            return identity + sin(angle_) * k + (1 - cos(angle_)) * k ** 2

        # Adding degree(s) of freedom of the joint ...........................

        # Revolute Joints  . . . . . . . . . . . . . . . . . . . . . . . . . .

        if self.joint_type == 'revolute':

            # 1 degree of freedom around the axis

            # Degree of freedom
            theta = Symbol('theta_' + self.name)

            if consider_limits:
                theta_limit = Min(Max(theta, self.limit_lower),
                                  self.limit_upper)
                T[0:3, 0:3] *= rodrigues(self.axis, theta_limit)
            else:
                T[0:3, 0:3] *= rodrigues(self.axis, theta)

        # Continuous Joints  . . . . . . . . . . . . . . . . . . . . . . . . .

        elif self.joint_type == 'continuous':

            # 1 degree of freedom around the axis

            # Degree of freedom
            theta = Symbol('theta_' + self.name)

            T[0:3, 0:3] *= rodrigues(self.axis, theta)

        # Prismatic Joints . . . . . . . . . . . . . . . . . . . . . . . . . .

        elif self.joint_type == 'prismatic':

            # 1 degree of freedom along the axis
            # Since axis is normalized, the translation value is multiplied by
            # the axis to get its XYZ value

            d = Symbol('d_' + self.name)
            if consider_limits:
                d_limit = Min(Max(d, self.limit_lower), self.limit_upper)
                T[0:3, 3] += d_limit * self.axis
            else:
                T[0:3, 3] += d * self.axis

        # Fixed Joints . . . . . . . . . . . . . . . . . . . . . . . . . . . .

        elif self.joint_type == 'fixed':

            # Nothing to do here (no degrees of freedom)

            pass

        # Floating Joints  . . . . . . . . . . . . . . . . . . . . . . . . . .

        elif self.joint_type == 'floating':

            # 6 degrees of freedom (3 translations, 3 rotations)

            # Translation Symbols
            dx = Symbol('dx_' + self.name)
            dy = Symbol('dy_' + self.name)
            dz = Symbol('dz_' + self.name)

            # Translation transformations
            T[0, 3] += dx
            T[1, 3] += dy
            T[2, 3] += dz

            # Rotation Symbols
            droll = Symbol('roll_' + self.name)
            dpitch = Symbol('pitch_' + self.name)
            dyaw = Symbol('yaw_' + self.name)

            # Rotation transformations
            roll_rot = Matrix([[1, 0, 0],
                               [0, cos(droll), -sin(droll)],
                               [0, sin(droll), cos(droll)]])

            pitch_rot = Matrix([[cos(dpitch), 0, sin(dpitch)],
                                [0, 1, 0],
                                [-sin(dpitch), 0, cos(dpitch)]])

            yaw_rot = Matrix([[cos(dyaw), -sin(dyaw), 0],
                              [sin(dyaw), cos(dyaw), 0],
                              [0, 0, 1]])

            T[0:3, 0:3] *= (yaw_rot * pitch_rot * roll_rot)

        # Planar Joints  . . . . . . . . . . . . . . . . . . . . . . . . . . .

        elif self.joint_type == 'planar':

            # 2 degrees of freedom (translation in the normal plan)

            # Degrees of freedom
            # dx = Symbol('dx_' + self.name)
            # dy = Symbol('dy_' + self.name)

            print('Planar Joints not Supported yet')

        # A  bug  in  SymPy  is  not  rounding  float  values  if they are not
        # multiplied  by  a  Symbol.  To  fix  this, we multiply T by a random
        # Symbol, optimize and then divide by this Symbol.

        debug_sym = Symbol('debug_symbol')

        return nsimplify(T * debug_sym,
                         tolerance=tolerance).evalf() / debug_sym

    # Checks if the joint is valid ___________________________________________

    def valid(self):
        """
        Description
        -----------

        This function checks if the Joint object is valid, ie if it has :
            - A str name
            - A valid joint_type str
            - A 3 x 1 numpy.ndarray origin_xyz
            - A 3 x 1 numpy.ndarray origin_rpy
            - A list of valid parent links
            - A list of valid child links
            - A 3 x 1 numpy.ndarray axis
            - A float / int limit_lower
            - A float / int limit_upper
            - A float / int limit_effort
            - A float / int limit_velocity
        This  function  raises  an exception if any of these statements is not
        true.

        Returns
        -------
        True if no exceptions are raised

        """

        # Super call .........................................................

        super(JointURDF, self).valid()

        # self.joint_type ....................................................

        if type(self.joint_type) != str:
            raise TypeError("Joint type must be a str and is currently a " +
                            f"{type(self.joint_type)}")

        if self.joint_type not in ['revolute', 'continuous', 'prismatic',
                                   'fixed', 'floating', 'planar']:
            raise ValueError(f"Joint type '{self.joint_type}' is not " +
                             "correct.")

        # self.origin_xyz ....................................................

        if type(self.origin_xyz) != np.ndarray:
            raise TypeError(
                "Joint origin_xyz must be a numpy.ndarray and is" +
                f" currently a {type(self.origin_xyz)}")

        if self.origin_xyz.shape != (3, 1):
            raise ValueError("Joint origin_xyz shape must be (3, 1) and is " +
                             f"currently {self.origin_xyz.shape}")

        # self.origin_rpy ....................................................

        if type(self.origin_rpy) != np.ndarray:
            raise TypeError(
                "Joint origin_rpy must be a numpy.ndarray and is" +
                f" currently a {type(self.origin_rpy)}")

        if self.origin_rpy.shape != (3, 1):
            raise ValueError("Joint origin_rpy shape must be (3, 1) and is " +
                             f"currently {self.origin_rpy.shape}")

        # self.axis ..........................................................

        if type(self.axis) != np.ndarray:
            raise TypeError("Joint axis must be a numpy.ndarray and is" +
                            f" currently a {type(self.axis)}")

        if self.axis.shape != (3, 1):
            raise ValueError("Joint axis shape must be (3, 1) and is " +
                             f"currently {self.axis.shape}")

        # self.limit_lower ...................................................

        if self.limit_lower is None and self.joint_type not in \
                ['revolute', 'prismatic']:
            pass

        elif type(self.limit_lower) not in [float, int]:
            raise TypeError(
                "Joint limit_lower must be a float / int and is " +
                f"currently a {type(self.limit_lower)}")

        # self.limit_upper ...................................................

        if self.limit_upper is None and self.joint_type not in \
                ['revolute', 'prismatic']:
            pass

        elif type(self.limit_upper) not in [float, int]:
            raise TypeError(
                "Joint limit_upper must be a float / int and is " +
                f"currently a {type(self.limit_upper)}")

        # self.limit_effort ..................................................

        if self.limit_effort is None and self.joint_type not in \
                ['revolute', 'prismatic']:
            pass

        elif type(self.limit_effort) not in [float, int]:
            raise TypeError(
                "Joint limit_effort must be a float / int and is" +
                f" currently a {type(self.limit_effort)}")

        # self.limit_velocity ................................................

        if self.limit_velocity is None and self.joint_type not in \
                ['revolute', 'prismatic']:
            pass

        elif type(self.limit_velocity) not in [float, int]:
            raise TypeError(
                "Joint limit_velocity must be a float / int and " +
                f"is currently a {type(self.limit_velocity)}")

        return True

    def __str__(self):
        """
        Description
        -----------

        Converts the Joint object to str object

        Returns
        -------
        joint_str : str
            Converted joint to string

        """

        joint_str = f"Joint Name : {self.name}\n"
        joint_str += f"Joint Type : {self.joint_type}\n"
        joint_str += f"Joint origin XYZ (m):\n{self.origin_xyz}\n"
        joint_str += f"Joint origin RPY (rad) :\n{self.origin_rpy}\n"
        joint_str += f"Parent Link ID : {self.parent}\n"
        joint_str += f"Child Link ID : {self.child}\n"
        joint_str += f"Joint Axis :\n{self.axis}\n"
        joint_str += f"Joint Limits :\n\tLower : {self.limit_lower}\n"
        joint_str += f"\tUpper : {self.limit_upper}\n"
        joint_str += f"\tEffort : {self.limit_effort}\n"
        joint_str += f"\tVelocity : {self.limit_velocity}\n\n"

        return joint_str


# ----------------------------------------------------------------------------
# | JointDH Class                                                            |
# ----------------------------------------------------------------------------

class JointDH(Joint):
    """
    Description
    -----------

    Joint Class for .dhparams files.
    For more details, see :

    https://github.com/Teskann/URDFast/blob/master/documentation/dhparams_file_format.md

    Inherited from Joint
    ---------------------

    name : str
        Joint name. This field is required or an error will be thrown

    parent : int
        Parent  link id. Corresponds to the link_id of the link which name
        is  the  <parent>  element  of the <joint> element in the URDF XML
        representation.

    child : int
        Child  link id.  Corresponds to the link_id of the link which name
        is  the  <child>  element  of  the <joint> element in the URDF XML
        representation.

    T : sympy.matrices.immutable.ImmutableDenseMatrix
        Transition matrix of the joint

    Tinv : sympy.matrices.immutable.ImmutableDenseMatrix
        Inverse of the transition matrix of the joint

    Data Structure
    --------------

    __rot_trans : list of str
        List of all the transformations applied to the joint. Every element of
        the list must be a CSV value of the line 1 of a .dhparam file.

    __d : float or sympy.core.symbol.Symbol
        Offset along previous z to the common normal. Must be given in meters.

    __theta : float or sympy.core.symbol.Symbol
        Angle about previous z, from old x to new x. Must be given in radians.

    __r : float or sympy.core.symbol.Symbol
        Length of the common normal (aka a, but if using this notation, do not
        confuse  with  alpha).  Assuming  a revolute joint, this is the radius
        about previous z. Must be given in meters.

    __alpha : float or sympy.core.symbol.Symbol
        Angle  about  common  normal,  from  old z axis to new z axis. Must be
        given in radians.

    pmin : float or None
        Minimal value the degree of freedom can reach (position).
        Expressed  in  radians  for  angular degrees of freedom, in meters for
        translation degrees of freedom.

        Default is None

    pmax : float or None
        Maximal value the degree of freedom can reach (position).
        Expressed  in  radians  for  angular degrees of freedom, in meters for
        translation degrees of freedom.

        Default is None

    vmax : float or None
        Maximal reachable velocity (max of the derivative of theta / alpha for
        revolute  joints in radians per seconds, max of the derivative of r, d
        for prismatic joints in meters per seconds).

        Must be positive (the velocity is expressed as a norm).

        Default is None

    amax : float or None
        maximal  reachable acceleration (max of the second derivative of theta
        / alpha for revolute joints in radians per seconds², max of the second
        derivative of r, d for prismatic joints in meters per seconds²).

        Default is None

    type : str
        Joint type (prismatic, revolute, continuous, fixed)

    Example
    -------

    You can create a joint from a .dhparams file using the parser

    >>> from dh_params import dh
    >>> dh_obj = dh("./Examples/example_0.dhparams")
    >>> joint = JointDH(dh_obj, 0)
    """

    # Constructor ____________________________________________________________

    def __init__(self, dhparams, joint_number):
        """
        Parameters
        ----------

        dhparams : dh_params.DHParams
            DH params object of the robot possessing the joint

        joint_number : int
            joint_number in the list of dhparams.rows

        Example
        -------

        You can create a joint from a .dhparams file using the parser

        >>> from dh_params import dh
        >>> dh_obj = dh("./Examples/example_0.dhparams")
        >>> joint = JointDH(dh_obj, 0)

        """

        if joint_number >= len(dhparams.rows):
            raise KeyError("The joint you try to create does not exist. "
                           f"joint_number {joint_number} out of range ("
                           f"the object has {len(dhparams.rows)} joints)")

        # Transformations ....................................................

        self.__rot_trans = dhparams.rot_trans
        self.__d = dhparams.rows[joint_number].d
        self.__theta = dhparams.rows[joint_number].theta
        self.__r = dhparams.rows[joint_number].r
        self.__alpha = dhparams.rows[joint_number].alpha

        # Super call .........................................................

        super().__init__("joint_" + dhparams.rows[joint_number].name,
                         joint_number,
                         joint_number + 1)

        # Limits .............................................................

        # If there are no DoF ==> Ignoring limits
        if all([type(x) == float for x in [self.__d, self.__theta,
                                           self.__r, self.__alpha]]):
            self.pmin = None
            self.pmax = None
            self.vmax = None
            self.amax = None
            self.joint_type = "Fixed"
        # If there is a DoF
        else:
            self.pmin = dhparams.rows[joint_number].pmin
            self.pmax = dhparams.rows[joint_number].pmax
            self.vmax = dhparams.rows[joint_number].vmax
            self.amax = dhparams.rows[joint_number].amax

            # Alpha or theta are DoF
            if any([type(x) != float for x in [self.__theta, self.__alpha]]):
                if self.pmin is not None and self.pmax is not None:
                    self.joint_type = "Revolute"
                else:
                    self.joint_type = "Continuous"
            # r or d are DoF
            else:
                self.joint_type = "Prismatic"
        self.valid()

    # Transition Matrix ______________________________________________________

    def T_(self, consider_limits=False, tolerance=1e-10):
        """
        Transition matrix of the joint.
        Computed from the Denavit-Hartenberg parameters.

        For more details, see :

        https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters

        Parameters
        ----------

        consider_limits : bool
            If  True,  the  transition matrices ensure it's not possible to go
            beyond  self.pmin and self.pmax. This results adding
            max  and  min  functions in the transition matrix. For example, if
            the  degree  of  freedom is d and consider_limits is True, every d
            value will become :
                min(max(d, self.pmin), self.pmax)
            (if the limits are not None).

            This can cause derivation  problems so you can disable this option
            turning consider_limits to  False.

            Defaults to False

        tolerance : float
            Tolerance for simplification of the expression.

            Defaults to 1e-10

        Returns
        -------
        sympy.matrices.immutable.ImmutableDenseMatrix
            Transformation  matrix  from  the parent link to the child link in
            homogeneous coordinates. The shape is (4, 4)

        """

        T = Matrix([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

        # Finding the degree of freedom ......................................

        subs = {"d":     self.__d,
                "theta": self.__theta,
                "r":     self.__r,
                "alpha": self.__alpha}

        # Considering limits if asked
        for sym in subs:
            if type(subs[sym]) != float:

                if consider_limits and self.pmin is not None:
                    subs[sym] = Max(subs[sym], self.pmin)
                if consider_limits and self.pmax is not None:
                    subs[sym] = Min(subs[sym], self.pmax)
                break

        val = Symbol("__k__")
        c = cos(val)
        s = sin(val)

        matrices = {"TransX": Matrix([[1, 0, 0, val],
                                      [0, 1, 0, 0],
                                      [0, 0, 1, 0],
                                      [0, 0, 0, 1]]),
                    "TransZ": Matrix([[1, 0, 0, 0],
                                      [0, 1, 0, 0],
                                      [0, 0, 1, val],
                                      [0, 0, 0, 1]]),
                    "RotX": Matrix([[1, 0, 0, 0],
                                    [0, c, -s, 0],
                                    [0, s, c, 0],
                                    [0, 0, 0, 1]]),
                    "RotZ": Matrix([[c, -s, 0, 0],
                                    [s, c, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])}

        for transformation in self.__rot_trans:
            trans, param = transformation.split("..")
            T *= matrices[trans].subs(val, subs[param])

        # A  bug  in  SymPy  is  not  rounding  float  values  if they are not
        # multiplied  by  a  Symbol.  To  fix  this, we multiply T by a random
        # Symbol, optimize and then divide by this Symbol.

        debug_sym = Symbol('debug_symbol')

        return nsimplify(T * debug_sym,
                         tolerance=tolerance).evalf() / debug_sym

    # Validation _____________________________________________________________

    def valid(self):
        super().valid()
        return True
