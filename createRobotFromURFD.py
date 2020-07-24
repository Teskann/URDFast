# -*- coding: utf-8 -*-
"""
Created on Mon Jun  8 18:23:08 2020

@author: Clément
"""

import numpy as np
from anytree import Node, RenderTree, Walker
from URDF import URDF
from sympy import Matrix, Symbol, cos, sin, Min, Max, nsimplify, simplify

# Creates a robot from an URDF object
# See URDF.py for more details about this class

# ----------------------------------------------------------------------------
# | LINK CLASS                                                               |
# ----------------------------------------------------------------------------

class Link:
    """
        Description
        -----------

        Link Class.

        This class describes link  objects.  These objects represent URDF link
        elements. This class is used inside the Robot class (see below).
        The  link  element  describes  a  rigid  body  with an inertia, visual
        features, and collision properties.

        For the moment,  this  class  only deals with the <inertial> elements.
        <visual>  and  <collision>  elements could be useful for visualization
        and simulation purposes, which are not the goal of this project.

        For more details about this object description, refer to the URDF Link
        documentation : http://wiki.ros.org/urdf/XML/link

        Prerequisites
        -------------

        This class needs numpy dependency to work properly. You can install it
        using pip :

        >>> $ pip install numpy  

        or with Anaconda (installed by default) :

        >>> $ conda install numpy

        You also need URDF Library, included in this project.

        Data Structure
        --------------
        
        link_id : int
            Link identifier. It's the number of the link in the URDF file.

        name : str
            Link name. This field is required or an error will be thrown

        com : 3 x 1 numpy.ndarray
            Center  of  mass of the link. Corresponds to the "origin" field of
            the "inertial" field of a link in the URDF XML format.

            SI Unit : m

            Default : Null 3x1 Vector : numpy.zeros((3, 1))

        mass : float / int
            Link  mass  in  kilograms.  Corresponds to the "mass" field of the
            "inertial"  element  of  a link in the URDF XML format. It MUST be
            positive

            SI Unit : kg

            Default : 0 kg

        inertia : 3 x 3 numpy.ndarray
            Inertia matrix of the link.  This matrix is symetric.  Corresponds
            to  the  "inertia"  field of the "inertial" field of a link in the
            URDF XML format.

            SI Unit : kg.m^2

            Default : 3x3 Identity Matrix : numpy.eye(3)

        parent_joints : list of Joints
            All Joints that have the Link as parent

        child_joints : list of Joints
            All Joints that have the Link as child

        Constructor
        -----------

        Only one constructor is implemented.
        It creates a Link object from an URDF Object and a link number

        Parameters :

            urdfObject : URDF
                URDF object from URDF library
            link_number : int
                Link   Number   in  the URDF file tree. Must be fewer than the
                total number of links in the URDF object.

        Examples :

            Example  URDF  files  are  located  in  ./Examples  where . is the
            directory containing createRobotFromURDF.py

            - Running example_0.urdf URDF file (with no <inertial> elements) :

            >>> urdfObj = URDF("./Examples/example_0.urdf")
            >>> linkObj = Link(urdfObj, 1)
            >>> print(linkObj)
            Link Name : link2
            Link Center of Mass (m) : 
            [[0.]
             [0.]
             [0.]]
            Link Mass (kg) : 0.0
            Link Inertia Matrix (kg.m^2) : 
            [[1. 0. 0.]
             [0. 1. 0.]
             [0. 0. 1.]]
            Is parent of joints number : []
            Is child of joints number : [0]
            Terminal Link

            - Running example_1.urdf URDF file (with <inertial> elements) :
            >>> urdfObj = URDF("./Examples/example_1.urdf")
            >>> linkObj = Link(urdfObj, 0)
            >>> print(linkObj)
            Link Name : link1
            Link Center of Mass (m) : 
            [[1.]
             [1.]
             [1.]]
            Link Mass (kg) : 10.0
            Link Inertia Matrix (kg.m^2) : 
            [[1. 2. 3.]
             [2. 4. 5.]
             [3. 5. 6.]]
            Is parent of joints number : [0, 1]
            Is child of joints number : []
            Root Link

    """
    # Data ===================================================================
    
    # Link ID
    
    link_id = None

    # Link Name
    name = None

    # Center of mass coordinates
    com = np.zeros((3, 1))

    # Link Mass
    mass = 0.0

    # Inertia Matrix
    inertia = np.eye(3)

    # List of parent Joints
    parent_joints = []

    # List of children Links
    child_joints = []

    # Is the link a terminal link ?
    is_terminal = False

    # If the link a root link ?
    is_root = False

    # Constructor ============================================================

    # Default, giving an URDF Object and a link number _______________________

    def __init__(self, urdfObject, link_number):
        """
        Description
        -----------

        Create a Link object from an URDF Object and a link number

        Parameters
        ----------

        urdfObject : URDF
            URDF object from URDF library
        link_number : int
            Link  Number  in  the URDF file tree. Must be fewer than the total
            number of links in the URDF object.

        Examples
        --------

        Example URDF files are located in ./Examples  where . is the directory
        containing createRobotFromURDF.py

        - Running example_0.urdf URDF file (with no <inertial> elements) :

        >>> urdfObj = URDF("./Examples/example_0.urdf")
        >>> linkObj = Link(urdfObj, 1)
        >>> print(linkObj)
        Link Name : link2
        Link Center of Mass (m) : 
        [[0.]
         [0.]
         [0.]]
        Link Mass (kg) : 0.0
        Link Inertia Matrix (kg.m^2) : 
        [[1. 0. 0.]
         [0. 1. 0.]
         [0. 0. 1.]]
        Is parent of joints number : []
        Is child of joints number : [0]
        Terminal Link

        - Running example_1.urdf URDF file (with <inertial> elements) :
        >>> urdfObj = URDF("./Examples/example_1.urdf")
        >>> linkObj = Link(urdfObj, 0)
        >>> print(linkObj)
        Link Name : link1
        Link Center of Mass (m) : 
        [[1.]
         [1.]
         [1.]]
        Link Mass (kg) : 10.0
        Link Inertia Matrix (kg.m^2) : 
        [[1. 2. 3.]
         [2. 4. 5.]
         [3. 5. 6.]]
        Is parent of joints number : [0, 1]
        Is child of joints number : []
        Root Link

        """

        # Checking if the link exists
        if link_number > urdfObject.nlinks():
            raise IndexError(f"link_number ({link_number}) out of range " +
                             f"({urdfObject.nlinks()})")

        urdfObjectLink = urdfObject.links[link_number]
        self.link_id = link_number

        # 1 - Getting the Link Name ..........................................

        try:
            self.name = urdfObjectLink['name']
        except KeyError:
            raise KeyError("The URDF Link must contain a 'name' property")

        # 2 - Getting com if exists ..........................................

        try:
            # Setting CoM as a column vector
            self.com = np.array(urdfObjectLink['inertial']['origin']['xyz'])\
                .reshape((3, 1))
        except KeyError:
            # If this property doesn't exist we keep the default value
            self.com = np.zeros((3, 1))

        # 3 - Getting mass if exists .........................................

        try:
            self.mass = urdfObjectLink['inertial']['mass']['value'][0]
        except KeyError:
            # If this property doesn't exist we keep the default value
            self.mass = 0.0

        # 4 - Getting inertia matrix if exists ...............................

        try:
            # Ixx
            self.inertia[0][0] = urdfObjectLink['inertial']['inertia']['ixx']\
                [0]

            # Ixy & Iyx (symetric)
            self.inertia[1][0] = urdfObjectLink['inertial']['inertia']['ixy']\
                [0]
            self.inertia[0][1] = urdfObjectLink['inertial']['inertia']['ixy']\
                [0]

            # Ixz & Izx (symetric)
            self.inertia[2][0] = urdfObjectLink['inertial']['inertia']['ixz']\
                [0]
            self.inertia[0][2] = urdfObjectLink['inertial']['inertia']['ixz']\
                [0]

            # Iyy
            self.inertia[1][1] = urdfObjectLink['inertial']['inertia']['iyy']\
                [0]

            # Iyz & Izy (symetric)
            self.inertia[1][2] = urdfObjectLink['inertial']['inertia']['iyz']\
                [0]
            self.inertia[2][1] = urdfObjectLink['inertial']['inertia']['iyz']\
                [0]

            # Izz
            self.inertia[2][2] = urdfObjectLink['inertial']['inertia']['izz']\
                [0]

        except KeyError:
            self.inertia = np.eye(3)

        # 5 - Parent and child joints ........................................

        self.parent_joints = []
        self.child_joints = []
        for i in range(urdfObject.njoints()):
            joint = urdfObject.joints[i]
            if self.name == joint['parent']['link']:
                self.parent_joints.append(i)
            if self.name == joint['child']['link']:
                self.child_joints.append(i)

        # 6 - Is the link Terminal ? .........................................

        self.is_terminal = self.parent_joints == []

        # 7 - Is the link Root ? .............................................

        self.is_root = self.child_joints == []

        # Be sure the created object is valid ................................

        self.__valid()

    # Methods ================================================================

    # Checks if the object is valid __________________________________________
    def __valid(self):
        """
        Description
        -----------

        This function checks if the Link object is valid, ie if it has :
            - A not-None name
            - A 3 x 1 numpy.ndarray com
            - A positive or null mass
            - A 3 x 3 symetric numpy.ndarray inertia
        This  function  raises  an exception if any of these statements is not
        true.

        Returns
        -------
        True if no exceptions are raised

        """
        # self.link_id .......................................................
        
        if type(self.link_id) != int:
            raise TypeError("Link id must be an integer")
            
        if self.link_id < 0:
            raise ValueError("Link id must be a positive integer")

        # self.name ..........................................................

        if self.name is None:
            raise ValueError("Link name is None. You must give it a valid " +
                             "name (str)")

        if type(self.name) != str:
            raise TypeError("Link name must be a str and is currently a " +
                            f"{type(self.name)}")

        # self.com ...........................................................

        if type(self.com) != np.ndarray:
            raise TypeError("Link com must be a numpy.ndarray and is " + 
                            f"currently a {type(self.com)}")

        if self.com.shape != (3, 1):
            raise ValueError("Link com shape must be (3, 1) and is currently"+
                             f" {self.com.shape}")

        # self.mass ..........................................................

        if type(self.mass) not in [float, int]:
            raise TypeError("Link mass must be a float and is currently a "+
                            f"{type(self.mass)}")
        if self.mass < 0.0:
            raise ValueError("Link mass must be greater or equal to 0 and is"+
                             f" currently {self.mass}")

        # self.inertia .......................................................

        if type(self.inertia) != np.ndarray:
            raise TypeError("Link inertia must be a numpy.ndarray and is " +
                            f"currently a {type(self.inertia)}")

        if self.inertia.shape != (3, 3):
            raise ValueError("Link inertia shape must be (3, 3) and is "+
                             f"currently {self.inertia.shape}")

        if not np.allclose(self.inertia, self.inertia.T):
            raise ValueError("Link inertia must be symetric")

        return True

    # Converting the Link Objecct to string __________________________________
    
    def __str__(self):
        tostring = f"Link Name : {self.name}\n"
        tostring += "Link Center of Mass (m) : \n"
        tostring += f"{self.com}\n"
        tostring += f"Link Mass (kg) : {self.mass}\n"
        tostring += f"Link Inertia Matrix (kg.m^2) : \n{self.inertia}\n"
        tostring += f"Is parent of joints number : {self.parent_joints}\n"
        tostring += f"Is child of joints number : {self.child_joints}\n"

        if self.is_root:
            tostring += "Root Link\n"
        if self.is_terminal:
            tostring += "Terminal Link\n"

        return tostring


# ----------------------------------------------------------------------------
# | JOINT CLASS                                                              |
# ----------------------------------------------------------------------------

class Joint:
    """
        Description
        -----------

        Joint Class.
        
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

        Prerequisites
        -------------

        This class needs numpy dependency to work properly. You can install it
        using pip :

        >>> $ pip install numpy

        or with Anaconda (installed by default) :

        >>> $ conda install numpy

        You also need URDF Library, included in this project.

        Data Structure
        --------------

        name : str
            Joint name. This field is required or an error will be thrown

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

        parent : int
            Parent  link id. Corresponds to the link_id of the link which name
            is  the  <parent>  element  of the <joint> element in the URDF XML
            representation.

        child : int
            Child  link id.  Corresponds to the link_id of the link which name
            is  the  <child>  element  of  the <joint> element in the URDF XML
            representation.

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
            
        T : sympy.matrices.immutable.ImmutableDenseMatrix
            Transition matrix of the joint
        
        Tinv : sympy.matrices.immutable.ImmutableDenseMatrix
            Inverse of the transition matrix of the joint

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
            directory containing createRobotFromURDF.py

            - Running example_0.urdf URDF file (pretty simple <joint>) :
            >>> urdf_obj = URDF("./Examples/example_0.urdf")
            >>> joint_obj = Joint(urdf_obj, 0)
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
            >>> joint_obj = Joint(urdf_obj, 0)
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
    
    def __init__(self, urdfObject, joint_number):
        """
        Description
        -----------

        Only one constructor is implemented.
        It creates a Joint from an URDF Object and a joint number

        Parameters
        ----------

        urdfObject : URDF
            URDF object from URDF library
        joint_number : int
            Joint  Number  in the URDF file tree. Must be fewer than the total
            number of joints in the URDF object.

        Examples
        --------

        Example URDF files are located in ./Examples  where . is the directory
        directory containing createRobotFromURDF.py

        - Running example_0.urdf URDF file (with no <inertial> elements) :

        TODO

        - Running example_1.urdf URDF file (with <inertial> elements) :

        TODO
        """
        # Checking if the joint exists
        if joint_number > urdfObject.njoints():
            raise IndexError(f"joint_number ({joint_number}) out of range " +
                             f"({urdfObject.njoints()})")

        urdfObjectJoint = urdfObject.joints[joint_number]
        
        # 1 - Joint Name .....................................................
        
        if 'name' in urdfObjectJoint.keys():
            self.name = urdfObjectJoint['name']
        else:
            raise KeyError("Joint must have a name")
        
        # 2 - Joint type .....................................................
        
        if 'type' in urdfObjectJoint.keys():
            self.joint_type = urdfObjectJoint['type']
        else:
            raise KeyError("Joint must have a type")
            
        # 3 - Origin XYZ & RPY ...............................................
        
        if 'origin' in urdfObjectJoint.keys():
            
            # Simplifying notation
            originDict = urdfObjectJoint['origin']
            
            # Origin XYZ . . . . . . . . . . . . . . . . . . . . . . . . . . .
            
            if 'xyz' in originDict.keys():
                # File Value
                self.origin_xyz = np.array(originDict['xyz']).reshape(3, 1)
            else:
                # Default Value
                self.origin_xyz = np.zeros((3, 1))
            
            # Origin RPY . . . . . . . . . . . . . . . . . . . . . . . . . . .
            
            if 'rpy' in originDict.keys():
                # File Value
                self.origin_rpy = np.array(originDict['rpy']).reshape(3, 1)
            else:
                # Defalut Value
                self.origin_rpy = np.zeros((3, 1))
            
        # 4 - Parent Link ....................................................
        
        if 'parent' in urdfObjectJoint.keys():
            
            # Simplifying notation
            parentDict = urdfObjectJoint['parent']
            
            # Must have a 'link' element
            if 'link' in parentDict.keys():
                
                # Searching the parent link in link list
                for i, linkURDF in enumerate(urdfObject.links):
                    
                    # If found, get the index value
                    if parentDict['link'] == linkURDF['name']:
                        self.parent = i
                        break
            else:
                raise KeyError("Joint Parent Link must have a Link property")
        else:
            raise KeyError("Joint must have a Parent Link")
            
        # 5 - Child Link .....................................................
        
        if 'child' in urdfObjectJoint.keys():
            
            # Simplifying notation
            childDict = urdfObjectJoint['child']
            
            # Must have a 'link' element
            if 'link' in childDict.keys():
                
                # Searching the child link in link list
                for i, linkURDF in enumerate(urdfObject.links):
                    
                    # If found, get the index value
                    if childDict['link'] == linkURDF['name']:
                        self.child = i
                        break
            else:
                raise KeyError("Joint Child Link must have a Link property")
        else:
            raise KeyError("Joint must have a child Link")
        
        # 6 - Axis ...........................................................
        
        if 'axis' in urdfObjectJoint.keys():
            # File Value
            if 'xyz' in urdfObjectJoint['axis'].keys():
                self.axis = np.array(urdfObjectJoint['axis']['xyz']).reshape(\
                            (3, 1))
                self.axis /= np.linalg.norm(self.axis)
            # Default Value
            else:
                self.axis = np.array([[1, 0, 0]]).T
                
        # 7 - Limits .........................................................
        
        if 'limit' in urdfObjectJoint.keys():
            
            # Simplifying Notation
            limitDict = urdfObjectJoint['limit']
            
            # Lower Limit  . . . . . . . . . . . . . . . . . . . . . . . . . .
            
            if 'lower' in limitDict:
                # File Value
                self.limit_lower = float(limitDict['lower'][0])
            else:
                # Default Value
                self.limit_lower = 0.0
                
            # Upper Limit  . . . . . . . . . . . . . . . . . . . . . . . . . .
            
            if 'upper' in limitDict:
                # File Value
                self.limit_upper = float(limitDict['upper'][0])
            else:
                # Default Value
                self.limit_upper = 0.0
            
            # Effort limit . . . . . . . . . . . . . . . . . . . . . . . . . .
            
            if 'effort' in limitDict:
                # File Value
                self.limit_effort = float(limitDict['effort'][0])
            else:
                # NO DEFAULT VALUE : REQUIRED for revolute & prismatic
                if self.joint_type in ['revolute', 'prismatic']:
                    raise KeyError(f"{self.joint_type} joint must have "+
                           "effort limit properties")
                # None for other types of joints (doesn't matter)
                else:
                    self.limit_effort = None
            
            # Velocity Limit . . . . . . . . . . . . . . . . . . . . . . . . .
            
            if 'velocity' in limitDict:
                # File Value
                self.limit_velocity = float(limitDict['velocity'][0])
            else:
                # NO DEFAULT VALUE : REQUIRED for revolute & prismatic
                if self.joint_type in ['revolute', 'prismatic']:
                    raise KeyError(f"{self.joint_type} joint must have "+
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
            
        self.T = self.__T()
        self.Tinv = simplify(self.T**(-1))
        
        # 8 - Checking if the joint is valid .................................
        
        self.__valid()
        

    # Methods ================================================================
    
    # Getting the transition Matrix T ________________________________________
    
    def __T(self, consider_limits=False, tolerance=1e-10):
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
            
            Defaluts to 1e-10

        Returns
        -------
        numpy.ndarray 
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
        yaw = Matrix([[cos(angle),  -sin(angle),  0],
                      [sin(angle),  cos(angle),   0],
                      [0,           0,            1]])
        
        # Pitch around Y axis  . . . . . . . . . . . . . . . . . . . . . . . .
        
        angle = self.origin_rpy[1, 0]
        pitch = Matrix([[cos(angle),   0,   sin(angle)],
                        [0,            1,   0         ],
                        [-sin(angle),  0,   cos(angle)]])

        # Roll around X axis . . . . . . . . . . . . . . . . . . . . . . . . .
        
        angle = self.origin_rpy[0, 0]
        roll = Matrix([[1,   0,            0          ],
                       [0,   cos(angle),   -sin(angle)],
                       [0,   sin(angle),   cos(angle) ]])
        
        # Yaw * Pitch * Roll
        T[0:3, 0:3] = (yaw * pitch * roll).evalf()
        
        # Rodrigues formula ..................................................
        
        def rodrigues(axis, angle):
            """
            Description
            -----------
            
            Computes the  Rogrigues rotation matrix from a rotation around the
            axis 'axis' with the rotation 'angle'
            https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
            
            Parameters
            ----------
            
            axis : numpy.ndarray
                Axis around the one the rotation is made.
                Has to be normalized and shape = (3, 1)
                
            angle : sympy.core.symbol.Symbol
                Symbol for angle rotation (in radians)
                
            Returns
            -------
            sympy.matrices.dense.MutableDenseMatrix
                Rotation matrix of the angle 'angle' around the axis 'axis'.
                Shape is 3x3
                
            """
            
            # Cross Product Matrix
            K = Matrix([[0,            -axis[2, 0],  axis[1, 0] ],
                        [axis[2, 0],   0,            -axis[0, 0]],
                        [-axis[1, 0],  axis[0, 0],   0         ]])
            
            identity = Matrix([[1, 0, 0],
                               [0, 1, 0],
                               [0, 0, 1]])
            
            return identity + sin(angle)*K + (1 - cos(angle))*K**2
            
        # Adding degree(s) of freedom of the joint ...........................
        
        # Revolute Joints  . . . . . . . . . . . . . . . . . . . . . . . . . .
        
        if self.joint_type == 'revolute':
            
            # 1 degree of freedom around the axis
            
            # Degree of freedom
            theta = Symbol('theta_'+self.name)
            
            if(consider_limits):
                theta_limit = Min(Max(theta, self.limit_lower),
                                  self.limit_upper)
                T[0:3, 0:3] *= rodrigues(self.axis, theta_limit)
            else:
                T[0:3, 0:3] *= rodrigues(self.axis, theta)
            
        # Continuous Joints  . . . . . . . . . . . . . . . . . . . . . . . . .
        
        elif self.joint_type == 'continuous':
            
            # 1 degree of freedom around the axis
            
            # Degree of freedom
            theta = Symbol('theta_'+self.name)
            
            T[0:3, 0:3] *= rodrigues(self.axis, theta)
            
        # Prismatic Joints . . . . . . . . . . . . . . . . . . . . . . . . . .
        
        elif self.joint_type == 'prismatic':
            
            # 1 degree of freedom along the axis
            # Since axis is normalized, the translation value is multiplied by
            # the axis to get its XYZ value
            
            d = Symbol('d_'+self.name)
            if(consider_limits):
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
            roll_rot = Matrix([[1,   0,            0          ],
                               [0,   cos(droll),   -sin(droll)],
                               [0,   sin(droll),   cos(droll) ]])
            
            pitch_rot = Matrix([[cos(dpitch),   0,   sin(dpitch)],
                                [0,             1,   0          ],
                                [-sin(dpitch),  0,   cos(dpitch)]])
            
            yaw_rot = Matrix([[cos(dyaw),  -sin(dyaw),  0],
                              [sin(dyaw),  cos(dyaw),   0],
                              [0,           0,          1]])
            
            T[0:3, 0:3] *= (yaw_rot * pitch_rot * roll_rot)
            
        # Planar Joints  . . . . . . . . . . . . . . . . . . . . . . . . . . .
            
        elif self.joint_type == 'planar':
            
            # 2 degrees of freedom (translation in the normal plan)
            
            # Degrees of freedom
            dx = Symbol('dx_' + self.name)
            dy = Symbol('dy_' + self.name)
            
            print('Planar Joints not Supported yet')
        
        
        # A  bug  in  SymPy  is  not  rounding  float  values  if they are not
        # multiplied  by  a  Symbol.  To  fix  this, we multiply T by a random
        # Symbol, simplify and then divide by this Symbol.
        
        debug_sym = Symbol('debug_symbol')
        
        return nsimplify(T*debug_sym, tolerance=tolerance).evalf()/debug_sym

    # Checks if the joint is valid ___________________________________________

    def __valid(self):
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
        # self.name ..........................................................
        
        if self.name is None:
            raise ValueError("Joint name is None. You must give it a valid " +
                             "name (str)")

        if type(self.name) != str:
            raise TypeError("Joint name must be a str and is currently a " +
                            f"{type(self.name)}")

        # self.joint_type ....................................................

        if type(self.joint_type) != str:
            raise TypeError("Joint type must be a str and is currently a " +
                            f"{type(self.joint_type)}")

        if self.joint_type not in ['revolute', 'continuous', 'prismatic',
                                   'fixed', 'floating', 'planar']:
            raise ValueError(f"Joint type '{self.joint_type}' is not "+
                             "correct.")

        # self.origin_xyz ....................................................

        if type(self.origin_xyz) != np.ndarray:
            raise TypeError("Joint origin_xyz must be a numpy.ndarray and is"+
                            f" currently a {type(self.origin_xyz)}")

        if self.origin_xyz.shape != (3, 1):
            raise ValueError("Joint origin_xyz shape must be (3, 1) and is " +
                             f"currently {self.origin_xyz.shape}")

        # self.origin_rpy ....................................................

        if type(self.origin_rpy) != np.ndarray:
            raise TypeError("Joint origin_rpy must be a numpy.ndarray and is"+
                            f" currently a {type(self.origin_rpy)}")

        if self.origin_rpy.shape != (3, 1):
            raise ValueError("Joint origin_rpy shape must be (3, 1) and is " +
                             f"currently {self.origin_rpy.shape}")

        # self.parent ........................................................

        if type(self.parent) != int:
            raise TypeError("Joint parent must be an integer and is "+
                                f"currently a {type(self.parent)}")
            
        if self.parent < 0:
            raise ValueError("Joint parent must be a positive integer")

        # self.child .........................................................
        
        if type(self.child) != int:
            raise TypeError("Joint child must be an integer and is "+
                                f"currently a {type(self.child)}")
            
        if self.child < 0:
            raise ValueError("Joint child must be a positive integer")

        # self.axis ..........................................................
    
        if type(self.axis) != np.ndarray:
            raise TypeError("Joint axis must be a numpy.ndarray and is"+
                            f" currently a {type(self.axis)}")
            
        if self.axis.shape != (3, 1):
            raise ValueError("Joint axis shape must be (3, 1) and is " +
                             f"currently {self.axis.shape}")
            
        # self.limit_lower ...................................................
        
        if self.limit_lower == None and self.joint_type not in\
            ['revolute', 'prismatic']:
            pass
        
        elif type(self.limit_lower) not in [float, int]:
            raise TypeError("Joint limit_lower must be a float / int and is "+
                            f"currently a {type(self.limit_lower)}")
            
        # self.limit_upper ...................................................
        
        if self.limit_upper == None and self.joint_type not in\
            ['revolute', 'prismatic']:
            pass
        
        elif type(self.limit_upper) not in [float, int]:
            raise TypeError("Joint limit_upper must be a float / int and is "+
                            f"currently a {type(self.limit_upper)}")
        
        # self.limit_effort ..................................................
        
        if self.limit_effort == None and self.joint_type not in\
            ['revolute', 'prismatic']:
            pass
        
        elif type(self.limit_effort) not in [float, int]:
            raise TypeError("Joint limit_effort must be a float / int and is"+
                            f" currently a {type(self.limit_effort)}")
            
        # self.limit_velocity ................................................
        
        if self.limit_velocity == None and self.joint_type not in\
            ['revolute', 'prismatic']:
            pass
        
        elif type(self.limit_velocity) not in [float, int]:
            raise TypeError("Joint limit_velocity must be a float / int and "+
                            f"is currently a {type(self.limit_velocity)}")
            
        return True
    
    def __str__(self):
        """
        Description
        -----------
        
        Converts the Joint object to str object

        Returns
        -------
        jointstr : str
            Converted joint to string

        """
        
        jointstr = f"Joint Name : {self.name}\n"
        jointstr += f"Joint Type : {self.joint_type}\n"
        jointstr += f"Joint origin XYZ (m):\n{self.origin_xyz}\n"
        jointstr += f"Joint origin RPY (rad) :\n{self.origin_rpy}\n"
        jointstr += f"Parent Link ID : {self.parent}\n"
        jointstr += f"Child Link ID : {self.child}\n"
        jointstr += f"Joint Axis :\n{self.axis}\n"
        jointstr += f"Joint Limits :\n\tLower : {self.limit_lower}\n"
        jointstr += f"\tUpper : {self.limit_upper}\n"
        jointstr += f"\tEffort : {self.limit_effort}\n"
        jointstr += f"\tVelocity : {self.limit_velocity}\n\n"
        
        return jointstr
    
# ----------------------------------------------------------------------------
# | ROBOT CLASS                                                              |
# ----------------------------------------------------------------------------

class Robot:
    """
    Description
    -----------
    
    Robot Class. Represents the tree representation of an URDF Robot.
    A robot is composed of joints and links
    
    Prerequisites
    -------------

    This class needs numpy dependency to work properly. You can install it
    using pip :

    >>> $ pip install numpy

    or with Anaconda (installed by default) :

    >>> $ conda install numpy
    
    To deal with tree representation, you also need to install anytree :
        
    >>> $ pip install anytree

    You finally need URDF Library, included in this project.
    
    Data Structure
    --------------
    
    name : str
        Robot Name. If the robot has no name, it will be named "no_name"
    
    links : List of 'Link'
        List containing all the Links of the Robot.
        
    joints : List of 'Joint'
        List containing all the Joints of the Robot.
        
    tree : anytree.render.RenderTree
        Tree  representation of the Robot. Tree nodes names are formatted like
        this :
            - For Links
                link_k
            - For Joints
                joint_k
            where k is the number of the Link/Joint in the list links/joints

    """
    
    # Data ===================================================================
    
    # Robot Name
    name = "no_name"
    
    # Robot Links
    links = []
    
    # Robot Joints
    joints = []
    
    # Tree representation
    tree = None
    
    # Methods ================================================================
    
    # Defalut Constructor ____________________________________________________
    
    def __init__(self, urdfObject, progressbar=None):
        """
        Description
        -----------
        
        Robot Constructor. You can construct a robot from an URDF Object.
        
        Parameters
        ----------
        
        urdfObject : URDF
            URDF Object from the URDF library
        
        progressbar : PyQt5.QtWidgets.QProgressBar or None, optional
                      default is None
            Progressbar to update during the robot creation (used in GUI)
            If it is None, no progressbar is updated
            
        Examples
        --------
        
        TODO !!!!!!!
        
        """
        
        # 1 - Robot Name .....................................................
        
        if 'name' in urdfObject.robot[0].keys():
            self.name = urdfObject.robot[0]['name']
        else:
            self.name = "no_name"
        
        # 2 - Robot Links ....................................................
        
        self.links = []
        
        for i in range(urdfObject.nlinks()):
            self.links.append(Link(urdfObject, i))
            
        # 3 - Robot Joints ...................................................
        
        self.joints = []
        
        for i in range(urdfObject.njoints()):
            if progressbar is not None:
                progressbar.setProperty("value", 
                                        100*(i+1)/urdfObject.njoints())
            self.joints.append(Joint(urdfObject, i))
            
        # 4 - Tree Representation ............................................
        
        # Creating a Node per Link . . . . . . . . . . . . . . . . . . . . . .
        
        allLinkNodes = []
        for i, _ in enumerate(self.links):
            allLinkNodes.append(Node('link_'+str(i)))
        
        # Creating a Node per Joint  . . . . . . . . . . . . . . . . . . . . .
        
        allJointNodes = []
        for i, joint in enumerate(self.joints):
            allJointNodes.append(Node('joint_'+str(i),
                                 parent=allLinkNodes[joint.parent]))
            
        # Setting parents for Link Nodes . . . . . . . . . . . . . . . . . . .
        
        root_link_id = 0
        for i, _ in enumerate(allLinkNodes):
            if self.links[i].is_root:
                root_link_id = i
                continue
            allLinkNodes[i].parent = allJointNodes[self.links[i]\
                                                   .child_joints[0]]
            
        # Setting Global Tree
        self.tree = RenderTree(allLinkNodes[root_link_id])
        
            
    # Number of Links ________________________________________________________
    
    def nlinks(self):
        """
        Description
        -----------
        
        Returns the number of links of the Robot
        
        Returns
        -------
        
        int : Number of links of the Robot
        
        Examples
        --------
        
        TODO
        
        """
        
        return len(self.links)
    
    # Number of Joints _______________________________________________________
    
    def njoints(self):
        """
        Description
        -----------
        
        Returns the number of joints of the Robot
        
        Returns
        -------
        
        int : Number of joints of the robot
        
        Examples
        --------
        
        TODO
        
        """
        
        return len(self.joints)
    
    # Get tree branch between 2 joints / links _______________________________
    
    def branch(self, origin, destination):
        """
        Description
        -----------
        
        Returns  the  path  in  the  tree  to go from the 'origin' node to the
        'destination' node.
        
        The result is returned as two lists :
            - upwards is a list of joint numbers to go upward to
            - downwards is a list of joint numbers to go downward to
            
        Every element of the list is the index of a joint from self.joints
        
        This  function  uses anytree Walker class, for more details, check the
        anytree documentation :
        https://anytree.readthedocs.io/en/latest/api/anytree.walker.html

        Parameters
        ----------
        origin : str
            Origin  element  of the tree. Is a string formatted like the nodes
            names of self.tree :
                type_number
                
                where type is 'joint' / 'link' and number is the number of the
                considered Joint / Link in self.joints / self.links.
                
                Example : origin='joint_2'
        destination : str
            Destination  element  of  the tree. Is a string formatted like the
            node names of self.tree :
                type_number
                
                where type is 'joint' / 'link' and number is the number of the
                considered Joint / Link in self.joints / self.links.
                
                Example : destination='link_0'

        Returns
        -------
        upwards : list of int
            List of  Joint index to go upward to. The transition matrix of the
            Joints is T()**(-1) to go from a Joint to the next in this list.
        downwards : list of int
            List  Joint index to go downward  to. The transition matrix of the
            Joints is T() to go from a Joint to the next in this list
            
        Examples
        --------
        
        TODO

        """
        
        # Upward List
        upwards = []
        
        # Downward List
        downwards = []
        
        # 1 - Finding the Node in the tree from origin and destination .......
        
        for _, _, node in self.tree:
            if node.name == origin:
                origin_node = node
            if node.name == destination:
                destination_node = node
                
        # 2 - Walk around the tree ...........................................
        
        # Tree Walker
        w = Walker()
        
        # Getting paths
        up_nodes, middle, down_nodes = w.walk(origin_node, destination_node)
        
        # As only Joints have transition matrices, we ignore link nodes
        
        # For upward nodes
        for node in up_nodes:
            # Getting the i & type from the node name "link_i" or "joint_i"
            node_type, node_nb = node.name.split('_')
            node_nb = int(node_nb)
            if node_type == 'joint':
                upwards.append(node_nb)
                
        # For downward nodes
        for node in down_nodes:
            # Getting the i & type from the node name "link_i" or "joint_i"
            node_type, node_nb = node.name.split('_')
            node_nb = int(node_nb)
            if node_type == 'joint':
                downwards.append(node_nb)
        
        return upwards, downwards
    
    # Get transition matrices between 2 Joints / Links _______________________
    
    def forward_kinematics(self, origin, destination):
        """
        Description
        -----------
        
        Computes  the  forward kinematics between the 'origin' Joint / Link to
        the 'destination' Joint / Link.
        
        The result is returned as a Sympy Matrix in homogeneous coordinates.
        
        Parameters
        ----------
        origin : str
            Origin  element  of the tree. Is a string formatted like the nodes
            names of self.tree :
                type_number
                
                where type is 'joint' / 'link' and number is the number of the
                considered Joint / Link in self.joints / self.links.
                
                Example : origin='joint_2'
        destination : str
            Destination  element  of  the tree. Is a string formatted like the
            node names of self.tree :
                type_number
                
                where type is 'joint' / 'link' and number is the number of the
                considered Joint / Link in self.joints / self.links.
                
                Example : destination='link_0'
        
        Returns
        -------
        
        T : sympy.matrices.dense.MutableDenseMatrix
            Forward kinematics from origin to destination in homogeneous
            coordinates.
            
            The shape of the matrix is (4x4)
        """
        
        # 1 - Getting the path in the tree ...................................
        
        upwards, downwards = self.branch(origin, destination)
        
        # 2 - Getting the matrix .............................................
        
        # Initialisation
        T = Matrix([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
        
        # First upwards joints
        for up_joint_nb in upwards:
            T *= simplify(self.joints[up_joint_nb].T**(-1))
            simplify(T)
            
        # Then downwards joints
        for down_joint_nb in downwards:
            T *= self.joints[down_joint_nb].T
            simplify(T)
            
        return T
        
    # Cartesian Jacobian _____________________________________________________
    
    def jacobian_3xN(self, origin, destination):
        """
        Description
        -----------
        
        Returns the Cartesian Jacobian between the origin and the destination.
        
        The Jacobian is given by the formula :
            
                  /                             \
                 | dx/dDOF_0    ...    dx/dDOF_n |
            Jx = | dy/dDOF_0    ...    dy/dDOF_n |
                 | dz/dDOF_0    ...    dz/dDOF_n |
                  \                             /
                  
        where  dx,  dy  and  dz  are  the  differentiation  of  the  cartesian
        coordinates of the destination frame in the origin frame,
        
        dDOF_k is the differentiation of the kth degree of freedom encountered
        in the kineatics chain from origin to destination in alphabetic order,
        
        n  is  the  total  number of DOF in the kinematic chain from origin to
        destination,
        
        Jx is a 3xN matrix representing the Jacobian.
        
        This  can  be  seen  as  the  Jacobian  of the position of the forward
        kinematics from origin to destination  (it's the ways it's computed).
        
        
        Parameters
        ----------
        origin : str
            Origin  element  of the tree. Is a string formatted like the nodes
            names of self.tree :
                type_number
                
                where type is 'joint' / 'link' and number is the number of the
                considered Joint / Link in self.joints / self.links.
                
                Example : origin='joint_2'
        destination : str
            Destination  element  of  the tree. Is a string formatted like the
            node names of self.tree :
                type_number
                
                where type is 'joint' / 'link' and number is the number of the
                considered Joint / Link in self.joints / self.links.
                
                Example : destination='link_0'

        Returns
        -------
        Jx : sympy.matrices.dense.MutableDenseMatrix
            Jacobian matrix between origin and destination
            
        list_symbols : list of sympy.core.symbol.Symbol
            List of all the derivatives variables (DOF_k) (alphabetic order)
            The kth element of this list is the DOF_k of the Jacobian

        """
        
        fk = self.forward_kinematics(origin, destination)
        
        # Getting all the symbols (DOFs)
        list_symbols = list(fk.free_symbols)
        
        # Sort by name ascending
        list_symbols.sort(key = lambda sym: sym.name)
        
        Jx = fk[0:3, 3].jacobian(list_symbols)
        
        return simplify(Jx), list_symbols
    
    # Converting to String ___________________________________________________
    
    def __str__(self):
        """
        Description
        -----------
        
        Converts the Robot object to a string
        The  string  is the tree representaiton which nodes are link and joint
        names.
        For  more  details  on  how  this  is  done,  check  out  the  anytree
        documentation : https://anytree.readthedocs.io/en/latest/

        Returns
        -------
        
        String tree representation of the robot

        """
        # This string will be returned
        robstr = f'Robot Name : {self.name}\n'
        robstr += f'Number of Links : {self.nlinks()}\n'
        robstr += f'Number of Joints : {self.njoints()}\n'
        
        # Iterating over the tree
        for pre, _, node in self.tree:
            # Getting the i & type from the node name "link_i" or "joint_i"
            node_type, node_nb = node.name.split('_')
            node_nb = int(node_nb)
            
            # Looking for the name corresponding to this type and number
            
            # Joint type
            if node_type == 'joint':
                real_node_name = self.joints[node_nb].name
                
            # Link Type
            else:
                real_node_name = self.links[node_nb].name
            
            # Adding to the string
            robstr += pre + real_node_name + '\n'
            
        return robstr
    
# ----------------------------------------------------------------------------
# | MAIN - RUNNING TESTS                                                     |
# ----------------------------------------------------------------------------

if __name__ == '__main__':
    
    # Link Class Tests _______________________________________________________
    
    # exemple_0.urdf .........................................................
    
    print("\n==================================\n")
    print("Test : Link - example_0.urdf\n")
    urdf_obj = URDF("./Examples/example_0.urdf")
    link_obj = Link(urdf_obj, 1)
    print(link_obj)
    
    # exemple_1.urdf .........................................................
    
    print("\n==================================\n")
    print("Test : Link - example_1.urdf\n")
    urdf_obj = URDF("./Examples/example_1.urdf")
    link_obj = Link(urdf_obj, 0)
    print(link_obj)
    
    # Joint Class Tests ______________________________________________________
    
    # example_0.urdf .........................................................
    
    print("\n==================================\n")
    print("Test : Joint - example_0.urdf\n")
    urdf_obj = URDF("./Examples/example_0.urdf")
    joint_obj = Joint(urdf_obj, 0)
    print(joint_obj)
    
    # example_1.urdf .........................................................
    
    print("\n==================================\n")
    print("Test : Joint - example_1.urdf\n")
    urdf_obj = URDF("./Examples/example_1.urdf")
    joint_obj = Joint(urdf_obj, 0)
    print(joint_obj)
    
    # Robot Class Tests ______________________________________________________
    
    # example_0.urdf .........................................................

    print("\n==================================\n")
    print("Test : Robot - example_0.urdf\n")
    urdf_obj = URDF("./Examples/kuka.urdf")
    robot_obj = Robot(urdf_obj)
    print(robot_obj)
    robot_obj.joints[0].T
    print(robot_obj.forward_kinematics('joint_5', 'joint_4'))