"""
Link elements
"""

import numpy as np


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
        Inertia matrix of the link.  This matrix is symmetric.  Corresponds
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
        directory containing robots.py

        - Running example_0.urdf URDF file (with no <inertial> elements) :
        >>> from URDF import URDF
        >>> urdf_obj = URDF("./Examples/example_0.urdf")
        >>> link_obj = Link(urdf_obj, 1)
        >>> print(link_obj)
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

    def __init__(self, urdf_object, link_number):
        """
        Description
        -----------

        Create a Link object from an URDF Object and a link number

        Parameters
        ----------

        urdf_object : URDF.URDF
            URDF object from URDF library
        link_number : int
            Link  Number  in  the URDF file tree. Must be fewer than the total
            number of links in the URDF object.

        Examples
        --------

        Example URDF files are located in ./Examples  where . is the directory
        containing robots.py

        - Running example_0.urdf URDF file (with no <inertial> elements) :
        >>> from URDF import URDF
        >>> urdf_obj = URDF("./Examples/example_0.urdf")
        >>> link_obj = Link(urdf_obj, 1)
        >>> print(link_obj)
        Link Name : link2 "www.google.com"
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
        >>> from URDF import URDF
        >>> urdf_obj = URDF("./Examples/example_1.urdf")
        >>> link_obj = Link(urdf_obj, 0)
        >>> print(link_obj)
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
        if link_number > urdf_object.nlinks():
            raise IndexError(f"link_number ({link_number}) out of range " +
                             f"({urdf_object.nlinks()})")

        urdf_object_link = urdf_object.links[link_number]
        self.link_id = link_number

        # 1 - Getting the Link Name ..........................................

        try:
            self.name = urdf_object_link['name']
        except KeyError:
            raise KeyError("The URDF Link must contain a 'name' property")

        # 2 - Getting com if exists ..........................................

        try:
            # Setting CoM as a column vector
            self.com = np.array(urdf_object_link['inertial']
                                ['origin']['xyz']).reshape((3, 1))
        except KeyError:
            # If this property doesn't exist we keep the default value
            self.com = np.zeros((3, 1))

        # 3 - Getting mass if exists .........................................

        try:
            self.mass = urdf_object_link['inertial']['mass']['value'][0]
        except KeyError:
            # If this property doesn't exist we keep the default value
            self.mass = 0.0

        # 4 - Getting inertia matrix if exists ...............................

        try:
            # Ixx
            self.inertia[0][0] = (urdf_object_link['inertial']['inertia']
                                  ['ixx'][0])

            # Ixy & Iyx (symmetric)
            self.inertia[1][0] = (urdf_object_link['inertial']['inertia']
                                  ['ixy'][0])
            self.inertia[0][1] = (urdf_object_link['inertial']['inertia']
                                  ['ixy'][0])

            # Ixz & Izx (symmetric)
            self.inertia[2][0] = (urdf_object_link['inertial']['inertia']
                                  ['ixz'][0])
            self.inertia[0][2] = (urdf_object_link['inertial']['inertia']
                                  ['ixz'][0])

            # Iyy
            self.inertia[1][1] = (urdf_object_link['inertial']['inertia']
                                  ['iyy'][0])

            # Iyz & Izy (symmetric)
            self.inertia[1][2] = (urdf_object_link['inertial']['inertia']
                                  ['iyz'][0])
            self.inertia[2][1] = (urdf_object_link['inertial']['inertia']
                                  ['iyz'][0])

            # Izz
            self.inertia[2][2] = (urdf_object_link['inertial']['inertia']
                                  ['izz'][0])

        except KeyError:
            self.inertia = np.eye(3)

        # 5 - Parent and child joints ........................................

        self.parent_joints = []
        self.child_joints = []
        for i in range(urdf_object.njoints()):
            joint = urdf_object.joints[i]
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
            - A 3 x 3 symmetric numpy.ndarray inertia
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
            raise ValueError(
                "Link com shape must be (3, 1) and is currently" +
                f" {self.com.shape}")

        # self.mass ..........................................................

        if type(self.mass) not in [float, int]:
            raise TypeError("Link mass must be a float and is currently a " +
                            f"{type(self.mass)}")
        if self.mass < 0.0:
            raise ValueError(
                "Link mass must be greater or equal to 0 and is" +
                f" currently {self.mass}")

        # self.inertia .......................................................

        if type(self.inertia) != np.ndarray:
            raise TypeError("Link inertia must be a numpy.ndarray and is " +
                            f"currently a {type(self.inertia)}")

        if self.inertia.shape != (3, 3):
            raise ValueError("Link inertia shape must be (3, 3) and is " +
                             f"currently {self.inertia.shape}")

        if not np.allclose(self.inertia, self.inertia.T):
            raise ValueError("Link inertia must be symmetric")

        return True

    # Converting the Link Object to string ___________________________________

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
