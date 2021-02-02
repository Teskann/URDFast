# -*- coding: utf-8 -*-
"""
Robot Objects
"""

from anytree import Node, RenderTree, Walker
from URDF import URDF
from sympy import Matrix, zeros, factor, ones, eye, nsimplify
from joints import JointURDF, JointDH
from links import LinkURDF, LinkDH
from dh_params import dh


# ----------------------------------------------------------------------------
# | ROBOT CLASS                                                              |
# ----------------------------------------------------------------------------

class Robot:
    """
    Description
    -----------
    
    Robot Class. Represents the tree representation of an URDF Robot.
    A robot is composed of joints and links
    
    Data Structure
    --------------
    
    name : str
        Robot Name. If the robot has no name, it will be named "no_name"
    
    links : List of links.Link
        List containing all the Links of the Robot.
        
    joints : List of joints.Joint
        List containing all the Joints of the Robot.
        
    tree : anytree.render.RenderTree
        Tree  representation of the Robot. Tree nodes names are formatted like
        this :
            - For Links
                link_k
            - For Joints
                joint_k
            where k is the number of the Link/Joint in the list links/joints

    mass : float
        Mass of the robot

    saved_fk : dict of dict of sympy.matrices.dense.MutableDenseMatrix
        Variable saving the forward kinematics that have already been computed
        before  to  save time if there is a need to compute it again. The keys
        of  the  first dict are the origins and the keys of the second are the
        destinations.

    saved_jac : dict of dict of sympy.matrices.dense.MutableDenseMatrix
        Variable  saving  the jacobians that have already been computed before
        to  save  time if there is a need to compute it again. The keys of the
        first  dict  are  the  origins  and  the  keys  of  the second are the
        destinations.

    saved_com : sympy.matrices.dense.MutableDenseMatrix or None
        Variable saving the center of mass expression of the robot.

    saved_com_jac : sympy.matrices.dense.MutableDenseMatrix or None
        Variable saving the jacobian of the center of mass of the robot.

    dof : list of sympy.core.symbol.Symbol
        List of all the degrees of freedom of the robot (alphabetical order)

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

    # Saved FK
    saved_fk = {}

    # Saved Jacobians
    saved_jac = {}

    # Saved center of mass
    saved_com = None

    # Saved center of mass jacobian
    saved_com_jac = None

    # Robot mass
    mass = 0

    # Methods ================================================================

    # Constructor ____________________________________________________________

    def __init__(self):
        """
        Constructor to call from children classes at the end of the creation
        """

        self.saved_fk = {}
        self.saved_jac = {}
        self.saved_com = None
        self.saved_com_jac = None

        self.dof = []
        for joint in self.joints:
            self.dof += list(joint.T.free_symbols)

        self.dof.sort(key=lambda x: x.name)

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

        origin_node = None
        destination_node = None
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

    def forward_kinematics(self, origin, destination, content="xyzo",
                           optimization_level=0):
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

        content : str
            Content of the FK ("xyz", "xyzo", "o", ...)

        optimization_level : int
            Optimization level of the generated code.
            - 0 : Numeric  computation  of  FK  using  the transition matrices
            functions (not applicable here)
            - 1 : Computes  directly  the  FK  without  using  the  transition
            matrices functions, but the expression is not simplified
            - 2 : Computes  directly  the  FK  without  using  the  transition
            matrices functions, and the expression is factored
            - 3 : Computes  directly  the  FK  without  using  the  transition
            matrices  functions,  and the expression is simplified. This might
            take a long time to compute for only a few gains compared to 2.
        
        Returns
        -------
        
        T : sympy.matrices.dense.MutableDenseMatrix
            Forward kinematics from origin to destination in homogeneous
            coordinates.
            
            The shape of the matrix is (4x4)
        """

        # 0 - Check if it has already been computed ..........................

        if origin in self.saved_fk.keys()\
            and destination in self.saved_fk[origin].keys()\
                and self.saved_fk[origin][destination][1] >= \
                        optimization_level:
                    T = self.saved_fk[origin][destination][0]

        else:
            # 1 - Getting the path in the tree ...............................

            upwards, downwards = self.branch(origin, destination)

            # 2 - Getting the matrix .........................................

            # Initialisation
            T = Matrix([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

            # First upwards joints
            for up_joint_nb in upwards:
                T *= self.joints[up_joint_nb].Tinv

            # Then downwards joints
            for down_joint_nb in downwards:
                T *= self.joints[down_joint_nb].T

            if optimization_level == 1:
                T = nsimplify(T.evalf(), tolerance=1e-10).evalf()

            else:
                T = nsimplify(T.factor().evalf(), tolerance=1e-10).evalf()
            if optimization_level == 3:
                T = nsimplify(T.simplify(), tolerance=1e-10).evalf()

            # Save the FK
            if origin not in self.saved_fk.keys():
                self.saved_fk[origin] = {}
            self.saved_fk[origin][destination] = [T, optimization_level]

        # Select content .....................................................

        if content == "xyzo":
            return T
        elif content == "o":
            return T[0:3, 0:3]
        elif content == "xyz":
            return T[0:3, 3]
        elif content == "xy":
            return T[0:2, 3]
        elif content == "yz":
            return T[1:3, 3]
        elif content == "xz":
            return Matrix(2, 1, [T[0, 3], T[2, 3]])
        elif content == "x":
            return T[0:1, 3]
        elif content == "y":
            return T[1:2, 3]
        elif content == "z":
            return T[2:3, 3]
        else:
            raise ValueError("FK content is not valid.")

    # Geometric Jacobian _____________________________________________________

    def jacobian(self, origin, destination, content="xyzrpY",
                 optimization_level=1):
        """
        Description
        -----------
        
        Returns the Geometric Jacobian between the origin and the destination.
        
        The Jacobian is given by the formula :
            
                  /                             \
                 | dx/dDOF_0    ...    dx/dDOF_n |
                 | dy/dDOF_0    ...    dy/dDOF_n |
             J = | dz/dDOF_0    ...    dz/dDOF_n |
                 | dr/dDOF_0    ...    dr/dDOF_n |
                 | dp/dDOF_0    ...    dp/dDOF_n |
                 | dY/dDOF_0    ...    dY/dDOF_n |
                  \                             /
                  
        where  dx,  dy  and  dz  are  the  differentiation  of  the  cartesian
        coordinates of the destination frame in the origin frame,

        dr/dDOF_k, dp/dDOF_k and dY/dDOF_k are the angular velocities,
        
        dDOF_k is the differentiation of the kth degree of freedom encountered
        in  the  kinematics  chain  from  origin  to destination in alphabetic
        order,
        
        N  is  the  total  number of DOF in the kinematic chain from origin to
        destination,
        
        J is a 6xN matrix representing the Jacobian.
        
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

        content : str
            Content of the Jacobian ("xyzrpY", ...)

        optimization_level : int
            0 or 1 => The Jacobian is not simplified at all
            2 => The Jacobian is factored
            3 => The Jacobian is simplified

        Returns
        -------
        J : sympy.matrices.dense.MutableDenseMatrix
            Jacobian matrix between origin and destination
            
        list_symbols : list of sympy.core.symbol.Symbol
            List of all the derivatives variables (DOF_k) (alphabetical order)
            The kth element of this list is the DOF_k of the Jacobian

        """

        # 0 - Check if it has already been computed ..........................

        if origin in self.saved_jac.keys()\
            and destination in self.saved_jac[origin].keys()\
                and self.saved_jac[origin][destination][1] >= \
                        optimization_level:
                    JJ = self.saved_jac[origin][destination][0]

        else:

            fk = self.forward_kinematics(origin, destination)

            Jx = fk[0:3, 3].jacobian(self.dof)
            Jo = zeros(*Jx.shape)
            upwards, downwards = self.branch(origin, destination)

            for i, j_nb in enumerate(upwards + downwards):
                if self.joints[j_nb].joint_type.lower() in ["continuous",
                                                            "revolute"]:
                    Jo[0:3, i] = self\
                        .forward_kinematics(origin, f"joint_{j_nb}",
                                            optimization_level=1)[0:3, 2]

            JJ = Matrix([[Jx], [Jo]])

            if optimization_level > 1:
                JJ = factor(JJ).evalf().nsimplify(tolerance=1e-10).evalf()
            if optimization_level > 2:
                JJ = JJ.simplify().nsimplify(tolerance=1e-10).evalf()

            # Save the Jac
            if origin not in self.saved_jac.keys():
                self.saved_jac[origin] = {}
            self.saved_jac[origin][destination] = [JJ,
                                                   optimization_level]

        all_lines = "xyzrpY"
        to_del = []
        Jret = JJ.copy()
        for i, line in enumerate(all_lines):
            if line not in content:
                to_del.append(i)
        for ddel in to_del[::-1]:
            tmp = Jret.row_del(ddel)
            if tmp is not None:
                Jret = tmp

        return Jret

    # Center of mass _________________________________________________________

    def com(self, content, optimization_level):
        """
        Computes the center of mass of the robot.
        Returned as a 3x1 vector.

        Parameters
        ----------

        content : str
            Content of the center of mass ("xyz", "xz", ...)

        optimization_level : int
            0 or 1 => The CoM is not simplified at all
            2 => The CoM is factored
            3 => The CoM is simplified

        Returns
        -------

        com : sympy.matrices.dense.MutableDenseMatrix
            Center of mass of the robot in the root (world) frame

        Raises
        ------

        ValueError : If the robot mass is null
        """

        if self.mass == 0:
            raise ValueError("Cannot compute the robot center of mass "
                             "because its mass is equal to 0 kg")

        if self.saved_com is not None\
                and self.saved_com[1] >= optimization_level:
            com_expr = self.saved_com[0]
        else:
            def recursive_com(link, frame):
                """
                Recursive function to compute the center of mass
                Parameters
                ----------
                link : links.Link
                    Link from which you want to compute the center of mass.

                frame : links.link
                    Frame in which you want to express the CoM

                Returns
                -------

                com : sympy.matrices.dense.MutableDenseMatrix
                    Jacobian matrix between origin and destination

                """
                m = link.mass / self.mass
                hc = ones(4, 1)
                hc[:3, :] = link.com
                if link.is_root:
                    T = eye(4, 4)
                else:
                    T = self.forward_kinematics(f"link_{frame.link_id}",
                                                f"joint_"
                                                f"{link.child_joints[0]}")
                cm = m * (T @ hc)
                if link.is_terminal:
                    return cm
                else:
                    for child in link.parent_joints:
                        cm += recursive_com(self.links[self.joints[child]
                                            .child],
                                            frame)
                    return cm

            com_expr = None
            for link in self.links:
                if link.is_root:
                    com_expr = recursive_com(link, link)
                    break

            if optimization_level > 1:
                com_expr = factor(com_expr).evalf()\
                    .nsimplify(tolerance=1e-10)\
                    .evalf()
            if optimization_level > 2:
                com_expr = com_expr.simplify().nsimplify(tolerance=1e-10)\
                    .evalf()

            com_expr = com_expr[:3, :]
            self.saved_com = [com_expr, optimization_level]

        all_lines = "xyz"
        to_del = []
        comret = com_expr.copy()
        for i, line in enumerate(all_lines):
            if line not in content:
                to_del.append(i)
        for ddel in to_del[::-1]:
            tmp = comret.row_del(ddel)
            if tmp is not None:
                comret = tmp

        return comret

    # CoM Jacobian ___________________________________________________________

    def com_jacobian(self, content, optimization_level):
        """
        Computes  the  jacobian  of  the  center  of  mass  (only position, no
        orientation).

        Parameters
        ----------

        content : str
            Content of the CoM Jacobian ("xy", "xyz", ...)

        optimization_level : int
            0 or 1 => The CoM Jacobian is not simplified at all
            2 => The CoM Jacobian is factored
            3 => The CoM Jacobian is simplified

        Returns
        -------

        com_jac : sympy.matrices.dense.MutableDenseMatrix
            Jacobian  of  the  center of mass of the robot in the root (world)
            frame.
        """

        if self.saved_com_jac is not None\
                and self.saved_com_jac[1] >= optimization_level:
            com_jac = self.saved_com_jac[0]
        else:
            com_expr = self.com("xyz", optimization_level)

            com_jac = com_expr.jacobian(self.dof)

            if optimization_level > 1:
                com_jac = factor(com_jac).evalf().nsimplify(tolerance=1e-10) \
                    .evalf()
            if optimization_level > 2:
                com_jac = com_jac.simplify().nsimplify(tolerance=1e-10)\
                    .evalf()

            self.saved_com_jac = [com_jac, optimization_level]

        all_lines = "xyz"
        to_del = []
        Jret = com_jac.copy()
        for i, line in enumerate(all_lines):
            if line not in content:
                to_del.append(i)
        for ddel in to_del[::-1]:
            tmp = Jret.row_del(ddel)
            if tmp is not None:
                Jret = tmp

        return Jret

    # Converting to String ___________________________________________________

    def __str__(self):
        """
        Description
        -----------
        
        Converts the Robot object to a string
        The  string  is the tree representation which nodes are link and joint
        names.
        For  more  details  on  how  this  is  done,  check  out  the  anytree
        documentation : https://anytree.readthedocs.io/en/latest/

        Returns
        -------
        
        String tree representation of the robot

        """
        # This string will be returned
        rob_str = f'Robot Name : {self.name}\n'
        rob_str += f'Number of Links : {self.nlinks()}\n'
        rob_str += f'Number of Joints : {self.njoints()}\n'

        # Iterating over the tree
        for pre, _, node in self.tree:
            # Getting the i & type from the node name "link_i" or "joint_i"
            node_type, node_nb = node.name.split('_')
            node_nb = int(node_nb)

            # Looking for the name corresponding to this type and number

            # Joint type
            if node_type == 'joint':
                real_node_name = self.joints[node_nb].name
                # Display degrees of freedom
                try:
                    dof = "(" + str(self.joints[node_nb].T
                                    .free_symbols)[1:-1] + ")"
                    real_node_name += dof
                except AttributeError:
                    pass

            # Link Type
            else:
                real_node_name = self.links[node_nb].name

            # Adding to the string
            rob_str += pre + real_node_name + '\n'

        return rob_str


# ----------------------------------------------------------------------------
# | URDF Robots                                                              |
# ----------------------------------------------------------------------------

class RobotURDF(Robot):
    """
    Robot for URDF files.

    Only implements a constructor for URDF files.
    For more details, see Robot class

    Examples
    --------

    You can create a robot from an URDF file using the parser :

    >>> from URDF import URDF
    >>> urdf_obj = URDF("./Examples/example_0.urdf")
    >>> robot_obj = RobotURDF(urdf_obj)

    """

    # URDF Constructor _______________________________________________________

    def __init__(self, urdf_object, progressbar=None):
        """
        Description
        -----------

        Robot Constructor. You can construct a robot from an URDF Object.

        Parameters
        ----------

        urdf_object : URDF.URDF
            URDF Object from the URDF library

        progressbar : PyQt5.QtWidgets.QProgressBar or None, optional
                      default is None
            Progressbar to update during the robot creation (used in GUI)
            If it is None, no progressbar is updated

        Examples
        --------

        Examples
        --------

        You can create a robot from an URDF file using the parser :

        >>> from URDF import URDF
        >>> urdf_obj = URDF("./Examples/example_0.urdf")
        >>> robot_obj = RobotURDF(urdf_obj)

        """

        # 1 - Robot Name .....................................................

        if 'name' in urdf_object.robot[0].keys():
            self.name = urdf_object.robot[0]['name']
        else:
            self.name = "no_name"

        # 2 - Robot Links ....................................................

        self.links = []

        for i in range(urdf_object.nlinks()):
            self.links.append(LinkURDF(urdf_object, i))

        # 3 - Robot Joints ...................................................

        self.joints = []

        for i in range(urdf_object.njoints()):
            if progressbar is not None:
                progressbar.setProperty("value",
                                        100 * (i + 1) / urdf_object.njoints())
            self.joints.append(JointURDF(urdf_object, i))

        # 4 - Tree Representation ............................................

        # Creating a Node per Link . . . . . . . . . . . . . . . . . . . . . .

        all_link_nodes = []
        for i, _ in enumerate(self.links):
            all_link_nodes.append(Node('link_' + str(i)))

        # Creating a Node per Joint  . . . . . . . . . . . . . . . . . . . . .

        all_joint_nodes = []
        for i, joint in enumerate(self.joints):
            all_joint_nodes.append(Node('joint_' + str(i),
                                        parent=all_link_nodes[joint.parent]))

        # Setting parents for Link Nodes . . . . . . . . . . . . . . . . . . .

        root_link_id = 0
        for i, _ in enumerate(all_link_nodes):
            if self.links[i].is_root:
                root_link_id = i
                continue
            all_link_nodes[i].parent = (all_joint_nodes[self.links[i]
                                        .child_joints[0]])

        # Setting Global Tree
        self.tree = RenderTree(all_link_nodes[root_link_id])

        self.mass = 0
        for link in self.links:
            self.mass += link.mass

        super().__init__()


# ----------------------------------------------------------------------------
# | DH params robot                                                          |
# ----------------------------------------------------------------------------

class RobotDH(Robot):
    """
    Robot for URDF files.

    Only implements a constructor for URDF files.
    For more details, see Robot class

    Example
    -------

    You can create a RobotDH from a .dhparams file using the parser :

    >>> from dh_params import dh
    >>> dhparams_obj = dh("./Examples/example_0.dhparams")
    >>> robot_obj = RobotDH(dhparams_obj)
    """

    # Dhparams Constructor ___________________________________________________

    def __init__(self, dhparams_object):
        """
        Construct a Robot from a DHParams object

        Parameters
        ----------

        dhparams_object : dh_params.DHParams
            DHParams object of the robot you want to create

        Example
        -------

        You can create a RobotDH from a .dhparams file using the parser :

        >>> from dh_params import dh
        >>> dhparams_obj = dh("./Examples/example_0.dhparams")
        >>> robot_obj = RobotDH(dhparams_obj)
        """

        # Init Object attributes
        self.name = dhparams_object.name
        self.links = []
        self.joints = []
        all_link_nodes = []
        all_joint_nodes = []

        # Create the world link
        self.links.append(LinkDH(dhparams_object, 0, is_world=True))
        all_link_nodes.append(Node("link_0"))

        # Create all the joints and links
        for i, _ in enumerate(dhparams_object.rows):
            self.joints.append(JointDH(dhparams_object, i))
            self.links.append(LinkDH(dhparams_object, i))

            # Tree structure
            all_joint_nodes.append(Node(f"joint_{i}",
                                        parent=all_link_nodes[-1]))
            all_link_nodes.append(Node(f"link_{i + 1}",
                                       parent=all_joint_nodes[-1]))

        self.tree = RenderTree(all_link_nodes[0])

        self.mass = 0
        for link in self.links:
            self.mass += link.mass

        super().__init__()

# ----------------------------------------------------------------------------
# | MAIN - RUNNING TESTS                                                     |
# ----------------------------------------------------------------------------

if __name__ == '__main__':
    # Robot Class Tests ______________________________________________________

    # example_0.urdf .........................................................

    print("\n==================================\n")
    print("Test : Robot - example_0.urdf\n")
    urdf_obj = URDF("./Examples/kuka.urdf")
    robot_obj = RobotURDF(urdf_obj)
    print(robot_obj)
    print(robot_obj.forward_kinematics('joint_0', 'joint_1'))

    print("\n==================================\n")
    print("Test : Robot - example_0.dhparams\n")
    dhparams_obj = dh("./Examples/kuka.dhparams")
    robot_obj = RobotDH(dhparams_obj)
    print(robot_obj)
    print(robot_obj.forward_kinematics('joint_0', 'joint_1'))

