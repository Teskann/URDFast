# -*- coding: utf-8 -*-
"""
Robot Objects
"""

from anytree import Node, RenderTree, Walker
from URDF import URDF
from sympy import Matrix, simplify
from joints import Joint
from links import Link


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

    # Default Constructor ____________________________________________________

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
        
        TODO !!!!!!!
        
        """

        # 1 - Robot Name .....................................................

        if 'name' in urdf_object.robot[0].keys():
            self.name = urdf_object.robot[0]['name']
        else:
            self.name = "no_name"

        # 2 - Robot Links ....................................................

        self.links = []

        for i in range(urdf_object.nlinks()):
            self.links.append(Link(urdf_object, i))

        # 3 - Robot Joints ...................................................

        self.joints = []

        for i in range(urdf_object.njoints()):
            if progressbar is not None:
                progressbar.setProperty("value",
                                        100 * (i + 1) / urdf_object.njoints())
            self.joints.append(Joint(urdf_object, i))

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
            all_link_nodes[i].parent = all_joint_nodes[self.links[i]
                                                       .child_joints[0]]

        # Setting Global Tree
        self.tree = RenderTree(all_link_nodes[root_link_id])

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
            T *= simplify(self.joints[up_joint_nb].T ** (-1))
            simplify(T)

        # Then downwards joints
        for down_joint_nb in downwards:
            T *= self.joints[down_joint_nb].T
            simplify(T)

        return T.factor().cancel().nsimplify(tolerance=1e-10)

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
        in  the  kinematics  chain  from  origin  to destination in alphabetic
        order,
        
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
        list_symbols.sort(key=lambda sym: sym.name)

        Jx = fk[0:3, 3].jacobian(list_symbols)

        return simplify(Jx), list_symbols

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

            # Link Type
            else:
                real_node_name = self.links[node_nb].name

            # Adding to the string
            rob_str += pre + real_node_name + '\n'

        return rob_str


# ----------------------------------------------------------------------------
# | MAIN - RUNNING TESTS                                                     |
# ----------------------------------------------------------------------------

if __name__ == '__main__':
    # Link Class Tests _______________________________________________________

    # example_0.urdf .........................................................

    print("\n==================================\n")
    print("Test : Link - example_0.urdf\n")
    urdf_obj = URDF("./Examples/example_0.urdf")
    link_obj = Link(urdf_obj, 1)
    print(link_obj)

    # example_1.urdf .........................................................

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
    print(robot_obj.forward_kinematics('joint_5', 'joint_4'))
