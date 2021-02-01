"""
File containing all the functions to generate docstrings for URDFast
"""
from Language import Language


# Get the parameters from sympy ______________________________________________

def get_parameters(syms):
    """
    Get the parameters and their description from a list of symbols

    Parameters
    ----------
    syms : list of sympy.core.symbol.Symbol
        List of all the symbols you want to get the parameters from

    Returns
    -------

    params : list of dict
        Parameters created. Every element of the list has these keys :
            - name : str : Variable Name
            - type : str : Variable Type, can be 'double', 'vect' or 'mat'
            - description : str : Parameter description (for doc string).
            This field depends on the name of the symbol.
    """

    params = []
    for symbol in syms:
        param = {'name': str(symbol), 'type': 'double'}
        category = param['name'].split('_')[0]
        descr = ''
        if category == 'd':
            descr += 'Translation value (in meters) along the '
            descr += param['name'][2:] + ' prismatic joint axis.'

        elif category == 'dx':
            descr += 'Translation value (in meters) along the X axis of the '
            descr += param['name'][3:] + ' joint.'

        elif category == 'dy':
            descr += 'Translation value (in meters) along the Y axis of the '
            descr += param['name'][3:] + ' joint.'

        elif category == 'dz':
            descr += 'Translation value (in meters) along the Z axis of the '
            descr += param['name'][3:] + ' joint.'

        elif category == 'dz':
            descr += 'Translation value (in meters) along the Z axis of the '
            descr += param['name'][3:] + ' joint.'

        elif category == 'theta':
            descr += 'Rotation value (in radians) around the '
            descr += param['name'][6:] + ' joint axis.'

        elif category == 'roll':
            descr += 'Rotation value (in radians) around the X axis of the '
            descr += param['name'][5:] + ' joint.'

        elif category == 'pitch':
            descr += 'Rotation value (in radians) around the Y axis of the '
            descr += param['name'][6:] + ' joint.'

        elif category == 'yaw':
            descr += 'Rotation value (in radians) around the Z axis of the '
            descr += param['name'][4:] + ' joint.'

        param['description'] = descr

        params.append(param)
    return params


# Convert content ____________________________________________________________

def convert_content(content):
    """
    Converts the content ("xzyrpY", ...) to make it readable and pretty

    Parameters
    ----------
    content : str
        Content you want to convert to a pretty string

    Returns
    -------

    str
        Converted content (see examples for details)

    Examples
    --------

    >>> convert_content('xyz')
    'position (x, y, z)'

    >>> convert_content('rpY')
    'orientation (roll, pitch, yaw)'

    >>> convert_content("xyp")
    'position (x, y) and orientation (pitch)'
    """

    pos = False
    cmd = ''
    if any(x in 'xyz' for x in content):
        pos = True
        cmd += 'position ('
        first = True
        for char in 'xyz':
            if char in content:
                cmd += (', ' if not first else '') + char
                first = False
        cmd += ')'

    if any(x in 'rpY' for x in content):
        if pos:
            cmd += " and "
        cmd += 'orientation ('
        first = True
        val = {'r': 'roll',
               'p': 'pitch',
               'Y': 'yaw'}
        for char in 'rpY':
            if char in content:
                cmd += (', ' if not first else '') + val[char]
                first = False
        cmd += ')'
    return cmd


# Split content ______________________________________________________________

def split_content(content):
    """
    Split content to return 2 variables with position and orientation

    Parameters
    ----------
    content : str
        Content ("xyz", "xyzrpY", ...)

    Returns
    -------

    position : str
        Part of the content representing the position
    orientation : str
        Part of the content representing the orientation

    Examples
    --------

    >>> split_content("xyzrpY")
    ('xyz', 'rpY')

    >>> split_content("xrY")
    ('x', 'rY')

    >>> split_content("xy")
    ('xy', '')

    """

    position = ""
    orientation = ""
    for c in content:
        if c in 'xyz':
            position += c
        else:
            orientation += c
    return position, orientation


# Description of the content for a vector ____________________________________

def descr_from_content(content, var_name, language=Language('python')):
    """
    Returns the description of the vector var_name with respect to the content

    Parameters
    ----------
    content : str
        Content of the string ("xyz", "xy", "z", ...) (no rpY)
    var_name : str
        Name of the vector you want to describe
    language : Language.Language
        Language of the generated docstring (used for vector subscription)

        Defaults to Language('python')

    Returns
    -------

    description : str
        Description of the vector

    Examples
    --------

    >>> my_description = descr_from_content("xyz", "Xd")
    >>> print(my_description)
    This is a (3 x 1) numpy.ndarray where :
        - Xd[0] is the x coordinate of Xd,
        - Xd[1] is the y coordinate of Xd,
        - Xd[2] is the z coordinate of Xd

    >>> from Language import Language
    >>> my_description = descr_from_content("xz", "Xd", Language('Julia'))
    >>> print(my_description)
    This is a (2 x 1) Matrix where :
        - Xd[1] is the x coordinate of Xd,
        - Xd[2] is the z coordinate of Xd

    >>> from Language import Language
    >>> my_description = descr_from_content("z", "Xd", Language('MATLAB'))
    >>> print(my_description)
    This is a (1 x 1) double where :
        - Xd(1) is the z coordinate of Xd
    """

    descr = f"This is a ({len(content)} x 1) {language.matrix_type} where :\n"

    for i, c in enumerate(content):
        qp = language.slice_mat(var_name, i, None, None, None)
        coma = ",\n" if i < len(content) - 1 else ""
        descr += f"    - {qp} is the {c} coordinate of {var_name}{coma}"

    return descr


# Robot degrees of freedom description _______________________________________

def robot_dof_descr(robot, var_name, language=Language('python')):
    """
    Returns  the  description  of  the  vector  containing  all the degrees of
    freedom of the robot.

    Parameters
    ----------
    robot : robots.Robot
        Robot you want to describe
    var_name : str
        Name of the vector variable
    language : Language.Language
        Language of the generated docstring (used for vector subscription)

        Defaults to Language('python')

    Returns
    -------

    description : str
        Description of the vector

    Examples
    --------

    >>> from robots import RobotURDF
    >>> from URDF import URDF
    >>> from Language import Language
    >>> rob = RobotURDF(URDF("./Examples/example_0.urdf"))
    >>> print(robot_dof_descr(rob, 'q', Language("julia")))
    This is a (3 x 1) Matrix containing the state of all the joints where :
            - q[1] = theta_joint1 :
                  Rotation value (in radians) around the joint1 joint axis.
            - q[2] = theta_joint2 :
                  Rotation value (in radians) around the joint2 joint axis.
            - q[3] = theta_joint3 :
                  Rotation value (in radians) around the joint3 joint axis.

    """

    params = get_parameters(robot.dof)

    descrq = (f"This is a ({len(robot.dof)} x 1) {language.matrix_type} "
              f"containing the state of all the joints where :\n")
    for i_p, param in enumerate(params):
        qp = language.slice_mat(var_name, i_p, None, None, None)
        descrq += f'        - {qp} = {param["name"]}'
        descrq += f' :\n              {param["description"]}\n'
    return descrq[:-1]


# Control loops dosctrings ___________________________________________________

def docstr_control_loop(loop):
    """
    Generates the docstring for the control loop

    Parameters
    ----------
    loop : dict
        Dictionary with the following structure :

        {"type" : str
            "effector" or "com" for task 1
         "type_2" : str or None
            "effector" or "com" or None for task 2
         "ids" : list of str
            Origin, destination and content for task 1
         "ids_2" : list of str
            Origin, destination and content for task 2
         "trajectory" : str
            Polynomial trajectory used
         "control_type" : str
            "geometric", "positions" or "velocities"
         "coppelia" : bool
            True if coppelia sim support is enabled
         "constraints" : bool
            True if the constraints are enabled
        }

    Returns
    -------

    doc : str
        Docstring containing the control loop description

    Examples
    --------

    >>> my_loop = {"type" : "effector",\
                   "type_2": "com",\
                   "ids":["world", "link_5", "xrpY"],\
                   "ids_2":[None, None, "z"],\
                   "trajectory" : "(None)",\
                   "control_type": "positions",\
                   "coppelia": True,\
                   "constraints": True}
    >>> my_docstr = docstr_control_loop(my_loop)

    >>> my_loop = {'type': 'effector',\
                   'ids': ['world', 'link_A6', 'xyzrpY'],\
                   'type_2': None,\
                   'ids_2': [None, None, None],\
                   'trajectory': '(None)',\
                   'control_type': 'geometric',\
                   'coppelia': False,\
                   'constraints': False}
    >>> my_docstr = docstr_control_loop(my_loop)

    """
    cmds = []
    cfgs = []
    ltypes = [loop["type"], loop["type_2"]]
    lids = [loop["ids"], loop["ids_2"]]
    for i in range(2):
        if ltypes[i] is None:
            break
        elif ltypes[i] == 'effector':
            cmd = (f"effector ({lids[i][1]} in the "
                   f"{lids[i][0]} frame) ")
        else:
            cmd = f"center of mass "
        cmd += convert_content(lids[i][2])

        suffix = "" if ltypes[1] is None else f"_{i + 1}"
        if "center of mass" in cmd:
            cfg = f"center of mass position comd{suffix}"
        elif "position" in cmd and "orientation" in cmd:
            cfg = f"configuration Td{suffix}"
        elif 'position' in cmd:
            cfg = f"position Xd{suffix}"
        else:
            cfg = f"orientation Ad{suffix}"
        cmds.append(cmd)
        cfgs.append(cfg)

    if loop["trajectory"] == "(None)":
        traj = ("This loop does not use any trajectory generation to work."
                " Be careful when you use it as you may not have control "
                "over joint velocities.")
    else:
        traj = ("This loop uses the trajectory generation given by "
                f"{loop['trajectory']}.")

    if loop['coppelia']:
        coppelia = ("These functions are included in this file for simulation"
                    " using Coppelia Sim. For a physical robot, you will "
                    "have to write them yourself.")
    else:
        coppelia = (" These functions are not defined by URDFast, so you need"
                    " to define them yourself.")

    if loop["control_type"] == "geometric":
        ctl_type = ("This control loop is geometric : it does not send any "
                    "data to a simulated nor a physical robot. You can not "
                    "control a real / simulated robot with this function.")
        q0 = "the robot initial configuration q0"
    else:
        ctl_type = (f"This control loop receives / sends joint "
                    f"{loop['control_type']} data to your "
                    "simulated / physical robot through the functions "
                    f"get_positions() and "
                    f"set_{loop['control_type']}(q). {coppelia}")
        q0 = "the robot current configuration (given by get_positions())"

    if loop['constraints']:
        constr = ("The solutions found by the kinematic controller consider "
                  "the physical limits of the robot (max/min positions/"
                  "velocities).")
    else:
        constr = ("The solutions found by the kinematic controller do NOT "
                  "consider the physical limits of the robot (max/min "
                  "positions/velocities)")

    if loop["type_2"] is None:
        intr = (f"Kinematic control loop to command the {cmds[0]} in the task"
                f" space. It brings it to the desired {cfgs[0]} from "
                f"{q0}.")
    else:
        intr = (f"Kinematic control loop to command 2 tasks.\n    - The high "
                f"priority task commands the {cmds[0]}. It brings it to "
                f"the desired {cfgs[0]} from {q0},\n    - The low "
                f"priority task commands the {cmds[1]}. It brings it to "
                f"the desired {cfgs[1]} from {q0}.")

    docstr = (f"{intr}\n\n{constr}\n\n{traj}\n\n{ctl_type}\n\n"
              f"The block scheme representation of this control loop is "
              "available here : "  # TODO(Insert URL when possible)
              "(NOT AVAILABLE YET)\n")
    return docstr