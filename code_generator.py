# -*- coding: utf-8 -*-
"""
Created on Fri Jun 19 21:44:37 2020

@author: Clément
"""

from URDF import URDF
from robots import Robot
from sympy import pretty, Symbol
from Language import Language
from datetime import datetime
from code_optimization import replace_var, optimize
from polynomial_trajectory import get_solution, verify_solution, get_equations
from anytree import PreOrderIter
from docstrings import *


# Update progressbar _________________________________________________________

def increment_progressbar(progressbar, increment):
    """
    Increment the progressbar

    Parameters
    ----------

    progressbar : PyQt5.QtWidgets.QProgressBar or None, optional
        default is None
        Progressbar to update during the robot creation (used in GUI)
        If it is None, no progressbar is updated

    increment : float
        Progressbar value between 0 and 100
    """

    if progressbar is None:
        return
    else:
        progressbar.setProperty("value", progressbar.value() + increment)


# Generate Python code from Sympy Matrix _____________________________________

def generate_code_from_sym_mat(sympy_matrix, fname,
                               language=Language('python'),
                               docstr=None,
                               input_is_vector=False,
                               dof=None):
    """
    Description
    -----------
    
    Generates code from the Sympy Matrix expression.
    
    Python generated code uses numpy dependency to deal with matrices.
    
    This only generates the body of the function
    
    Parameters
    ----------
    
    sympy_matrix : sympy.matrices.dense.MutableDenseMatrix
        Sympy matrix to convert to code
        
    fname : str:
        Function Name
        
    language : Language
        Language for code generation. Supported values are :
            - Language('Python')
            
        Defaluts to 'python'
    
    docstr : str, optional
        Docstring of the function
        Default is None

    input_is_vector : bool
        Set  to  True if you want the parameters of the code of your matrix to
        be passed as a vector instead of many variables.

        Default is False.

    dof : list of sympy.core.symbol.Symbol or None
        List  of  all the degrees of freedom of the robot. Must not be None if
        inut_input_is_vector is not True
        
    Returns
    -------
    
    code : str
        string containing the generated code

    """

    # 1 - Getting the Sympy Matrix as string .................................

    # Optimizing code
    varss, expr = optimize(str(sympy_matrix), incl_lists=False)

    # Matrix containing code samples
    code_mat = []
    expr = expr[9:-3].replace(' ', '')
    elems = expr.split('],[')

    for elem in elems:
        code_mat.append(elem.split(','))

    # 2 - Getting function parameters ........................................

    syms = list(sympy_matrix.free_symbols)
    syms.sort(key=lambda x: str(x))
    params = get_parameters(syms)

    r = language.generate_fct("mat", fname, params, code_mat, varss=varss,
                              docstr=docstr,
                              input_is_vector=input_is_vector,
                              matrix_dims=(len(code_mat), len(code_mat[0])),
                              dof=dof)

    return r


# Generate all matrices ______________________________________________________

def generate_all_matrices(robot, list_ftm, list_btm,
                          language=Language('python'),
                          progressbar=None,
                          progress_increment=0):
    """
    Description
    -----------
    
    Generate all transformation matrices for each joint of the robot
    
    Parameters
    ----------
    
    robot : robots.Robot
        Robot you want to generate the matrices from
    
    list_ftm : list of str
        List of all the forward transition matrices to generate
        Every element is a string formatted like the nodes
        names of robot.tree :
            type_number
            
            where  type  is 'joint' (in this case) and number is the number of
            the considered Joint in robot.joints.
            
            Example : list_ftm=['joint_3', 'joint_2']

    list_btm : list of str
        List of all the backward transition matrices to generate
        Every element is a string formatted like the nodes
        names of robot.tree :
            type_number
            
            where  type  is 'joint' (in this case) and number is the number of
            the considered Joint in robot.joints
            
            Example : list_btm=['joint_3', 'joint_2']
    
    language : Language.Language, optional
        Language you want the code to be generated to
        Default is Language('python')

    progressbar : PyQt5.QtWidgets.QProgressBar or None, optional
        default is None
        Progressbar to update during the robot creation (used in GUI)
        If it is None, no progressbar is updated

    progress_increment : float
        Progressbar  increment.  Default  is  0.  If progressbar is None, this
        parameter is ignored.
    
    Returns
    -------
    
    str :
        String containing the code of all the functions
    
    """

    code = ''

    if list_ftm:
        code += language.title('FORWARD TRANSITION MATRICES', 0)

        code += '\n\n'

    # For every joint ........................................................

    kk = 1
    for jj in list_ftm:
        print(f"Generating Forward Transition Matrix {kk}/{len(list_ftm)}")
        _, i_j = jj.split('_')
        i_j = int(i_j)
        joint = robot.joints[i_j]

        code += language.title('Joint ' + str(i_j), 1) + '\n\n'

        # Function name
        fname = 'T_' + joint.name

        # Docstring
        docstr = "Transition Matrix to go from link "
        docstr += robot.links[joint.parent].name + ' to link '
        docstr += robot.links[joint.child].name + '.\nThis joint is '
        docstr += joint.joint_type + '. The matrix is :\n\n'
        docstr += pretty(joint.T, num_columns=language.max_line_length - 4,
                         use_unicode=False)

        joint_str = generate_code_from_sym_mat(joint.T, fname,
                                               language, docstr)

        code += joint_str + '\n\n'

        increment_progressbar(progressbar, progress_increment)
        kk += 1

    if list_btm:
        code += language.title('BACKWARD TRANSITION MATRICES', 0)

        code += '\n\n'

    # For every joint ........................................................

    kk = 1
    for jj in list_btm:
        print(f"Generating Backward Transition Matrix {kk}/{len(list_ftm)}")
        _, i_j = jj.split('_')
        i_j = int(i_j)
        joint = robot.joints[i_j]

        T = joint.Tinv

        code += language.title('Joint ' + str(i_j) + ' Inverse', 1) + '\n\n'

        # Function name
        fname = 'T_' + joint.name + '_inv'

        # Docstring
        docstr = "Transition Matrix to go from link "
        docstr += robot.links[joint.child].name + ' to link '
        docstr += robot.links[joint.parent].name + '.\nThis joint is '
        docstr += joint.joint_type + '. The matrix is :\n\n'
        docstr += pretty(T, num_columns=language.max_line_length - 4,
                         use_unicode=False) + '\n'

        joint_str = generate_code_from_sym_mat(T, fname, language, docstr)

        code += joint_str
        if i_j < len(robot.joints) - 1:
            code += '\n\n'

        increment_progressbar(progressbar, progress_increment)
        kk += 1

    return code


# Generate Forward Kinematics ________________________________________________

def generate_fk(robot, origin, destination, content, optimization_level,
                language=Language('python')):
    """
    Description
    -----------
    
    Generate function to compute forward kinematics.
    This   function    generates    code   using   generated   matrices   from
    generate_all_matrices.  Consider  using this function after having run the
    generate_all_matrices function
    
    Parameters
    ----------
    
    robot : robots.Robot
        Robot you want to generate the matrices from
    
    origin : str
        Origin  element  of the FK. Is a string formatted like the nodes
        names of self.tree :
            type_number
            
            where type is 'joint' / 'link' and number is the number of the
            considered Joint / Link in self.joints / self.links.
            
            Example : origin='joint_2'
    destination : str
        Destination  element  of  the FK. Is a string formatted like the
        node names of self.tree :
            type_number
            
            where type is 'joint' / 'link' and number is the number of the
            considered Joint / Link in self.joints / self.links.
            
            Example : destination='link_0'

    content : str
        Content of the FK ('xyz', "xyzo", "o", ...)

    optimization_level : int
        Optimization level of the generated code.
            - 0 : Numeric  computation  of  FK  using  the transition matrices
            functions
            - 1 : Computes  directly  the  FK  without  using  the  transition
            matrices functions, but the expression is not simplified
            - 2 : Computes  directly  the  FK  without  using  the  transition
            matrices functions, and the expression is factored
            - 3 : Computes  directly  the  FK  without  using  the  transition
            matrices  functions,  and the expression is simplified. This might
            take a long time to compute for only a few gains compared to 2.
    
    language : Language.Language, optional
        Language you want the code to be generated to
        Default is Language('python')
        
    Returns
    -------
    
    str :
        String  containing the forward kinematics function in the language you
        want
    
    """

    # Adding Title
    code = language.title('Forward Kinematics from ' + origin + ' to ' +
                          destination, 1)
    code += '\n\n'

    # Function properties ....................................................

    index_origin = int(origin.split('_')[1])
    type_origin = origin.split('_')[0]
    origin_name = robot.joints[index_origin].name if type_origin == 'joint' \
        else robot.links[index_origin].name
    index_dest = int(destination.split('_')[1])
    type_dest = destination.split('_')[0]
    dest_name = robot.joints[index_dest].name if type_dest == 'joint' \
        else robot.links[index_dest].name

    if "o" not in content:
        dim_ret = (f"({len(content)} x 1) {language.matrix_type}, giving the"
                   " position")
    elif content == "o":
        dim_ret = (f"(3 x 3) {language.matrix_type} (rotation matrix), "
                   f"giving the orientation")
    else:
        dim_ret = (f"(4 x 4) {language.matrix_type} in homogeneous "
                   f"coordinates, giving the position and the orientation")
    docstr = (f'Computes the forward kinematics from the {type_origin} '
              f"{origin_name} to the {type_dest} {dest_name}. The result is "
              f"returned as a {dim_ret} of "
              f"{dest_name} in the {origin_name} frame.")

    fname = 'fk_' + origin_name + '_' + dest_name + "_" + content

    # Optimised version ......................................................

    if optimization_level > 0:
        fk = robot.forward_kinematics(origin, destination, content=content,
                                      optimization_level=optimization_level)
        return generate_code_from_sym_mat(fk, fname, language, docstr,
                                          input_is_vector=True, dof=robot.dof)

    # Not optimised version ..................................................

    # 1 - Getting the path in the tree .......................................

    upwards, downwards = robot.branch(origin, destination)

    # 2 - Getting the matrix .................................................

    # Intermediate variables
    varss = []

    # FK function parameters
    params = []

    for up_joint_nb in upwards:
        joint = robot.joints[up_joint_nb]

        # Parameters
        all_sym = joint.T.free_symbols
        all_sym.sort(key=lambda x: str(x))
        params_tmp = get_parameters(all_sym)
        params += get_parameters(all_sym)

        val = 'MATLAB_PREFIXT_' + joint.name + '_inv('

        for i_p, par in enumerate(params_tmp):
            val += par['name']
            if i_p < len(params_tmp) - 1:
                val += ','
            else:
                val += ')'

        variable = {'name': 'MATLAB_PREFIXT_' + str(up_joint_nb) + '_inv',
                    'value': val,
                    'type': 'mat'}
        varss.append(variable)

    # Then downwards joints
    for down_joint_nb in downwards:
        joint = robot.joints[down_joint_nb]
        # Parameters
        all_sym = joint.T.free_symbols
        params_tmp = get_parameters(all_sym)
        params += get_parameters(all_sym)
        val = 'MATLAB_PREFIXT_' + joint.name + '('

        params_tmp.sort(key=lambda x: x['name'])

        for i_p, par in enumerate(params_tmp):
            val += par['name']
            if i_p < len(params_tmp) - 1:
                val += ','
            else:
                val += ')'

        variable = {'name': 'MATLAB_PREFIXT_' + str(down_joint_nb),
                    'value': val,
                    'type': 'mat'}
        varss.append(variable)

    if len(varss) == 0:
        variable = {'name': 'T',
                    'value': '___eye__4__4___',
                    'type': 'mat'}
        varss.append(variable)
    expr = ''
    for i_v, var in enumerate(varss):

        expr += str(var['name'])
        if i_v < len(varss) - 1:
            expr += '@'

    params.sort(key=lambda x: x['name'])

    code += language.generate_fct("mat", fname, params, expr, varss, docstr,
                                  matrix_dims=(1, 1), input_is_vector=True,
                                  dof=robot.dof)

    return code


# Generate all FK functions __________________________________________________

def generate_all_fk(robot, list_origin, list_dest, list_content,
                    optimization_level,
                    language=Language('python'),
                    progressbar=None,
                    progress_increment=0):
    """
    Description
    -----------
    
    Generate all forward kinematics functions
    
    Parameters
    ----------
    
    robot : robots.Robot
        Robot you want to generate the matrices from
    
    list_origin : list of str
        List of all origins elements.
        Every element is a string formatted like the nodes
        names of self.tree :
            type_number
            
            where type is 'joint' / 'link' and number is the number of the
            considered Joint / Link in self.joints / self.links.
            
            Example : origin='joint_2'
    list_dest : list of str
        Every element is a string formatted like the
        node names of self.tree :
            type_number
            
            where type is 'joint' / 'link' and number is the number of the
            considered Joint / Link in self.joints / self.links.
            
            Example : destination='link_0'
    list_content : list of str
        Every element is a string containing the content of the FK
        ("xyz", "xyzo", "o", ...)

    optimization_level : int
        Optimization level of the generated code.
            - 0 : Numeric  computation  of  FK  using  the transition matrices
            functions
            - 1 : Computes  directly  the  FK  without  using  the  transition
            matrices functions, but the expression is not simplified
            - 2 : Computes  directly  the  FK  without  using  the  transition
            matrices functions, and the expression is factored
            - 3 : Computes  directly  the  FK  without  using  the  transition
            matrices  functions,  and the expression is simplified. This might
            take a long time to compute for only a few gains compared to 2.
    
    language : Language.Language, optional
        Language you want the code to be generated to
        Default is Language('python')

    progressbar : PyQt5.QtWidgets.QProgressBar or None, optional
        default is None
        Progressbar to update during the robot creation (used in GUI)
        If it is None, no progressbar is updated

    progress_increment : float
        Progressbar  increment.  Default  is  0.  If progressbar is None, this
        parameter is ignored.
        
    Returns
    -------
    
    str :
        String  containing the forward kinematics function in the language you
        want
    
    """

    if len(list_origin) == 0:
        return ''

    # Adding Title
    code = language.title("FORWARD KINEMATICS", 0)
    code += '\n\n'

    for i, origin, in enumerate(list_origin):
        print(f"Generating Forward Kinematics {i + 1}/{len(list_origin)}")
        code += generate_fk(robot, origin, list_dest[i],
                            list_content[i], optimization_level,
                            language=language)
        if i < len(list_origin) - 1:
            code += '\n\n'

        increment_progressbar(progressbar, progress_increment)

    return code


# Generate Jacobian Function _________________________________________________

def generate_jacobian(robot, origin, destination, content,
                      language=Language('python'), optimization_level=0):
    """
    Description
    -----------
    
    Generate function to compute 6xn Jacobian where n is the number  of DoF of
    the robot from origin to destination
    This   function    generates    code   using   generated   matrices   from
    generate_all_matrices.  Consider  using this function after having run the
    generate_all_matrices function
    
    Parameters
    ----------
    
    robot : robots.Robot
        Robot you want to generate the matrices from
    
    origin : str
        Origin  element  of the FK. Is a string formatted like the nodes
        names of self.tree :
            type_number
            
            where type is 'joint' / 'link' and number is the number of the
            considered Joint / Link in self.joints / self.links.
            
            Example : origin='joint_2'
            
    destination : str
        Destination  element  of  the FK. Is a string formatted like the
        node names of self.tree :
            type_number
            
            where type is 'joint' / 'link' and number is the number of the
            considered Joint / Link in self.joints / self.links.
            
            Example : destination='link_0'

    content : str
        Content of the jacobian ("xyzrpY", ...)
    
    language : Language.Language, optional
        Language you want the code to be generated to
        Default is Language('python')

    optimization_level : int
        - 0 => Jacobian is computed numerically (fastest to generate, slowest
        code
        - 1 => Jacobian is computed analytically but is not simplified
        - 2 => Jacobian is computed analytically and is factored
        - 3 => Jacobian is computed analytically and is simplified
        
    Returns
    -------
    
    str :
        String  containing the forward kinematics function in the language you
        want
    
    """

    # Adding Title
    code = language.title('Jacobian of the ' + destination + ' position ' +
                          'and orientation', 1)
    code += '\n\n'
    jac = None

    # Jacobian function parameters
    params = []

    if optimization_level == 0:

        # 1 - Getting the path in the tree ...................................

        upwards, downwards = robot.branch(origin, destination)

        # 2 - Getting the matrix .............................................

        # Intermediate variables
        varss = []

        T_fcts = []

        # First upwards joints
        for up_joint_nb in upwards:
            joint = robot.joints[up_joint_nb]

            # Paramters
            all_sym = joint.T.free_symbols
            params_tmp = get_parameters(all_sym)
            params += get_parameters(all_sym)

            val = 'MATLAB_PREFIXT_' + joint.name + '_inv('

            params_tmp.sort(key=lambda x: x['name'])

            for i_p, par in enumerate(params_tmp):
                val += par['name']
                if i_p < len(params_tmp) - 1:
                    val += ','
                else:
                    val += ')'

            T_fcts.append([f'MATLAB_PREFIXT_{up_joint_nb}_inv', val])

        # Then downwards joints
        for down_joint_nb in downwards:
            joint = robot.joints[down_joint_nb]
            # Parameters
            all_sym = joint.T.free_symbols
            params_tmp = get_parameters(all_sym)
            params += get_parameters(all_sym)
            val = 'MATLAB_PREFIXT_' + joint.name + '('

            params_tmp.sort(key=lambda x: x['name'])

            for i_p, par in enumerate(params_tmp):
                val += par['name']
                if i_p < len(params_tmp) - 1:
                    val += ','
                else:
                    val += ')'

            T_fcts.append(['MATLAB_PREFIXT_' + str(down_joint_nb), val])

        # Jacobian variables .................................................

        nb_dof = len(params)

        varss.append({'name': 'Jac',
                      'value': f"___zeros__6__{nb_dof}___",
                      'type': 'mat'})

        varss.append({'name': 'T',
                      'value': f'{T_fcts[0][1]}',
                      'type': 'mat'})

        varss.append({'name': 'L',
                      'value': f'p0-{language.slice_mat("T", 0, 2, 3, None)}',
                      'type': 'mat'})

        varss.append({'name': 'Z',
                      'value': language.slice_mat("T", 0, 2, 2, None),
                      'type': 'mat'})

        # Compute Jacobian column 1:3
        varss.append({'name': language.slice_mat("Jac", 0, 2, 0, None),
                      'value': 'cross(Z,L)',
                      'type': ''})

        # Compute Jacobian column 4:6
        varss.append({'name': language.slice_mat("Jac", 3, 5, 0, None),
                      'value': 'Z',
                      'type': ''})

        for i in range(1, nb_dof):
            # Multiplying by T
            varss.append({'name': 'T',
                          'value': f'T@{T_fcts[i][1]}',
                          'type': ''})

            # Compute L
            varss.append({'name': 'L',
                          'value':
                              f"p0-{language.slice_mat('T', 0, 2, 3, None)}",
                          'type': ''})
            # Compute Z
            varss.append({'name': 'Z',
                          'value': language.slice_mat("T", 0, 2, 2, None),
                          'type': ''})

            # Compute Jacobian column 1:3
            varss.append({'name': language.slice_mat("Jac", 0, 2, i, None),
                          'value': 'cross(Z,L)',
                          'type': ''})

            # Compute Jacobian column 4:6
            varss.append({'name': language.slice_mat("Jac", 3, 5, i, None),
                          'value': 'Z',
                          'type': ''})

    else:
        jac = robot\
            .jacobian(origin, destination, content,
                      optimization_level=optimization_level)

        params += get_parameters(robot.dof)
    expr = 'Jac'
    index_origin = int(origin.split('_')[1])
    type_origin = origin.split('_')[0]
    origin_name = robot.joints[index_origin].name if type_origin == 'joint' \
        else robot.links[index_origin].name
    index_dest = int(destination.split('_')[1])
    type_dest = destination.split('_')[0]
    dest_name = robot.joints[index_dest].name if type_dest == 'joint' \
        else robot.links[index_dest].name

    descrq = f'Vector of length {len(params)} containing all the degrees of ' + \
             'freedom of the robot between ' + \
             f'{origin_name} and {dest_name} chain. This vector contains :'
    for i_p, param in enumerate(params):
        descrq += f'\n        - q[{i_p + language.indexing_0}] = ' + \
                  param['name']
        if param['description'] != '':
            descrq += ' :\n              ' + param['description']

    paramq = {'name': 'q', 'type': 'vect', 'description': descrq}

    p0 = {'name': 'p0', 'type': 'vect', 'description':
        f"Point in the {origin_name} frame where you want to compute the" + \
        " Jacobian Matrix. p0 is a (3 x 1) vector."}
    parameters = [paramq, p0]

    docstr = (f'Computes the Jacobian Matrix of the {dest_name} coordinates '
              f'in the {origin_name} frame from the point p0. This matrix is '
              f'returned as a ({len(content)} x {len(robot.dof)}) '
              f'matrix where every column '
              f'is the derivative of the position/orientation with respect to'
              f' a degree of freedom. \n')
    for i_cc, cc in enumerate(content):
        docstr += f'    - The line {i_cc} is the '
        if cc in "xyz":
            docstr += f"derivative of {cc.upper()} position "
        else:
            d = {"r": "X", "p": "Y", "Y": "Z"}
            docstr += f"the angular velocity about the {d[cc]} axis "
        docstr += f"of {dest_name} in the {origin_name} frame,\n"
    docstr += 'Here is the list of all the derivative variables :'
    for i_p, param in enumerate(robot.dof):
        docstr += f'\n    - Column {language.indexing_0 + i_p} : ' + \
                  f'{param.name}'

    fname = 'jacobian_' + origin_name + '_to_' + dest_name + "_" + content

    if optimization_level == 0:
        for i_v, var in enumerate(varss):
            for i_p, param in enumerate(params):
                qp = language.\
                    slice_mat("q", robot.dof.index(Symbol(param['name'])),
                              None, None, None)
                varss[i_v]['value'] = replace_var(var['value'],
                                                  param['name'], qp)
        code += language.generate_fct("mat", fname, parameters, expr, varss,
                                      docstr,
                                      matrix_dims=(1, 1))
    else:
        code += generate_code_from_sym_mat(jac, fname, language, docstr,
                                           input_is_vector=True,
                                           dof=robot.dof)

    return code


# Generate all Jacobian Matrices _____________________________________________

def generate_all_jac(robot, list_origin, list_dest, list_content,
                     optimization_level,
                     language=Language('python'),
                     progressbar=None,
                     progress_increment=0):
    """
    Description
    -----------
    
    Generate all Jacobian functions
    
    Parameters
    ----------
    
    robot : robots.Robot
        Robot you want to generate the matrices from
    
    list_origin : list of str
        List of all origins elements.
        Every element is a string formatted like the nodes
        names of self.tree :
            type_number
            
            where type is 'joint' / 'link' and number is the number of the
            considered Joint / Link in self.joints / self.links.
            
            Example : origin='joint_2'
    list_dest : list of str
        Every element is a string formatted like the
        node names of self.tree :
            type_number
            
            where type is 'joint' / 'link' and number is the number of the
            considered Joint / Link in self.joints / self.links.
            
            Example : destination='link_0'

    list_content : list of str
        Every element is the content of the jacobian ("xyzrpY", ...)

    optimization_level : int
        - 0 => Jacobian is computed numerically (fastest to generate, slowest
        code
        - 1 => Jacobian is computed analytically but is not simplified
        - 2 => Jacobian is computed analytically and is factored
        - 3 => Jacobian is computed analytically and is simplified
    
    language : Language.Language, optional
        Language you want the code to be generated to
        Default is Language('python')

    progressbar : PyQt5.QtWidgets.QProgressBar or None, optional
        default is None
        Progressbar to update during the robot creation (used in GUI)
        If it is None, no progressbar is updated

    progress_increment : float
        Progressbar  increment.  Default  is  0.  If progressbar is None, this
        parameter is ignored.
        
    Returns
    -------
    
    str :
        String  containing the jacobian functions in the language you want
    
    """

    if len(list_origin) == 0:
        return ''

    # Adding Title
    code = language.title("JACOBIANS", 0)
    code += '\n\n'

    for i, origin, in enumerate(list_origin):

        print(f"Generating Jacobian {i+1}/{len(list_origin)}")
        code += generate_jacobian(robot, origin, list_dest[i],
                                  list_content[i],
                                  optimization_level=optimization_level,
                                  language=language)
        if i < len(list_origin) - 1:
            code += '\n\n'

        increment_progressbar(progressbar, progress_increment)

    return code


# Generate Center of Mass Position ___________________________________________

def generate_com(robot, optimization_level, language=Language('python'),
                 progressbar=None, progress_increment=0):
    """
    Generate the code for the center of mass of the robot

    Parameters
    ----------
    robot : robots.Robot
        Robot you want to generate the center of mass function from

    optimization_level : int
        - 0 => CoM is computed numerically (fastest to generate, slowest code
        - 1 => CoM is computed analytically but is not simplified
        - 2 => CoM is computed analytically and is factored
        - 3 => CoM is computed analytically and is simplified
    
    language : Language.Language, optional
        Language you want the code to be generated to
        Default is Language('python')

    progressbar : PyQt5.QtWidgets.QProgressBar or None, optional
        default is None
        Progressbar to update during the robot creation (used in GUI)
        If it is None, no progressbar is updated

    progress_increment : float
        Progressbar  increment.  Default  is  0.  If progressbar is None, this
        parameter is ignored.


    Returns
    -------
    str : 
        Code of the CoM function in the desired language

    """

    print(f"Generating Center of Mass")

    # Adding Title
    code = language.title("Center of Mass of the Robot", 0)
    code += '\n\n'

    # Total Mass of the robot ................................................

    mass = 0
    for link in robot.links:
        mass += link.mass

    if mass == 0:
        return language.comment_line + ' Center of mass function can not be' + \
               ' generated because the robot mass is null.'

    params = []
    varss = []
    if optimization_level == 0:

        # Tree iteration .....................................................

        # Initial transformation
        var = {'name': 'T',
               'value': '___eye__4__4___',
               'type': 'mat'}

        varss.append(var)

        saved_joints_T = []
        params = []
        expr = ''

        last_u = 0

        for node in PreOrderIter(robot.tree.node):
            obj_type = node.name.split('_')[0]
            obj_nb = int(node.name.split('_')[1])

            if obj_type == 'link':
                relative_mass = robot.links[obj_nb].mass / mass
                # Ignoring null mass links
                if relative_mass == 0:
                    continue
                cm = robot.links[obj_nb].com
                pos_val = f'#mat#4#1#{str(cm[0, 0])}#{str(cm[1, 0])}#' + \
                          f'{str(cm[2, 0])}#1.0#endmat&'
                pos_var = {'name': f'com_{obj_nb}_xyz',
                           'value': pos_val,
                           'type': 'vect'}
                varss.append(pos_var)
                var = {'name': f'com_{obj_nb}',
                       'value': f'{relative_mass}*T@com_{obj_nb}_xyz',
                       'type': 'vect'}
                varss.append(var)
                expr += f'+com_{obj_nb}'

                last_u = len(varss)

            elif obj_type == 'joint':
                joint = robot.joints[obj_nb]

                # Parameters
                all_sym = joint.T.free_symbols
                params_tmp = get_parameters(all_sym)
                params += get_parameters(all_sym)
                T_fct = 'MATLAB_PREFIXT_' + joint.name + '('

                params_tmp.sort(key=lambda x: x['name'])

                for i_p, par in enumerate(params_tmp):
                    T_fct += par['name']
                    if i_p < len(params_tmp) - 1:
                        T_fct += ','
                T_fct += ')'

                if len(robot.links[joint.child].parent_joints) > 1:
                    var2 = {'name': f'MATLAB_PREFIXT_{obj_nb}',
                            'value': f'T@{T_fct}',
                            'type': 'mat'}
                    varss.append(var2)
                    saved_joints_T.append(obj_nb)
                    var = {'name': 'T',
                           'value': f'MATLAB_PREFIXT_{obj_nb}',
                           'type': ''}
                    varss.append(var)

                elif any(x in robot.links[joint.parent].child_joints for x in \
                         saved_joints_T):
                    num = robot.links[joint.parent].child_joints[0]
                    var = {'name': 'T',
                           'value': f'MATLAB_PREFIXT_{num}@{T_fct}',
                           'type': ''}
                    varss.append(var)
                else:
                    var = {'name': 'T',
                           'value': f'T@{T_fct}',
                           'type': ''}
                    varss.append(var)
        # Removing useless variables
        to_remove = []
        last_T = None
        T_nb = 0
        for i_v, var in enumerate(varss):
            if var['name'] == 'T':
                if last_T == i_v - 1 and var['value'][:2] != 'T@':
                    to_remove.append(last_T)
                    if T_nb == 2:
                        varss[i_v]['value'] = varss[i_v]['value'][2:]
                        varss[i_v]['type'] = 'mat'
                last_T = i_v

        varss = varss[:last_u]
        to_remove.sort(reverse=True)
        for rem in to_remove:
            varss.pop(rem)
    else:
        com = robot.com(optimization_level)

        all_sym = robot.dof

        params += get_parameters(all_sym)

    descrq = (f'Vector of length {len(params)} containing all the degrees of '
              'freedom of the robot that have an effect on the center of mass'
              ' position. This vector contains:')
    for i_p, param in enumerate(params):
        qp = language.slice_mat("q", robot.dof.index(Symbol(param['name'])),
                                None, None, None)
        descrq += f'\n        - {qp} = ' + \
                  param['name']
        descrq += f' :\n              ' + param['description']

    paramq = {'name': 'q', 'type': 'vect', 'description': descrq}

    for i_v, var in enumerate(varss):
        for i_p, param in enumerate(params):
            if not varss[i_v]['value'][0] == '#':
                varss[i_v]['value'] = \
                    replace_var(var['value'], param['name'],
                                language.slice_mat("q", i_p, None, None,
                                                   None))

    if optimization_level == 0:
        dimensions = (f'(4 x 1) {language.matrix_type} in homogeneous '
                      'coordinates. The first three coordinates represent the'
                      ' X, Y and Z positions of the CoM and the 4th coordinat'
                      'e is always equal to 1.')
    else:
        dimensions = f'(3 x 1) {language.matrix_type} (X, Y and Z).'
    docstr = 'Returns the center of mass of the robot in the root link frame.' + \
             ' The center of mass of the whole structure is computed. The ' \
             'result' + \
             f' is returned as a {dimensions}'

    if optimization_level == 0:
        code += language.generate_fct("mat", 'com', [paramq], expr[1:], varss,
                                      docstr, matrix_dims=(1, 1))
    else:
        code += generate_code_from_sym_mat(com, 'com', language, docstr,
                                           input_is_vector=True,
                                           dof=robot.dof)

    increment_progressbar(progressbar, progress_increment)
    return code


# Generate Center of Mass Jacobian ___________________________________________

def generate_com_jacobian(robot, optimization_level,
                          language=Language('python'),
                          progressbar=None,
                          progress_increment=0,
                          content="xyz"):
    """
    Generate the code for the jacobian of the center of mass of the robot

    Parameters
    ----------
    robot : robots.Robot
        Robot you want to generate the center of mass jacobian function from

    optimization_level : int
        - 0 => CoM  Jacobian  is  computed  numerically  (fastest to generate,
        slowest code
        - 1 => CoM Jacobian is computed analytically but is not simplified
        - 2 => CoM Jacobian is computed analytically and is factored
        - 3 => CoM Jacobian is computed analytically and is simplified
    
    language : Language.Language, optional
        Language you want the code to be generated to
        Default is Language('python')

    progressbar : PyQt5.QtWidgets.QProgressBar or None, optional
        default is None
        Progressbar to update during the robot creation (used in GUI)
        If it is None, no progressbar is updated

    progress_increment : float
        Progressbar  increment.  Default  is  0.  If progressbar is None, this
        parameter is ignored.

    content : str
        Content of the center of mass of the robot ("xyz", "xy")


    Returns
    -------
    str : 
        Code of the CoM Jacobian function in the desired language

    """

    print(f"Generating Center of Mass Jacobian")

    # Adding Title
    code = language.title("Jacobian of the Center of Mass of the Robot", 0)
    code += '\n\n'

    params = []
    varss = []
    if optimization_level == 0:
        # Total Mass of the robot ............................................

        mass = 0
        nb_dof = 0
        for link in robot.links:
            if link.mass != 0:
                mass += link.mass
                nb_dof += 1

        if mass == 0:
            return language.comment_line + ' Center of mass jacobian function ' + \
                   'can not be generated because the robot mass is null.'

        # Tree iteration .....................................................

        varss = []

        # Init Jacobian
        varss.append({'name': 'Jac',
                      'value': f"___zeros__3__{nb_dof}___",
                      'type': 'mat'})

        # Initial transformation
        var = {'name': 'T',
               'value': '___eye__4__4___',
               'type': 'mat'}

        varss.append(var)

        saved_joints_T = []
        params = []
        expr = 'Jac'
        l_declared = False
        z_declared = False
        i_jac = 0
        last_u = 0

        for node in PreOrderIter(robot.tree.node):
            obj_type = node.name.split('_')[0]
            obj_nb = int(node.name.split('_')[1])

            if obj_type == 'link':
                relative_mass = robot.links[obj_nb].mass / mass
                # Ignoring null mass links
                if relative_mass == 0:
                    continue
                cm = robot.links[obj_nb].com
                pos_val = f'#mat#4#1#{str(cm[0, 0])}#{str(cm[1, 0])}#' + \
                          f'{str(cm[2, 0])}#1.0#endmat&'
                pos_var = {'name': f'com_{obj_nb}_xyz',
                           'value': pos_val,
                           'type': 'vect'}
                varss.append(pos_var)
                var = {'name': 'com',
                       'value': f'{relative_mass}*T@com_{obj_nb}_xyz',
                       'type': 'vect'}
                varss.append(var)
                l_type = '' if l_declared else 'vect'
                com_val = (f"{language.slice_mat('com0', 0, 2, None, None)}"
                           f"-{language.slice_mat('com', 0, 2, None, None)}")
                var_l = {'name': 'L',
                         'value': com_val,
                         'type': l_type}
                l_declared = True
                varss.append(var_l)
                z_type = '' if z_declared else 'vect'
                var_z = {'name': 'Z',
                         'value': language.slice_mat("T", 0, 2, 2, None),
                         'type': z_type}
                z_declared = True
                varss.append(var_z)
                var_j = {'name':language.slice_mat("Jac", 0, 2, i_jac, None),
                         'value': 'cross(Z,L)',
                         'type': ''}
                varss.append(var_j)
                last_u = len(varss)
                i_jac += 1

            elif obj_type == 'joint':
                joint = robot.joints[obj_nb]

                # Parameters
                all_sym = joint.T.free_symbols
                params_tmp = get_parameters(all_sym)
                params += get_parameters(all_sym)
                T_fct = 'MATLAB_PREFIXT_' + joint.name + '('

                params_tmp.sort(key=lambda x: x['name'])

                for i_p, par in enumerate(params_tmp):
                    T_fct += par['name']
                    if i_p < len(params_tmp) - 1:
                        T_fct += ','
                    else:
                        T_fct += ')'

                if len(robot.links[joint.child].parent_joints) > 1:
                    var2 = {'name': f'MATLAB_PREFIXT_{obj_nb}',
                            'value': f'T@{T_fct}',
                            'type': 'mat'}
                    varss.append(var2)
                    saved_joints_T.append(obj_nb)
                    var = {'name': 'T',
                           'value': f'MATLAB_PREFIXT_{obj_nb}',
                           'type': ''}
                    varss.append(var)

                elif any(x in robot.links[joint.parent].child_joints for x in \
                         saved_joints_T):
                    num = robot.links[joint.parent].child_joints[0]
                    var = {'name': 'T',
                           'value': f'MATLAB_PREFIXT_{num}@{T_fct}',
                           'type': ''}
                    varss.append(var)
                else:
                    var = {'name': 'T',
                           'value': f'T@{T_fct}',
                           'type': ''}
                    varss.append(var)

        # Removing useless variables
        to_remove = []
        last_T = None
        T_nb = 0
        for i_v, var in enumerate(varss):
            if var['name'] == 'T':
                T_nb += 1
                if last_T == i_v - 1 and var['value'][:2] != 'T@':
                    to_remove.append(last_T)
                    if T_nb == 2:
                        varss[i_v]['value'] = varss[i_v]['value'][2:]
                        varss[i_v]['type'] = 'mat'
                last_T = i_v

        varss = varss[:last_u]
        to_remove.sort(reverse=True)
        for rem in to_remove:
            varss.pop(rem)
    else:
        jac = robot.com_jacobian(optimization_level)
        all_sym = robot.dof
        params += get_parameters(all_sym)
        nb_dof = len(all_sym)

    descrq = 'Vector of all the degrees of freedom of the robot that have ' \
             'an effect on the center of mass position. This vector ' \
             'contains : '
    for i_p, param in enumerate(params):
        qp = language.slice_mat("q", robot.dof.index(Symbol(param['name'])),
                                None, None, None)
        descrq += f'\n        - {qp} = ' + param['name']
        descrq += f' :\n              ' + param['description']

    paramq = {'name': 'q', 'type': 'vect', 'description': descrq}

    for i_v, var in enumerate(varss):
        for i_p, param in enumerate(params):
            varss[i_v]['value'] = replace_var(var['value'], param['name'],
                                              language.slice_mat("q", i_p,
                                                                 None, None,
                                                                 None))
    paar = [paramq]
    if optimization_level == 0:
        param_com0 = {'name': 'com0',
                      'type': 'vect',
                      'description': 'Point from which you want to compute the'
                                     'Jacobian of the center of Mass. This '
                                     'point is '
                                     'expressed in homogeneous coordinates as a '
                                     '(4 x 1)'
                                     f' {language.vector_type}. '
                                     'The first three coordinates represent the '
                                     'X, Y and Z '
                                     'positions of the CoM and the 4th '
                                     'coordinate must always be equal'
                                     ' to 1'}
        paar.append(param_com0)

    origin_name = 'world'
    dest_name = 'the center of mass'

    docstr = (f'Computes the Jacobian Matrix of the center of mass of the '
              f'robobt. This matrix is '
              f'returned as a ({len(content)} x {len(robot.dof)}) '
              f'matrix where every column '
              f'is the derivative of the position/orientation with respect to'
              f' a degree of freedom. \n')
    for i_cc, cc in enumerate(content):
        docstr += f'    - The line {i_cc} is the derivative of '
        if cc in "xyz":
            docstr += f"{cc.upper()} position "
        else:
            d = {"r": "roll", "p": "pitch", "Y": "yaw"}
            docstr += f"the {d[cc]} orientation "
        docstr += f"of {dest_name} in the {origin_name} frame,\n"
    docstr += 'Here is the list of all the derivative variables :'
    for i_p, param in enumerate(robot.dof):
        docstr += f'\n    - Column {language.indexing_0 + i_p} : ' + \
                  f'{param.name}'

    if optimization_level == 0:
        code += language.generate_fct("mat", 'jacobian_com', paar, expr,
                                      varss,
                                      docstr, matrix_dims=(1, 1))
    else:
        code += generate_code_from_sym_mat(jac, 'jacobian_com', language,
                                           docstr, input_is_vector=True,
                                           dof=robot.dof)

    increment_progressbar(progressbar, progress_increment)
    return code


# Generate Polynomial trajectory _____________________________________________

def generate_polynomial_trajectory(conditions, function_name, language):
    """
    Description
    -----------

    Generates all the functions of the polynomial trajectory following the
    conditions you wish. All the derivatives are generated.

    Parameters
    ----------
    conditions : list of lists containing 3 elements
        [..., [derivative_order, time_value, equals], ...]

        derivative_order : str
            String containing a positive integer.
            Order  of  the derivative. If the time is your derivative variable
            and  the  function  you  want to create describes your position, 0
            corresponds to the position, 1 to the speed, 2 to the acceleration
            3 to the jerk and so on.
        time_value : str
            Time value on which you want your condition to be set. This can be
            a Float string ("1", "25.04") or a literal expression representing
            a variable ("t", "t0", "tf", ...)
        equals : str
            Value  of  the  function  for  the  given time. This can bea Float
            string ("1", "25.04") or a literal expression representing
            a variable ("t", "t0", "tf", ...)

    function_name : str
        Name of the function of the trajectory

    language : Language.Language
        Language of the generated code

    Returns
    -------

    code : str
        Code of the functions of all the polynomial derivatives.

        The generated code will be optimized automatically.

        The  name  of  the functions are d<k>_<function_name> where <k> is the
        derivative  order  and  <function_name>  is the function name given as
        parameter.

        For example, if your function name is "poly_1", the generated function
        for the 2nd derivative (acceleration) will be called "d2_poly_1()".
        The 0th derivative (position) will give "d0_poly_1()".

    parameters : dict of list of dict
        Parameters of the trajectory (for position)

    """

    conditions_ = [condition.copy() for condition in conditions]

    # If conditions are empty ................................................

    if not conditions_:
        return language.comment_line + " No conditions have been given for " \
                                       "the trajectory \"" + \
               function_name + \
               "\", so nothing has been generated."

    # Conditions -> String for documentation .................................

    conditions_.sort(key=lambda x: int(x[0]))

    conditions_str = "The followed conditions are :"
    for condition in conditions_:
        conditions_str += "\n    - " + ("d" + ("^" + condition[0] if
                                               int(condition[0]) > 1
                                               else "") + '_' if
                                        int(condition[0]) > 0 else "") + \
                          function_name + ("/dt" + ("^" + condition[0] if
                                                    int(condition[0]) > 1
                                                    else "") if
                                           int(condition[0]) > 0 else "") + \
                          "(" + condition[1] + ") = " + condition[2]

    # Conditions -> Symbols ..................................................

    def str_to_sym_or_float(string):
        """
        Description
        -----------

        Convert the expression in a string to a Sympy variable or a float.

        Parameters
        ----------
        string : string containing the value.
            This must NOT contain a mathematical expression.
            Allowed values are for example "2", "10.5", "k0", "t" ...

        Returns
        -------

        sym : sympy.core.symbol.Symbol or float
            If  the  string  represents  a  number, the function will return a
            float.  Else,  it  will  return a Sympy symbol associated with the
            string.

        """

        try:
            return float(string)
        # The string is not a float value
        except ValueError:
            return Symbol(string, real=True)

    for condition in conditions_:
        condition[0] = int(condition[0])
        condition[1] = str_to_sym_or_float(condition[1])  # time_value
        condition[2] = str_to_sym_or_float(condition[2])  # equals

    # Conditions -> Polynomial ...............................................

    sym_vars, derivatives, equations = get_equations(conditions_,
                                                     Symbol('t'))
    solution = get_solution(equations, sym_vars, derivatives)

    # Verifying solution
    if not verify_solution(conditions_, solution, Symbol('t')):
        raise ArithmeticError('The conditions you gave for trajectory '
                              "generation can not be followed.")

    # Polynomials -> Code ....................................................

    code = language.title("Trajectory " + function_name, 1) + '\n\n'
    parameters_0 = []
    for i, polynomial in enumerate(solution):
        fname = "d" + str(i) + "_" + function_name
        docstring = "Trajectory polynomial. This function returns the " + \
                    ("position" if i == 0 else ("speed" if i == 1 else (
                        "acceleration" if i == 2 else ("jerk" if i == 3 else
                                                       str(i) + "th " +
                                                       "derivative of the " +
                                                       "position")
                    ))) + " as a function of time. The polynomial was " \
                          "created following some " + \
                    "conditions. " + conditions_str + "\n\nBE CAREFUL : " + \
                    "This function does not take into account the " + \
                    "physical limits of the robot (maximum velocities, " + \
                    "accelerations, positions ...)."

        parameters = []
        for param in polynomial.free_symbols:
            parameters.append(dict(name=str(param),
                                   type="double",
                                   description="Time variable" if
                                   param == Symbol('t') else ""))
        if i == 0:
            parameters_0 = parameters

        varss, expr = optimize(str(polynomial))

        code += language.generate_fct("mat", fname, parameters, expr, varss,
                                      docstring, (1, 1), False) + '\n\n'

    return code, parameters_0


# Generate all the polynomial trajectories ___________________________________

def generate_all_polynomial_trajectories(trajectories, language,
                                         progressbar=None,
                                         progress_increment=0):
    """
    Description
    -----------

    Generates  the  code for all the polynomial trajectories. For more details
    on how it works, check generate_polynomial_trajectory() (in this file).

    Parameters
    ----------

    trajectories : list of dict
        list of all the polynomial trajectories to generate.

        Every item of this list must be a dict with the following structure :

        {"name": str : Name of the trajectory,

         "conditions" : list of list of 3 str :

            [..., [k, t, x], ...]

            k : str representing an integer
                Order  of  the  derivative.  If  the  time  is your derivative
                variable  and  the function you  want to create describes your
                position,  0 corresponds to the position, 1 to the speed, 2 to
                the acceleration, 3 to the jerk and so on.
            t : str representing a float or a symbol
                Time value on which you want your condition to be set
            x : str representing a float or a symbol
                Value  of the  function  for  the  given  time.  This can be a
                symbolic variable
        }

    language : Language.Language
        Language of the generated code

    progressbar : PyQt5.QtWidgets.QProgressBar or None, optional
        default is None
        Progressbar to update during the robot creation (used in GUI)
        If it is None, no progressbar is updated

    progress_increment : float
        Progressbar  increment.  Default  is  0.  If progressbar is None, this
        parameter is ignored.


    Returns
    -------

    code : str
        Code containing all the functions for each trajectory.

        The generated code will be optimized automatically.

        The  name  of  the functions are d<k>_<function_name> where <k> is the
        derivative  order  and  <function_name>  is the function name given as
        parameter.

        For example, if your function name is "poly_1", the generated function
        for the 2nd derivative (acceleration) will be called "d2_poly_1()".
        The 0th derivative (position) will give "d0_poly_1()".

    poly_parameters : dict of list of dict
        Parameters of the polynomial trajectories

    """

    code = "\n\n" + language.title("Polynomial Trajectories", 0) + "\n\n"

    poly_parameters = {}
    for trajectory in trajectories:
        code_, par = generate_polynomial_trajectory(trajectory["conditions"],
                                                    trajectory["name"],
                                                    language)
        code += code_
        poly_parameters[trajectory["name"]] = par
        increment_progressbar(progressbar, progress_increment)

    return code, poly_parameters


# Control loops ______________________________________________________________

def generate_control_loop(loop, robot, traj_parameters, language):
    """
    Generate the code for control loop in the desired language

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

    robot : robots.Robot
        Robot for which you want to create the control loop

    traj_parameters : dict of list of dict
        All  the  parameters  of  the  polynomial  trajectory  that  have been
        generated before. The keys of the dict are the trajectories names.
        Values are the list of parameters.

    language : Language.Language
        Language of the generated code

    Returns
    -------

    code : str
        Generated code
    """

    # Function name, docstring ...............................................

    fname = f"kinematic_control_loop_{loop['type']}_{loop['ids'][2]}"
    docstr = docstr_control_loop(loop)

    # defining Parameters ....................................................

    param_q0 = {"name": "q0",
                "type": "mat",
                "description": ("Initial configuration of the robot. " +
                                robot_dof_descr(robot, "q0", language))}
    param_Td = {"name": "Td__I__",
                "type": "mat",
                "description": "Desired configuration to reach__IT__. Must "
                               "be given as a (4 x 4) matrix in homogeneous "
                               "coordinates."}
    param_Xd = {"name": "Xd__I__",
                "type": "mat",
                "description": "Desired position to reach__IT__."}
    param_Od = {"name": "Ad__I__",
                "type": "mat",
                "description": "Desired orientation to reach__IT__. Must "
                               "be given as a (3 x 3) rotation matrix."}
    param_comd = {"name": "comd__I__",
                  "type": "mat",
                  "description": "Desired position of the center of mass."}
    param_kp = {"name": "kp__I__",
                "type": "double",
                "description": "Proportional controller for __POSITION__"
                               "__IT__. "
                               "For more details, check out the block scheme "
                               "of this control loop."}

    param_ko = {"name": "ko__I__",
                "type": "double",
                "description": "Proportional controller for __ORIENTATION__"
                               "__IT__. "
                               "For more details, check out the block scheme "
                               "of this control loop."}

    param_err_max = {"name": "err_max",
                     "type": "double",
                     "description": "Maximum euclidean norm of the error. If "
                                    "the measured error is less than this "
                                    "value, the task is considered as "
                                    "done (the loop breaks)."}

    param_beta = {"name": "beta",
                  "type": "double",
                  "description": "Regularizer for constraints on joint "
                                 "positions. For more details, check out the "
                                 "block scheme of this control loop."}

    additional_parameters = []
    if loop["trajectory"] != "(None)":
        for param in traj_parameters[loop["trajectory"]]:
            if param["name"] != "t":
                additional_parameters.append(param)

    # Find fk and jacobians ..................................................

    # Task 1 only
    if loop["type_2"] is None:
        suffix = [""]
        it = [""]
        ids = [loop["ids"]]
        types = [loop["type"]]
        n = 1

    # Tasks 1 et 2
    else:
        suffix = ["_1", "_2"]
        it = [" for task 1", " for task 2"]
        ids = [loop["ids"], loop["ids_2"]]
        types = [loop["type"], loop["type_2"]]
        n = 2

    jacobians = []
    fks = []

    for i in range(n):
        pos, ori = split_content(ids[i][2])
        if types[i] == "effector":
            if ori != "":
                ct = f"{pos}o"
            else:
                ct = ids[i][2]
            fks.append(f"MATLAB_PREFIXfk_{ids[i][0]}_{ids[i][1]}_{ct}(q)")
            jacobians.append(f"MATLAB_PREFIXjacobian_{ids[i][0]}_to_"
                             f"{ids[i][1]}_{ids[i][2]}(q)")
        else:
            fks.append(f"MATLAB_PREFIXcom_{pos}(q)")
            jacobians.append(f"MATLAB_PREFIXjacobian_com_{pos}(q)")

    # Selecting parameters ...................................................

    params = []

    fk_vars = []
    fkd_vars = []
    X_vars = []
    Xd_vars = []
    O_vars = []
    Od_vars = []
    X0_vars = []
    O0_vars = []
    Ak_vars = [[[], [], []], [[], [], []]]
    Ak_d_traj_vars = [[[], [], []], [[], [], []]]
    Ak_d_vars = [[[], [], []], [[], [], []]]

    for i in range(n):
        pos, ori = split_content(ids[i][2])

        # Position and orientation
        if pos != "" and ori != "":
            params.append(param_Td.copy())
            params.append(param_kp.copy())
            params.append(param_ko.copy())
            fk_vars.append(f"T{suffix[i]}")
            fkd_vars.append(f"Td{suffix[i]}")
            X_vars.append(language.slice_mat(fk_vars[i], 0, 2, 3, None))
            Xd_vars.append(language.slice_mat(fkd_vars[i], 0, 2, 3, None))
            O_vars.append(language.slice_mat(fk_vars[i], 0, 2, 0, 2))
            Od_vars.append(language.slice_mat(fkd_vars[i], 0, 2, 0, 2))
            X0_vars.append(language.slice_mat(f"T0{suffix[i]}", 0, 2, 3,
                                              None))
            O0_vars.append(language.slice_mat(f"T0{suffix[i]}", 0, 2, 0, 2))
            for k in range(3):
                for l in range(3):
                    Ak_vars[i][k].append(language.slice_mat(fk_vars[i], k,
                                                            None, l, None))
                    Ak_d_vars[i][k].append(language.slice_mat(fkd_vars[i], k,
                                                           None, l, None))
                    Ak_d_traj_vars[i][k].append(language
                                             .slice_mat(f"Ad_tmp{suffix[i]}",
                                                        k, None, l, None))

        # Only Position
        elif pos != "":
            if types[i] == "effector":
                params.append(param_Xd.copy())
                fk_vars.append(f"X{suffix[i]}")
                fkd_vars.append(f"Xd{suffix[i]}")
            else:
                params.append(param_comd.copy())
                fk_vars.append(f"com{suffix[i]}")
                fkd_vars.append(f"comd{suffix[i]}")
            params[-1]["description"] += \
                descr_from_content(pos, params[-1]["name"], language)
            params.append(param_kp.copy())
            X_vars.append(fk_vars[i])
            Xd_vars.append(fkd_vars[i])
            X0_vars.append(fks[i])
            O_vars.append(None)
            Od_vars.append(None)
            O0_vars.append(None)
            for k in range(3):
                for _ in range(3):
                    Ak_vars[i][k].append(None)
                    Ak_d_vars[i][k].append(None)
                    Ak_d_traj_vars[i][k].append(None)

        # Only orientation
        else:
            params.append(param_Od.copy())
            params.append(param_ko.copy())
            fk_vars.append(f"A{suffix[i]}")
            fkd_vars.append(f"Ad{suffix[i]}")
            X_vars.append(None)
            Xd_vars.append(None)
            X0_vars.append(None)
            O_vars.append(fk_vars[i])
            Od_vars.append(fkd_vars[i])
            O0_vars.append(fks[i])
            for k in range(3):
                for l in range(3):
                    Ak_vars[i][k].append(language.slice_mat(fk_vars[i], k,
                                                            None, l, None))
                    Ak_d_vars[i][k].append(language.slice_mat(fkd_vars[i], k,
                                                           None, l, None))
                    Ak_d_traj_vars[i][k].append(language
                                             .slice_mat(f"Ad_tmp{suffix[i]}",
                                                        k, None, l, None))

        for param in params:
            param["name"] = param["name"].replace("__I__", suffix[i])
            param["description"] = param["description"] \
                .replace("__IT__", it[i])\
                .replace("__I__", suffix[i]) \
                .replace("__POSITION__", f"the position of the {types[i]}") \
                .replace("__ORIENTATION__",
                         f"the orientation of the {types[i]}")

    params.append(param_err_max)
    if loop["constraints"]:
        params.append(param_beta)
    if loop["control_type"] == "geometric":
        params.append(param_q0)
    params += additional_parameters

    # Variables and function body ............................................

    varss = []

    # Time variable
    var_t = {"name": "t",
             "type": "double",
             "value": "0"}
    varss.append(var_t)

    # dt variable
    var_dt = {"name": "dt",
              "type": "double",
              "value": "0.01"}
    if loop["constraints"]:
        varss.append(var_dt)

    # Initial q
    var_q = {"name": "q",
             "type": "mat",
             "value": "q0" if loop["control_type"] == "geometric" else
                      "get_positions()"}
    varss.append(var_q)

    # Initial configurations
    if loop["trajectory"] != "(None)":
        for i in range(n):
            pos, ori = split_content(ids[i][2])
            if pos != "" and ori != "":
                var_T0 = {"name": f"T0{suffix[i]}",
                          "type":"mat",
                          "value": fks[i]}
                varss.append(var_T0)

            if X0_vars[i] is not None:
                var_X0 = {"name": f"X0{suffix[i]}",
                          "type": "mat",
                          "value": X0_vars[i]}
                varss.append(var_X0)
            if O0_vars[i] is not None:
                var_O0 = {"name": f"A0{suffix[i]}",
                          "type": "mat",
                          "value": O0_vars[i]}
                varss.append(var_O0)
                A = f"A0_err{suffix[i]}"
                var_A = {"name": A,
                         "type": "mat",
                         "value": f"{Od_vars[i]} @ transpose({O0_vars[i]})"}
                varss.append(var_A)

                a11 = language.slice_mat(A, 0, None, 0, None)
                a22 = language.slice_mat(A, 1, None, 1, None)
                a33 = language.slice_mat(A, 2, None, 2, None)
                var_theta_f = {"name": f"theta_f{suffix[i]}",
                               "type": "double",
                               "value": f"acos(0.5*({a11}+{a22}+{a33}-1))"}
                varss.append(var_theta_f)

                var_u0 = {"name": f"u{suffix[i]}",
                          "type": "mat",
                          "value": "#mat#3#1#0#0#0#endmat&"}
                varss.append(var_u0)

                var_if_u0 = {"name": "__IF__",
                             "type": "",
                             "value": f"abs(theta_f{suffix[i]}) > 1e-10"}
                varss.append(var_if_u0)

                er0 = (f"{language.slice_mat(A, 2, None, 1, None)}-"
                       f"{language.slice_mat(A, 1, None, 2, None)}")
                er1 = (f"{language.slice_mat(A, 0, None, 2, None)}-"
                       f"{language.slice_mat(A, 2, None, 1, None)}")
                er2 = (f"{language.slice_mat(A, 1, None, 0, None)}-"
                       f"{language.slice_mat(A, 0, None, 1, None)}")
                var_u = {"name": f"u{suffix[i]}",
                         "type": "mat",
                         "value": f"0.5/sin(theta_f{suffix[i]})*"
                                  f"#mat#3#1#{er0}#{er1}#{er2}#endmat&"}
                varss.append(var_u)

                var_end_if = {"name": "__ENDLOOP__",
                              "type": "",
                              "value": ""}
                varss.append(var_end_if)

    # Start counting time
    var_time_start = {"name": "__TIMESTART__",
                      "type": "",
                      "value": ""}
    varss.append(var_time_start)

    # While loop variable
    var_while = {"name": "__WHILE__",
                 "type": "",
                 "value": "1"}
    varss.append(var_while)

    # Compute FKs
    for i in range(n):
        var_fk = {"name": fk_vars[i],
                  "type": "mat",
                  "value": fks[i]}
        varss.append(var_fk)

    # Compute errors
    err_vars = ""
    for i in range(n):
        pos, ori = split_content(ids[i][2])
        X_val = X_vars[i]
        Xd_val = Xd_vars[i]
        O_val = O_vars[i]
        Od_val = Od_vars[i]
        if pos != "":
            var_err_pos = {"name": f"err_pos{suffix[i]}",
                           "type": "mat",
                           "value": f"norm({Xd_val} - {X_val})"}
            varss.append(var_err_pos)
            err_vars += "+" + var_err_pos["name"]
        if ori != "":
            var_A = {"name": f"E{suffix[i]}",
                     "type": "mat",
                     "value": f"{Od_val} @ transpose({O_val})"}
            varss.append(var_A)
            er0 = (f"{language.slice_mat(f'E{suffix[i]}', 2, None, 1, None)}-"
                   f"{language.slice_mat(f'E{suffix[i]}', 1, None, 2, None)}")
            er1 = (f"{language.slice_mat(f'E{suffix[i]}', 0, None, 2, None)}-"
                   f"{language.slice_mat(f'E{suffix[i]}', 2, None, 1, None)}")
            er2 = (f"{language.slice_mat(f'E{suffix[i]}', 1, None, 0, None)}-"
                   f"{language.slice_mat(f'E{suffix[i]}', 0, None, 1, None)}")
            var_err_o = {"name": f"err_o{suffix[i]}",
                         "type": "mat",
                         "value": f"0.5*#mat#3#1#{er0}#{er1}#{er2}#endmat&"}
            varss.append(var_err_o)
            err_vars += f"+norm({var_err_o['name']})"

    # If err_max statement
    var_if = {"name": "__IF__",
              "type": "",
              "value": f"{err_vars[1:]} < err_max"}
    varss.append(var_if)

    # Break statement
    var_break = {"name": "__BREAK__",
                 "type": "",
                 "value": ""}
    varss.append(var_break)

    # End if statement
    var_endif = {"name": "__ENDLOOP__",
                 "type": "",
                 "value": ""}
    varss.append(var_endif)

    # Trajectory generation
    if loop["trajectory"] != "(None)":
        for i in range(n):
            pos, ori = split_content(ids[i][2])
            param_trj = ""
            for i_p, prm in enumerate(traj_parameters[loop['trajectory']]):
                param_trj += prm["name"]
                if i_p < len(traj_parameters[loop['trajectory']]) - 1:
                    param_trj += ', '
            r_t = f"MATLAB_PREFIXd0_{loop['trajectory']}({param_trj})"
            r_dot_t = f"MATLAB_PREFIXd1_{loop['trajectory']}({param_trj})"
            if pos != '':
                var_Xd_tmp = {"name": f"Xd_tmp{suffix[i]}",
                              "type": "mat",
                              "value": f"X0{suffix[i]}+({Xd_vars[i]}-"
                                       f"X0{suffix[i]})*{r_t}"}
                varss.append(var_Xd_tmp)

                var_Xdd_tmp = {"name": f"Xdot_d_tmp{suffix[i]}",
                               "type": "mat",
                               "value": f"({Xd_vars[i]}-X0{suffix[i]})*"
                                        f"{r_dot_t}"}
                varss.append(var_Xdd_tmp)

                var_X_dot = {"name": f"Xdot{suffix[i]}",
                             "type": "mat",
                             "value": f"(Xd_tmp{suffix[i]}-{X_vars[i]})"
                                      f"*kp{suffix[i]}+Xdot_d_tmp{suffix[i]}"}
                varss.append(var_X_dot)
            if ori != '':
                var_theta_t = {"name": f"theta_t{suffix[i]}",
                               "type": "double",
                               "value": f"theta_f{suffix[i]}*{r_t}"}
                varss.append(var_theta_t)

                S12 = "-" + language.slice_mat(f"u{suffix[i]}", 2, None, None,
                                               None)
                S13 = language.slice_mat(f"u{suffix[i]}", 1, None, None, None)
                S21 = language.slice_mat(f"u{suffix[i]}", 2, None, None, None)
                S23 = "-" + language.slice_mat(f"u{suffix[i]}", 0, None, None,
                                               None)
                S31 = "-" + language.slice_mat(f"u{suffix[i]}", 1, None, None,
                                               None)
                S32 = language.slice_mat(f"u{suffix[i]}", 0, None, None, None)
                val_S = (f"#mat#3#3#0#{S12}#{S13}#{S21}#0#{S23}#{S31}#{S32}"
                         f"#0#endmat&")
                var_S = {"name": f"S{suffix[i]}",
                         "type": "mat",
                         "value": val_S}
                varss.append(var_S)

                var_Od_tmp = {"name": f"Ad_tmp{suffix[i]}",
                              "type": "mat",
                              "value": "#mat#3#3#1#0#0#0#1#0#0#0#1#endmat& + "
                                       f"S{suffix[i]}*sin(theta_t{suffix[i]})"
                                       f"+(S{suffix[i]}@S{suffix[i]})*(1-cos"
                                       f"(theta_t{suffix[i]}))@{Od_vars[i]}"}
                varss.append(var_Od_tmp)
                var_wd_tmp = {"name": f"wd_tmp{suffix[i]}",
                              "type": "mat",
                              "value": f"{r_dot_t}*theta_f{suffix[i]}*"
                                       f"u{suffix[i]}"}
                varss.append(var_wd_tmp)

                # L Matrix
                L_val = "-0.5*( S0 + S1 + S2 )"
                for l in range(3):
                    val_A = (f"#mat#3#3#0#-{Ak_vars[i][2][l]}#"
                             f"{Ak_vars[i][1][l]}#"
                             f"{Ak_vars[i][2][l]}#0#-{Ak_vars[i][0][l]}#-"
                             f"{Ak_vars[i][1][l]}#{Ak_vars[i][0][l]}"
                             f"#0#endmat&")
                    val_Ad = (f"#mat#3#3#0#-{Ak_d_traj_vars[i][2][l]}#"
                             f"{Ak_d_traj_vars[i][1][l]}#"
                             f"{Ak_d_traj_vars[i][2][l]}#0#"
                             f"-{Ak_d_traj_vars[i][0][l]}#-"
                             f"{Ak_d_traj_vars[i][1][l]}#"
                             f"{Ak_d_traj_vars[i][0][l]}#0#endmat&")

                    L_val = L_val.replace(f"S{l}", f"{val_A} @ {val_Ad}")
                var_L = {"name": f"L{suffix[i]}",
                         "type": "mat",
                         "value": L_val}
                varss.append(var_L)

                var_w = {"name": f"w{suffix[i]}",
                         "type": "mat",
                         "value": f"L{suffix[i]}**(-1) @ (err_o{suffix[i]}"
                                  f"*ko{suffix[i]} + transpose(L{suffix[i]})"
                                  f"@wd_tmp{suffix[i]})"}
                varss.append(var_w)

    # No trajectory Generation
    else:
        for i in range(n):
            pos, ori = split_content(ids[i][2])
            if pos != "":
                var_X_dot = {"name": f"Xdot{suffix[i]}",
                             "type": "mat",
                             "value": f"({Xd_vars[i]}-{X_vars[i]})"
                                      f"*kp{suffix[i]}"}
                varss.append(var_X_dot)
            if ori != "":
                # L Matrix
                L_val = "-0.5*( S0 + S1 + S2 )"
                for l in range(3):
                    val_A = (f"#mat#3#3#0#-{Ak_vars[i][2][l]}#"
                             f"{Ak_vars[i][1][l]}#"
                             f"{Ak_vars[i][2][l]}#0#-{Ak_vars[i][0][l]}#-"
                             f"{Ak_vars[i][1][l]}#{Ak_vars[i][0][l]}"
                             f"#0#endmat&")
                    val_Ad = (f"#mat#3#3#0#-{Ak_d_vars[i][2][l]}#"
                              f"{Ak_d_vars[i][1][l]}#"
                              f"{Ak_d_vars[i][2][l]}#0#"
                              f"-{Ak_d_vars[i][0][l]}#-"
                              f"{Ak_d_vars[i][1][l]}#"
                              f"{Ak_d_vars[i][0][l]}#0#endmat&")

                    L_val = L_val.replace(f"S{l}", f"{val_A} @ {val_Ad}")
                var_L = {"name": f"L{suffix[i]}",
                         "type": "mat",
                         "value": L_val}
                varss.append(var_L)

                var_w = {"name": f"w{suffix[i]}",
                         "type": "mat",
                         "value": f"L{suffix[i]}**(-1) @ (err_o{suffix[i]}"
                                  f"*ko{suffix[i]})"}
                varss.append(var_w)

    # Concatenation position & orientation
    concat_names = []
    for i in range(n):
        pos, ori = split_content(ids[i][2])
        if pos != "" and ori != "":
            concat_names.append(f"Xdot_w{suffix[i]}")
            var_concat = {"name": f"Xdot_w{suffix[i]}",
                          "type": "mat",
                          "value": f"___vcat__Xdot{suffix[i]}__w{suffix[i]}"
                                   f"___"}
            varss.append(var_concat)
        elif pos != "":
            concat_names.append(f"Xdot{suffix[i]}")
        else:
            concat_names.append(f"w{suffix[i]}")

    # Jacobians
    if not loop["constraints"]:
        if n == 1:
            var_qdot = {"name": f"qdot{suffix[0]}",
                        "type": "mat",
                        "value": f"pinv({jacobians[0]}) @ {concat_names[0]}"}
            varss.append(var_qdot)
        else:
            var_j_1 = {"name": "J_1",
                       "type": "mat",
                       "value": f"{jacobians[0]}"}
            varss.append(var_j_1)

            var_pinv = {"name": f"pinv_J_1",
                        "type": "mat",
                        "value": f"pinv(J_1)"}
            varss.append(var_pinv)

            var_qdot1 = {"name": f"qdot_1",
                         "type": "mat",
                         "value": f"pinv_J_1 @ {concat_names[0]}"}
            varss.append(var_qdot1)

            var_j_2 = {"name": "J_2",
                       "type": "mat",
                       "value": f"{jacobians[1]}"}
            varss.append(var_j_2)

            dim_I = f"{len(robot.dof)}__{len(robot.dof)}"
            var_qdot2 = {"name": "qdot",
                         "type": "mat",
                         "value": f"qdot_1+pinv(J_2 @ ( ___eye__{dim_I}___- "
                                  f"pinv_J_1@J_1)) @ ({concat_names[1]}-"
                                  f"J_2@qdot_1)"}
            varss.append(var_qdot2)

    # TODO Quadprog
    else:
        pass

    if loop["control_type"] == "velocities":
        var_set_vel = {"name": "",
                       "type": "function",
                       "value": "set_velocities(qdot)"}
        varss.append(var_set_vel)
        var_get_pos = {"name": "q",
                       "type": "",
                       "value": "get_positions()"}
        varss.append(var_get_pos)

    # dt
    var_dt = {"name": "__TIMEDT__",
              "type": "",
              "value": ""}
    varss.append(var_dt)
    varss.append(var_time_start)
    var_t = {"name": "t",
             "type": "function",
             "value": "___pluseq__t__dt___"}
    varss.append(var_t)

    if loop["control_type"] != "velocities":
        # Integration
        var_q = {"name": "",
                 "type": "function",
                 "value": "___pluseq__q__qdot*dt___"}
        varss.append(var_q)

    if loop["control_type"] == "positions":
        var_set_pos = {"name": "",
                       "type": "function",
                       "value": "set_positions(q)"}
        varss.append(var_set_pos)
        var_get_pos = {"name": "q",
                       "type": "",
                       "value": "get_positions()"}
        varss.append(var_get_pos)

    # Closing while loop
    varss.append(var_endif)

    # Generate function ......................................................

    return language.generate_fct("void", fname, params, "", varss,
                                 docstr=docstr,
                                 matrix_dims=(1, 1))


# All control loops __________________________________________________________

def generate_all_control_loops(control_loops_list,
                               robot,
                               traj_parameters,
                               language,
                               progressbar=None,
                               progress_increment=0):
    code = "\n\n" + language.title("Control Loops", 0) + "\n\n"

    for loop in control_loops_list:
        code += generate_control_loop(loop, robot, traj_parameters, language)
        code += "\n\n"
        increment_progressbar(progressbar, progress_increment)

    return code


# Generate Everything ________________________________________________________

def generate_everything(robot, list_ftm, list_btm, list_fk, list_jac, com,
                        com_jac, polynomial_trajectories, control_loops_list,
                        optimization_level,
                        language, filename, progressbar=None):
    """
    Description
    -----------
    
    Generates  all  the  desired  functions in the file 'filename.xx' where xx
    stands for the language file extension.

    Parameters
    ----------
    
    robot : robots.Robot
        Robot object for code generation
    
    list_ftm : list of str
        List of all the forward transition matrices to generate
        Every element is a string formatted like the nodes
        names of robot.tree :
            type_number
            
            where  type  is 'joint' (in this case) and number is the number of
            the considered Joint in robot.joints.
            
            Example : list_ftm=['joint_3', 'joint_2']

    list_btm : list of str
        List of all the backward transition matrices to generate
        Every element is a string formatted like the nodes
        names of robot.tree :
            type_number
            
            where  type  is 'joint' (in this case) and number is the number of
            the considered Joint in robot.joints
            
            Example : list_btm=['joint_3', 'joint_2']
            
    list_fk : list of list of 2 str
        List of all the forward kinematics functions to generate
        Every element of the list contains 2 elements :
            - origin
            - destination
            
        origin and destination are strings formatted like the nodes
        names of robot.tree :
            type_number
            
            where type is 'joint' / 'link' and number is the number of the
            considered Joint / Link in self.joints / self.links.
            
            Example : origin = 'joint_3'
            
    list_jac : list of list of 2 str
        List of all the jacobian functions to generate
        Every element of the list contains 2 elements :
            - origin
            - destination
            
        origin and destination are strings formatted like the nodes
        names of robot.tree :
            type_number
            
            where type is 'joint' / 'link' and number is the number of the
            considered Joint / Link in self.joints / self.links.
            
            Example : origin = 'joint_3'
    
    com : bool
        Set  this parameter to True if you want to generate the Center of Mass
        function.

    com_jac : bool
        Set  this parameter to True if you want to generate the Center of Mass
        Jacobian.

    polynomial_trajectories : list of dict
        list of all the polynomial trajectories to generate.

        Every item of this list must be a dict with the following structure :

        {"name": str : Name of the trajectory,

         "conditions" : list of list of 3 str :

            [..., [k, t, x], ...]

            k : str representing an integer
                Order  of  the  derivative.  If  the  time  is your derivative
                variable  and  the function you  want to create describes your
                position,  0 corresponds to the position, 1 to the speed, 2 to
                the acceleration, 3 to the jerk and so on.
            t : str representing a float or a symbol
                Time value on which you want your condition to be set
            x : str representing a float or a symbol
                Value  of the  function  for  the  given  time.  This can be a
                symbolic variable
        }

    control_loops_list : list of dict
        list of all the control_loops to generate.

        Every item of this list must be a dict with the following structure :

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
    
    language : Language.Language
        Language of the generated code

    optimization_level : int
        Optimization level of the generated code :
        - 0 : Numerical functions
        - 1 : Analytical functions
        - 2 : Analytical factored functions
        - 3 : Analytical simplified functions
    
    filename : str
        Name  of  the  output  file  containing  the  generated  code (without
        extension)

    progressbar : PyQt5.QtWidgets.QProgressBar or None, optional
        default is None
        Progressbar to update during the robot creation (used in GUI)
        If it is None, no progressbar is updated

    Returns
    -------
    None.

    """

    # NUmber of functions to generate
    total_fcts = len(list_ftm + list_btm + list_fk + list_jac +
                     polynomial_trajectories + control_loops_list) + com +\
                     com_jac
    progress_0 = 0
    progress_increment = 100/total_fcts

    # Opening the file in write mode

    with open(filename + '.' + language.extension, 'w') as f:
        header = ("The code in this file has been generated by URDFast Code "
                  "Generator on " + datetime.now()
                  .strftime("%m/%d/%Y, %H:%M:%S") +
                  ". Consider testing this code before using it as errors "
                  "remain "
                  "possible. For more details, check out the github "
                  "repository "
                  "of this project at https://github.com/Teskann/URDFast.")

        code = language.comment_par_beg + '\n' + \
               language.justify(header) + \
               '\n' + language.comment_par_end + '\n\n' + \
               language.header + '\n'

        if language.name == "matlab":
            code += f"\nclassdef {filename.split('/')[-1]}\nmethods(Static)\n"
            code += "\n"

        code += generate_all_matrices(robot, list_ftm, list_btm, language,
                                      progressbar=progressbar,
                                      progress_increment=progress_increment)

        if list_fk:
            code += '\n\n'
            list_origin = []
            list_dest = []
            list_content = []

            for fk in list_fk:
                list_origin.append(fk[0])
                list_dest.append(fk[1])
                list_content.append(fk[2])

            code += generate_all_fk(robot, list_origin, list_dest,
                                    list_content,
                                    optimization_level,
                                    language,
                                    progressbar=progressbar,
                                    progress_increment=progress_increment)

        if list_jac:
            code += '\n\n'
            list_origin = []
            list_dest = []
            list_content = []

            for jac in list_jac:
                list_origin.append(jac[0])
                list_dest.append(jac[1])
                list_content.append(jac[2])

            code += generate_all_jac(robot, list_origin, list_dest,
                                     list_content, optimization_level,
                                     language,
                                     progressbar=progressbar,
                                     progress_increment=progress_increment)

        if com:
            code += '\n\n'
            code += generate_com(robot, optimization_level, language,
                                 progressbar=progressbar,
                                 progress_increment=progress_increment
                                 )

        if com_jac:
            code += '\n\n'
            code += generate_com_jacobian(
                robot, optimization_level, language,
                progressbar=progressbar,
                progress_increment=progress_increment)

        par = None
        if polynomial_trajectories:
            code_, par = generate_all_polynomial_trajectories(
                         polynomial_trajectories,
                         language,
                         progressbar=progressbar,
                         progress_increment=progress_increment)
            code += code_

        if control_loops_list:
            code += generate_all_control_loops(
                control_loops_list,
                robot,
                par,
                language,
                progressbar=progressbar,
                progress_increment=progress_increment)

        code += '\n'

        if language.name == "matlab":
            code += "\nend\nend\n"
            code = code.replace("MATLAB_PREFIX",
                                f"{filename.split('/')[-1]}.")
        else:
            code = code.replace("MATLAB_PREFIX", "")

        f.write(code)
        f.close()

        print("Done")


if __name__ == '__main__':
    print("\n==================================\n")
    print("Test : Robot - example_1.urdf\n")
    urdf_obj = URDF("./Examples/example_1.urdf")
    robot_obj = Robot(urdf_obj)
    print(robot_obj)

    list_tm = ['joint_0', 'joint_1', 'joint_2']

    lang = Language('julia')
    s = ''
    s += generate_all_matrices(robot_obj, list_tm, list_tm, language=lang)
    origins = ['link_0', 'joint_0']
    destinations = ['link_1', 'link_3']
    s += '\n\n' + generate_all_fk(robot_obj, origins, destinations, lang)

    s += '\n\n' + generate_all_jac(robot_obj, origins, destinations, lang)

    # print(s)

    s += '\n\n' + generate_com(robot_obj, lang)

    s += '\n\n' + generate_com_jacobian(robot_obj, lang)

    conds = [["0", "0", "0"],
             ["0", "1", "1"],
             ["1", "0", "0"],
             ["1", "1", "0"],
             ["2", "0", "0"],
             ["2", "1", "0"],
             ["3", "0", "0"],
             ["3", "1", "0"],
             ["4", "0", "0"],
             ["4", "1", "0"],
             ["5", "0", "0"],
             ["5", "1", "0"],
             ["6", "0", "0"],
             ["6", "1", "0"],
             ["7", "0", "0"],
             ["7", "1", "0"],
             ["8", "0", "0"],
             ["8", "1", "0"],
             ["9", "0", "0"],
             ["9", "1", "0"],
             ]
    s += '\n\n' + generate_polynomial_trajectory(conds, "f", lang)
    with open("Output2." + lang.extension, "w") as text_file:
        text_file.write(s)
