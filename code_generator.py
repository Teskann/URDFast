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


# Generate Python code from Sympy Matrix _____________________________________

def generate_code_from_sym_mat(sympy_matrix, fname,
                               language=Language('python'),
                               docstr=None,
                               input_is_vector=False):
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

    r = language.generate_fct(fname, params, code_mat, varss=varss,
                              docstr=docstr,
                              input_is_vector=input_is_vector,
                              matrix_dims=(len(code_mat), len(code_mat[0])))

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

    docstr = (f'Computes the forward kinematics from the {type_origin} '
              f"{origin_name} to the {type_dest} {dest_name}. The result is "
              f"returned as a 4x4 {language.matrix_type} in  homogeneous "
              "coordinates, giving the position and the orientation of "
              f"{dest_name} in the {origin_name} frame.")

    fname = 'fk_' + origin_name + '_' + dest_name + "_" + content

    # Optimised version ......................................................

    if optimization_level > 0:
        fk = robot.forward_kinematics(origin, destination, content=content,
                                      optimization_level=optimization_level)
        return generate_code_from_sym_mat(fk, fname, language, docstr,
                                          input_is_vector=True)

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
                    'value': '_eye_4_4_',
                    'type': 'mat'}
        varss.append(variable)
    expr = ''
    for i_v, var in enumerate(varss):

        expr += str(var['name'])
        if i_v < len(varss) - 1:
            expr += '@'

    params.sort(key=lambda x: x['name'])

    code += language.generate_fct(fname, params, expr, varss, docstr,
                                  matrix_dims=(1, 1), input_is_vector=True)

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
                      'value': f"_zeros_6_{nb_dof}_",
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
        jac, all_sym = robot\
            .jacobian(origin, destination, content,
                      optimization_level=optimization_level)

        params += get_parameters(all_sym)
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
              f'returned as a (6 x {len(params)}) matrix where every column '
              f'is the derivative of the position/orientation with respect to'
              f' a degree of freedom. \n'
              f'    - The line 1 is the derivative of X position of '
              f'{dest_name} in the {origin_name} frame,\n'
              f'    - The line 2 is the derivative of Y position of '
              f'{dest_name}'
              f' in the {origin_name} frame,\n'
              f'    - The line 3 is the derivative of Z position of '
              f'{dest_name}'
              f' in the {origin_name} frame,\n'
              '    - The line 4 is the derivative of the roll orientation of'
              f' {dest_name} in the {origin_name} frame,\n'
              '    - The line 5 is the derivative of the pitch orientation of'
              f' {dest_name} in the {origin_name} frame,\n'
              '    - The line 6 is the derivative of the yaw orientation of'
              f' {dest_name} in the {origin_name} frame,\n'
              'Here is the list of all the derivative variables :')
    for i_p, param in enumerate(params):
        docstr += f'\n    - Column {language.indexing_0 + i_p} : ' + \
                  f'{param["name"]}'

    fname = 'jacobian_' + origin_name + '_to_' + dest_name + "_" + content

    if optimization_level == 0:
        for i_v, var in enumerate(varss):
            for i_p, param in enumerate(params):
                varss[i_v]['value'] = \
                    replace_var(var['value'], param['name'],
                                language.slice_mat("q", i_p, None, None,
                                                   None))
        code += language.generate_fct(fname, parameters, expr, varss, docstr,
                                      matrix_dims=(1, 1))
    else:
        code += generate_code_from_sym_mat(jac, fname, language, docstr,
                                           input_is_vector=True)

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
               'value': '_eye_4_4_',
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
                          f'{str(cm[2, 0])}#1.0#'
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

        all_sym = list(com.free_symbols)

        all_sym.sort(key=lambda sym: sym.name)

        params += get_parameters(all_sym)

    descrq = (f'Vector of length {len(params)} containing all the degrees of '
              'freedom of the robot that have an effect on the center of mass'
              ' position. This vector contains:')
    for i_p, param in enumerate(params):
        descrq += f'\n        - q[{i_p + language.indexing_0}] = ' + \
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
        code += language.generate_fct('com', [paramq], expr[1:], varss,
                                      docstr, matrix_dims=(1, 1))
    else:
        code += generate_code_from_sym_mat(com, 'com', language, docstr,
                                           input_is_vector=True)

    increment_progressbar(progressbar, progress_increment)
    return code


# Generate Center of Mass Jacobian ___________________________________________

def generate_com_jacobian(robot, optimization_level,
                          language=Language('python'),
                          progressbar=None,
                          progress_increment=0):
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
                      'value': f"_zeros_3_{nb_dof}_",
                      'type': 'mat'})

        # Initial transformation
        var = {'name': 'T',
               'value': '_eye_4_4_',
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
                          f'{str(cm[2, 0])}#1.0#'
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
        all_sym = list(jac.free_symbols)
        all_sym.sort(key=lambda sym: sym.name)
        params += get_parameters(all_sym)
        nb_dof = len(all_sym)

    descrq = 'Vector of all the degrees of freedom of the robot that have ' \
             'an effect on the center of mass position. This vector ' \
             'contains : '
    for i_p, param in enumerate(params):
        descrq += f'\n        - q[{i_p + language.indexing_0}] = ' + \
                  param['name']
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

    docstr = 'Returns the Jacobian of the center of mass of the robot. ' + \
             'This matrix is ' + \
             f'returned as a (3 x {nb_dof}) matrix where every column is the' + \
             ' derivative of the position of the CoM (X, Y and Z) with ' \
             'respect' + \
             ' to a degree ' + \
             'of freedom. The result is expressed in the root link frame.\n' + \
             f'    - The line 1 is the derivative of X position of {dest_name}' + \
             f' in the {origin_name} frame,\n' + \
             f'    - The line 2 is the derivative of Y position of {dest_name}' + \
             f' in the {origin_name} frame,\n' + \
             f'    - The line 3 is the derivative of Z position of {dest_name}' + \
             f' in the {origin_name} frame\n' + \
             'Here is the list of all the derivative variables :'
    for i_p, param in enumerate(params):
        docstr += f'\n    - Column {language.indexing_0 + i_p} : ' + \
                  f'{param["name"]}'

    if optimization_level == 0:
        code += language.generate_fct('jacobian_com', paar, expr, varss,
                                      docstr, matrix_dims=(1, 1))
    else:
        code += generate_code_from_sym_mat(jac, 'jacobian_com', language,
                                           docstr, input_is_vector=True)

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

    symbolic_variables = []
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

        varss, expr = optimize(str(polynomial))

        code += language.generate_fct(fname, parameters, expr, varss,
                                      docstring, (1, 1), False) + '\n\n'

    return code


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

    """

    code = "\n\n" + language.title("Polynomial Trajectories", 0) + "\n\n"

    for trajectory in trajectories:
        code += generate_polynomial_trajectory(trajectory["conditions"],
                                               trajectory["name"],
                                               language)
        increment_progressbar(progressbar, progress_increment)

    return code


# Generate Everything ________________________________________________________

def generate_everything(robot, list_ftm, list_btm, list_fk, list_jac, com,
                        com_jac, polynomial_trajectories, optimization_level,
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
                     polynomial_trajectories) + com + com_jac
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

        if polynomial_trajectories:
            code += generate_all_polynomial_trajectories(
                    polynomial_trajectories,
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
