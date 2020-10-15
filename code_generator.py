# -*- coding: utf-8 -*-
"""
Created on Fri Jun 19 21:44:37 2020

@author: Clément
"""

from URDF import URDF
from createRobotFromURFD import Link, Joint, Robot
from sympy import Matrix, Symbol, cos, sin, Min, Max, nsimplify, simplify,\
    pretty
from Language import Language
from datetime import datetime
from code_optimization import replace_var, optimize
from anytree import PreOrderIter

# Generate Python code from Sympy Matrix _____________________________________

def generate_code_from_sym_mat(sympy_matrix, fname,
                               language=Language('python'),
                               docstr=None):
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
    expr = expr[9:-3].replace(' ','')
    elems = expr.split('],[')
    
    for elem in elems:
        code_mat.append(elem.split(','))
    
    # 2 - Gertting function parameters .......................................
    
    params = []
    for symbol in sympy_matrix.free_symbols:
        param = {}
        param['name'] = str(symbol)
        param['type'] = 'double'
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
    
    r = language.generate_fct(fname, params, code_mat, varss=varss,
                              docstr=docstr)
    
    return r

# Generate all matrices ______________________________________________________

def generate_all_matrices(robot, list_ftm, list_btm,
                          language=Language('python')):
    """
    Description
    -----------
    
    Generate all transformation matrices for each joint of the robot
    
    Parameters
    ----------
    
    robot : createRobotFromURDF.Robot
        Robot you want to generate the matrices from
    
    list_ftm : list of str
        List of all the forward transition matrices to generate
        Every element is a string formatted like the nodes
        names of robot.tree :
            type_number
            
            where  type  is 'joint' (in this case) and number is the number of
            the considered Joint in robot.joints.
            
            Example : list_ftm=['joint_3', 'joint_2']

    list_ftm : list of str
        List of all the backward transition matrices to generate
        Every element is a string formatted like the nodes
        names of robot.tree :
            type_number
            
            where  type  is 'joint' (in this case) and number is the number of
            the considered Joint in robot.joints
            
            Example : list_btm=['joint_3', 'joint_2']
    
    language : Language.Language, optional
        Language you want the code to be generated to
        Defalut is Language('python')
    
    Returns
    -------
    
    str :
        String containing the code of all the functions
    
    """
    
    code = ''
    
    if list_ftm != []:
    
        code += language.title('FORWARD TRANSISION MATRICES', 0)
        
        code += '\n\n'
    
    # For every joint ........................................................
    
    for jj in list_ftm:
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
      
    if list_btm != []:
        code += language.title('BACKWARD TRANSISION MATRICES', 0)
    
        code += '\n\n'
    
    # For every joint ........................................................
    
    for jj in list_btm:
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
    
    return code

# Generate Forward Kinematics ________________________________________________

def generate_fk(robot, origin, destination, language=Language('python')):
    """
    Description
    -----------
    
    Generate function to compute forward kinematics.
    This   function    generates    code   using   generated   matrices   from
    generate_all_matrices.  Consider  using this function after having run the
    generate_all_matrices function
    
    Parameters
    ----------
    
    robot : createRobotFromURDF.Robot
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
    
    language : Language.Language, optional
        Language you want the code to be generated to
        Defalut is Language('python')
        
    Returns
    -------
    
    str :
        String  containing the forward kinematics function in the language you
        want
    
    """
    
    # Adding Title
    code = language.title('Forward Kinematics from ' + origin + ' to ' +\
                          destination, 1)
    code += '\n\n'
    
    # 1 - Getting the path in the tree .......................................
        
    upwards, downwards = robot.branch(origin, destination)
    
    # 2 - Getting the matrix .................................................
    
    # Intermediate variables
    varss = []
    
    # FK function parameters
    params = []
    
    # First upwards joints
    for up_joint_nb in upwards:
        joint = robot.joints[up_joint_nb]
        
        # Paramters
        all_sym = joint.T.free_symbols
        params_tmp = []
        for symbol in all_sym:
            
            param = {}
            param['name'] = str(symbol)
            param['type'] = 'double'
            category = param['name'].split('_')[0]
            descr = ''
            if category == 'd':
                descr += 'Translation value (in meters) along the '
                descr += param['name'][2:] + ' prismatic joint axis.'
            
            elif category == 'dx':
                descr += 'Translation value (in meters) along the X axis of '
                descr += 'the ' + param['name'][3:] + ' joint.'
                
            elif category == 'dy':
                descr += 'Translation value (in meters) along the Y axis of '
                descr += 'the ' + param['name'][3:] + ' joint.'
            
            elif category == 'dz':
                descr += 'Translation value (in meters) along the Z axis of '
                descr += 'the ' + param['name'][3:] + ' joint.'
            
            elif category == 'dz':
                descr += 'Translation value (in meters) along the Z axis of '
                descr += 'the ' + param['name'][3:] + ' joint.'
            
            elif category == 'theta':
                descr += 'Rotation value (in radians) around the '
                descr += param['name'][6:] + ' joint axis.'
            
            elif category == 'roll':
                descr += 'Rotation value (in radians) around the X axis of '
                descr += 'the ' + param['name'][5:] + ' joint.'
            
            elif category == 'pitch':
                descr += 'Rotation value (in radians) around the Y axis of '
                descr += 'the ' + param['name'][6:] + ' joint.'
            
            elif category == 'yaw':
                descr += 'Rotation value (in radians) around the Z axis of '
                descr += 'the ' + param['name'][4:] + ' joint.'
                
            param['description'] = descr
            
            params_tmp.append(param)
            params.append(param)
        
        
        val = 'T_' + joint.name + '_inv('
        
        params_tmp.sort(key=lambda x : x['name'])
        
        for i_p, par in enumerate(params_tmp):
            val += par['name']
            if i_p < len(params_tmp) - 1:
                val += ','
            else:
                val += ')'
        
        variable = {'name':'T_' + str(up_joint_nb) + '_inv',
                    'value' : val,
                    'type' : 'mat'}
        varss.append(variable)
            
        
    # Then downwards joints
    for down_joint_nb in downwards:
        joint = robot.joints[down_joint_nb]
        # Paramters
        all_sym = joint.T.free_symbols
        params_tmp = []
        for symbol in all_sym:
            
            param = {}
            param['name'] = str(symbol)
            param['type'] = 'double'
            category = param['name'].split('_')[0]
            descr = ''
            if category == 'd':
                descr += 'Translation value (in meters) along the '
                descr += param['name'][2:] + ' prismatic joint axis.'
            
            elif category == 'dx':
                descr += 'Translation value (in meters) along the X axis of '
                descr += 'the ' + param['name'][3:] + ' joint.'
                
            elif category == 'dy':
                descr += 'Translation value (in meters) along the Y axis of '
                descr += 'the ' + param['name'][3:] + ' joint.'
            
            elif category == 'dz':
                descr += 'Translation value (in meters) along the Z axis of '
                descr += 'the ' + param['name'][3:] + ' joint.'
            
            elif category == 'dz':
                descr += 'Translation value (in meters) along the Z axis of '
                descr += 'the ' + param['name'][3:] + ' joint.'
            
            elif category == 'theta':
                descr += 'Rotation value (in radians) around the '
                descr += param['name'][6:] + ' joint axis.'
            
            elif category == 'roll':
                descr += 'Rotation value (in radians) around the X axis of '
                descr += 'the ' + param['name'][5:] + ' joint.'
            
            elif category == 'pitch':
                descr += 'Rotation value (in radians) around the Y axis of '
                descr += 'the ' + param['name'][6:] + ' joint.'
            
            elif category == 'yaw':
                descr += 'Rotation value (in radians) around the Z axis of '
                descr += 'the ' + param['name'][4:] + ' joint.'
                
            param['description'] = descr
            
            params_tmp.append(param)
            params.append(param)
        val = 'T_' + joint.name + '('
        
        params_tmp.sort(key=lambda x : x['name'])
        
        for i_p, par in enumerate(params_tmp):
            val += par['name']
            if i_p < len(params_tmp) - 1:
                val += ','
            else:
                val += ')'

        variable = {'name':'T_' + str(down_joint_nb),
                    'value' : val,
                    'type' : 'mat'}
        varss.append(variable)
    
    if len(varss) == 0:
        variable = {'name':'T',
                    'value' : '_eye_4_4_',
                    'type' : 'mat'}
        varss.append(variable)
    expr = ''
    for i_v, var in enumerate(varss):
        
        expr += str(var['name'])
        if i_v < len(varss) - 1:
            expr += '@'
    
    index_origin = int(origin.split('_')[1])
    type_origin = origin.split('_')[0]
    origin_name = robot.joints[index_origin].name if type_origin == 'joint'\
        else robot.links[index_origin].name
    index_dest = int(destination.split('_')[1])
    type_dest = destination.split('_')[0]
    dest_name = robot.joints[index_dest].name if type_dest == 'joint'\
        else robot.links[index_dest].name
    docstr = 'Comuputes the forward kinematics from the ' + type_origin + ' '\
        + origin_name + ' to the ' +type_dest + ' ' + dest_name +\
            '. The result '
    docstr += 'is returned as a 4x4 ' + language.matrix_type + ' in ' + \
        'homogeneous coordinates.'
        
    fname = 'fk_' + origin_name + '_' + dest_name
    code += language.generate_fct(fname, params, expr, varss, docstr,
                                  matrix_dims=(1,1), input_is_vector=True)
        
    return code

# Generate all FK functions __________________________________________________

def generate_all_fk(robot, list_origin, list_dest, 
                    language=Language('python')):
    """
    Description
    -----------
    
    Generate all forward kinematics functions
    
    Parameters
    ----------
    
    robot : createRobotFromURDF.Robot
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
    
    language : Language.Language, optional
        Language you want the code to be generated to
        Defalut is Language('python')
        
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
        code += generate_fk(robot, origin, list_dest[i], language=language)
        if i < len(list_origin) - 1:
            code += '\n\n'
    
    return code

# Generate Jacobian Function _________________________________________________

def generate_jacobian(robot, origin, destination, 
                      language=Language('python')):
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
    
    robot : createRobotFromURDF.Robot
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
    
    language : Language.Language, optional
        Language you want the code to be generated to
        Defalut is Language('python')
        
    Returns
    -------
    
    str :
        String  containing the forward kinematics function in the language you
        want
    
    """
    
    # Adding Title
    code = language.title('Jacobian of the ' + destination + ' position ' +\
                          'and orientation', 1)
    code +='\n\n'
    
    # 1 - Getting the path in the tree .......................................
        
    upwards, downwards = robot.branch(origin, destination)
    
    # 2 - Getting the matrix .................................................
    
    # Intermediate variables
    varss = []
    
    # Jacobian function parameters
    params = []
    
    T_fcts = []
    
    # First upwards joints
    for up_joint_nb in upwards:
        joint = robot.joints[up_joint_nb]
        
        # Paramters
        all_sym = joint.T.free_symbols
        params_tmp = []
        for symbol in all_sym:
            
            param = {}
            param['name'] = str(symbol)
            param['type'] = 'double'
            category = param['name'].split('_')[0]
            descr = ''
            if category == 'd':
                descr += 'Translation value (in meters) along the '
                descr += param['name'][2:] + ' prismatic joint axis.'
            
            elif category == 'dx':
                descr += 'Translation value (in meters) along the X axis of '
                descr += 'the ' + param['name'][3:] + ' joint.'
                
            elif category == 'dy':
                descr += 'Translation value (in meters) along the Y axis of '
                descr += 'the ' + param['name'][3:] + ' joint.'
            
            elif category == 'dz':
                descr += 'Translation value (in meters) along the Z axis of '
                descr += 'the ' + param['name'][3:] + ' joint.'
            
            elif category == 'dz':
                descr += 'Translation value (in meters) along the Z axis of '
                descr += 'the ' + param['name'][3:] + ' joint.'
            
            elif category == 'theta':
                descr += 'Rotation value (in radians) around the '
                descr += param['name'][6:] + ' joint axis.'
            
            elif category == 'roll':
                descr += 'Rotation value (in radians) around the X axis of '
                descr += 'the ' + param['name'][5:] + ' joint.'
            
            elif category == 'pitch':
                descr += 'Rotation value (in radians) around the Y axis of '
                descr += 'the ' + param['name'][6:] + ' joint.'
            
            elif category == 'yaw':
                descr += 'Rotation value (in radians) around the Z axis of '
                descr += 'the ' + param['name'][4:] + ' joint.'
                
            param['description'] = descr
            
            params_tmp.append(param)
            params.append(param)
        
        
        val = 'T_' + joint.name + '_inv('
        
        params_tmp.sort(key=lambda x : x['name'])
        
        for i_p, par in enumerate(params_tmp):
            val += par['name']
            if i_p < len(params_tmp) - 1:
                val += ','
            else:
                val += ')'

        T_fcts.append(['T_' + str(up_joint_nb) + '_inv', val])
            
        
    # Then downwards joints
    for down_joint_nb in downwards:
        joint = robot.joints[down_joint_nb]
        # Paramters
        all_sym = joint.T.free_symbols
        params_tmp = []
        for symbol in all_sym:
            
            param = {}
            param['name'] = str(symbol)
            param['type'] = 'double'
            category = param['name'].split('_')[0]
            descr = ''
            if category == 'd':
                descr += 'Translation value (in meters) along the '
                descr += param['name'][2:] + ' prismatic joint axis.'
            
            elif category == 'dx':
                descr += 'Translation value (in meters) along the X axis of '
                descr += 'the ' + param['name'][3:] + ' joint.'
                
            elif category == 'dy':
                descr += 'Translation value (in meters) along the Y axis of '
                descr += 'the ' + param['name'][3:] + ' joint.'
            
            elif category == 'dz':
                descr += 'Translation value (in meters) along the Z axis of '
                descr += 'the ' + param['name'][3:] + ' joint.'
            
            elif category == 'dz':
                descr += 'Translation value (in meters) along the Z axis of '
                descr += 'the ' + param['name'][3:] + ' joint.'
            
            elif category == 'theta':
                descr += 'Rotation value (in radians) around the '
                descr += param['name'][6:] + ' joint axis.'
            
            elif category == 'roll':
                descr += 'Rotation value (in radians) around the X axis of '
                descr += 'the ' + param['name'][5:] + ' joint.'
            
            elif category == 'pitch':
                descr += 'Rotation value (in radians) around the Y axis of '
                descr += 'the ' + param['name'][6:] + ' joint.'
            
            elif category == 'yaw':
                descr += 'Rotation value (in radians) around the Z axis of '
                descr += 'the ' + param['name'][4:] + ' joint.'
                
            param['description'] = descr
            
            params_tmp.append(param)
            params.append(param)
        val = 'T_' + joint.name + '('
        
        params_tmp.sort(key=lambda x : x['name'])
        
        for i_p, par in enumerate(params_tmp):
            val += par['name']
            if i_p < len(params_tmp) - 1:
                val += ','
            else:
                val += ')'

        T_fcts.append(['T_' + str(down_joint_nb), val])
    
    # Jacobian variables .....................................................
    
    nb_dof = len(params)
    
    varss.append({'name': 'Jac',
                  'value' : f"_zeros_6_{nb_dof}_",
                  'type' : 'mat'})
    
    varss.append({'name' : 'T',
                  'value' : f'{T_fcts[0][1]}',
                  'type' : 'mat'})
    
    varss.append({'name' : 'L',
                  'value' : f'p0-T[{language.indexing_0}:' + \
                      f'{language.indexing_0+2+language.subscription}'+\
                          f',{language.indexing_0+3}]',
                  'type' : 'mat'})

    varss.append({'name' : 'Z',
                  'value' : f'T[{language.indexing_0}:' + \
                      f'{language.indexing_0+2+language.subscription},'+\
                          f'{language.indexing_0+2}]',
                      'type' : 'mat'})
    
    # Compute Jacobian column 1:3
    varss.append({'name' : f'Jac[{language.indexing_0}:' + \
                          f'{language.indexing_0+2+language.subscription},'+\
                              f'{language.indexing_0}]',
                      'value' : 'cross(Z,L)',
                      'type' : ''})
        
    # Compute Jacobian column 4:6
    varss.append({'name' : f'Jac[{language.indexing_0+3}:' + \
                      f'{language.indexing_0+5+language.subscription}'+\
                          f',{language.indexing_0}]',
                  'value' : 'Z',
                  'type' : ''})

    for i in range(1, nb_dof):
        # Multiplying by T
        varss.append({'name' : 'T',
                      'value' : f'T@{T_fcts[i][1]}',
                      'type' : ''})
        
        # Compute L
        varss.append({'name' : 'L',
                      'value' : f'p0-T[{language.indexing_0}:' + \
                          f'{language.indexing_0+2+language.subscription}'+\
                              f',{language.indexing_0+3}]',
                      'type' : ''})
        # Compute Z
        varss.append({'name' : 'Z',
                      'value' : f'T[{language.indexing_0}:' + \
                          f'{language.indexing_0+2+language.subscription}'+\
                              f',{language.indexing_0+2}]',
                      'type' : ''})
            
        # Compute Jacobian column 1:3
        varss.append({'name' : f'Jac[{language.indexing_0}:' + \
                          f'{language.indexing_0+2+language.subscription}'+\
                          f',{language.indexing_0+i}]',
                      'value' : 'cross(Z,L)',
                      'type' : ''})
        
        # Compute Jacobian column 4:6
        varss.append({'name' : f'Jac[{language.indexing_0+3}:' + \
                          f'{language.indexing_0+5+language.subscription}'+\
                              f',{language.indexing_0+i}]',
                      'value' : 'Z',
                      'type' : ''})
        
    expr='Jac'
    
    index_origin = int(origin.split('_')[1])
    type_origin = origin.split('_')[0]
    origin_name = robot.joints[index_origin].name if type_origin == 'joint'\
        else robot.links[index_origin].name
    index_dest = int(destination.split('_')[1])
    type_dest = destination.split('_')[0]
    dest_name = robot.joints[index_dest].name if type_dest == 'joint'\
        else robot.links[index_dest].name
    docstr = 'Comuputes the forward kinematics from the ' + type_origin + ' '\
        + origin_name + ' to the ' +type_dest + ' ' + dest_name +\
            '. The result '
    docstr += 'is returned as a 4x4 ' + language.matrix_type + ' in ' + \
        'homogeneous coordinates.'
    
    descrq =f'Vector of length {len(params)} containing all the degrees of '+\
        'freedom of the robot between '+\
        f'{origin_name} and {dest_name} chain. This vector contains :'
    for i_p, param in enumerate(params):
        descrq += f'\n        - q[{i_p+language.indexing_0}] = ' + \
            param['name']
        descrq += ' :\n              ' + param['description']
    
    paramq = {'name' : 'q', 'type' : 'vect', 'description' : descrq}
    
    p0 = {'name' : 'p0', 'type' : 'vect', 'description':
          f"Point in the {origin_name} frame where you want to compute the" +\
              " Jacobian Matrix. p0 is a (3 x 1) vector."}
    parameters = [paramq, p0]
    
    docstr = f'Computes the Jacobian Matrix of the {dest_name} coordinates '+\
        f'in the {origin_name} frame from the point p0. This matrix is ' +\
        f'returned as a (6 x {nb_dof}) matrix where every column is the' +\
        ' derivative of the position/orientation with respect to a degree '+\
        'of freedom. \n'+\
        f'    - The line 1 is the derivative of X position of {dest_name}'+\
        f' in the {origin_name} frame,\n'+\
        f'    - The line 2 is the derivative of Y position of {dest_name}'+\
        f' in the {origin_name} frame,\n'+\
        f'    - The line 3 is the derivative of Z position of {dest_name}'+\
        f' in the {origin_name} frame,\n'+\
        '    - The line 4 is the derivative of the roll orientation of'+\
        f' {dest_name} in the {origin_name} frame,\n'+\
        '    - The line 5 is the derivative of the pitch orientation of'+\
        f' {dest_name} in the {origin_name} frame,\n'+\
        '    - The line 6 is the derivative of the yaw orientation of'+\
        f' {dest_name} in the {origin_name} frame,\n'+\
            'Here is the list of all the derivative variables :'
    for i_p, param in enumerate(params):
        docstr += f'\n    - Column {language.indexing_0+i_p} : '+\
            f'{param["name"]}'
    
    for i_v, var in enumerate(varss):
        for i_p, param in enumerate(params):
            varss[i_v]['value'] = replace_var(var['value'], param['name'],
                                             f'q[{i_p+language.indexing_0}]')
    
    fname = 'jacobian_' + origin_name + '_to_' + dest_name
    
    code += language.generate_fct(fname, parameters, expr, varss, docstr,
                                  matrix_dims=(1,1))
    
    return code
           
# Generate all Jacobian Matrices _____________________________________________

def generate_all_jac(robot, list_origin, list_dest, 
                    language=Language('python')):
    """
    Description
    -----------
    
    Generate all Jacobian functions
    
    Parameters
    ----------
    
    robot : createRobotFromURDF.Robot
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
    
    language : Language.Language, optional
        Language you want the code to be generated to
        Defalut is Language('python')
        
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
        code += generate_jacobian(robot, origin, list_dest[i],
                                  language=language)
        if i < len(list_origin) - 1:
            code += '\n\n'
    
    return code

# Generate Center of Mass Position ___________________________________________

def generate_com(robot, language=Language('python')):
    """
    Generate the code for the center of mass of the robot

    Parameters
    ----------
    robot : createRobotFromURDF.Robot
        Robot you want to generate the center of mass function from
    
    language : Language.Language, optional
        Language you want the code to be generated to
        Defalut is Language('python')

    Returns
    -------
    str : 
        Code of the CoM function in the desired language

    """
    
    # Adding Title
    code = language.title("Center of Mass of the Robot", 0)
    code += '\n\n'
    
    # Total Mass of the robot ................................................
    
    mass = 0
    for link in robot.links:
        mass+=link.mass
    
    if mass==0:
        return language.comment_line + ' Center of mass function can not be'+\
            ' generated because the robot mass is null.'
    
    # Tree iteration .........................................................
    
    varss = []
    
    # Initial transformation
    var = {'name'  : 'T',
           'value' : '_eye_4_4_',
           'type'  : 'mat'}
    
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
            pos_val =f'#mat#4#1#{str(cm[0,0])}#{str(cm[1,0])}#'+\
                f'{str(cm[2,0])}#1.0#'
            pos_var = {'name' : f'com_{obj_nb}_xyz',
                       'value' : pos_val,
                       'type' : 'vect'}
            varss.append(pos_var)
            var = {'name' : f'com_{obj_nb}',
                   'value' : f'{relative_mass}*T@com_{obj_nb}_xyz',
                   'type' : 'vect'}
            varss.append(var)
            expr += f'+com_{obj_nb}'
            
            last_u = len(varss)
        
        elif obj_type == 'joint':
            joint = robot.joints[obj_nb]
            
            # Paramters
            all_sym = joint.T.free_symbols
            params_tmp = []
            for symbol in all_sym:
                
                param = {}
                param['name'] = str(symbol)
                param['type'] = 'double'
                category = param['name'].split('_')[0]
                descr = ''
                if category == 'd':
                    descr += 'Translation value (in meters) along the '
                    descr += param['name'][2:] + ' prismatic joint axis.'
                
                elif category == 'dx':
                    descr += 'Translation value (in meters) along the X axis'
                    descr += ' of the ' + param['name'][3:] + ' joint.'
                    
                elif category == 'dy':
                    descr += 'Translation value (in meters) along the Y axis'
                    descr += ' of the ' + param['name'][3:] + ' joint.'
                
                elif category == 'dz':
                    descr += 'Translation value (in meters) along the Z axis'
                    descr += ' of the ' + param['name'][3:] + ' joint.'
                
                elif category == 'dz':
                    descr += 'Translation value (in meters) along the Z axis'
                    descr += ' of the ' + param['name'][3:] + ' joint.'
                
                elif category == 'theta':
                    descr += 'Rotation value (in radians) around the '
                    descr += param['name'][6:] + ' joint axis.'
                
                elif category == 'roll':
                    descr += 'Rotation value (in radians) around the X axis'
                    descr += ' of the ' + param['name'][5:] + ' joint.'
                
                elif category == 'pitch':
                    descr += 'Rotation value (in radians) around the Y axis'
                    descr += ' of the ' + param['name'][6:] + ' joint.'
                
                elif category == 'yaw':
                    descr += 'Rotation value (in radians) around the Z axis'
                    descr += ' of the ' + param['name'][4:] + ' joint.'
                    
                param['description'] = descr
                
                params_tmp.append(param)
                params.append(param)
            T_fct = 'T_' + joint.name + '('
            
            params_tmp.sort(key=lambda x : x['name'])
            
            for i_p, par in enumerate(params_tmp):
                T_fct += par['name']
                if i_p < len(params_tmp) - 1:
                    T_fct += ','
            T_fct += ')'        
            
            if len(robot.links[joint.child].parent_joints) > 1:
                var2 = {'name' : f'T_{obj_nb}',
                        'value' : f'T@{T_fct}',
                        'type' : 'mat'}
                varss.append(var2)
                saved_joints_T.append(obj_nb)
                var = {'name' : 'T',
                       'value' : f'T_{obj_nb}',
                       'type' : ''}
                varss.append(var)
            
            elif any(x in robot.links[joint.parent].child_joints for x in\
                     saved_joints_T):
                num = robot.links[joint.parent].child_joints[0]
                var = {'name' : 'T',
                       'value' : f'T_{num}@{T_fct}',
                       'type' : ''}
                varss.append(var)
            else:
                var = {'name' : 'T',
                       'value' : f'T@{T_fct}',
                       'type' : ''}
                varss.append(var)
    
    descrq =f'Vector of length {len(params)} containing all the degrees of '+\
        'freedom of the robot that have '+\
        'an effect on the center of mass position. This vector contains :'
    for i_p, param in enumerate(params):
        descrq += f'\n        - q[{i_p+language.indexing_0}] = ' + \
            param['name']
        descrq += f' :\n              ' + param['description']
    
    paramq = {'name' : 'q', 'type' : 'vect', 'description' : descrq}
    
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
    
    for i_v, var in enumerate(varss):
        for i_p, param in enumerate(params):
            if not varss[i_v]['value'][0] == '#':
                varss[i_v]['value'] = replace_var(var['value'], param['name'],
                                             f'q[{i_p+language.indexing_0}]')
            
    docstr='Returns the center of mass of the robot in the root link frame.'+\
        ' The center of mass of the whole structure is computed. The result'+\
        f' is returned as a (4 x 1) {language.matrix_type} in homogeneous '+\
        'coordinates. The first three coordinates represent the X, Y and Z '+\
        'positions of the CoM and the 4th coordinate is always equal to 1'
    
    code += language.generate_fct('com', [paramq], expr[1:], varss, docstr,
                                  matrix_dims=(1,1))
    return code

# Generate Center of Mass Jacobian ___________________________________________

def generate_com_jacobian(robot, language=Language('python')):
    """
    Generate the code for the jacobian of the center of mass of the robot

    Parameters
    ----------
    robot : createRobotFromURDF.Robot
        Robot you want to generate the center of mass jacobian function from
    
    language : Language.Language, optional
        Language you want the code to be generated to
        Defalut is Language('python')

    Returns
    -------
    str : 
        Code of the CoM Jacobian function in the desired language

    """
    
    # Adding Title
    code = language.title("Jacobian of the Center of Mass of the Robot", 0)
    code += '\n\n'
    
    # Total Mass of the robot ................................................
    
    mass = 0
    nb_dof = 0
    for link in robot.links:
        if link.mass != 0:
            mass+=link.mass
            nb_dof += 1
    
    if mass==0:
        return language.comment_line + ' Center of mass jacobian function '+\
            'can not be generated because the robot mass is null.'
    
    # Tree iteration .........................................................
    
    varss = []
    
    # Init Jacobian
    varss.append({'name': 'Jac',
                  'value' : f"_zeros_3_{nb_dof}_",
                  'type' : 'mat'})
    
    # Initial transformation
    var = {'name'  : 'T',
           'value' : '_eye_4_4_',
           'type'  : 'mat'}
    
    varss.append(var)
    
    saved_joints_T = []
    params = []
    expr = 'Jac'
    l_declared = False
    z_declared = False
    i_jac = language.indexing_0
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
            pos_val =f'#mat#4#1#{str(cm[0,0])}#{str(cm[1,0])}#'+\
                f'{str(cm[2,0])}#1.0#'
            pos_var = {'name' : f'com_{obj_nb}_xyz',
                       'value' : pos_val,
                       'type' : 'vect'}
            varss.append(pos_var)
            var = {'name' : 'com',
                   'value' : f'{relative_mass}*T@com_{obj_nb}_xyz',
                   'type' : 'vect'}
            varss.append(var)
            l_type = '' if l_declared else 'vect'
            var_l = {'name' : 'L',
                     'value' :  f'com0[{language.indexing_0}:' +\
                     f'{language.indexing_0+2+language.subscription}'+\
                         f']-com[{language.indexing_0}:' +\
                     f'{language.indexing_0+2+language.subscription}]',
                     'type' : l_type}
            l_declared = True
            varss.append(var_l)
            z_type = '' if z_declared else 'vect'
            var_z = {'name' : 'Z',
                     'value' : f'T[{language.indexing_0}:'+\
                         f'{language.indexing_0+2+language.subscription},'+\
                             f'{language.indexing_0+2}]',
                     'type' : z_type}
            z_declared = True
            varss.append(var_z)
            var_j = {'name' : f'Jac[{language.indexing_0}:'+\
                         f'{language.indexing_0+2+language.subscription}'+\
                             f',{i_jac}]',
                     'value' : 'cross(Z,L)',
                     'type' : ''}
            varss.append(var_j)
            last_u = len(varss)
            i_jac += 1
        
        elif obj_type == 'joint':
            joint = robot.joints[obj_nb]
            
            # Paramters
            all_sym = joint.T.free_symbols
            params_tmp = []
            for symbol in all_sym:
                
                param = {}
                param['name'] = str(symbol)
                param['type'] = 'double'
                category = param['name'].split('_')[0]
                descr = ''
                if category == 'd':
                    descr += 'Translation value (in meters) along the '
                    descr += param['name'][2:] + ' prismatic joint axis.'
                
                elif category == 'dx':
                    descr += 'Translation value (in meters) along the X axis'
                    descr += ' of the ' + param['name'][3:] + ' joint.'
                    
                elif category == 'dy':
                    descr += 'Translation value (in meters) along the Y axis'
                    descr += ' of the ' + param['name'][3:] + ' joint.'
                
                elif category == 'dz':
                    descr += 'Translation value (in meters) along the Z axis'
                    descr += ' of the ' + param['name'][3:] + ' joint.'
                
                elif category == 'dz':
                    descr += 'Translation value (in meters) along the Z axis'
                    descr += ' of the ' + param['name'][3:] + ' joint.'
                
                elif category == 'theta':
                    descr += 'Rotation value (in radians) around the '
                    descr += param['name'][6:] + ' joint axis.'
                
                elif category == 'roll':
                    descr += 'Rotation value (in radians) around the X axis'
                    descr += ' of the ' + param['name'][5:] + ' joint.'
                
                elif category == 'pitch':
                    descr += 'Rotation value (in radians) around the Y axis'
                    descr += ' of the ' + param['name'][6:] + ' joint.'
                
                elif category == 'yaw':
                    descr += 'Rotation value (in radians) around the Z axis'
                    descr += ' of the ' + param['name'][4:] + ' joint.'
                    
                param['description'] = descr
                
                params_tmp.append(param)
                params.append(param)
            T_fct = 'T_' + joint.name + '('
            
            params_tmp.sort(key=lambda x : x['name'])
            
            for i_p, par in enumerate(params_tmp):
                T_fct += par['name']
                if i_p < len(params_tmp) - 1:
                    T_fct += ','
                else:
                    T_fct += ')'
            
            if len(robot.links[joint.child].parent_joints) > 1:
                var2 = {'name' : f'T_{obj_nb}',
                        'value' : f'T@{T_fct}',
                        'type' : 'mat'}
                varss.append(var2)
                saved_joints_T.append(obj_nb)
                var = {'name' : 'T',
                       'value' : f'T_{obj_nb}',
                       'type' : ''}
                varss.append(var)
            
            elif any(x in robot.links[joint.parent].child_joints for x in\
                     saved_joints_T):
                num = robot.links[joint.parent].child_joints[0]
                var = {'name' : 'T',
                       'value' : f'T_{num}@{T_fct}',
                       'type' : ''}
                varss.append(var)
            else:
                var = {'name' : 'T',
                       'value' : f'T@{T_fct}',
                       'type' : ''}
                varss.append(var)
    
    descrq = 'Vector of all the degrees of freedom of the robot that have ' +\
        'an effect on the center of mass position. This vector contains :'
    for i_p, param in enumerate(params):
        descrq += f'\n        - q[{i_p+language.indexing_0}] = ' + \
            param['name']
        descrq += f' :\n              ' + param['description']
    
    paramq = {'name' : 'q', 'type' : 'vect', 'description' : descrq}
    
    param_com0 = {'name' : 'com0',
                  'type' : 'vect',
                  'description' : 'Point from which you want to compute the'+\
                      ' Jacobian of the center of Mass. This point is '+\
         'expressed in homogeneous coordinates as a (4 x 1)'+\
             f' {language.vector_type}. '+\
              'The first three coordinates represent the X, Y and Z '+\
        'positions of the CoM and the 4th coordinate must always be equal'+\
            ' to 1'}
    
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
    
    for i_v, var in enumerate(varss):
        for i_p, param in enumerate(params):
            varss[i_v]['value'] = replace_var(var['value'], param['name'],
                                             f'q[{i_p+language.indexing_0}]')
    paar = [paramq, param_com0]
    
    origin_name='world'
    dest_name = 'the center of mass'
            
    docstr='Returns the Jacobian of the center of mass of the robot. '+\
        'This matrix is ' +\
        f'returned as a (3 x {nb_dof}) matrix where every column is the' +\
        ' derivative of the position of the CoM (X, Y and Z) with respect'+\
            ' to a degree '+\
        'of freedom. The result is expressed in the root link frame. '+\
        f'    - The line 1 is the derivative of X position of {dest_name}'+\
        f' in the {origin_name} frame,\n'+\
        f'    - The line 2 is the derivative of Y position of {dest_name}'+\
        f' in the {origin_name} frame,\n'+\
        f'    - The line 3 is the derivative of Z position of {dest_name}'+\
        f' in the {origin_name} frame,\n'+\
        '    - The line 4 is the derivative of the roll orientation of'+\
        f' {dest_name} in the {origin_name} frame,\n'+\
        '    - The line 5 is the derivative of the pitch orientation of'+\
        f' {dest_name} in the {origin_name} frame,\n'+\
        '    - The line 6 is the derivative of the yaw orientation of'+\
        f' {dest_name} in the {origin_name} frame,\n'+\
            'Here is the list of all the derivative variables :'
    for i_p, param in enumerate(params):
        docstr += f'\n    - Column {language.indexing_0+i_p} : '+\
            f'{param["name"]}'
    
    code += language.generate_fct('jacobian_com',paar,expr, varss, docstr,
                                  matrix_dims=(1,1))
    return code

# Generate Everything ________________________________________________________

def generate_everything(robot, list_ftm, list_btm, list_fk, list_jac, com,
                        com_jac, language, filename):
    """
    Description
    -----------
    
    Generates  all  the  desired  functions in the file 'filename.xx' where xx
    stands for the language file extensiton.

    Parameters
    ----------
    
    robot : createRobotFromURDF.Robot
        Robot object for code generation
    
    list_ftm : list of str
        List of all the forward transition matrices to generate
        Every element is a string formatted like the nodes
        names of robot.tree :
            type_number
            
            where  type  is 'joint' (in this case) and number is the number of
            the considered Joint in robot.joints.
            
            Example : list_ftm=['joint_3', 'joint_2']

    list_ftm : list of str
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
    
    language : Language.Language
        Language of the generated code
    
    filename : str
        Name  of  the  output  file  containing  the  generated  code (without
        extension)

    Returns
    -------
    None.

    """
    
    # Openning the file in write mode
    
    with open(filename + '.' + language.extension, 'w') as f:
        header = "The code in file has been generated by URDFast Code "+\
        "Generator on " + datetime.now().strftime("%m/%d/%Y, %H:%M:%S") +\
        ". Consider testing this code before using it as errors remain "+\
        "possible. For more details, check out the github repository " +\
        "of this projets at https://github.com/Teskann/URDFast."
        
        code = language.comment_par_beg + '\n\n' + language.justify(header) +\
            '\n\n'+ language.comment_par_end + '\n\n' + language.header + '\n'
        
        code += generate_all_matrices(robot, list_ftm, list_btm, language)
        
        if list_fk != []:
            code += '\n\n'
            list_origin = []
            list_dest = []
            
            for fk in list_fk:
                list_origin.append(fk[0])
                list_dest.append(fk[1])
            
            code += generate_all_fk(robot, list_origin, list_dest, language)
            
        
        if list_jac != []:
            code += '\n\n'
            list_origin = []
            list_dest = []
            
            for fk in list_jac:
                list_origin.append(fk[0])
                list_dest.append(fk[1])
                
            code += generate_all_jac(robot, list_origin, list_dest, language)
        
        if com:
            code += '\n\n'
            code += generate_com(robot, language)
        
        if com_jac:
            code += '\n\n'
            code += generate_com_jacobian(robot, language)
        
        code += '\n'
        
        f.write(code)
        f.close()
    
if __name__ == '__main__':
    print("\n==================================\n")
    print("Test : Robot - example_1.urdf\n")
    urdf_obj = URDF("./Examples/example_1.urdf")
    robot_obj = Robot(urdf_obj)
    print(robot_obj)
    robot_obj.joints[0].T
    
    list_tm = ['joint_0', 'joint_1', 'joint_2']
    
    lang = Language('julia')
    s = ''
    s += generate_all_matrices(robot_obj, [], list_tm, language=lang)
    origins = ['link_0', 'joint_0']
    destinations = ['link_1' ,'link_3']
    s += '\n\n' + generate_all_fk(robot_obj, origins, destinations, lang)
    
    s += '\n\n' + generate_all_jac(robot_obj, origins, destinations, lang)
    
   # print(s)
    
    s+= '\n\n'+ generate_com(robot_obj, lang)
    
    s += '\n\n' + generate_com_jacobian(robot_obj, lang)
    with open("Output2."+lang.extension, "w") as text_file:
        text_file.write(s)