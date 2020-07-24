# -*- coding: utf-8 -*-
"""
Created on Sat Jun 20 14:50:36 2020

@author: Clément
"""

import re
from anytree import Node

# Getting operators with the same precedence _________________________________

def same_precedence_opers(op):
    """
    Description
    -----------
    
    Returns the list of all the operators having the same precedence as op

    Parameters
    ----------
    op : str
        String od the operator

    Returns
    -------
    list of str:
        list of string containing all operators with the same precedence
        
    Examples
    --------
    
    TODO

    """
    
    # List of operators with the same predecence
    list_op = [
        ['[]'],
        ['**'],
        ['+u', '-u', '~'],
        ['/'],
        ['//'],
        ['%'],
        ['*'],
        ['@'],
        ['-'],
        ['+'],
        ['<<', '>>'],
        ['&'],
        ['^'],
        ['|'],
        ['in', 'not in', 'is', 'is not', '<', '<=', '>', '>=', '!=',
         '=='],
        ['not'],
        ['and'],
        ['or'],
        ]
    
    for oper in list_op:
        if op in oper:
            return oper
    
    return []

# Getting operators with higher priority _____________________________________

def higher_priority_oper(op):
    """
    Description
    -----------
    
    Returns  all  the  operators  that  have  a  higher  priority than the
    operator op.
    
    Result is returned as a list with priority descending

    Parameters
    ----------
    op : str
        String of the Python operator

    Returns
    -------
    list : list of str
        List of strings containing operators having a higher priority
        level than the operator op
        
    Examples
    --------
    
    TODO
    """
    
    if op == '[]':
        return []
    if op == '**':
        return ['[]']
    if op == '-u' or op == '+u' or op=='~':
        return ['[]', '**']
    if op == '/':
        return ['[]', '**', '-u', '+u', '~']
    if op == '//':
        return ['[]', '**', '-u', '+u', '~', '/']
    if op == '%':
        return ['[]', '**', '-u', '+u', '~', '/',  '//']
    if op == '*':
        return ['[]', '**', '-u', '+u', '~', '/',  '//', '%']
    if op == '@':
        return ['[]', '**', '-u', '+u', '~', '/',  '//', '*', '%']
    if op == '-':
        return ['[]', '**', '-u', '+u', '~', '/', '//', '*', '%', '@']
    if op == '+':
        return ['[]', '**', '-u', '+u', '~', '/', '//', '*', '%', '@', '-']
    if op == '<<' or op == '>>':
        return ['[]', '**', '-u', '+u', '~', '/', '//', '*', '%', '+', '-',
                '@']
    if op == "&":
        return ['[]', '**', '-u', '+u', '~', '/', '//', '*', '%', '+', '-',
                '<<', '>>', '@']
    if op == "^":
        return ['[]', '**', '-u', '+u', '~', '/', '//', '*', '%', '+', '-',
                '<<', '>>', '&', '@']
    if op == "|":
        return ['[]', '**', '-u', '+u', '~', '/', '//', '*', '%', '+', '-',
                '<<', '>>', '&', '^', '@']
    if op == "==" or op == "!=" or op == '>' or op == '>=' or op == '<'\
        or op == '<=' or op == 'is' or op== 'is not' or op == 'in' or\
            op == 'not in':
        return ['[]', '**', '-u', '+u', '~', '/', '//', '*', '%', '+', '-',
                '<<', '>>', '&', '^', '|', '@']
    if op == 'not':
        return ['[]', '**', '-u', '+u', '~', '/', '//', '*', '%', '+', '-',
                '<<', '>>', '&', '^', '|', '==', '!=', '>', '>=', '<',
                '<=', 'is', 'is not', 'in', 'not in', '@']
    if op == 'and':
        return ['[]', '**', '-u', '+u', '~', '/', '//', '*', '%', '+', '-',
                '<<', '>>', '&', '^', '|', '==', '!=', '>', '>=', '<',
                '<=', 'is', 'is not', 'in', 'not in', 'not', '@']
    if op == 'or':
        return ['[]', '**', '-u', '+u', '~', '/', '//', '*', '%', '+', '-',
                '<<', '>>', '&', '^', '|', '==', '!=', '>', '>=', '<',
                '<=', 'is', 'is not', 'in', 'not in', 'not', 'and', '@']
    return []

# Catch operator _____________________________________________________________

def catch_operator(string, operator):
    '''
    Catches the whole sequence around an operator or inside a function string.
    
    Returns  a  list  containing  one list per match. Every sublist contains a
    tuple  containing  the beginning index and the end index of the operand so
    that string[beginning:end] matches exactly the operand.

    Parameters
    ----------
    string : str
        String in which one you want to catch operator
        
    operator : str
        operator to look for

    Returns
    -------
    List of list of tuple with 2 elements
        Returns a list containing one list per match. Every sublist contains a
        tuple containing  the beginning index and the end index of the operand
        so that string[beginning:end] matches exactly the operand.
        
    Examples
    --------
    
    TODO !!!
    '''
    
    # Unary Operator checker .................................................
    
    def check_unary(string, index):
        """
        Description
        -----------
        
        Checks if the operator at the char i of the string is unary or binary.
        
        An operator is considered as unary  if and only if the first non-space
        character on its left is an operator,  an  openning bracket, a coma or
        nothing.
        
        Parameters
        ----------
        
        string : str
            String containing a mathematical function
        index : int
            String index where the operator is located
            
        Returns
        -------
        
        str :
            String  of  the operator followed by 'u' if the operator is unary,
            operator else.
        
        Examples
        --------
        
        TODO
        
        """
        # '+' and '-' are the only unary / binary operators
        if string[index] in ['+', '-']:
        
            # If  we  are  at  the  beginning  of the string, there is no left
            # operand, the operator is thus unary
            if index == 0:
                return string[index] + 'u'
            
            # Finding  the  first  non-space  character  on  the left. This is
            # equivalent to find the last non-space character in the substring
            # going from 0 to i
            
            # Pattern matching every non-space character
            pattern = re.compile(r'[\S]')
            
            # Find, and keep the last occurence
            try:
                last_nspace_char = re.findall(pattern, string[0:index])[-1]
            except IndexError:      # if no match
                return string[index] + 'u'
            
            # If the char is an operand-like char => Binary
            if last_nspace_char.isalnum() or last_nspace_char in ')}]_':
                return string[index]
            
            # If the char is an operator-like char => Unary
            else:
                return string[index] + 'u'
            
        # If it's not a '+' or a '-', it's not ambiguous
        else:
            return string[index]
        
    # Finding a more than one char operator ..................................
        
    def find_whole_operator(string, index, forward=True):
        """
        Description
        -----------
        
        Returns  the  whole operator which at least one char is located at the
        char 'index' in the string 'string'

        Parameters
        ----------
        string : str
            String representing a mathematical function containing operators.
        index : int
            String index where a char of an operator is located
        forward : bool, optional
            If true, looks for the rest of the operator forward, else backward
            The default is True.

        Returns
        -------
        str
            String containing the whole operator
            
        Examples
        --------
        
        TODO

        """
        s = string[index]
        
        # Searching Forward  . . . . . . . . . . . . . . . . . . . . . . . . .
        
        if forward:
            
            # 1 char operators
            if s in '~%+-^|&@':
                return check_unary(string, index)
            
            # 2 chars operators
            if index + 1 < len(string):
                if s == '*':
                    if string[index+1] == '*':
                        return '**'
                    else:
                        return '*'
                if s == '/':
                    if string[index+1] == '/':
                        return '//'
                    else:
                        return '/'
                if s == '>':
                    if string[index+1] == '>':
                        return '>>'
                    elif string[index+1] == '=':
                        return '>='
                    else:
                        return '>'
                if s == '<':
                    if string[index+1] == '<':
                        return '<<'
                    elif string[index+1] == '=':
                        return '<='
                    else:
                        return '<'
                if s == '!' and string[index+1] == '=':
                    return '!='
                if s == '=' and string[index+1] == '=':
                    return '=='
                
        # Searching Backward . . . . . . . . . . . . . . . . . . . . . . . . .
        
        else:
            
            # One char operators
            if s in '~%+-^|&@':
                return check_unary(string, index)
            
            # 2 chars operators
            if index - 1 >=0:
                if s == '*':
                    if string[index-1] == '*':
                        return '**'
                    else:
                        return '*'
                if s == '/':
                    if string[index-1] == '/':
                        return '//'
                    else:
                        return '/'
                if s == '>':
                    if string[index-1] == '>':
                        return '>>'
                    else:
                        return '>'
                if s == '<':
                    if string[index-1] == '<':
                        return '<<'
                    else:
                        return '<'
                if s == '=' and string[index-1] == '!':
                    return '!='
                if s == '=' and string[index-1] == '=':
                    return '=='
                if s == '=' and string[index-1] == '<':
                    return '<='
                if s == '=' and string[index-1] == '>':
                    return '>='
        return ""
    
    # Main function ..........................................................
    
    if 'u' in operator:
        is_unary = True
    else:
        is_unary = False
    
    # List of commutative operators
    commutative_op = ['+', '*']
    
    # Right-to-left associative operators
    right_to_left_asso_op = ['**']
    
    # List containing index of commutative operators with mort than 2 operands
    extra_operand_indices = []
        
    # Matches exactly the operator sequence if it's surrounded by a word char,
    # a spacing char or an openning / closing bracket
    op = r"((?<=([\s\w\]\[\)\(,]))|(?<=^))"
    for char in operator.replace('u', ''):
        op += '[' + char + ']'
    op += r"(?=([\s\w\]\[\)\(,]))"
    spec = re.compile(op)
    
    # Finding all the operators
    all_op = spec.finditer(string)
    
    all_matches = []
    
    # Find the left and right members of the operator  . . . . . . . . . . . .
    
    for num_oper, oper in enumerate(all_op):
        
        if find_whole_operator(string, oper.span()[0]) != operator:
            continue
        
        extra_operand_indices.append([])
        
        # Managing the left operand
        # Seraching the beginning index of the number / variable
        # Iterating for i from span[0] to 0
        beg_left_index = 0
        i = oper.span()[0] - 1
        
        # Until the begenning of the string
        while i >= 0:
            
            # If there is a closing parenthesis, we catch the openning one
            if string[i] == ")":
                closebr=1
                i-=1
                while closebr !=0:
                    if string[i] == ')':
                        closebr+=1
                    elif string[i] == '(':
                        closebr-=1
                    if i==0:
                        break
                    i-=1
                    
            # If there is a closing bracket, we catch the openning one
            if string[i] == "]":
                closebr=1
                i-=1
                while closebr !=0:
                    if string[i] == ']':
                        closebr+=1
                    elif string[i] == '[':
                        closebr-=1
                    if i==0:
                        break
                    i-=1
                    
            # if there is a closing curly brace, we catch the openning one
            if string[i] == "}":
                closebr=1
                i-=1
                while closebr !=0:
                    if string[i] == '}':
                        closebr+=1
                    elif string[i] == '{':
                        closebr-=1
                    if i==0:
                        break
                    i-=1
                    
            # If we bump into an operator-like char
            if not (string[i].isalnum() or string[i] in "]. ){}_"):
                
                # Find whole operator searching backward (on the left)
                o = find_whole_operator(string, i, False)
                
                # Checking if the operator is the same as 'operator'
                if operator == o and operator in commutative_op:
                    extra_operand_indices[num_oper].append(i)
                elif operator == o and operator in right_to_left_asso_op:
                    beg_left_index = i+1
                    break
                
                # Checking if the operator has a higher priority level
                elif o in higher_priority_oper(operator) or o in \
                    same_precedence_opers(operator):
                    
                    # If so, we skip it and continue searching
                    i-=len(o.replace('u', ''))-1
                    
                else:
                    
                    # If  not,  i  is  the  index of the beginning of the left
                    # operand
                    beg_left_index = i+1 # beginning is right after this one
                    break
                
            # Going to the next char on the left
            i-=1

        # Finding the right operand  . . . . . . . . . . . . . . . . . . . . .

        # Managing the right operand
        # Seraching the ending index of the number / variable / expression
        # Iterating for span[1] to end of string
        end_right_index = len(string)
        i = oper.span()[1]
        
        # Until the end of the string
        while i < len(string):
            
            # If there is an openning parenthesis, we catch the closing one
            if string[i] == "(":
                closebr=1
                i+=1
                while closebr !=0:
                    if string[i] == '(':
                        closebr+=1
                    elif string[i] == ')':
                        closebr-=1
                    if i==len(string)-1:
                        break
                    i+=1
                    
            # If there is an openning bracket, we catch the closing one
            if string[i] == "[":
                closebr=1
                i+=1
                while closebr !=0:
                    if string[i] == '[':
                        closebr+=1
                    elif string[i] == ']':
                        closebr-=1
                    if i==len(string)-1:
                        break
                    i+=1
                    
            # If there is an openning curly brace, we catch the closing one
            if string[i] == "{":
                closebr=1
                i+=1
                while closebr !=0:
                    if string[i] == '{':
                        closebr+=1
                    elif string[i] == '}':
                        closebr-=1
                    if i==len(string)-1:
                        break
                    i+=1
                    
            # If we bump into an operator-like char
            if not (string[i].isalnum() or string[i] in "[.({} _"):
                
                # Find the whole operator searching froward (on the right)
                o = find_whole_operator(string, i)
                
                # Checking if the operator is the same as 'operator'
                if operator == o and operator in commutative_op:
                    extra_operand_indices[num_oper].append(i)
                elif operator == o and operator not in right_to_left_asso_op:
                    end_right_index = i
                    break
                
                # Checking if the operator has a higher priority level
                elif o in higher_priority_oper(operator) or o in \
                    same_precedence_opers(operator):
                    
                    # If so, we skip it and continue searching
                    i+=len(o.replace('u', ''))-1
                    
                else:
                    
                    # If not, i is the index of the end of the right operand
                    end_right_index = i
                    break
                
            # Going to the next char on the right
            i+=1
        
        # [left begin, left end, right begin, right end]
        if is_unary:
            all_matches.append([[oper.span()[0], oper.span()[0]], 
                                [oper.span()[1], end_right_index]])
        else:
            all_matches.append([[beg_left_index, oper.span()[0]], 
                                [oper.span()[1], end_right_index]])
    
    # Splitting at every same operator
    for i, _ in enumerate(all_matches):
        extra_operand_indices[i].sort()
        for j, operand in enumerate(all_matches[i]):
            for extra_op in extra_operand_indices[i]:
                if operand[0] < extra_op < operand[1]:
                    all_matches[i].append([extra_op+1, operand[1]])
                    all_matches[i][j][1] = extra_op
            all_matches[i][j] = tuple(all_matches[i][j])
        all_matches[i].sort()
    return [list(item) for item in set(tuple(row) for row in all_matches)]

# Find all functions in the string ___________________________________________

def find_all_functions(string, char='('):
    """
    Description
    -----------
    
    Finds all the function calls in the string. Functions calls are defined
    by a word character (\w) followed by an openning parenthesis.
    
    Parameter
    ---------
    
    string : str
        String containing a mathematical function
    
    char : str, optional default is '('
        '(' for functions, '[' for lists
    
    Returns
    -------
    
    set of str :
        Set containing all the names of the functions called in the string
        
    Examples
    --------
    
    TODO
    
    """
    
    # Functions are at least one word char followed by '('
    if char == '(':
        pattern = re.compile(r'\w+(?=([\(]))')
    else:
        pattern = re.compile(r'\w+(?=([\[]))')
    
    all_matches = re.finditer(pattern, string)
    
    all_functions = []
    
    for match in all_matches:
        all_functions.append(match.group(0))
    
    return set(all_functions)

# Catch function calls in the string _________________________________________

def catch_function(string, func_name, char = '('):
    """
    Description
    -----------
    
    Returns all the occurences of the 'func_name' function calls in the string
    
    Parameters
    ----------
    
    string : str
        String  of  the  mathematical  function in which you want to catch the
        function
    func_name : str
        Name of the function you want to catch
    char : str, optional, default is '('
        '(' for functions, '[' for lists subscriptions
    
    Returns
    -------
    
    list of list of tuple of int :
        List of int containing all the function calls :
            [[(beg_arg_0_0, end_arg_0_0), ... (beg_arg_k_0, end_arg_k_0))]
                          .
                          . For m function calls
                          .
            [(beg_arg_0_n, end_arg_0_n), ... (beg_arg_k_n, end_arg_k_n))]
        
    Examples
    --------
    
    TODO
    
    """
    
    # Function call pattern (function name + openning parenthesis not preceded
    # by a word charcter or at the beginnning of the string)
    if char == '(':
        pattern = re.compile(r'(?:^|(?<=(\W)))'+func_name + r'(?=([\(]))')
    else:
        pattern = re.compile(r'(?:^|(?<=(\W)))'+func_name + r'(?=([\[]))')
    
    other_char = '(' if char == '[' else '['
    
    other_char_close = ')' if char == '[' else ']'
    
    char_close = ')' if char == '(' else ']'
    
    all_matches = re.finditer(pattern, string)
    
    all_func_calls = []
    
    for m, match in enumerate(all_matches):
        all_func_calls.append([])
        i = match.span()[1]   # Index of openning parenthesis
        closebr=1
        i+=1
        beg_index = i
        end_index = i
        while closebr !=0:
            
            # If there is an openning bracket, we catch the closing one
            if string[i] == other_char:
                closebra=1
                i+=1
                while closebra !=0:
                    if string[i] == other_char:
                        closebra+=1
                    elif string[i] == other_char_close:
                        closebra-=1
                    if i==len(string)-1:
                        break
                    i+=1
                    
            # If there is an openning curly brace, we catch the closing one
            if string[i] == "{":
                closebra=1
                i+=1
                while closebra !=0:
                    if string[i] == '{':
                        closebra+=1
                    elif string[i] == '}':
                        closebra-=1
                    if i==len(string)-1:
                        break
                    i+=1
            
            # New argument
            if string[i] == ',' and closebr == 1:
                end_index = i
                all_func_calls[m].append((beg_index, end_index))
                beg_index = i+1
            if string[i] == char:
                closebr+=1
            elif string[i] == char_close:
                closebr-=1
            if i==len(string)-1:
                i+=1
                break
            i+=1
        all_func_calls[m].append((beg_index, i-1))
        
    return all_func_calls

# Find all lists (special operator) __________________________________________

def find_all_lists(string):
    """
    Description
    -----------
    
    Finds all the lists operations with the operator []
    
    Parameter
    ---------
    
    string : str
        String containing a mathematical function
    
    Returns
    -------
    
    list of list of tuple of int :
        List of int containing all the lists :
            [[(beg_arg_0_0, end_arg_0_0), ... (beg_arg_k_0, end_arg_k_0))]
                          .
                          . For m list operations
                          .
            [(beg_arg_0_n, end_arg_0_n), ... (beg_arg_k_n, end_arg_k_n))]
        
    Examples
    --------
    
    TODO
    
    """
    
    # Matching openning brackets
    pattern = re.compile(r'(?:^|(?<=(\W)))' + r'\[')
    
    all_matches = re.finditer(pattern, string)
    
    all_lists = []
    
    for m, match in enumerate(all_matches):
        all_lists.append([])
        i = match.span()[0]   # Index of openning bracket
        closebr=1
        i+=1
        beg_index = i
        end_index = i
        while closebr !=0:
            
            # If there is an openning bracket, we catch the closing one
            if string[i] == "(":
                closebra=1
                i+=1
                while closebra !=0:
                    if string[i] == '(':
                        closebra+=1
                    elif string[i] == ')':
                        closebra-=1
                    if i==len(string)-1:
                        break
                    i+=1
                    
            # If there is an openning curly brace, we catch the closing one
            if string[i] == "{":
                closebra=1
                i+=1
                while closebra !=0:
                    if string[i] == '{':
                        closebra+=1
                    elif string[i] == '}':
                        closebra-=1
                    if i==len(string)-1:
                        break
                    i+=1
            
            # New element
            if string[i] == ',' and closebr == 1:
                end_index = i
                all_lists[m].append((beg_index, end_index))
                beg_index = i+1
            if string[i] == '[':
                closebr+=1
            elif string[i] == ']':
                closebr-=1
            if i==len(string)-1:
                i+=1
                break
            i+=1
        all_lists[m].append((beg_index, i-1))
    return all_lists

# Find every operation and function __________________________________________

def find_everything(string):
    """
    Description
    -----------
    
    Returns all the operations in the string
    
    Parameter
    ---------
    
    string : str
        Mathematical expression
    
    Returns
    -------
    
    list of dict :
        List of all the operations in the same order than the tree list.
        Every dict is formatted like this :
            'operator' : operator string or function name
            'operation' : dict formatted like this :
                'indices' : list of tuple containing the beginning and the end
                indices  of  all  the  operands / arguments of the operation /
                function
                'str_val' : string value of the arguments / operands
                'len' : length of the operation
            'is_fct' : True if it is a funciton, false if it's an operator
    
    """
    # List containing all func calls and operations (no key)
    all_operations = []
    
    string = ' ' + string.replace(' ', '') + '  '
    
    # All functions ..........................................................
    
    all_fct = find_all_functions(string)
    
    # Element of all_operations
    operation = {'operator' : '', 'operation' : {}, 'is_fct' : False}
    
    for fct in all_fct:
        
        # Getting all calls of this function
        all_fct_calls = catch_function(string, fct)
        
        for call in all_fct_calls:
            dict_oper = {'indices' : [], 'str_val' : [], 'len' : 0,
                         'children' : []}
            for arg in call:
                dict_oper['indices'].append(arg)
                dict_oper['str_val'].append(string[arg[0]:arg[1]])
                dict_oper['children'].append(None)
                
            # Length of the function call
            dict_oper['len'] = dict_oper['indices'][-1][1] - \
                dict_oper['indices'][0][0]
           
                
            operation['operation'] = dict_oper.copy()
            operation['operator'] = fct
            operation['priority'] = fct
            operation['is_fct'] = True
            
            all_operations.append(operation.copy())
    
    # All subscriptions ......................................................
    
    all_fct = find_all_functions(string, char='[')
    
    # Element of all_operations
    operation = {'operator' : '', 'operation' : {}, 'is_fct' : False}
    
    for fct in all_fct:
        
        # Getting all calls of this function
        all_fct_calls = catch_function(string, fct, char='[')
        
        for call in all_fct_calls:
            dict_oper = {'indices' : [], 'str_val' : [], 'len' : 0,
                         'children' : []}
            for arg in call:
                dict_oper['indices'].append(arg)
                dict_oper['str_val'].append(string[arg[0]:arg[1]])
                dict_oper['children'].append(None)
                
            # Length of the function call
            dict_oper['len'] = dict_oper['indices'][-1][1] - \
                dict_oper['indices'][0][0]
           
                
            operation['operation'] = dict_oper.copy()
            operation['operator'] = fct+'[]'
            operation['priority'] = '[]'
            operation['is_fct'] = True
            
            all_operations.append(operation.copy())
    
    # All lists  .............................................................
    
    all_lists = find_all_lists(string)

    for lis in all_lists:
        dict_oper = {'indices' : [], 'str_val' : [],'len' : 0,
                     'children' : []}
        
        for elem in lis:
            dict_oper['indices'].append(elem)
            dict_oper['str_val'].append(string[elem[0]:elem[1]])
            dict_oper['children'].append(None)
        
        # Length of the operation
        dict_oper['len'] = dict_oper['indices'][-1][1] - \
            dict_oper['indices'][0][0]
        
        operation['operation'] = dict_oper.copy()
        operation['operator'] = '[]'
        operation['priority'] = '[]'
        operation['is_fct'] = False
        
        all_operations.append(operation.copy())
                
    # Retrieving all operators ...............................................
    
    for operator in ['+', '-', '*', '@', '/', '**', '-u', '+u']:
        
        # Getting all operations with this operator
        all_op_calls = catch_operator(string, operator)
        
        for call in all_op_calls:
            dict_oper = {'indices' : [], 'str_val' : [],'len' : 0,
                         'children' : []}
            for operand in call:
                dict_oper['indices'].append(operand)
                dict_oper['str_val'].append(string[operand[0]:operand[1]])
                dict_oper['children'].append(None)
            
            # Length of the operation
            dict_oper['len'] = dict_oper['indices'][-1][1] - \
                dict_oper['indices'][0][0]
            
            operation['operation'] = dict_oper.copy()
            operation['operator'] = operator
            operation['priority'] = operator
            operation['is_fct'] = False
            
            all_operations.append(operation.copy())
    
    return all_operations
    
# Build the operations tree __________________________________________________

def get_tree(operations_list):
    """
    Description
    -----------
    
    Create  the  operations  /  functions  call  tree  from  a list of dift of
    operations
    
    Parameter
    ---------
    
    operations_list : list of dict
        List of all the operations from whiches you want to create the tree.
        Every dict is formatted like this :
            'operator' : operator string or function name
            'operation' : dict formatted like this :
                'indices' : list of tuple containing the beginning and the end
                indices  of  all  the  operands / arguments of the operation /
                function
                'str_val' : string value of the arguments / operands
                'len' : length of the operation
            'is_fct' : True if it is a funciton, false if it's an operator
    
    Returns
    -------
    
    list of anytree.node.node.Node
        List of all Nodes of operations (linked)
    """
    
    # List of all index of operations / functions that have already a Node
    have_a_node = []
    
    # Find the root operation ................................................
    
    def find_leaf(min_index, max_index, exclude_op, general=False):
        """
        Description
        -----------
        
        Returns the index of the leaf  operation between the indices min_index
        and max_index
        
        Parameters
        ----------
        
        min_index : int
            Minimum index
        max_index : int
            Maximum index
        exclude_op : int
            Index of the operation you want to ignore
        general : bool
            If True, the three above arguments are ignored
            Default is False
            
        Returns
        -------
        
        Index of the leave (longest) operation between the 2 indices
        None if there is no leaf
        
        Global Variables Used
        ---------------------
        
        - operations_list
        
        """
        max_len = -1
        index = None
        for i_op, op in enumerate(operations_list):
            # Checking the index
            if not general:
                if op['operation']['indices'][0][0] < min_index or\
                    op['operation']['indices'][-1][1] > max_index or\
                        i_op == exclude_op:
                            continue
            if op['operation']['len'] > max_len:
                max_len = op['operation']['len']
                index = i_op
        return index
    
    # Main function ..........................................................
    
    all_nodes = []
    
    # Creating a Node per operation
    for i_op, op in enumerate(operations_list):
        all_nodes.append(Node(str(i_op)))
        
    # Linking Nodes toghther
    for i_op, op in enumerate(operations_list):
        for i_i, leaf_index in enumerate(op['operation']['indices']):
            leaf = find_leaf(leaf_index[0], leaf_index[1], i_op)
            if leaf is not None:
                all_nodes[leaf].parent = all_nodes[i_op]
                operations_list[i_op]['operation']['children'][i_i] = leaf
    return all_nodes

# Operation to string ________________________________________________________

def render(operation_dict, parentheses=False):
    """
    Description
    -----------
    
    Converts the operation dict to the string representation
    
    Parameter
    ---------
    
    operation_dict : dict
        Operation you want to convert to string
        Every dict is formatted like this :
            'operator' : operator string or function name
            'operation' : dict formatted like this :
                'indices' : list of tuple containing the beginning and the end
                indices  of  all  the  operands / arguments of the operation /
                function
                'str_val' : string value of the arguments / operands
                'len' : length of the operation
            'is_fct' : True if it is a funciton, false if it's an operator
            
    Returns
    -------
    
    str
        String representation of the operation
    parentheses : bool, optional
        If True, add parentheses around the expression. Default is False
    
    Examples
    --------
    
    TODO
    
    """
    # Rendering functions ....................................................
    
    if operation_dict['is_fct']:
        
        # Subscription
        if '[]' in operation_dict['operator']:
            string = operation_dict['operator'][:-2] + '['
        
        # Function name
        else:
            string = operation_dict['operator'] + '('
        
        # Arguments
        for i, arg in enumerate(operation_dict['operation']['str_val']):
            string += arg
            
            if i == len(operation_dict['operation']['str_val']) - 1:
                string += ']' if '[]' in operation_dict['operator'] else ')'
            else:
                string += ','
        
    # Rendering operations ...................................................
    
    else:
        # If it is a list
        if operation_dict['operator'] == '[]':
            string = '['
            
            # Elements
            for i, elem in enumerate(operation_dict['operation']['str_val']):
                string += elem
                
                if i == len(operation_dict['operation']['str_val']) - 1:
                    string += ']'
                else:
                    string += ','
            return string
        
        # Classic operator
        string = '(' if parentheses else ''
        
        for i, operand in enumerate(operation_dict['operation']['str_val']):
            string += operand
            
            if i != len(operation_dict['operation']['str_val']) - 1:
                string += operation_dict['operator'].replace('u', '')
        
        string += ')' if parentheses else ''
        
    return string.replace(' ', '')

# Render whole expression from tree __________________________________________

def render_from_tree(tree, all_op):
    """
    Description
    -----------
    
    Render the whole expression from a tree representation
    
    Parameters
    ----------
    
    tree : list of anytree.node.node.Node
        List of all Nodes of operations (linked)
    
    all_op : list of dict
        List of all the operations in the same order than the tree list.
        Every dict is formatted like this :
            'operator' : operator string or function name
            'operation' : dict formatted like this :
                'indices' : list of tuple containing the beginning and the end
                indices  of  all  the  operands / arguments of the operation /
                function
                'str_val' : string value of the arguments / operands
                'len' : length of the operation
            'is_fct' : True if it is a funciton, false if it's an operator
    
    Returns
    -------
    
    str :
        String expression of the tree representation
    
    """
    
    # 1 - Copying the list of operations .....................................
    
    def copy(old):
        return {'operator' : old['operator'],
                'priority' : old['priority'],
               'operation': {
                    'indices' : old['operation']['indices'].copy(),
                    'str_val' : old['operation']['str_val' ].copy(),
                    'len' : old['operation']['len'],
                    'children' : old['operation']['children'].copy()
                    },
               'is_fct' : old['is_fct']
               }
    
    # Finding root of the tree
    i_root = 0
    for i_n, node in enumerate(tree):
        if node.is_root:
            i_root = i_n
            break
    
    def recursive_render(index):
        if tree[index].is_leaf:
            if tree[index].parent is None:
                par = False
            elif all_op[int(tree[index].name)]['is_fct']:
                par = False
            elif all_op[int(tree[index].parent.name)]['priority'] == '[]':
                par = False
            elif all_op[int(tree[index].parent.name)]['priority'] in \
                higher_priority_oper(all_op[int(tree[index].name)]\
                                     ['priority']) or \
                all_op[int(tree[index].parent.name)]['priority'] in \
                same_precedence_opers(all_op[int(tree[index].name)]\
                                     ['priority']):
                    par = True
            else:
                par = False
            return render(all_op[index], parentheses = par)
        else:
            operands = []
            for i_c, child in enumerate(all_op[index]['operation']\
                                        ['children']):
                if child is not None:
                    operands.append(recursive_render(child))
                else:
                    operands.append(all_op[index]['operation']\
                                    ['str_val'][i_c])
            operation = copy(all_op[index])
            operation['operation']['str_val'] = operands
            
            if tree[index].parent is None:
                par = False
            
            elif all_op[int(tree[index].name)]['is_fct']:
                par = False
            
            elif all_op[int(tree[index].parent.name)]['priority'] == '[]':
                par = False
            
            elif all_op[int(tree[index].parent.name)]['priority'] in \
                higher_priority_oper(all_op[int(tree[index].name)]\
                                     ['priority']) or \
                all_op[int(tree[index].parent.name)]['priority'] in \
                same_precedence_opers(all_op[int(tree[index].name)]\
                                     ['priority']):
                    par = True
            else:
                par = False
            return render(operation, parentheses = par)

    return recursive_render(i_root)

# Replace an operator / function by another __________________________________

def replace(string, operator, new_operator):
    """
    Description
    -----------
    
    Replace  an  operator  /  function  in an expression by another operator /
    function in the string.  May  be  useful  to  convert an expression from a
    language to another.
    
    Parameters
    ----------
    
    string : str
        String representing a mathematical expression
    
    operator : list
        Operator / function you want to replace
        List that contains :
            - At index 0 => operator string value
            - At index 1 => bool set to true if it is a function
    new_operator : list
        New value of the function / operation
        List that contains :
            - At index 0 => operator string value
            - At index 1 => bool set to true if it is a function
    
    Returns
    -------
    
    str :
        Copy of the string with the substitution done
    
    Examples
    --------
    
    TODO
    
    """
    
    all_op = find_everything(string)
    
    if len(all_op) == 0:
        return string
    
    tree = get_tree(all_op)
    
    for op in all_op:
        if op['operator'] == operator[0] and op['is_fct'] == operator[1]:
            op['operator'] = new_operator[0]
            op['is_fct'] = new_operator[1]
        
    return render_from_tree(tree, all_op)

# Replace a variable _________________________________________________________

def replace_var(string, var, new_var):
    """
    Description
    -----------
    
    Replace a variable by another variable or expression
    
    Parameters
    ----------

    string : str
        Mathematical expression string
    var : str
        Variable you want to replace
    new_var : str
        New value of the replaced variable

    Returns
    -------
    str :
        Value of the new mathematical expression

    """
    
    all_op = find_everything(string)
    
    if len(all_op) == 0:
        return new_var if string == var else string
    
    tree = get_tree(all_op)
    
    for op in all_op:
        for i_a, arg in enumerate(op['operation']['str_val']):
            if arg == var:
                op['operation']['str_val'][i_a] = new_var
        
    return render_from_tree(tree, all_op)

# Optimize a function ________________________________________________________

def optimize(string):
    """
    Description
    -----------
    
    Optimize a string function finding redundant operations.
    Returns all intermediate variables and the simplified expression
    
    Function only supports operators :
        +
        -
        *
        /
        **
    and every function call with eventually many parameters.
    
    Parameter
    ---------
    
    string : str
        String representing a mathematical expression
        
    Returns
    -------
    
    variables : list of dict of strings
        List containing the definition of every variable
        Every element of this list is a dict containing keys :
            - 'name' (str) : variable name
            - 'value' (str) : value of this variable
    expression : str
        New expression of the function string
    
    Examples
    --------
    
    TODO
    
    """
    
    string = string.replace(' ', '')
    string = ' ' + string + ' '
    
    # Retrieving all function calls ..........................................
    
    # List containing all func calls and operations (no key)
    all_operations = []
    
    all_fct = find_all_functions(string)
    
    # Element of all_operations
    operation = {'operator' : '', 'operation' : {}, 'is_fct' : False}
    
    for fct in all_fct:
        
        # Getting all calls of this function
        all_fct_calls = catch_function(string, fct)
        
        for call in all_fct_calls:
            dict_oper = {'indices' : [], 'str_val' : [], 'len' : 0,
                         'children' : []}
            for arg in call:
                dict_oper['indices'].append(arg)
                dict_oper['str_val'].append(string[arg[0]:arg[1]])
                dict_oper['children'].append(None)

            # Length of the function call
            dict_oper['len'] = dict_oper['indices'][-1][1] - \
                dict_oper['indices'][0][0]
                
            operation['operation'] = dict_oper.copy()
            operation['operator'] = fct
            operation['priority'] = fct
            operation['is_fct'] = True
            
            all_operations.append(operation.copy())
                
                
    # Retrieving all operators ...............................................
    
    for operator in ['+', '-', '*', '@', '/', '**', '-u', '+u']:
        
        # Getting all operations with this operator
        all_op_calls = catch_operator(string, operator)
        
        for call in all_op_calls:
            dict_oper = {'indices' : [], 'str_val' : [],'len' : 0,
                         'children' : []}
            for operand in call:
                dict_oper['indices'].append(operand)
                dict_oper['str_val'].append(string[operand[0]:operand[1]])
                dict_oper['children'].append(None)
            
            # Length of the operation
            dict_oper['len'] = dict_oper['indices'][-1][1] - \
                dict_oper['indices'][0][0]
            
            operation['operation'] = dict_oper.copy()
            operation['operator'] = operator
            operation['priority'] = operator
            operation['is_fct'] = False
            
            all_operations.append(operation.copy())
    
    # Processing the tree representation .....................................
    
    tree = get_tree(all_operations)
    
    render_from_tree(tree, all_operations)
    
    # Find redundant computations ............................................
    
    # Redundancies renderings
    all_redundancies = []
    
    # Redundant operators / function names
    all_redundancies_oper = []
    
    all_redundancies_oper_nodes = []
    
    all_redundancies_oper_no_dbl = []
    
    all_redundancies_oper_no_dbl_nodes = []
    
    def copy(old):
        return {'operator' : old['operator'],
               'operation': {
                    'indices' : old['operation']['indices'].copy(),
                    'str_val' : old['operation']['str_val' ].copy(),
                    'len' : old['operation']['len']
                    },
               'is_fct' : old['is_fct']
               }

    for i_n, node in enumerate(tree):
        for i_n_, _ in enumerate(tree[i_n+1:]):
            i_n_2 = i_n_ + i_n+1
            if i_n_2 != i_n:
                if all_operations[i_n]['operator'] == \
                    all_operations[i_n_2]['operator']:
                    if all_operations[i_n]['operator'] in ['+','*']:
                        print('TOTO +* !!!!!!!!!!!!!!!!!!!!!')
                        pass
                    else:
                        if render(all_operations[i_n]) == \
                            render(all_operations[i_n_2]):
                                
                            if  render(all_operations[i_n]) not in \
                            all_redundancies:
                                
                                all_redundancies.append(render(\
                                    all_operations[i_n]))
                                all_redundancies_oper_no_dbl.append(\
                                    all_operations[i_n])
                                all_redundancies_oper_no_dbl_nodes.\
                                    append(i_n)
                                    
                            new = copy(all_operations[i_n])
                            
                            if new not in all_redundancies_oper:
                                all_redundancies_oper.append(new)
                                all_redundancies_oper_nodes.append(i_n)
                                
                            new = copy(all_operations[i_n_2])
                            if new not in all_redundancies_oper:
                                all_redundancies_oper.append(new)
                                all_redundancies_oper_nodes.append(i_n_2)
    
    all_redundancies_oper_no_dbl_2 = []
    for elem in all_redundancies_oper_no_dbl:
        all_redundancies_oper_no_dbl_2.append(copy(elem))
    
    all_operations_2 = []
    for elem in all_operations:
        all_operations_2.append(copy(elem))
    
    # Sorting redundant expressions in the order of definition
    sorted_tree = tree.copy()
    sorted_tree.sort(key=lambda x: x.depth, reverse=True)
    op_index = [int(x.name) for x in sorted_tree]
    sorted_oper = []
    sorted_oper_nodes = []
    sorted_oper_no_dbl = []
    sorted_oper_no_dbl_nodes = []
    sorted_oper_no_dbl_2 = []
    for i in op_index:
        
        for oper in all_redundancies_oper:
            if oper == all_operations[i]:
                sorted_oper.append(oper)
                
        for node in all_redundancies_oper_nodes:
            if node == i:
                sorted_oper_nodes.append(node)
                
        for oper in all_redundancies_oper_no_dbl:
            if oper == all_operations[i]:
                sorted_oper_no_dbl.append(oper)
                
        for node in all_redundancies_oper_no_dbl_nodes:
            if node == i:
                sorted_oper_no_dbl_nodes.append(node)
                
        for oper in all_redundancies_oper_no_dbl_2:
            if oper == all_operations[i]:
                sorted_oper_no_dbl_2.append(oper)
    
    all_redundancies_oper = sorted_oper
    all_redundancies_oper_no_dbl = sorted_oper_no_dbl
    all_redundancies_oper_no_dbl_2 = sorted_oper_no_dbl_2
    all_redundancies_oper_nodes = sorted_oper_nodes
    all_redundancies_oper_no_dbl_nodes = sorted_oper_no_dbl_nodes
    all_redundancies = []
    for i_op, op in enumerate(all_redundancies_oper):
        if render(op) not in all_redundancies:
            all_redundancies.append(render(op))
    
    # Variable Names .........................................................
    
    # All variables and function names
    all_names = list(all_fct.copy())
    
    # All variable names
    all_var_names = []
    
    # Getting names in the leaves of the tree
    for i_n, node in enumerate(tree):
        for var in all_operations[i_n]['operation']['str_val']:
            no_fct_calls = True
            # Checking if the operand has function calls
            for f in all_fct:
                if catch_function(var, f) != []:
                    no_fct_calls = False
                    break
            if no_fct_calls:
                # Checking if the operand has operations
                no_op_calls = True
                for o in ['+', '-', '*', '@', '/', '**', '-u', '+u']:
                    if catch_operator(var, o) != []:
                        no_op_calls = False
                        break
            # No fct and no op => variable or number
            if no_fct_calls and no_op_calls and var != '':
                # Keeping only variables (starting by a letter)
                if var[0].isalpha() or var[0] == '_' \
                    and var not in all_var_names:
                    all_var_names.append(var)
    all_var_names = list(set(all_var_names))
    
    # Appending variable names to all the defined var / func
    all_names += all_var_names
    
    def new_name(var_operator):
        """
        Description
        -----------
        
        Get a name for a variable which is not already used
        
        Parameter
        ---------
        
        var_operator : str
            Operator or function of the variable expression
        
        Returns
        -------
        
        str
            Variable name
        """
        
        prefix=''
        
        # Operator variable names
        if var_operator == '+':
            prefix = 'v_sum'
        elif var_operator == '-':
            prefix = 'v_sub'
        elif var_operator == '*':
            prefix = 'v_prod'
        elif var_operator == '/':
            prefix = 'v_div'
        elif var_operator == '**':
            prefix = 'v_exp'
        elif var_operator == '-u':
            prefix = 'v_minus'
        elif var_operator == '+u':
            prefix = 'v_plus'
            
        # Function variable name
        else:
            prefix = 'f_' + var_operator[0:3]
        
        while prefix in all_names:
            prefix += '_'
        
        suffix = '0'
        while prefix + suffix in all_names:
            suffix = str(int(suffix) + 1)
        
        return prefix + suffix
    
    # Dict of variable names with their expressions
    variable_names = {}
    
    for i, var in enumerate(all_redundancies):
        v_name = new_name(all_redundancies_oper_no_dbl[i]['operator'])
        variable_names[v_name] = var
        all_names.append(v_name)
        
    # Re-writing variable expressions ........................................
    # Variables sometimes depend on each other
    
    for i_var, var in enumerate(variable_names):
        rendering = variable_names[var].replace(' ','')
        for i_op, op in enumerate(all_operations):
            if render(op) == rendering:
                i_node= i_op
                rend = render(all_redundancies_oper_no_dbl[i_var])
                rendp = '(' + rendering + ')'
                subs = var
                
                print(rend,  '===>')

                # Til the root if necessary
                while tree[i_node].parent != None:
                    print(rend,  '=>', subs)
                    parent_node = tree[i_node].parent
                    
                    # Searching the operation parent in the redundancies list
                    parent_red_ind = None
                    no_dbl_ind = None
                    
                    # Exact parent
                    for i_r, red_op in enumerate(all_redundancies_oper_nodes):
                        if int(parent_node.name) == red_op:
                            parent_red_ind = i_r
                            print(parent_red_ind)
                            break
                    
                    # Render parent
                    for i_, d_op in enumerate(all_redundancies_oper_no_dbl):
                        if render(all_operations[int(parent_node.name)])\
                            == render(d_op):
                            no_dbl_ind = i_
                            break
                    # If the operation in not in the redundancies, we skip it
                    # (It means no other variable depends on 'var')
                    if parent_red_ind == None:
                        print(all_operations_2[int(parent_node.name)])
                        break
                    parent_op = all_redundancies_oper[parent_red_ind]
                    found = False
                    # Searching which operand leads to the rendering
                    for i, operand in enumerate(parent_op['operation']\
                                                ['str_val']):
                        if operand.replace(' ','') == rend or\
                            operand.replace(' ','') == rendp:
                            found = True
                            # Next expression to match
                            rend = render(parent_op)
                            rendp = '(' + rend + ')'
                            # Substitution
                            all_redundancies_oper[parent_red_ind]\
                                ['operation']['str_val'][i] = subs
                            all_operations_2[int(parent_node.name)]\
                                ['operation']['str_val'][i] = subs
                            if no_dbl_ind != None:
                                all_redundancies_oper_no_dbl[no_dbl_ind]\
                                ['operation']['str_val'][i] = subs
                            subs = render(parent_op, parentheses=True)
                            i_node = int(parent_node.name)
                            
                    # If the expression can't be found
                    if not found:
                        break
        for i, var_2 in enumerate(variable_names):
            variable_names[var_2] = render(all_redundancies_oper_no_dbl[i])
    
    # # Sorting redundant expressions in the order of definition
    # sorted_tree = tree.copy()
    # sorted_tree.sort(key=lambda x: x.depth, reverse=True)
    # op_index = [int(x.name) for x in sorted_tree]
    # sorted_oper = []
    # for i in op_index:
    #     for oper in all_redundancies_oper:
    #         if oper == all_operations[i]:
    #             sorted_oper.append(oper)
    
    # Re-rendering the variable values
    
    for i, var in enumerate(variable_names):
        variable_names[var] = render(all_redundancies_oper_no_dbl[i])
    
    # Re-writing function expression .........................................

    for var in variable_names:
        # Searching all operations leading to this rendering
        rendering = variable_names[var].replace(' ','')
        for i_op, op in enumerate(all_operations):
            if render(op) == rendering:
                # Substituing by the variable til the root
                i_node = i_op
                rend = rendering
                rendp = '(' + rendering + ')'
                subs = var
                while tree[i_node].parent != None:
                    parent_node = tree[i_node].parent
                    parent_op = all_operations[int(parent_node.name)]
                    found = False
                    # Searching which operand leads to the rendering
                    for i, operand in enumerate(parent_op['operation']\
                                                ['str_val']):
                        if operand.replace(' ','') == rend or\
                            operand.replace(' ','') == rendp:
                            found = True
                            # Next expression to match
                            rend = render(parent_op).replace(' ','')
                            rendp = '(' + rend + ')'
                            # Substitution
                            all_operations[int(parent_node.name)]\
                                ['operation']['str_val'][i] = subs
                            subs = render(parent_op, parentheses=True)
                            i_node = int(parent_node.name)
                            break
                    # If the expression can't be found
                    if not found:
                        break
    
    # Looking for the root
    max_height = 0
    root = 0
    for i, node in enumerate(tree):
        if node.height > max_height:
            max_height = node.height
            root = i
            
    expression = render(all_operations[root])
    
    print(all_redundancies, all_names, variable_names, '\n', expression)
    
    
                        
    return variable_names, expression
    

# ----------------------------------------------------------------------------
# | MAIN - RUNNING TESTS                                                     |
# ----------------------------------------------------------------------------

if __name__ == '__main__':
    
    # Testing with a function string _________________________________________
    
    func_str = '[abs(cos(a1**(3-6)**5, 5-4)*arccos(x**2+5+0.1))-8-5*t**s'+\
        '(k[0]/1)+a1**(3-6)**5*r*3, -6*R**3]'
    oper = '-u'
    matches = catch_operator(func_str, oper)
    print(func_str)
    for match in matches:
        print('\n'+oper+' :\n')
        for operand in match:
            print(func_str[operand[0]:operand[1]])
      
    print('\n\n\n================\n',find_all_functions(func_str))
    
    replace_var(func_str, 'ze','ee')
    
    func = 'cos'
    #func_str = func_str.replace(' ', '')
    func_calls = catch_function(func_str, func)
    
    for call in func_calls:
        print('\n'+func+' :\n')
        for arg in call:
            print(func_str[arg[0]:arg[1]])
    
    print('Lists : ____________________\n')
    lists = find_all_lists(func_str)
    for lis in lists:
        print('\n')
        for elem in lis:
            print(func_str[elem[0]:elem[1]])
            
    print('OP___________________________________________________\n')
            
    #liste, expr= optimize(func_str)
    
    test = replace(func_str, ['**', False], ['^', False])
    print(func_str)
    print(test)