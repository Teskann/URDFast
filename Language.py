# -*- coding: utf-8 -*-
"""
Created on Mon Jun 29 10:19:19 2020

@author: Clément
"""

from code_optimization import replace, replace_var, replace_many
import re


def indent(number):
    """
    Description
    -----------
    
    Returns 4*number of whitespace string
    
    Parameter
    ---------
    
    number : int
        Indentation value
    
    Returns
    -------
    
    str
        whitespaces to indent code generation
    
    Example
    -------
    
    >>> indent(2)
    '        '
    
    """

    ind = ''

    for _ in range(number):
        ind += '    '

    return ind


# Language object ____________________________________________________________

class Language:
    """
    Description
    -----------

    Langauge objects. Describe the languages to generate code.
    
    Member Data
    -----------
    
    name : str
        Language Name
    
    fct_prefix : str
        Function declaration string
        Example, for Python :
            "def function("
    
    fct_suffix : str
        Function declaration suffix
        Example, for Python "):"
    
    fct_end : str
        String marking the end of a function
        Example, for C / C++:
            "}"
    
    param_separator : str:
        String for function parameters serparation
        Example, for Python :
            ","
    max_line_length : int
        Maximum line length.
        Example for Python:
            79
    
    comment_line : str
        String to comment a line.
        Example for Python :
            "#"
    
    comment_par_beg : str
        String to comment a paragraph
        Example, for C / C++ :
            "/*"
            
    comment_par_end : str
        String to close the commenting of a paragraph
        Example, for C / C++ :
            "*/"
        
    indexing_0 : int
        Index of the first element of a list
        Example for Python :
            0
    
    end_of_line : str
        End of line character
        For examples, for C / C++ :
            ";"
        
    is_typed : bool
        True if the language is typed, False else
        Example for Python: 
            False
    
    double_type : str
        Type of double variables (if the langauge is typed)
    
    vector_type : str
        Type of vector variables (if the language is typed)
        
    matrix_type : str
        Type of matrix variables (if the language is typed)
    
    can_return_list : bool
        True if the langauge accepts returning a List
    
    operators : dict
        Language operators. Dict keys are :
            - ** : exponent notation
            - * : multiplication
            - @ : matrix multiplication
            - / : division
            - + : addition
            - - : substraction
            - [] : list  subscription. ("[]", "()", "<>" ... )
        Every  item  is  a  list of 2 elements containing the value and a bool
        that is True if the operator is written as a function.
    
    fcts : dict
        Language functions. Dict keys are :
            - eye : Matrix Identity
            - zeros : Matrix Zeros
            - cross : Cross product
    
    docstr_before : bool
        True if the docstring is written before the function declaration
    
    extension : str
        File extension
    
    mat_obj_start : str
        First characters for matrix declaration
    
    mat_obj_end : str
        Last characters for matrix declaration
    
    mat_col_separator : str
        Matrix column separator
    
    mat_line_separator : str
        Matrix line separator
    
    mat_new_line : list of str
        New line of a matrix
    
    file_header : string
        File header, containing includes and stuff
        
    subscription : int
        1 if the last index of the subscription is not included

    return_ : str
        Return statement of the language. For example, in Python "return"

    end_loop : str
        End of a loop. For example, in C++: "}", in Python: ""

    """

    # Constructor ============================================================

    def __init__(self, name):
        """
        Description
        -----------
        
        Construct a Langauge object from a function name and parameters
        
        Parameters
        ----------
        
        name : str
            Function Name. Not case sensitive
        
        """

        name = name.lower()

        # Python .............................................................

        if name == 'python':
            self.name = 'python'

            self.fct_prefix = "def _fname_("
            self.fct_suffix = "):"
            self.fct_end = ""
            self.param_separator = ","
            self.max_line_length = 79
            self.comment_line = "#"
            self.comment_par_beg = '"""'
            self.comment_par_end = '"""'
            self.indexing_0 = 0
            self.end_of_line = ''
            self.is_typed = False
            self.double_type = 'float'
            self.vector_type = 'numpy.ndarray'
            self.matrix_type = 'numpy.ndarray'
            self.can_return_list = True
            self.operators = {'**': ['**', False],
                              '*':  ['*', False],
                              '@':  ['dot', True],
                              '/':  ['/', False],
                              '+':  ['+', False],
                              '-':  ['-', False],
                              "[]": ["[]", True]}
            self.fcts = {'eye': 'eye(__param1__, __param2__)',
                         'zeros': 'zeros((__param1__, __param2__))',
                         'cross': 'cross(__param1__, __param2__'}
            self.docstr_before = False
            self.extension = 'py'
            self.mat_obj_start = 'array(['
            self.mat_obj_end = '])'
            self.mat_col_separator = ','
            self.mat_line_separator = ','
            self.mat_new_line = ['[', ']']

            self.header = "from math import cos, sin\nfrom numpy import " + \
                          "array, cross, dot, zeros, eye\n"

            self.subscription = 1
            self.return_ = "return"
            self.end_loop = ""

        # Julia ..............................................................

        elif name == "julia":
            self.name = 'julia'
            self.fct_prefix = "function _fname_("
            self.fct_suffix = ")"
            self.fct_end = "end"
            self.param_separator = ","
            self.max_line_length = 92
            self.comment_line = "#"
            self.comment_par_beg = '"""'
            self.comment_par_end = '"""'
            self.indexing_0 = 1
            self.end_of_line = ''
            self.is_typed = False
            self.double_type = 'Float64'
            self.vector_type = 'Vector'
            self.matrix_type = 'Matrix'
            self.can_return_list = True
            self.operators = {'**': ['^', False],
                              '*': ['*', False],
                              '@': ['*', False],
                              '/': ['/', False],
                              '+': ['+', False],
                              '-': ['-', False],
                              '[]': ['[]', True]}
            self.fcts = {'eye': 'Matrix(I,__param1__, __param2__)',
                         'zeros': 'zeros(__param1__, __param2__)',
                         'cross': 'cross(__param1__, __param2__'}
            self.docstr_before = True
            self.extension = 'jl'
            self.mat_obj_start = 'vcat('
            self.mat_obj_end = ')'
            self.mat_col_separator = ' '
            self.mat_line_separator = ','
            self.mat_new_line = ['[', ']']

            self.header = "using LinearAlgebra\n"

            self.subscription = 0
            self.return_ = "return"
            self.end_loop = "end"

        # MATLAB .............................................................

        elif name == "matlab":
            self.name = 'matlab'
            self.fct_prefix = "function return_value = _fname_("
            self.fct_suffix = ")"
            self.fct_end = "end"
            self.param_separator = ","
            self.max_line_length = 75
            self.comment_line = "%"
            self.comment_par_beg = '%{\n'
            self.comment_par_end = '\n%}'
            self.indexing_0 = 1
            self.end_of_line = ';'
            self.is_typed = False
            self.double_type = 'double'
            self.vector_type = 'double'
            self.matrix_type = 'double'
            self.can_return_list = True
            self.operators = {'**': ['.^', False],
                              '*': ['.*', False],
                              '@': ['*', False],
                              '/': ['./', False],
                              '+': ['+', False],
                              '-': ['-', False],
                              '[]': ['()', True]}
            self.fcts = {'eye': 'eye(__param1__, __param2__)',
                         'zeros': 'zeros(__param1__, __param2__)',
                         'cross': 'cross(__param1__, __param2__'}
            self.docstr_before = False
            self.extension = 'm'
            self.mat_obj_start = '['
            self.mat_obj_end = ']'
            self.mat_col_separator = ','
            self.mat_line_separator = ';'
            self.mat_new_line = ['', '']

            self.header = ""

            self.subscription = 0
            self.return_ = "return_value ="
            self.end_loop = "end"

    def matrix(self, mat_list):
        """
        Description
        -----------
        
        Convert a list of string to its equivalent in the language as a matrix
        declaration
        
        Parameter
        ---------
        
        mat_list : list of list of str
            Matrix expression in Python
        
        Returns
        -------
        
        String expression of the matrix in the language
        
        """

        matrix = self.mat_obj_start
        for i, line in enumerate(mat_list):
            matrix += self.mat_new_line[0]
            mat_line = ''
            for j, val in enumerate(line):
                mat_line += self.convert(val)
                mat_line += self.mat_col_separator if j < len(line) - 1 \
                    else self.mat_new_line[1]
            matrix += mat_line
            matrix += self.mat_line_separator if i < len(mat_list) - 1 \
                else self.mat_obj_end
        return matrix

    # Matrix from matrix label ===============================================

    def matrix_from_label(self, label):
        """
        Description
        -----------
        
        Convert a matrix label #mat#i#j#...# to its equivalent in the language
        as a matrix declaration
        
        Parameter
        ---------
        
        mat_list : list of list of str
            Matrix expression in Python
        
        Returns
        -------
        
        String expression of the matrix in the language
        
        """

        elems = label.split('#')

        nbl = int(elems[2])
        nbc = int(elems[3])

        elements = elems[4:-1]

        matrix = self.mat_obj_start
        i_e = 0

        for i in range(nbl):
            matrix += self.mat_new_line[0]
            mat_line = ''
            for j in range(nbc):
                mat_line += self.convert(elements[i_e])
                mat_line += self.mat_col_separator if j < nbc - 1 \
                    else self.mat_new_line[1]
                i_e += 1
            matrix += mat_line
            matrix += self.mat_line_separator if i < nbl - 1 \
                else self.mat_obj_end
        return matrix

    # Convert expr to language ===============================================

    def convert(self, expression):
        """
        Description
        -----------
        
        Converts  a  string  of a mathematical expression written in Python to
        the same expression in the current language
        
        Parameter
        ---------
        
        expression : str
            Mathematical expression string
        
        Returns
        -------
        
        str :
            String containing the converted expression
        
        """

        def scistrtodblstr(scistr, double=True):
            """
            Converts a scientific notation string to a double notation string

            Parameters
            ----------
            scistr : string
                scientific notation double
            double : boolean
                if False, adds a f after the number : 1.0 become 1.0f

            Returns
            -------
            Double notation string

            """
            if 'e' not in scistr:
                return scistr
            e = scistr.find('e')
            if '.' not in scistr:
                afterPoint = e
            else:
                afterPoint = e - scistr.find('.')
            afterPoint -= int(scistr[e + 1:])
            fl = float(scistr)
            forma = f"%.{max(afterPoint - 1, 1)}f"
            return str(forma % fl) + ('' if double else 'f')

        def convert_all_sci_to_dbl(string):
            """
            Converts  all the numbers written in scientific notation to double
            notation

            Parameters
            ----------
            string : str
                String to remove scientific notations from

            Returns
            -------
            String containing double notations instead of scientific notations
            """

            # Matching scientific numbers
            pr = re.compile(r'-?[\d.]+(?:e[\+\-]?\d+)')
            numbers = pr.findall(string)

            for number in set(numbers):
                string = string.replace(number, scistrtodblstr(number))
            return string

        # . . . . . . . . . . . . . .

        expression = convert_all_sci_to_dbl(expression)

        list_op = [[op, False] for op in self.operators]
        list_op[-1][1] = True  # Subscription is considered as a function
        list_new_ops = [self.operators[op] for op in self.operators]

        expression = replace_many(expression, list_op, list_new_ops)

        for function in self.fcts:
            pattern = re.compile(r'(?:^|(?<=(\W)))_' + function + \
                                 '_\d+_\d+_(?:$|(?:(\W)))')
            all_occur = re.finditer(pattern, expression)

            for occur in all_occur:
                match = expression[occur.span()[0]:occur.span()[1]]
                arg1 = match.split('_')[2]
                arg2 = match.split('_')[3]
                repl = self.fcts[function].replace('__param1__', arg1). \
                    replace("__param2__", arg2)
                expression = expression.replace(match, repl)

        # Converting matrix labels
        pattern = re.compile(r'(?:^|(?<=(\W)))#mat#([\w\-.*/+\(\)\[\]]+#)+' + \
                             r'(?:$|(?:(\W)))')
        all_occur = re.finditer(pattern, expression)

        for occur in all_occur:
            match = expression[occur.span()[0]:occur.span()[1]]
            expression = expression.replace(match,
                                            self.matrix_from_label(match))

        return expression

    # Justifying docstring ===================================================

    def justify(self, docstring, is_a_paragraph=True):
        """
        Description
        -----------
        
        Justify the doc string so it looks great
        
        Parameters
        ----------
        
        docstring : str
            Docstring to justify

        is_a_paragraph : bool
            Set  to  True  if the  text  you want to justify is commented as a
            paragraph.  If  False, it will be commented as many lines preceded
            by a comment character.

            Default is True
            
        Returns
        -------
        
        str :
            Justified docstring
        """

        all_lines = docstring.split('\n')

        for i, line in enumerate(all_lines):
            if len(line) > self.max_line_length:
                # Indentation Level
                ind = 0
                i_c = 0
                minus_ok = False
                while i_c < len(line) - 1:
                    char = line[i_c]
                    if char == '-' and not minus_ok:
                        minus_ok = True
                        ind += 1
                    elif char != ' ':
                        break
                    else:
                        ind += 1
                    i_c += 1

                max_len = self.max_line_length - ind
                if not is_a_paragraph:
                    max_len -= len(self.comment_line) + 1

                all_words = []
                all_w = line.split(' ')
                for w in all_w:
                    if w != '':
                        all_words.append(w)

                if len(' '.join(all_words)) <= max_len:
                    break

                for i_w, word in enumerate(all_words):
                    if len(' '.join(all_words[:i_w + 1])) > max_len:
                        new_line = ' '.join(all_words[:i_w])
                        rest_of_line = ' '.join(all_words[i_w:])
                        break

                justified = len(new_line) == max_len
                nb_spaces = 1
                while not justified:
                    justified = len(new_line) == max_len
                    i_c = 0
                    for char in new_line:
                        if new_line[i_c] == ' ':
                            new_line = new_line[:i_c] + ' ' + new_line[i_c:]
                            justified = len(new_line) == max_len
                            i_c += nb_spaces
                            if justified:
                                break
                        i_c += 1
                        if i_c >= len(new_line):
                            break
                    nb_spaces += 1

                all_lines[i] = ' ' * ind + new_line
                all_lines.insert(i + 1, ' ' * ind + rest_of_line)

        if is_a_paragraph:
            join_char = "\n"
        else:
            join_char = "\n" + self.comment_line + " "
        return join_char[1:] + join_char.join(all_lines)

    # Generate function code =================================================

    def generate_fct(self, fname, params, expr, varss=[], docstr=None,
                     matrix_dims=(4, 4), input_is_vector=False):
        """
        Description
        -----------
        
        Generate a function code from its name, parameters and expression.
        
        Parameters
        ----------
        
        fname : str
            Function name
        
        params : list of dict
            Function parameters. Every element of the list has these keys :
                - name : str : Variable Name
                - type : str : Variable Type, can be 'double', 'vect' or 'mat'
                - description : str : Parameter description (for doc string)
        
        expr : str or list of list of str
            Return value of the function (mathematical expression)
            If your expression is a matrix, pass a list of list of str type.
        
        varss : list of list, optional
            Temporary variables. Default is []
        
        docstr : str, optional
            Docstring of the function. Default is None
            
        matrix_dims : tuple of 2 ints, optional
            Dimensions of the return value
            If !=(1, 1) the parameters are converted to a vector of parameters
            Default is (4, 4)
            
        input_is_vector : bool, optional
            If  True, the parameters are converted to an unique parameter that
            is a vector
            Default is False
        
        Returns
        -------
        
        str
            String of generated code
            
        Examples
        --------
        
        TODO
        """

        # Function declaration ...............................................

        code = self.fct_prefix.replace('_fname_', fname)

        # Parameters .........................................................

        # Sorting parameters in alphabetical order
        params.sort(key=lambda x: x['name'])

        docstrparams = [param.copy() for param in params]

        if input_is_vector:
            if self.is_typed:
                code += self.vector_type + ' '
            code += 'q'

            descrq = 'Vector of variables where :'
            for i_p, param in enumerate(params):
                descrq += f'\n        - q[{i_p + self.indexing_0}] = ' + \
                          param['name']
                descrq += ' :\n              ' + param['description']

            paramq = {'name': 'q', 'type': 'vect', 'description': descrq}
            docstrparams = [paramq]

        else:
            for i, param in enumerate(params):
                if self.is_typed:
                    typ = self.double_type if param['type'] == 'double' else \
                        (self.vector_type if param['type'] == 'vect' else
                         self.matrix_type if param['type'] == 'mat' else None)
                    code += typ + ' '
                code += param['name']

                if i < len(params) - 1:
                    code += self.param_separator + ' '

        code += self.fct_suffix + '\n' + indent(1)

        # Docstring ..........................................................

        real_docstr = ''
        if docstr is not None:
            if self.name != "matlab":
                real_docstr += self.comment_par_beg
            if self.name == 'julia':
                docstr = docstr.replace('\\', '\\\\')

            if self.name in ['matlab', 'julia']:
                real_docstr += '\n    ' + \
                               ' '.join(code.split('\n')[0].split(' ')[1:])
                real_docstr += '\n'
            if self.name == "matlab":
                real_docstr += fname + "\n"
            if self.name in ['python', 'julia', 'matlab']:
                real_docstr += '\nDescription\n-----------\n\n' + docstr
                real_docstr += '\n\nParameters\n----------\n\n'
                for param in docstrparams:
                    real_docstr += param['name']
                    typ = self.double_type if param['type'] == 'double' else \
                        (self.vector_type if param['type'] == 'vect' else
                         self.matrix_type if param['type'] == 'mat' else '')
                    real_docstr += ' : ' + typ + '\n    '
                    real_docstr += param['description'] + '\n\n'

            if self.name != "matlab":
                real_docstr += self.comment_par_end

            par = self.name != "matlab"

            if self.docstr_before:
                real_docstr = self.justify(real_docstr, is_a_paragraph=par)
                code = real_docstr + '\n' + code + '\n    '
            else:
                real_docstr = self.justify(
                    real_docstr.replace('\n', '\n    '), is_a_paragraph=par)
                code += real_docstr + '\n\n    '

        # Variables ..........................................................

        loops = 0  # Indent level
        for i_var, var in enumerate(varss):
            if var["name"] == "__FOR__":
                loops += 1
                code += (self.for_loop(var['value'][0], var['value'][1]) +
                         f"\n{indent(loops + 1)}")
                continue
            elif var['name'] == "__ENDLOOP__":
                loops -= 1
                code += f"\n{indent(1 + loops)}{self.end_loop}"
                continue
            if self.is_typed:
                typ = self.double_type if var['type'] == 'double' else \
                      (self.vector_type if var['type'] == 'vect' else
                       self.matrix_type if var['type'] == 'mat' else '')
                if typ != '':
                    code += typ + ' '

            if input_is_vector:
                for i_p, param in enumerate(params):
                    varss[i_var]['value'] = replace_var(varss[i_var]['value'],
                                                        param['name'],
                                                        f'q[{i_p + self.indexing_0}]')
            code += var['name'] + ' = ' + self.convert(varss[i_var]['value'])
            code += self.end_of_line + '\n' + indent(1 + loops)

        if len(varss) > 0:
            code += '\n' + indent(1)

        # Matrix Return ......................................................

        if matrix_dims != (1, 1):
            if fname == 'T_CHEST_R_SFE_joint_inv':
                print('azer')
                pass
            # Matrix name
            mat_name = 'mat'
            while mat_name in [var['name'] for var in varss] or mat_name \
                    in params:
                mat_name += "0"

            # Matrix declaration
            code += self.comment_line + ' Returned Matrix\n' + indent(1)

            code += mat_name + ' = ' + self.mat_obj_start

            # For 0 to the number of rows
            for i in range(matrix_dims[0]):
                # Empty matrix line
                if i > 0:
                    code += '\n' + indent(1) + (3 + len(mat_name)) * ' '
                code += self.mat_new_line[0]
                line = ''

                # For 0 to the number of columns
                for j in range(matrix_dims[1]):
                    element = expr[i][j]
                    if input_is_vector:
                        for i_p, param in enumerate(params):
                            element = replace_var(element, param['name'],
                                                  f'q[{i_p + self.indexing_0}]')
                    line += self.convert(element)
                    line += self.mat_col_separator if j < matrix_dims[1] - 1 \
                        else self.mat_new_line[1]

                # Adding the created line to the matrix
                code += line

                code += self.mat_line_separator if i < matrix_dims[0] - 1 \
                    else self.mat_obj_end

            code += self.end_of_line + f'\n\n    {self.return_} '\
                    + mat_name + \
                    self.end_of_line + '\n' + self.fct_end

        # Scalar return ......................................................

        else:
            if input_is_vector:
                for i_p, param in enumerate(params):
                    expr = replace_var(expr, param['name'],
                                       f'q[{i_p + self.indexing_0}]')
            code += self.return_ + ' ' + self.convert(expr) + \
                    self.end_of_line + '\n' + self.fct_end

        return code

    # Generating titles ______________________________________________________

    def title(self, text, level):
        """
        Description
        -----------
        
        Generate title string

        Parameters
        ----------
        text : str
            Title text
        level : int
            Level of the title.
            0 => farmed title
            1 => Title _____________
            2 => Title .............

        Returns
        -------
        str :
            Title string

        """
        if self.name == "matlab" and level == 0:
            code = "%% "
        else:
            code = self.comment_line + ' '
        if level == 0:
            for _ in range(self.max_line_length - 2 - (self.name == "matlab")):
                code += '-'
            code += '\n' + self.comment_line + ' |'

            is_symetrical = (self.max_line_length - 3 - \
                             len(self.comment_line) - len(text)) % 2 == 0
            spaces = (self.max_line_length - 3 - \
                      len(self.comment_line) - len(text)) // 2
            left_spaces = spaces
            right_spaces = spaces if is_symetrical else spaces + 1

            for _ in range(left_spaces):
                code += ' '
            code += text.upper()
            for _ in range(right_spaces):
                code += ' '
            code += f'|\n{self.comment_line} '
            for _ in range(self.max_line_length - 2):
                code += '-'

        elif level == 1:
            code += text + ' '
            for _ in range(self.max_line_length - len(text) - 3):
                code += '_'

        return code

    # For loop _______________________________________________________________

    def for_loop(self, i0, ilt):
        """
        Creates the for loop like : for i in range(i0, ilt):

        The iteration variable is i

        Parameters
        ----------
        i0 : int
            Beginning of the iteration
        ilt : int
            End of the iteration

        Returns
        -------
        for_ : str
            For loop statement
        """

        if self.name == "python":
            return f"for i in range({i0}, {ilt}):"
        elif self.name == "julia":
            return f"for i={i0+1}:{ilt}"
        elif self.name == "matlab":
            return f"for i={i0+1}:{ilt}"


if __name__ == '__main__':
    func_str = '1+(cos(a-b)-sin(a-b))/(cos(a-b)-sin(a-b))-((cos(a-b)-sin(a-b))/(cos(a-b)-sin(a-b)))'
    varss = {'v_sub0': 'a-b',
             'f_cos0': 'cos(v_sub0)',
             'f_sin0': 'sin(v_sub0)',
             'v_sub1': 'cos(v_sub0)-sin(v_sub0)',
             'v_div0': 'v_sub1/(cos(v_sub0)-sin(v_sub0))'}

    lan = Language('python')

    code = lan.generate_fct('test', ['a', 'b'], func_str, varss)

    print(code)
