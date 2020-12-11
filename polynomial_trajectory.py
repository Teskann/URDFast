# -*- coding: utf-8 -*-
"""
Trajectory Generation.
All  the  function  in  this file are used to generate polynomial trajectories
starting from conditions on the different derivatives.
"""

from sympy import symbols, diff, simplify
from sympy.solvers import solve, linsolve


# Get polynomial expression __________________________________________________

def get_polynomial(sym_list, variable):
    """
    Description
    -----------

    Get the polynomial expression from a vector of coefficients.

    Parameters
    ----------

    sym_list : Iterable of SymPy symbols
        List of all polynomial coefficients (power ascending)
    variable: SymPy Symbol
        Polynomial variable (is often t)

    Returns
    -------

    SymPy polynomial expression

    Example
    -------

    >>> import sympy as sp
    >>> coefficients = sp.symbols("a0:5")
    >>> var = sp.Symbol("t")
    >>> get_polynomial(coefficients, var)
    a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4

    """
    pol = 0  # Polynomial expression
    for i, coeff in enumerate(sym_list):
        pol += coeff * variable ** i

    return pol


# Get equations ______________________________________________________________

def get_equations(conditions, variable):
    """
    Description
    -----------

    Transforming conditions to polynomial equations

    Parameters
    ----------

    conditions : list of lists containing 3 elements
        [..., [derivative_order, time_value, equals], ...]
        derivative_order : must be an integer
            order  of  the derivative. If the time is your derivative variable
            and  the  function  you  want to create describes your position, 0
            corresponds to the position, 1 to the speed, 2 to the acceleration
            3 to the jerk and so on.
        time_value : Float or sympy.core.symbol.Symbol
            Time value on which you want your condition to be set
        equals : Float or sympy.core.symbol.Symbol
            Value  of the  function for the given time. This can be a symbolic
            variable

    variable : sympy.core.symbol.Symbol
        Derivative variable

    Returns
    -------

    symbolic_variables : tuple of sympy.core.symbol.Symbol
        List  of  all  symbolic  variables  noted from a0 to ak where k is the
        number of conditions given (the length of 'conditions' parameter).
    der : list of Sympy expressions
        All  polynomial  derivatives.  If the time is your derivative variable
        and  the  function  you want to create describes your position, der[0]
        will  contain  the  position  polynomial, der[1] the speed polynomial,
        der[2]  the  acceleration  polynomial  and  so on. The list contains k
        elements  where  k  is the maximum derivative order value given in the
        'conditions' parameter.
    eqns : list of Sympy expressions
        List  containing  all  the equations. Every equation is supposed to be
        equal to 0 to solve the system.

    Example
    -------

    >>> import sympy as sp
    >>> t, t0, tf = sp.symbols("t t0 tf", reals=True)
    >>> cond = [[2, t0, 0],
                [2, tf, 0],
                [1, t0, 0],
                [1, tf, 0],
                [0, t0, 0],
                [0, tf, 1]]
    >>> sym_vars, derivatives, equations = get_equations(cond, t)
    >>> print(sym_vars)
    (a0, a1, a2, a3, a4, a5)
    >>> print(*derivatives, sep="\n")
    a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
    a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
    2*a2 + 6*a3*t + 12*a4*t**2 + 20*a5*t**3
    >>> print(*equations, sep="\n")
    2*a2 + 6*a3*t0 + 12*a4*t0**2 + 20*a5*t0**3
    2*a2 + 6*a3*tf + 12*a4*tf**2 + 20*a5*tf**3
    a1 + 2*a2*t0 + 3*a3*t0**2 + 4*a4*t0**3 + 5*a5*t0**4
    a1 + 2*a2*tf + 3*a3*tf**2 + 4*a4*tf**3 + 5*a5*tf**4
    a0 + a1*t0 + a2*t0**2 + a3*t0**3 + a4*t0**4 + a5*t0**5
    a0 + a1*tf + a2*tf**2 + a3*tf**3 + a4*tf**4 + a5*tf**5 - 1

    """

    # Getting the max derivative order
    d_max = max(list(map(list, zip(*conditions)))[0])

    # Computing polynomial order
    order = max(len(conditions), d_max + 1)

    # Creating symbolic variables for coefficients
    symbolic_variables = symbols("a0:" + str(order), reals=True)

    # Getting polynomial
    pol = get_polynomial(symbolic_variables, variable)

    # Computing all derivatives
    der = [pol]
    for i in range(1, d_max + 1):
        der.append(diff(der[i - 1], variable))

    # Array of equations
    eqns = []
    for cond in conditions:
        eqns.append(der[cond[0]].subs(variable, cond[1]) - cond[2])

    return symbolic_variables, der, eqns


# Custom Sympy Equations Solver ______________________________________________

def poly_solve(eqns_list, unknowns):
    """
    Description
    -----------

    Custom solver for linear system of symbolic equations.
    As  Sympy  solve()  function gets pretty slow on not that big systems (~10
    equations  and  10  unknowns  containing  symbolic variables), this solver
    might be faster for big systems.

    The  most  important  feature  is that it is possible to keep track of the
    solving  process (printed on the console), so the user can have an idea of
    the remaining time.

    This solver might however be slower if there are no symbolic variables.

    Parameters
    ----------

    eqns_list : list of Sympy expressions
        List of expressions (considered equal to 0)
    unknowns : sympy symbol list
        Variables to solve for

    Returns
    -------

    solution : dict
        Dictionary containing solutions

    """

    eqns = eqns_list.copy()

    # Choosing solver
    nb_sym = 0
    for eq in eqns:
        fs = eq.free_symbols
        fs.difference_update(unknowns)
        if len(fs) > 0:
            nb_sym += 1
    custom_solver = (nb_sym > 0)

    if custom_solver:
        # System solutions
        sol = {}
        solr = {xi: 0 for xi in unknowns}

        # Found solutions
        found_vars = []

        for w, eqn in enumerate(eqns):
            print(w / len(eqns))
            variable = 0
            for var in unknowns:
                if var in eqn.free_symbols and var not in found_vars:
                    variable = var
                    break
            if variable == 0:
                continue
            sol[variable] = list(linsolve([eqn], [variable]))[0][0]
            solr[variable] = sol[variable]
            for i, eq in enumerate(eqns):
                eqns[i] = eq.subs(sol).simplify()
            found_vars.append(variable)

        # Simplifying solution
        print("Simplifying")
        # dct = [[len(solr[a].free_symbols) for a in solr],
        #        [a for a in solr]]
        dct = []
        for a in solr:
            try:
                sym_len = len(solr[a].free_symbols)
            except AttributeError:
                sym_len = 0
            dct.append([sym_len, a])
        # dct = list(map(list, zip(*dct)))
        dct.sort(key=lambda x: x[0])
        for j, s in enumerate(dct):
            print(j / len(dct))
            for i, sol_i in enumerate(solr):
                try:
                    solr[sol_i] = solr[sol_i].subs(s[1], solr[s[1]]) \
                        .simplify()
                except AttributeError:
                    pass
        return solr
    else:
        return solve(eqns, unknowns)


# Getting solution to the equations __________________________________________

def get_solution(equations, unknowns, derivatives):
    """
    Description
    -----------

    Get the solutions to the equations to generate the trajectory.

    Parameters
    ----------

    equations : list of Sympy expressions
        List containing all the equations to solve.
        Every equation must be given with its expression equal to 0

    unknowns : tuple of sympy.core.symbol.Symbol
        List of all the unknowns in the equations

    derivatives : list of Sympy expressions
        All  polynomial  derivatives.  If the time is your derivative variable
        and  the  function  you want to create describes your position, der[0]
        contains  the position polynomial, der[1] the speed polynomial, der[2]
        the acceleration polynomial and so on.

    Returns
    -------

    sol : list of Sympy expressions
        List   containing  the  expression  of  the  polynomial  and  all  its
        derivatives.

    Example
    -------

    Here is an example to generate a polynomial trajectory with null speed and
    acceleration at the beginning and at the end of the trajectory :
        - f(t0) = 0
        - f(tf) = 1
        - f'(t0) = 0
        - f'(tf) = 0
        - f''(t0) = 0
        - f''(tf) = 0

    In this example, t is the time variable.

    >>> import sympy as sp
    >>> t, t0, tf = sp.symbols("t t0 tf", reals=True)
    >>> cond = [
                [0, t0, 0],
                [0, tf, 1],
                [1, t0, 0],
                [1, tf, 0],
                [2, t0, 0],
                [2, tf, 0]
               ]
    >>> sym_vars, derivatives, equations = get_equations(cond, t)
    >>> sol = get_solution(equations, sym_vars, derivatives)
    >>> sp.pprint(sol[0], use_unicode=False)  # Position (f(t))
             3 /     2                 2                                2\
    -(t - t0) *\6.0*t  - 15.0*t*tf + t0  + t0*(3.0*t - 5.0*tf) + 10.0*tf /
    -----------------------------------------------------------------------
                                            5
                                   (t0 - tf)
    >>> sp.pprint(sol[1], use_unicode=False)  # Speed  (f'(t))
                  2         2
    -30.0*(t - t0) *(t - tf)
    --------------------------
                     5
            (t0 - tf)
    >>> sp.pprint(sol[2], use_unicode=False)  # Acceleration (f''(t))
    -60.0*(t - t0)*(t - tf)*(2.0*t - t0 - tf)
    ------------------------------------------
                             5
                    (t0 - tf)
    """

    sol_unknowns = poly_solve(equations, unknowns)

    for unknown in unknowns:
        if unknown not in sol_unknowns:
            sol_unknowns[unknown] = 0

    # Getting rid of unknowns in solutions
    derivative_sol = derivatives
    for i, der in enumerate(derivatives):
        derivative_sol[i] = der.subs(sol_unknowns)
        while any(x in unknowns for x in derivative_sol[i].free_symbols):
            derivative_sol[i] = derivative_sol[i].subs(sol_unknowns)

    # Simplifying solutions
    for i, d in enumerate(derivative_sol):
        derivative_sol[i] = derivative_sol[i].simplify()

    for i, _ in enumerate(derivative_sol):
        derivative_sol[i] = derivative_sol[i].factor().evalf() \
            .collect(derivative_sol[i].free_symbols)

    return derivative_sol


# Verify solution ____________________________________________________________

def verify_solution(conditions, solution, function_variable):
    """
    Description
    -----------

    Verifies that the found solution is actually correct, ie that the solution
    follows the desired conditions.

    This function also verifies that the derivatives are actually correct.

    Parameters
    ----------

    conditions : list of lists containing 3 elements
        [..., [derivative_order, time_value, equals], ...]
        derivative_order : must be an integer
            order  of  the derivative. If the time is your derivative variable
            and  the  function  you  want to create describes your position, 0
            corresponds to the position, 1 to the speed, 2 to the acceleration
            3 to the jerk and so on.
        time_value : Float or sympy.core.symbol.Symbol
            Time value on which you want your condition to be set
        equals : Float or sympy.core.symbol.Symbol
            Value  of the  function for the given time. This can be a symbolic
            variable
    solution : list of Sympy expressions
        All  polynomial  derivatives.  If the time is your derivative variable
        and  the  function  you want to create describes your position, der[0]
        contains  the position polynomial, der[1] the speed polynomial, der[2]
        the acceleration polynomial and so on.
    function_variable : sympy.core.symbol.Symbol
        Function variable (often t for time-based polynomials)

    Returns
    -------

    Boolean :
        True if the solution is correct, False else.

    Examples
    --------

    >>> import sympy as sp
    >>> t, t0, tf = sp.symbols("t t0 tf", reals=True)
    >>> cond = [
                [0, t0, 0],
                [0, tf, 1],
                [1, t0, 0],
                [1, tf, 0],
                [2, t0, 0],
                [2, tf, 0],
                ]
    >>> sym_vars, derivatives, equations = get_equations(cond, t)
    >>> sol = get_solution(equations, sym_vars, derivatives)
    >>> verify_solution(cond, sol, t)
    True

    """

    # Verifying conditions ...................................................

    for cond in conditions:
        if simplify(solution[cond[0]].subs(t, cond[1]) - cond[2]). \
                nsimplify(tolerance=1e-10) != 0:
            print('Err :', cond, solution[cond[0]].subs(t, cond[1]) - cond[2],
                  '\n\n')
            return False

    # Verifying derivatives ..................................................

    if len(solution) <= 1:
        return True

    der = solution[0]

    for i, der_plus_1 in enumerate(solution[1:]):

        if (simplify(diff(der, function_variable) - der_plus_1))\
                .nsimplify(tolerance=1e-10) != 0:
            return False

        der = der_plus_1

    return True


# Main : Running a simple test _______________________________________________

if __name__ == "__main__":
    import sympy as sp

    t, t0, tf = sp.symbols("t t0 tf", reals=True)
    cond = [
        [0, t0, 0],
        [0, tf, 1],
        [1, t0, 0],
        [1, tf, 0],
        [2, t0, 0],
        [2, tf, 0],
    ]
    sym_vars, derivatives, equations = get_equations(cond, t)
    sol = get_solution(equations, sym_vars, derivatives)
    sp.pprint(sol[0], use_unicode=False)
    sp.pprint(sol[1], use_unicode=False)
    sp.pprint(sol[2], use_unicode=False)

    valid = verify_solution(cond, sol, t)
    print(valid)
