"""
This file contains the parser for DH Parameters file format.
For more details, read the specifications of this file format :
https://github.com/Teskann/URDFast/blob/master/documentation/dhparams_file_format.md
"""

from sympy import Symbol


class DHparamsRow:
    """
    Object representing a row of a .dhparams file once it has been parsed.

    Data structure
    ---------------

    name : str
        Name of the link / joint.

    d : float or sympy.core.symbol.Symbol
        Offset along previous z to the common normal. Must be given in meters.

    theta : float or sympy.core.symbol.Symbol
        Angle about previous z, from old x to new x. Must be given in radians.

    r : float or sympy.core.symbol.Symbol
        Length of the common normal (aka a, but if using this notation, do not
        confuse  with  alpha).  Assuming  a revolute joint, this is the radius
        about previous z. Must be given in meters.

    alpha : float or sympy.core.symbol.Symbol
        Angle  about  common  normal,  from  old z axis to new z axis. Must be
        given in radians.

    pmin : float or None
        Minimal value the degree of freedom can reach (position).
        Expressed  in  radians  for  angular degrees of freedom, in meters for
        translation degrees of freedom.

        Default is None

    pmax : float or None
        Maximal value the degree of freedom can reach (position).
        Expressed  in  radians  for  angular degrees of freedom, in meters for
        translation degrees of freedom.

        Default is None

    vmax : float or None
        Maximal reachable velocity (max of the derivative of theta / alpha for
        revolute  joints in radians per seconds, max of the derivative of r, d
        for prismatic joints in meters per seconds).

        Must be positive (the velocity is expressed as a norm).

        Default is None

    amax : float or None
        maximal  reachable acceleration (max of the second derivative of theta
        / alpha for revolute joints in radians per seconds², max of the second
        derivative of r, d for prismatic joints in meters per seconds²).

        Default is None

    com : list of float
        Center of mass of the link : [x, y, z]

    mass : float / int
        Link mass in kilograms. MUST be positive
    """

    def __init__(self, name, d, theta, r, alpha, com,
                 pmin=None, pmax=None, vmax=None, amax=None, mass=0):
        """
        Parameters
        ----------

        name : str
            Joint / Link name.

        d : float or sympy.core.symbol.Symbol
            Offset  along  previous  z  to the common normal. Must be given in
            meters.

        theta : float or sympy.core.symbol.Symbol
            Angle  about  previous  z,  from  old x to new x. Must be given in
            radians.

        r : float or sympy.core.symbol.Symbol
            Length of the common normal (aka a, but if using this notation, do
            not  confuse  with  alpha). Assuming a revolute joint, this is the
            radius about previous z. Must be given in meters.

        alpha : float or sympy.core.symbol.Symbol
            Angle  about common normal, from old z axis to new z axis. Must be
            given in radians.

        pmin : float or None
            Minimal value the degree of freedom can reach (position).
            Expressed in radians for angular degrees of freedom, in meters for
            translation degrees of freedom.

            Default is None

        pmax : float or None
            Maximal value the degree of freedom can reach (position).
            Expressed in radians for angular degrees of freedom, in meters for
            translation degrees of freedom.

            Default is None

        vmax : float or None
            Maximal reachable velocity (max of the derivative of theta / alpha
            for  revolute joints in radians per seconds, max of the derivative
            of r, d for prismatic joints in meters per seconds).

            Must be positive (the velocity is expressed as a norm).

            Default is None

        amax : float or None
            maximal  reachable  acceleration  (max of the second derivative of
            theta / alpha  for revolute joints in radians per seconds², max of
            the  second  derivative of r, d for prismatic joints in meters per
            seconds²).

            Default is None
        """

        self.d = d
        self.theta = theta
        self.r = r
        self.alpha = alpha
        self.name = name
        self.com = com
        self.pmin = pmin
        self.pmax = pmax
        self.vmax = vmax
        self.amax = amax
        self.mass = mass

    def __str__(self):
        """
        Convert the object to string

        Returns
        -------

        str
            Object converted to string for printing purposes.

        """

        formatter = "{:<8.8} " * 10 + "{:<8}"
        return formatter.format(self.name, str(self.d), str(self.theta),
                                str(self.r), str(self.alpha), str(self.pmin),
                                str(self.pmax), str(self.vmax),
                                str(self.amax), str(self.mass), str(self.com))


class DHparams:
    """
    Object representing a .dhparams file once it has been parsed.

    Data structure
    ---------------

    rot_trans : list of str
        List of all the transformations applied to the joint. Every element of
        the list must be a CSV value of the line 1 of a .dhparam file.

    rows : list of DHparamsRow
        All the rows of the .dhparams file

    Examples
    --------

    You can create objects using the parser calling the dh() function

    >>> from dh_params import dh
    >>> obj = dh("./Examples/example_0.dhparams")

    >>> from dh_params import dh
    >>> obj = dh("./Examples/example_1.dhparams")

    """

    def __init__(self, rot_trans, rows):
        """
        Init the object

        Parameters
        ----------
        rot_trans : list of str
            List  of  all  the  transformations  applied  to  the joint. Every
            element of the list must be a CSV value of the line 1 of a
            .dhparam file.

        rows : list of DHparamsRow
            All the rows of the .dhparams file
        """

        self.rot_trans = rot_trans
        self.rows = rows

    def __str__(self):
        """
        Convert the object to string

        Returns
        -------

        str
            Object converted to string
        """

        s = ("DHparam Object\n---------------\n\nTransformations : " +
             ', '.join(self.rot_trans) + "\n")
        formatter = "{:<8.8} " * 10 + "{:<8}"
        s += formatter.format("name", "d", "theta", "r", "alpha", "pmin",
                              "pmax", "vmax", "amax", "mass", "com") + "\n\n"
        for row in self.rows:
            s += str(row) + "\n"
        return s


# Parser _____________________________________________________________________

def parse_dhparams(text):
    """
    Parse a .dhparams file content to create a Robot

    This function might raise some errors if the text parameter is not valid.

    Parameters
    ----------
    text : str
        Content of the .dhparams file

    Returns
    -------

    DHparams
        Object of the parsed file

    Examples
    --------

    >>> filename = "./Examples/example_0.dhparams"
    >>> obj = parse_dhparams(filename)

    >>> filename = "./Examples/example_0.dhparams"
    >>> obj = parse_dhparams(filename)

    """

    text = text.replace(" ", "")

    lines = text.split('\n')

    # First line : Transformations ...........................................

    trans = lines[0].split(',')
    for t in trans:
        if t not in ["TransX..d", "TransX..r", "TransZ..d", "TransZ..r",
                     "RotX..theta", "RotX..alpha", "RotZ..theta",
                     "RotZ..alpha"]:
            raise SyntaxError("Error at line 1 of .dhparams file. "
                              "Unknown transformation \"" + t + "\". "
                              "Please read the documentation for more "
                              "details.")
    if len(trans) != 4:
        raise ValueError("The line 1 of the .dhparams file must contain"
                         " 4 transformations.")

    # Line 3 : Column headers ................................................

    headers = {"name": None,
               "d": None,
               "theta": None,
               "r": None,
               "alpha": None,
               "pmin": None,
               "pmax": None,
               "vmax": None,
               "amax": None,
               "com": None,
               "mass": None}

    for i, user_header in enumerate(lines[2].split(",")):
        if user_header not in headers.keys():
            raise SyntaxError("Unknown header \"" + user_header + "\". "
                              " Please see the .dhparams documentation"
                              " for more details.")
        else:
            headers[user_header] = i

    # Line 5 -> End : content ................................................

    rows = []

    for i, line in enumerate(lines[4:]):
        values = line.split(",")

        # Name . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

        if headers["name"] is not None:
            if headers["name"] >= len(values):
                raise SyntaxError("Error at line " + str(i + 5) + " of "
                                  ".dhparams file. Expected a value for "
                                  "name.")
            name = values[headers["name"]]
            if not name.isidentifier():
                raise SyntaxError("Error at line " + str(i + 5) + " of "
                                  ".dhparams file. Name is not valid.")
        else:
            name = str(i)

        # d, r, theta, alpha . . . . . . . . . . . . . . . . . . . . . . . . .

        mat = {"d": None,
               "r": None,
               "theta": None,
               "alpha": None}
        for k in ["d", "r", "theta", "alpha"]:
            if headers[k] is not None:
                if headers[k] >= len(values):
                    raise SyntaxError("Error at line " + str(i + 5) + " of "
                                      ".dhparams file. Expected a value "
                                      "for " + k + ".")
                try:
                    mat[k] = float(values[headers[k]])
                except ValueError:
                    if not values[headers[k]].isidentifier():
                        raise SyntaxError("Error at line " + str(i + 5) +
                                          " of .dhparams file. " + k +
                                          " value is not valid. It must "
                                          "be either a float or a symbol"
                                          " that is an identifier. Read "
                                          "the documentation for more "
                                          "details.")
                    mat[k] = Symbol(values[headers[k]])
            else:
                raise SyntaxError("Error at line 3 of .dhparams "
                                  "file. " + k + " header must be "
                                                 "declared.")

        # pmin, pmax, vmax, amax, mass . . . . . . . . . . . . . . . . . . . .

        limits = {"pmin": None,
                  "pmax": None,
                  "vmax": None,
                  "amax": None,
                  "mass": 0}
        for k in ["pmin", "pmax", "vmax", "amax", "mass"]:
            if headers[k] is not None:
                if headers[k] >= len(values):
                    raise SyntaxError("Error at line " + str(i + 5) +
                                      " of .dhparams file. Expected a"
                                      "value for " + k + ".")
                try:
                    limits[k] = float(values[headers[k]])
                except ValueError:
                    raise ValueError("Error at line " + str(i + 5) +
                                      " of .dhparams file. "
                                     "Expected a float value for " + k + ".")

        if limits["mass"] < 0:
            raise ValueError("Error at line " + str(i + 5) + "of .dhparams"
                             "file. Link must be positive of null.")

        # com  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

        com = [0.0, 0.0, 0.0]  # Default value

        if headers["com"] is not None:
            if headers["com"] >= len(values):
                raise SyntaxError("Error at line " + str(i + 5) + " of "
                                  ".dhparams file. Expected a value for "
                                  "com")
            coords = values[headers["com"]].split(';')
            if len(coords) != 3:
                raise SyntaxError("Error at line " + str(i + 5) + " of "
                                  ".dhparams file. com value must have 3 "
                                  "coordinates (separated by \";\")")
            letters = ["x", "y", "z"]
            for i_c, coord in enumerate(coords):
                try:
                    com[i_c] = float(coord)
                except ValueError:
                    raise ValueError("Error at line " + str(i + 5) + " of "
                                     ".dhparams file. com " + letters[i_c] +
                                     " coordinate must be a float value.")

        # Create the row object  . . . . . . . . . . . . . . . . . . . . . . .

        rows.append(DHparamsRow(name=name,
                                d=mat["d"],
                                theta=mat["theta"],
                                r=mat["r"],
                                alpha=mat["alpha"],
                                com=com,
                                pmin=limits["pmin"],
                                pmax=limits["pmax"],
                                vmax=limits["vmax"],
                                amax=limits["amax"],
                                mass=limits["mass"]))

    # Return the object ......................................................

    return DHparams(rot_trans=trans, rows=rows)


# Create the object from a file ______________________________________________

def dh(filename):
    """
    Creates the DHarams object from a *.dhparams file

    Parameters
    ----------
    filename : str
        File to open

    Returns
    -------

    DHparams
        Object corresponding to the parsed file

    Examples
    --------

    >>> dh("./Examples/example_0.dhparams")

    >>> dh("./Examples/example_1.dhparams")

    """

    with open(filename, "r") as f:
        return parse_dhparams(f.read())

# Main (running tests) _______________________________________________________


if __name__ == "__main__":
    print(dh("./Examples/example_0.dhparams"))
    print(dh("./Examples/example_1.dhparams"))