## (This feature is still in development)
# Denavit–Hartenberg Parameters File Format

This is the documentation for the [Denavit–Hartenberg parameters](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters)
file format (.dhparams). This format is similar to [CSV](https://en.wikipedia.org/wiki/Comma-separated_values),
but with specific fields. It has been created for URDFast, as this representation
is pretty used as well as [URDF](http://wiki.ros.org/urdf) format.

This format not only gives DH parameters, but also other properties (see below).
In that way, it contains almost as many informations as an URDF file.

Note however that you can not represent tree-like robot architectures with this
format.

## Prerequisites

You need to know what are DH Parameters to understand this documentation.
Please, refer to the [Wikipedia](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters)
page to learn more.

Denavit–Hartenberg parameters file format is a text format, so you need any
text editor to create or edit .dhparams files.

The .dhparams files parser is included in this project (see dh_params.py)

## Properties

**File extension** : .dhparams

**File format** : text, close to [CSV](https://en.wikipedia.org/wiki/Comma-separated_values)

## Specifications Line by Line

### Line 1 : Translations & Rotations

As many ways to represent the kinematic chain of the robot are possible with
the DH representation (like [modified DH parameters](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters)),
the first line of the file must describe the different translations and rotations
order. It must contain **4 values** separated by a coma. Each one of these 
values can be :
- `TransX..d` for a transition of the `d` parameter along the `x`axis
- `TransX..r` for a transition of the `r` parameter along the `x`axis
- `TransZ..d` for a transition of the `d` parameter along the `z`axis
- `TransZ..r` for a transition of the `r` parameter along the `z`axis
- `RotX..theta` for a rotation of the `theta` parameter around the `x` axis
- `RotX..alpha` for a rotation of the `alpha` parameter around the `x` axis
- `RotZ..theta` for a rotation of the `theta` parameter around the `z` axis
- `RotZ..alpha` for a rotation of the `alpha` parameter around the `z` axis

For example, the line 1 of a file using the common DH representation, described
[here](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Denavit%E2%80%93Hartenberg_matrix),
is :
```
TransZ..d,RotZ..theta,TransX..r,RotX..alpha
```

For the modified DH parameters representation, described [here](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters),
we get :
```csv
RotX..alpha,TransX..r,RotZ..theta,TransZ..d
```

Of course, you can use custom representations such as :
```csv
RotX..alpha,TransX..d,RotZ..theta,TransZ..r
```

In the last case, ensure you have actually used the 4 parameters (`d`, `theta`,
`r` and `alpha`) and the 4 transformations (`TransX`, `TransZ`, `RotX`, `RotZ`).
The parser doesn't throw any errors if you don't respect this condition, however,
the result you will get won't probably be what you expect.


### Line 2 : Empty line

The second line is ignored by the parser. You should keep it empty for
a better readability.

### Line 3 : Column Headers

This where you put the column headers of the DH parameters. You can write
the following headers in the order you want. Some of them are optional.

- `name` *(optional)* : name of the Link
- `d`: offset along previous `z` to the common normal. Must be given in meters.
- `theta`:  angle about previous `z`, from old `x` to new `x`. Must be given
  in radians.
- `r`: length of the common normal (aka `a`, but if using this notation, do 
  not confuse with `alpha`). Assuming a revolute joint, this is the radius 
  about previous `z`. Must be given in meters.
- `alpha`: angle about common normal, from old `z` axis to new `z` axis. Must
  be given in radians.
- `pmin` *(optional)*: minimal reachable position (min of `theta` / `alpha` value for
  revolute joints in radians, min of `r`, `d` value for prismatic joints in meters).
- `pmax` *(optional)*: maximal reachable position (max of `theta` / `alpha` value for
  revolute joints in radians, max of `r`, `d` value for prismatic joints in meters).
- `vmax` *(optional)*: maximal reachable velocity (max of the derivative of
  `theta` / `alpha` for revolute joints in radians per seconds, max of
  the derivative of `r`, `d` for prismatic joints in meters per seconds).
  Must be positive (velocity is expressed as a norm)
- `amax` *(optional)*: maximal reachable acceleration (max of the second derivative of
  `theta` / `alpha` for revolute joints in radians per seconds², max of
  the second derivative of `r`, `d` for prismatic joints in meters per seconds²).
- `com` *(optional)*: center of mass of the link coordinates. This must contain
3 Python-compatible floating-point numbers separated by semicolons `;`. The first one
is for the `x` coordinate, the second one for the `y` coordinate and the last 
one for the `z` coordinate. This must be given in meters.
If you do not specify this field, it will be set to a null vector by default.
- `mass` *(optional)*: mass of the link. Must be given in kilograms.
If you do not specify this field, it will be set to 0 kg.
  
Note that if you don't fill optional fields, they will default to `None`, whatever
their unit is. Only the name will default to `link_n` where `n` is the row number.

The `pmin`, `pmax`, `vmax` and `amax` will be applied for the
degree of freedom of the row (see below). If all the values are decimal, they will be
ignored.
  
For example, these are some valid 3rd lines :
```csv
name,d,theta,r,alpha,pmin,pmax,vmax,amax,com,mass
```
```csv
d,theta,r,alpha
```
```csv
name,pmin,pmax,d,alpha,theta,r,mass
```

### Line 4 : Empty line

The fourth line is ignored by the parser. You should keep it empty for
a better readability.

### Line 5 &rarr; End of File : Content

Starting from line 5, every line must describe a link / joint of your kinematic
chain. All the values must be separated by commas `,`.
The links must be given in the order of your kinematic chain.

If you wrote n header values upward, you must write n values for each row, in
the same order.

If you write the `name` header, the names must be string values
(do not write the quotes). It can not contain any non-alphanumeric character
except underscores `_` otherwise the generated code might not be valid !
In the general case, the `name` field is valid if and only if the following
Python code returns `True` :

```python
name = "arm_1"  # Any link / joint name
name.isidentifier()
```

All the other values (except one, see below) must be floating-point convertible
strings. The value will be valid if and only if the following Python code does not 
throw any errors :

```python
value = "-10.5"  # Any value that can be in your .dhparams file
float(value)
```

There is of course an exception for one value per row : the one representing
the degree of freedom of your joint. You can set it for `d`, `theta`, `r`, or
`alpha`.
In that way, you can write any non-numeric value but ensure it only contains
ASCII letters, numbers and underscores, so it creates a valid name as a
variable. For example, `theta_1` is a valid name whereas `1_theta`,
`θ1` or `theta-1` are not valid.
In general, the name of the degree of freedom
is valid if and only if the following Python code returns `True` :

```python
dof = "theta_1"  # Any degree of freedom name
dof.isidentifier()
```

If you have many degrees of freedom in the
same row, please consider splitting them into many rows, so you have only one
degree of freedom per row.

It is of course possible to have a 0 degrees of freedom row, if you have only
floating-point values in your row.

## Useful Informations

Whitespaces `" "` are ignored during parsing. You can add as many spaces
as you want to make your file look prettier.

Avoid empty lines, except when it is asked (lines 2 and 4), **including** at the
end of the file.

**Do not** add a semicolon `;` at the end of the lines. Split the lines with a
newline character.

**Do not** add a newline character at the end of the file.

Please use only ASCII characters in your file.

## Examples of Valid Files

Example 1 :

```csv
TransZ..d, RotZ..theta, TransX..r, RotX..alpha

name, d,    theta,  r,   alpha

L1,   0,    theta1, 0.5, 0
L2,   0,    theta2, 0,   1.57079633
L3,   1.04, theta3, 0,   0
```

Example 2 :

```csv
RotX..alpha, TransX..d, RotZ..theta, TransZ..r

name, alpha,               d, theta,  r,      pmin,     pmax,    vmax,         com
--------------------------------------------------------------------------------------------------------------
A1,   0,                   0, theta1, 0.3105, -2.96706, 2.96706, 1.9634954,    0.001340;  0.02622;   -0.087777
A2,   1.5707963267948966,  0, theta2, 0,      -2.0944,  2.0944,  1.9634954,    0.001340;  0.087777;  0.02622
E1,   -1.5707963267948966, 0, theta3, 0.4,    -2.96706, 2.96706, 1.9634954,    0.001340;  0.02622;   -0.087777
A3,   -1.5707963267948966, 0, theta4, 0,      -2.0944,  2.0944,  1.9634954,    -0.001340; -0.02622;  -0.087777
A4,   1.5707963267948966,  0, theta5, 0.39,   -2.96706, 2.96706, 3.1415926535, -0.000993; 0.026958;  -0.111650
A5,   1.5707963267948966,  0, theta6, 0,      -2.0944,  2.0944,  1.9634954,    -0.000259; -0.005328; 0.005956
A6,   -1.5707963267948966, 0, theta7, 0.078,  -2.96706, 2.96706, 1.9634954,    0.0;       0.0;       0.063
```

## Parse and Use

URDFast fully supports .dhparams files.

URDFast includes a Python parser for .dhparams files, see `dh_params.py`.

## Correspondence with URDF

*This part is here only for understanding purposes, and is not related to the
file format directly. This can help you understand how this format works
comparing it with URDF.*

For every row (line starting from the 5th) you can give a degree of freedom.
The nature of this degree of freedom determines the joint type. Here is the
correspondence with URDF joint types :

- If your DoF is set for `alpha` or `theta` and you set `pmin` and `pmax` headers :
  revolute joint
- If your Dof is set for `alpha` or `theta` and you didn't give any value for `pmin` and 
  `pmax` : continuous joint
- If your DoF is set for `r` or `d` **and** you set `pmin` and `pmax`
  headers : prismatic joint
- No DoF : fixed joint

In URDF, you have both &lt;joint&gt; and &lt;link&gt; elements. In .dhparams
file format, the link and the joints are described on the same row.
The `com` and `mass` properties refer to link whereas all the other properties
refer to the joint. One row describes the joint and its child link.