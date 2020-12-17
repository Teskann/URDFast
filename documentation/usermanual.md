# URDFast User Manual

## Installation

### 1. Prerequisties : Python & Packages

*URDFast* is a Python program. You then need to have Python installed on your computer to run it. If you don't have Python yet, consider [installing it using Anaconda](https://www.anaconda.com/products/individual). Anaconda automatically installs a lot of amazing packages and some of them are needed to run this program. If you don't plan to start working with your own Python codes, a [classic installation](https://www.python.org/downloads/) is recommended as it will use less storage space. You will then need to install packages manually as shown below.

This project uses various packages that need to be installed to run the program correctly :

* [Numpy](https://numpy.org/) (installed with Anaconda)
* [Sympy](https://www.sympy.org) (installed with Anaconda)
* [Anytree](https://anytree.readthedocs.io) (**not** installed with Anaconda)
* [PyQt5](https://pypi.org/project/PyQt5/) (**not** installed with Anaconda)

If you don't have these installed, you can install them running :

```bash
pip install numpy
pip install sympy
pip install anytree
pip install PyQt5
```

The others parkages used in this project should have been installed with Python. However, if you encounter a `ModuleNotFoundError` when running URDFast, consider installing the missing package running :

```bash
pip install <packageName>
```

### 2. Download URDFast

First, clone this repository running in a shell :
```bash
git clone https://github.com/Teskann/URDFast
```
You can also download this repository as ZIP file if you don't have `git` installed on your computer.

### 3. Launch URDFast

Once you have got the files, you can launch the program running `urdfast.py` file :
```bash
python urdfast.py
```

## Main window

The main window of the program looks like this :

![alt text](./Images/urdfast_main_window.png "The main window of URDFast")

The window is divided in three parts :
* Robot Information
* Code Generator
* Settings

### Open a File

To open a file, simply click on the *OPEN URDF FILE* button at the top center or click on *File > Open* (`Ctrl + O`).

This should open a file dialog in the *Examples* directory. You can pick one of the files in here to try the software or open your own URDF file.

File openning can take a few seconds, especially if you have exotic joints transformations. This is normal and if your OS says URDFast is not responding, just wait, it shouldn't have crashed. The progress bar at the bottom will give you an idea of the remaining time.

### Robot Information (left)

This section contains the robot information. You can not edit content in this section.

The "*Description*" tab is a text area where you can see some details about your robot :
* Robot Name
* Number of Joints
* Number of Links
* Total Mass

The "*Tree Representation*" tab displays the tree representation of the robot. Take a look at it to check if it corresponds to the architecture your URDF file is supposed to describe. There shouldn't be bugs here, if the architecture is not correct, your URDF file is probably wrong.

### Code Generator (middle)

This section is the heart of the software. It is where you select what you want to generate. It is composed of 4 tabs.

#### Transition Matrices

This is where you select which transition matrices you want to generate. Transition matrices are expressed as 4x4 matrices in [homogeneous coordinates](https://en.wikipedia.org/wiki/Homogeneous_coordinates). They are associated with joints. If a joint `J` links two links `L1` and `L2`, the forward transition matrix associated to `J` is the expression of `L2` coordinates in the `L1`frame. The backward transition matrix associated to `J` is the expression of `L1` coordinates in the `L2`frame (this is actually the invert of the forward one). These matrices can depend on variables if `J` has degrees of freedom.

By default, when you open a file, all joints are preselected for generation, forward and backward. You can remove some of them if you don't need them clicking on *Remove Selected* (multiple selection is allowed). To add a joint, select it in the combo box and click *Add*. Note that you can't add a joint that is already in the list.

You have to check the checkboxes if you want the code to be generated.

#### Forward Kinematics

This where you select which [forward kinematics](https://en.wikipedia.org/wiki/Forward_kinematics) functions you want to generate. The forward kinematics are expressed as 4x4 matrices in [homogeneous coordinates](https://en.wikipedia.org/wiki/Homogeneous_coordinates).

Let's suppose a robot has the chain `L1 => J1 => L2 => ... => JN-1 => LN` where `Lk`are links and `Jk` are joints for all `k` in `{1, ... , N}`. The forward kinematics function having `L1` as origin and `LN` as destination will return the coordinates of `LN` in `L1` frame. The forward kinematics function having `LN` as origin and `L1` as destination will return the coordinates of `L1` in `LN` frame.

The code generated by this tool will call transition matrices functions associated with all the joints in the chain between the origin and destination. In the example above, the generated code will call transition matrices for `J1, ..., JN-1`. **Make sure to generate these functions in the *Transition Matrices* tab or the code will call functions that are not defined**.

By default, when you open a file, all the paths from the root link of the robot to all the leaf links of the robot are preselected for generation. You can remove them clicking on *Remove Selected* (multiple selection is allowed). To add a forward kinematics function, select an origin object in the *origin* combo box and a destination object in the *destination* combo box. Then, click *Add*. Note that you can't add a forward kinematics function that is already in the list.

#### Jacobians

This where you select which [Jacobian matrices](https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant) functions you want to generate. The jacobians are expressed as 6xN matrices where N is the number of degrees of freedom of the robot between the origin and the destination (terms defined below).

Let's suppose a robot has the chain `L1 => J1 => L2 => ... => JN-1 => LN` where `Lk`are links and `Jk` are joints with one degree of freedom for all `k` in `{1, ... , N}`. The jacobian function having `L1` as origin and `LN` as destination will return the derivative of the coordinates of `LN` in `L1` frame with respect to all the `N` degrees of freedom as follows :

```
/ dX              dX  \
| ---     ...     --- |
| dD1             dDN |
|                     |
| dY              dY  |
| ---     ...     --- |
| dD1             dDN |
|                     |
| dZ              dZ  |
| ---     ...     --- |
| dD1             dDN |
|                     |
| dR              dR  |
| ---     ...     --- |
| dD1             dDN |
|                     |
| dP              dP  |
| ---     ...     --- |
| dD1             dDN |
|                     |
| dy              dy  |
| ---     ...     --- |
\ dD1             dDN /
```

where :
* `d` represents the differentiation operator, 
* `X`, `Y` and `Z` are the cartesian coordinates of `LN` in the `L1` frame,
* `R`, `P` and `y` represent the roll, pitch and yaw of `LN` in the `L1` frame
* `Dk` represents the variable associated with the degree of freedom of the joint `k` for `k` in `{1, ... , N}`.

The code generated by this tool will call transition matrices functions associated with all the joints in the chain between the origin and destination. In the example above, the generated code will call transition matrices for `J1, ..., JN-1`. **Make sure to generate these functions in the *Transition Matrices* tab or the code will call functions that are not defined**.

By default, when you open a file, all the paths from the root link of the robot to all the leaf links of the robot are preselected for generation. You can remove them clicking on *Remove Selected* (multiple selection is allowed). To add a jacobian function, select an origin object in the *origin* combo box and a destination object in the *destination* combo box. Then, click *Add*. Note that you can't add a jacobian function that is already in the list.

#### Center of Mass

In this section, you will find two combo boxes to generate the center of mass function and its jacobian matrix.

The center of mass is expressed as a 4x1 matrix in [homogeneous coordinates](https://en.wikipedia.org/wiki/Homogeneous_coordinates). The first three coordinates represent the cartesian coordinates of the center of mass of the robot in the world frame, and the 4th coordinate is always equal to 1. There is no way to change the frame for the expression of the center of mass yet. To express it in another frame, multiply the forward kinematics function going from the world frame to the desired frame by the center of mass vector. If the robot mass is null, the code is not generated as the center of mass is not defined. You have to write explicitly the mass of each link in the URDF file (the default mass for URDF links is 0 kg).

The [Jacobian matrix](https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant) of the center of mass is expressed as a 3xN matrix where N is the number of degrees of freedom of the robot that have an impact on the center of mass position. Here is what the matrix looks like :

```
/ dX              dX  \
| ---     ...     --- |
| dD1             dDN |
|                     |
| dY              dY  |
| ---     ...     --- |
| dD1             dDN |
|                     |
| dZ              dZ  |
| ---     ...     --- |
\ dD1             dDN /
```

where :
* `d` represents the differentiation operator, 
* `X`, `Y` and `Z` are the cartesian coordinates of the center of mass in the world frame,
* `Dk` represent the variable associated with the degree of freedom of the joint `k` for `k` in `{1, ... , N}`.

Both the center of mass and the jacobian of the center of mass call transition matrices functions associated with all the joints having an impact on the center of mass position. **Make sure to generate these functions in the *Transition Matrices* tab or the code will call functions that are not defined**.

### Settings

In this section are located the settings of URDFast. You can change the language or the output file name (don't add the file extension to the file name, it will be added automatically during code generation).

Don't forget to click on *Apply* after you made your changes otherwise they won't be applied.

## Supported Files and Robot Types

This program only supports XML URDF formatted files. XACRO files are not supported. If you don't know what is the URDF format, take a look at the [official documentation](http://wiki.ros.org/urdf/XML).
Moreover, this program only supports **robots with a tree-like representation**. If you have loops in your physical architecture, it won't work because of infinite loops. This kind of robots may never be supported by this software as the tree representation is fundamental in the URDFast algorithms.

Try to avoid commented lines `<!--...-->` in your URDF file, as some parsing errors have been detected with this, for some reason. This should be patched in a future version. If you can't parse the file althrough it should be correct, try to keep only `<link>` and `<joint>` elements (they are the only one used by URDFast). If the parser keeps failing, consider reporting the bug.

If you open a not-supported file, it will throw an error and this can lead to the program dies, depending on your operating system (errors are not handeled in the GUI yet).

## Generated Code

The generated files are stored in the `generated` directory.

All the generated functions come with their docstring explaining how to use them correctly.

### Python

The Python generated code uses [Numpy](https://numpy.org/) package to work with matrix computations. Consider installing it on the device on which you want to implement your controller running :
```bash
pip install numpy
```
The other functions like `cos` or `sin` are imported from `math` package, which is built-in.

[Here](../generated/Examples/test_robot.py) is an example of what the generated code looks like running on `example_0.urdf` with the default setting.

### Julia

The Julia generated code uses the [LinearAlgebra](https://docs.julialang.org/en/v1/stdlib/LinearAlgebra/) module to work with matrix computations. This package should be installed with Julia.

[Here](../generated/Examples/test_robot.jl) is an example of what the generated code looks like running on `example_0.urdf` with the default settings.
