# Developper Documentation

Welcome to the developper documentation. Thanks for your interest in this project. You will find here all you need to contribute to the project.

## Project Overview

This project generates code from URDF files. The geometrical informations stored in URDF files are processed by the software to generate the code. All the computations are done with [Sympy](https://www.sympy.org), which is a tool for symbolic computation. To manage the tree representation of the robot (and mathematical expressions strings operation decomposition), the package [Anytree](https://anytree.readthedocs.io) is used.

## Coding Conventions

To keep the same shape for every file in the project, please follow the following coding conventions :
* Keep the line lengths under 79 characters (max 78 are used in this project but Python standards use 79)
* Use [snake case](https://en.wikipedia.org/wiki/Snake_case) for variable and function names
* Write a docstring for each function following the [Numpydoc](https://numpydoc.readthedocs.io/en/latest/) format. The docstring must contain the function description, the parameters names values and types, the return value, global variables used (if used).
* Please split the files with ascii separator. For big titles consider writing them like this :
```
# ----------------- ... (79 characters) ... -----
# | Title           ... (79 characters) ...     |
# ----------------- ... (79 characters) ... -----
```
For h2, h3 and h4 titles please write your title followed by `_`, `....` or `. . .` until 79 characters

## Online Documentation

You will find all the documentation of URDFast Python files here : https://teskann.github.io/urdfast

## Files & Folders

This project contains the following folders.

### URDFast

This folder is the root of the project. It is where all the backend scripts are located. It contains the following files :
* **code_generator.py** : Script used for code generation. It is where you can find all the functions to generate transition matrices, forward kinematics and so on. If you wish to add a new category of functions to generate, this is the file where you should add your feature.

* **code_optimization.py** : Script used to work with mathematical expressions as strings. There are tools to detect all the operations in a string expression, replace some operators by functions and so on.

* **createRobotFromURDF.py** : File defining the `Joint`, `Link` and `Robot` objects so the data can be prossessed in Python from URDF objects.

* **Language.py** : This is the file defining the `Language` objects. These objects contain methods to generate functions in a specified language.

* **urdfast.py** : This fils is the main program to run to launch URDFast.

* **URDF.py** : URDF parser

### URDFast/documentation

This folder contains all the documentation files.

### URDFast/documentation/Images

This folder contains images that are supposed to be displayed in the documentation markdown files.

### URDFast/Examples

This folder contains 2 basic URDF files that are used as examples.

### URDFast/generated

This is the place where all the generated files are located.

### URDFast/generated/Examples

Contains the generated files of the examples URDF files.

### URDFast/GUI

This folder contains all the files used to run the GUI.

You will find :

* **dark.qss** : Qt style sheet file to style the GUI

* **main_gui.py** : main function of the GUI, setting up all the functions for buttons, actions etc ...

* **main_window.py** : Automatically generated file from the `main_window.ui` file.

### URDFast/GUI/dark

Contains all the SVG files used in the URDFast theme.

### URDFast/GUI/Images

Contains images such as URDFast logo.