# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 18:20:27 2020

@author: Clément
"""

import logging
import sys

from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import QUrl
from PyQt5.QtGui import QDesktopServices, QPalette
from PyQt5.QtGui import QFontDatabase
from PyQt5.QtWidgets import QFileDialog, QTreeWidgetItem, QListWidgetItem, \
    QTableWidgetItem

sys.path.insert(1, '../')

import URDF
import robots as cr
from dh_params import dh

from code_generator import generate_everything
from Language import Language

# make the example runnable without the need to install

import main_window

# Global variables ___________________________________________________________

ftm_list = []  # Forward transition matrices list
btm_list = []  # Backward transition matrices list
fk_list = []  # Forward Kinematics List
jac_list = []  # Jacobian list
polynomial_trajectories = []  # Polynomial trajectories list

robot_obj = 0  # Robot Object

settings = {"language": "Julia",
            "filename": "out"}

path = './GUI/'


# Add joint to a list from combobox __________________________________________

def add_joint_to_list(list_widget, combo_box, add_btn, del_btn, forward):
    """
    Description
    -----------
    
    Add an item to the List Widget from the selected item in the combo box.
    The add button may be disabled if all the joints are already in the list.
    
    Parameters
    ----------
    
    list_widget : PyQt5.QtWidgets.QListWidget
        List to add an element to
        
    combo_box : PyQt5.QtWidgets.QComboBox
        Combo box wrom which the item is added
    
    add_btn : PyQt5.QtWidgets.QPushButton
        Add button that may be disabled
    
    del_btn : PyQt5.QtWidgets.QPushButton
        Delete button that has to be reenabled when an element is added
        
    forward : bool
        True if the list to update is the Forward transition matrices list
        False if the list to update is the Backward transition matrices list
    
    Global Variables Used
    ---------------------
    
    ftm_list : list of string
        list of all the joints to generate the code
    
    btm_list : list of string
        list of all the joints to generate the code
    
    robot_obj : robots.Robot
        Robot Object
    
    Returns
    -------
    
    None.
    
    """

    global ftm_list  # Forward transition matrices list
    global btm_list  # Backward transition matrices list
    global robot_obj

    # Getting the current item
    ind = combo_box.currentIndex()

    # Finding the associated joint
    i_joint = 0
    for _, _, node in robot_obj.tree:
        type_, nb = node.name.split('_')
        nb = int(nb)

        if type_ == 'joint':
            if forward:
                if 'joint_' + str(nb) in ftm_list:
                    i_joint += 1
                    continue
            else:
                if 'joint_' + str(nb) in btm_list:
                    i_joint += 1
                    continue
            if ind == nb:
                text = robot_obj.joints[nb].name
                list_widget.addItem(text)

                # Disabling the item in the combo box
                combo_box.model().item(i_joint).setEnabled(False)

                # If all the joints are added
                if list_widget.count() == combo_box.count():
                    add_btn.setEnabled(False)
                del_btn.setEnabled(True)

                if forward:
                    ftm_list.append("joint_" + str(nb))
                else:
                    btm_list.append("joint_" + str(nb))

            i_joint += 1


# Remove a joint from a list _________________________________________________

def del_joint_from_list(list_widget, combo_box, add_btn, del_btn, forward):
    """
    Description
    -----------
    
    Removes the selected item in the list widget
    
    Parameters
    ----------
    
    list_widget : PyQt5.QtWidgets.QListWidget
        List in which you want to delete the item
        
    combo_box : PyQt5.QtWidgets.QComboBox
        Combo box linked to this list (to enable the item)
    
    add_btn : PyQt5.QtWidgets.QPushButton
        Add button that has to be enabled when an item is deleted
    
    del_btn : PyQt5.QtWidgets.QPushButton
        Delete button that may be disabled
        
    forward : bool
        True if the list to update is the Forward transition matrices list
        False if the list to update is the Backward transition matrices list
    
    Global Variables Used
    ---------------------
    
    ftm_list : list of string
        list of all the joints to generate the code
    
    btm_list : list of string
        list of all the joints to generate the code
    
    robot_obj : robots.Robot
        Robot Object
    
    Returns
    -------
    
    None.
    
    """

    global ftm_list  # Forward transition matrices list
    global btm_list  # Backward transition matrices list
    global robot_obj

    # Getting the selected items
    selection = list_widget.selectedItems()

    for item in selection:
        # Finding the associated joint
        i_joint = 0
        for _, _, node in robot_obj.tree:
            type_, nb = node.name.split('_')
            nb = int(nb)

            if type_ == 'joint':
                if robot_obj.joints[nb].name == item.text():
                    list_widget.takeItem(list_widget.row(item))

                    # Enabling the item in the combo box
                    combo_box.model().item(i_joint).setEnabled(True)

                    # If all the joints are added
                    if list_widget.count() == 0:
                        del_btn.setEnabled(False)
                    add_btn.setEnabled(True)

                    if forward:
                        ftm_list.remove("joint_" + str(nb))
                    else:
                        btm_list.remove("joint_" + str(nb))

                i_joint += 1


# Add FK or Jacobian _________________________________________________________

def add_fk_jac(list_widget, combo_box_o, combo_box_d, fk):
    """
    Description
    -----------
    
    Adds the fk or the jacobian to the list
    
    Parameters
    ----------
    
    list_widget : PyQt5.QtWidgets.QListWidget
        List in which you want to add the item
        
    combo_box_o : PyQt5.QtWidgets.QComboBox
        Combo box linked to this list containing origins
    
    combo_box_d : PyQt5.QtWidgets.QComboBox
        Combo box linked to this list containing origins
        
    fk : bool
        True if the list to update is the Forward Kinematics list
        False if the list to update is the Jacobian matrices list
    
    Global Variables Used
    ---------------------
    
    fk_list : list of string
        list of all the joints to generate the code
    
    jac_list : list of string
        list of all the joints to generate the code
    
    robot_obj : robots.Robot
        Robot Object
    
    Returns
    -------
    
    None.
    
    """

    global fk_list
    global jac_list

    # Getting the current item
    ind_o = combo_box_o.currentIndex()
    ind_d = combo_box_d.currentIndex()

    names = [0, 0]

    ids = [0, 0]

    # Finding the corresponding object
    i = 0
    for _, _, node in robot_obj.tree:
        for od, ind in enumerate([ind_o, ind_d]):
            if i == ind:
                type_, nb = node.name.split('_')
                nb = int(nb)

                if type_ == 'joint':
                    names[od] = robot_obj.joints[nb].name
                else:
                    names[od] = robot_obj.links[nb].name

                ids[od] = node.name
                continue

        i += 1

    if fk:
        if ids in fk_list:
            return
        fk_list.append(ids)
    else:
        if ids in jac_list:
            return
        jac_list.append(ids)

    list_widget.addItem(names[0] + '  ==>  ' + names[1])


# Remove FK or Jacobian ______________________________________________________

def del_fk_jac(list_widget, fk):
    """
    Description
    -----------
    
    Removes the fk or the jacobian from the list
    
    Parameters
    ----------
    
    list_widget : PyQt5.QtWidgets.QListWidget
        List in which you want to remove the item
        
    fk : bool
        True if the list to update is the Forward Kinematics list
        False if the list to update is the Jacobian matrices list
    
    Global Variables Used
    ---------------------
    
    fk_list : list of string
        list of all the joints to generate the code
    
    jac_list : list of string
        list of all the joints to generate the code
    
    robot_obj : robots.Robot
        Robot Object
    
    Returns
    -------
    
    None.
    
    """

    global fk_list
    global jac_list
    global robot_obj

    # Getting the selected items
    selection = list_widget.selectedItems()

    for item in selection:

        index = list_widget.row(item)

        list_widget.takeItem(index)

        if fk:
            del fk_list[index]
        else:
            del jac_list[index]


# Add a polynomial trajectory ________________________________________________

def new_polynomial_trajectory(ui):
    """
    Description
    -----------

    Create a new polynomial trajectory

    Parameters
    ----------

    ui : main_window.Ui_MainWindow
        GUI to update

    Global Variables Used
    ---------------------

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

    Returns
    -------

    None.

    """

    global polynomial_trajectories

    # Finding a Name .........................................................

    k = 0
    while "r" + str(k) in [t["name"] for t in polynomial_trajectories]:
        k += 1

    trajectory = {"name": "r" + str(k),
                  "conditions": []}

    # Deselect everything ....................................................

    ui.listWidget_poly.clearSelection()

    # Adding to the list widget ..............................................

    item = QListWidgetItem()
    item.setText(trajectory["name"])
    ui.listWidget_poly.addItem(item)
    ui.lineEdit_poly_fname.setText("r" + str(k))
    polynomial_trajectories.append(trajectory)
    ui.listWidget_poly.setCurrentRow(ui.listWidget_poly.count() - 1)
    ui.lineEdit_poly_fname.setStyleSheet("color: #efefef;")

    if len(polynomial_trajectories) == 1:
        ui.pushButton_poly_del.setEnabled(True)


# Remove polynomial trajectory _______________________________________________

def del_polynomial_trajectory(ui):
    """
    Description
    -----------

    Removes the selected polynomial trajectory

    Parameters
    ----------

    ui : main_window.Ui_MainWindow
        GUI to update

    Global Variables Used
    ---------------------

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

    Returns
    -------

    None.

    """

    global polynomial_trajectories

    # Getting the selected items
    selection = ui.listWidget_poly.selectedItems()

    for item in selection:
        index = ui.listWidget_poly.row(item)

        ui.listWidget_poly.takeItem(index)
        del polynomial_trajectories[index]

    if not polynomial_trajectories:
        ui.pushButton_poly_del.setEnabled(False)


# Update trajectory name on text change ______________________________________

def update_trajectory_name(ui):
    """
    Description
    -----------

    Update the trajectory name on editing name Line Edit.

    Parameters
    ----------

    ui : main_window.Ui_MainWindow
        GUI to update

    Global Variables Used
    ---------------------

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

    Returns
    -------

    None.

    """

    global polynomial_trajectories

    selection = ui.listWidget_poly.selectedItems()

    index = None
    for item in selection:
        index = ui.listWidget_poly.row(item)

    if index is None:
        return

    content = ui.lineEdit_poly_fname.text()
    original_content = content

    # Avoiding non alphanumeric characters ...................................

    i = 0
    while i < len(content):
        char = content[i]
        # Avoiding whitespaces and -
        if char in [" ", "-", ".", ",", ";"]:
            content = content[:i] + "_" + content[i + 1:]
            i += 1
        elif not (char.isascii() and char.isalnum() or char == "_"):
            content = content[:i] + content[i + 1:]
        else:
            i += 1

    # Avoiding empty name ....................................................

    if content == "":
        k = 0
        while "r" + str(k) in [t["name"] for t in polynomial_trajectories]:
            k += 1
        content = "r" + str(k)

    # Avoiding name starting by a number .....................................

    if content[0:1].isdigit():
        content = "r" + content

    # Avoiding duplicates ....................................................

    k = 0
    while content in [t["name"] for t in
                      polynomial_trajectories[:index] +
                      polynomial_trajectories[index+1:]]:
        if k == 0:
            content += '_0'
        else:
            content = '_'.join(content.split('_')[:-1]) + '_' + str(k)
        k += 1

    # Color text if the name is invalid ......................................

    if content == original_content:
        ui.lineEdit_poly_fname.setStyleSheet("color: #efefef;")
    else:
        ui.lineEdit_poly_fname.setStyleSheet("color: #ff2020;")

    # Set content in the list ................................................

    for item in selection:
        index = ui.listWidget_poly.row(item)
        item.setText(content)
        polynomial_trajectories[index]["name"] = content


# Display conditions on item selected ________________________________________

def display_trajectory_conditions(ui):
    """
    Description
    -----------

    Display  the  trajectory  details  (conditions)  on the bottom part of the
    "Polynomial  Trajectories"  tab.  The displayed trajectory is the selected
    one.  This function is supposed to be called when the trajectory selection
    is updated.

    Parameters
    ----------

    ui : main_window.Ui_MainWindow
        GUI to update

    Global Variables Used
    ---------------------

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

    Returns
    -------

    None.

    """

    # Clear table content ....................................................

    ui.tableWidget_poly_conditions.setRowCount(0)

    # Get the current selected item ..........................................

    global polynomial_trajectories

    selection = ui.listWidget_poly.selectedItems()

    index = None
    for item in selection:
        index = ui.listWidget_poly.row(item)

    # Nothing is selected => Disable the content
    # Item selected => Enable the content
    ui.lineEdit_poly_fname.setEnabled(index is not None)
    ui.tableWidget_poly_conditions.setEnabled(index is not None)
    ui.pushButton_poly_new_condition.setEnabled(index is not None)
    ui.pushButton_poly_del_condition.setEnabled(index is not None)

    if index is None:
        return

    ui.lineEdit_poly_fname.setText(polynomial_trajectories[index]["name"])

    # Fill content with conditions ...........................................

    for condition in polynomial_trajectories[index]["conditions"]:
        pos = ui.tableWidget_poly_conditions.rowCount()
        ui.tableWidget_poly_conditions.insertRow(pos)

        ui.tableWidget_poly_conditions.setItem(pos, 0,
                                               QTableWidgetItem(condition[0]))
        ui.tableWidget_poly_conditions.setItem(pos, 1,
                                               QTableWidgetItem(condition[1]))
        ui.tableWidget_poly_conditions.setItem(pos, 2,
                                               QTableWidgetItem(condition[2]))


# Add a condition to the selected polynomial trajectory ______________________

def new_condition_polynomial_trajectory(ui):
    """
    Description
    -----------

    Add a condition to the polynomial trajectory. This basically adds a row to
    the table with default conditions (k = 0 ; t = 0 ; x = 0).

    Parameters
    ----------

    ui : main_window.Ui_MainWindow
        GUI to update

    Global Variables Used
    ---------------------

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

    Returns
    -------

    None.

    """

    # Add the row ............................................................

    pos = ui.tableWidget_poly_conditions.rowCount()
    ui.tableWidget_poly_conditions.insertRow(pos)

    ui.tableWidget_poly_conditions.setItem(pos, 0, QTableWidgetItem("0"))
    ui.tableWidget_poly_conditions.setItem(pos, 1, QTableWidgetItem("0"))
    ui.tableWidget_poly_conditions.setItem(pos, 2, QTableWidgetItem("0"))

    # Save to global variable ................................................

    global polynomial_trajectories

    selection = ui.listWidget_poly.selectedItems()

    index = None
    for item in selection:
        index = ui.listWidget_poly.row(item)
    if index is None:
        return

    polynomial_trajectories[index]["conditions"].append(["0", "0", "0"])


# Delete polynomial trajectory condition(s) __________________________________

def delete_polynomial_trajectory_condition(ui):
    """
    Description
    -----------

    Delete all the conditions (rows) that have at least one cell selected.

    Parameters
    ----------

    ui : main_window.Ui_MainWindow
        GUI to update

    Global Variables Used
    ---------------------

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

    Returns
    -------

    None.

    """

    # Get selected list item .................................................

    global polynomial_trajectories

    selection = ui.listWidget_poly.selectedItems()

    index_list = None
    for item in selection:
        index_list = ui.listWidget_poly.row(item)
    if index_list is None:
        return

    # Find rows to delete ....................................................

    rows = []
    for item in ui.tableWidget_poly_conditions.selectedIndexes():
        row = item.row()
        if row not in rows:
            rows.append(row)

    # Delete the rows ........................................................

    rows.sort(reverse=True)
    for row in rows:
        polynomial_trajectories[index_list]["conditions"].pop(row)
        ui.tableWidget_poly_conditions.removeRow(row)


# Edit polynomial trajectory condition _______________________________________

def edit_polynomial_trajectory_condition(ui):
    """
    Description
    -----------

    Save the edited content from the table to polynomial_trajectories. This is
    supposed to be called when the table is edited.

    Parameters
    ----------

    ui : main_window.Ui_MainWindow
        GUI to update

    Global Variables Used
    ---------------------

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

    Returns
    -------

    None.

    """

    if ui.tableWidget_poly_conditions.currentItem() is None:
        return

    # Get selected list item .................................................

    global polynomial_trajectories

    selection = ui.listWidget_poly.selectedItems()

    index_list = None
    for item in selection:
        index_list = ui.listWidget_poly.row(item)
    if index_list is None:
        return

    # Find the cell to edit ..................................................

    row = ui.tableWidget_poly_conditions.currentItem().row()
    col = ui.tableWidget_poly_conditions.currentItem().column()

    content = ui.tableWidget_poly_conditions.currentItem().text()

    # Check if the number is correct .........................................

    content = content.replace(",", ".")

    try:
        float(content)

    # It's a symbolic variable
    except ValueError:

        # Avoiding non alphanumeric characters ...............................

        i = 0
        while i < len(content):
            char = content[i]
            # Avoiding whitespaces and -
            if char in [" ", "-", ",", ";"]:
                content = content[:i] + "_" + content[i + 1:]
                i += 1
            elif not (char.isascii() and char.isalnum() or char == "_"):
                content = content[:i] + content[i + 1:]
            else:
                i += 1

        # Avoiding name starting by a number .................................

        if content[0:1].isdigit():
            letters = ["k", "t", "x"]
            content = letters[col] + content

    if col == 0:
        try:
            int(content)
            if int(content) > 0:
                content = str(int(content))
            else:
                content = "0"
        except ValueError:
            content = "0"

    ui.tableWidget_poly_conditions.currentItem().setText(content)

    if content == "":
        content = "0"

    polynomial_trajectories[index_list]["conditions"][row][col] = content


# Update GUI State from Robot Object _________________________________________

def init_gui_from_robot(gui, robot):
    """
    Description
    -----------
    
    Updates the GUI with respect to the robot given as parameter
    
    Parameters
    ----------
    
    gui : main_window.Ui_MainWindow
        GUI to update
    
    robot : robots.Robot
        Robot Object to update the GUI
    
    Global Variables Used
    ---------------------
    
    ftm_list : list of string
        list of all the joints to generate the code
    
    btm_list : list of string
        list of all the joints to generate the code
    
    fk_list : list of string
        List of all the forward kinematics to generate
    
    jac_list : list of string
        List of all the jacobians
    
    Returns
    -------
    
    None.
    
    """

    global ftm_list  # Forward transition matrices list
    global btm_list  # Backward transition matrices list
    global fk_list
    global jac_list

    # Robot Information ......................................................

    # Paragraph syntax
    p = '<p style=" margin-top:0px; margin-bottom:0px; margin-left:0px; ' + \
        'margin-right:0px; -qt-block-indent:0; text-indent:0px;">'
    header = '<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" ' + \
             '"http://www.w3.org/TR/REC-html40/strict.dtd">\n<html><head' \
             '><meta' + \
             'name="qrichtext" content="1" /><style type="text/css">\np, ' \
             'li { ' + \
             'white-space: pre-wrap; }\n</style></head><body style=" ' \
             'font-family' + \
             ':\'MS Shell Dlg 2\'; font-size:10pt; font-weight:400; ' + \
             'font-style:normal;">\n'

    html = header + p + "<b>Robot Name</b> : " + f"{robot.name}</p><br/>\n"
    html += p + "<b>Number of Joints</b> : " + f"{robot.njoints()}</p><br/>\n"
    html += p + "<b>Number of Links</b> : " + f"{robot.nlinks()}</p><br/>\n"
    mass = 0
    for link in robot.links:
        mass += link.mass
    html += p + "<b>Mass</b> : " + '%.3f' % mass + " kg</p>"
    html += "</body></html>"

    gui.textEdit_info.setHtml(html)

    # Tree Representation ....................................................

    gui.treeWidget_info.clear()
    gui.treeWidget_info.setHeaderLabels(['Name', 'Type'])

    list_items = []
    list_nodes = []

    i = 0

    # Iterating over the tree
    for _, _, node in robot.tree:

        # Finding parent
        if node.is_root:
            parent = gui.treeWidget_info
        else:
            parent = list_items[list_nodes.index(node.parent)]

        item = QTreeWidgetItem(parent)

        # Text
        type_, nb = node.name.split('_')
        nb = int(nb)

        if type_ == 'joint':
            text = robot.joints[nb].name
            text1 = robot.joints[nb].joint_type + " Joint"
        else:
            text = robot.links[nb].name
            text1 = "Link"

        item.setText(0, text)
        item.setText(1, text1.title())

        list_items.append(item)
        list_nodes.append(node)
        i += 1

        # Expand the whole tree
        gui.treeWidget_info.expandItem(item)

    # Transition Matrices ....................................................

    ftm_list = []  # Forward transition matrices list
    btm_list = []  # Backward transition matrices list

    # Adding each matrix to both lists
    gui.listWidget_ftm.clear()
    gui.listWidget_btm.clear()
    gui.comboBox_btm_joint.clear()
    gui.comboBox_ftm_joint.clear()
    i = 0
    for pre, _, node in robot.tree:
        # Text
        type_, nb = node.name.split('_')
        nb = int(nb)

        if type_ == 'joint':
            text = robot.joints[nb].name
            gui.listWidget_ftm.addItem(text)
            gui.listWidget_btm.addItem(text)

            # Combo boxes
            gui.comboBox_btm_joint.addItem(pre + robot.joints[nb].name)
            gui.comboBox_ftm_joint.addItem(pre + robot.joints[nb].name)

            gui.comboBox_ftm_joint.model().item(i).setEnabled(False)
            gui.comboBox_btm_joint.model().item(i).setEnabled(False)

            ftm_list.append('joint_' + str(i))
            btm_list.append('joint_' + str(i))
            i += 1

    gui.pushButton_ftm_add.setEnabled(False)
    gui.pushButton_btm_add.setEnabled(False)

    # Froward Kinematics and Jacobians .......................................

    fk_list = []
    jac_list = []

    gui.listWidget_fk.clear()
    gui.listWidget_jac.clear()
    gui.comboBox_fk_origin.clear()
    gui.comboBox_jac_origin.clear()
    gui.comboBox_fk_destination.clear()
    gui.comboBox_jac_destination.clear()

    all_roots = []
    all_leaves = []

    # Adding every root --> leaf path to the FK list
    for pre, _, node in robot.tree:

        # Text
        type_, nb = node.name.split('_')
        nb = int(nb)

        if node.is_root:
            all_roots.append(node.name)

        if node.is_leaf:
            all_leaves.append(node.name)

        # Combo boxes
        if type_ == 'joint':
            name = robot.joints[nb].name
        else:
            name = robot.links[nb].name

        gui.comboBox_fk_origin.addItem(pre + name)
        gui.comboBox_jac_origin.addItem(pre + name)
        gui.comboBox_fk_destination.addItem(pre + name)
        gui.comboBox_jac_destination.addItem(pre + name)

    for root in all_roots:
        root_type, rnb = root.split('_')
        rnb = int(rnb)
        if root_type == 'joint':
            root_name = robot.joints[rnb].name
        else:
            root_name = robot.links[rnb].name

        for leaf in all_leaves:
            fk_list.append([root, leaf])
            jac_list.append([root, leaf])
            leaf_type, lnb = leaf.split('_')
            lnb = int(lnb)
            if leaf_type == 'joint':
                leaf_name = robot.joints[lnb].name
            else:
                leaf_name = robot.links[lnb].name

            gui.listWidget_fk.addItem(root_name + '  ==>  ' + leaf_name)
            gui.listWidget_jac.addItem(root_name + '  ==>  ' + leaf_name)

    # Checking checkboxes
    gui.checkBox_ftm.setChecked(True)
    gui.checkBox_btm.setChecked(False)
    gui.checkBox_fk.setChecked(True)
    gui.checkBox_jac.setChecked(True)
    gui.checkBox_com.setChecked(True)
    gui.checkBox_com_jac.setChecked(True)

    # Polynomial Trajectories ................................................

    gui.pushButton_poly_del.setEnabled(True)
    gui.pushButton_poly_new_traj.setEnabled(True)
    gui.pushButton_poly_del_condition.setEnabled(False)
    gui.pushButton_poly_new_condition.setEnabled(False)
    gui.listWidget_poly.setEnabled(True)
    gui.tableWidget_poly_conditions.setEnabled(False)
    gui.lineEdit_poly_fname.setEnabled(True)

    gui.lineEdit_fname.setText(robot.name)
    update_settings(gui)

    gui.pushButton_generate.setEnabled(True)


# Open a file dialog _________________________________________________________

def open_file_dialog(gui, progress_bar):
    """
    Description
    -----------
    
    Open a QT file dialog and open the file + parse URDF
    
    Parameters
    ----------
    
    gui : main_window.Ui_MainWindow
        GUI to update after the file is opened
    
    progress_bar : PyQt5.QtWidgets.QProgressBar
        Progressbar to update

    Returns
    -------
    
    robots.Robot
        Robot object of the opened file

    """

    # File dialog
    fname, ftype = QFileDialog\
        .getOpenFileName(caption="Open URDF File",
                         filter="Supported files (*.urdf *.dhparams)"
                                ";;All files (*)",
                         directory=path + '../Examples')
    if fname == '':
        return
    global robot_obj
    # Open the file
    if fname.split(".")[-1].lower() == "urdf":
        with open(fname) as file:
            urdf_obj = URDF.URDF(file)
            robot_obj = cr.RobotURDF(urdf_obj, progress_bar)

    elif fname.split(".")[-1].lower() == "dhparams":
        dh_obj = dh(fname)
        robot_obj = cr.RobotDH(dh_obj)
    init_gui_from_robot(gui, robot_obj)



def update_settings(gui):
    """
    Description
    -----------
    
    Updates the settings window
    
    Parameters
    ----------
    
    gui : main_window.Ui_MainWindow
        GUI to update
        
    Global Variable Used
    --------------------
    
    settings : dict
        Dictionary of the settings
    
    """

    settings["filename"] = gui.lineEdit_fname.text()
    ind = gui.comboBox_language.currentIndex()
    settings["language"] = gui.comboBox_language.itemText(ind)


def generate(gui):
    """
    Description
    -----------
    
    Generates the code from selection
    
    Parameters
    ----------
    
    gui : main_window.Ui_MainWindow
        GUI to update
        
    Global Variables Used
    ---------------------
    
    robot_obj : robots.Robot
        Robot object
    
    ftm_list : list of string
        list of all the joints to generate the code
    
    btm_list : list of string
        list of all the joints to generate the code
    
    fk_list : list of string
        List of all the forward kinematics to generate
    
    jac_list : list of string
        List of all the jacobians
    
    settings : dict
        Dictionnary of the settings for the code generation
    
    Returns
    -------
    
    None.
    
    """

    global robot_obj
    global ftm_list
    global btm_list
    global fk_list
    global jac_list

    ftm = ftm_list if gui.checkBox_ftm.isChecked() else []
    btm = btm_list if gui.checkBox_btm.isChecked() else []
    fk = fk_list if gui.checkBox_fk.isChecked() else []
    jac = jac_list if gui.checkBox_jac.isChecked() else []

    com = gui.checkBox_com.isChecked()
    comjac = gui.checkBox_com_jac.isChecked()

    language = Language(settings["language"])

    generate_everything(robot_obj, ftm, btm,
                        fk, jac, com, comjac,
                        polynomial_trajectories,
                        language,
                        path + '../generated/' + settings["filename"])


# UI Main ____________________________________________________________________

def main():
    """
    Application entry point
    """

    import ctypes
    myappid = u'teskann.urdfast.1.0'  # arbitrary string
    ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)

    global path

    logging.basicConfig(level=logging.DEBUG)
    # create the application and the main window
    app = QtWidgets.QApplication(sys.argv)
    # app.setStyle(QtWidgets.QStyleFactory.create("fusion"))
    window = QtWidgets.QMainWindow()

    # setup ui
    ui = main_window.Ui_MainWindow()
    ui.setupUi(window)

    # Actions ................................................................

    ui.actionOpen.triggered.connect(lambda:
                                    open_file_dialog(ui, ui.progressBar))

    ui.actionClose.triggered.connect(window.close)

    link_str = "https://github.com/Teskann/NYXX/blob/master/documentation" + \
               "/usermanual.md"

    ui.actionDocumentation.triggered.connect(lambda:
                                             QDesktopServices.openUrl(
                                                 QUrl(link_str)))

    link_str2 = "https://github.com/Teskann/NYXX"

    ui.actionView_Github.triggered.connect(lambda:
                                           QDesktopServices.openUrl(
                                               QUrl(link_str2)))

    link_str3 = "https://github.com/Teskann/NYXX/issues/new"

    ui.actionReport_a_Bug.triggered.connect(lambda:
                                            QDesktopServices.openUrl(
                                                QUrl(link_str3)))

    # Opening button .........................................................

    ui.pushButton.clicked.connect(
        lambda: open_file_dialog(ui, ui.progressBar))

    # Transition Matrices ....................................................

    ui.listWidget_btm.selectionMode()

    ui.pushButton_ftm_add.clicked \
        .connect(lambda:
                 add_joint_to_list(ui.listWidget_ftm,
                                   ui.comboBox_ftm_joint,
                                   ui.pushButton_ftm_add,
                                   ui.pushButton_ftm_del,
                                   True))

    ui.pushButton_ftm_del.clicked \
        .connect(lambda:
                 del_joint_from_list(
                     ui.listWidget_ftm,
                     ui.comboBox_ftm_joint,
                     ui.pushButton_ftm_add,
                     ui.pushButton_ftm_del, True))

    ui.pushButton_btm_add.clicked \
        .connect(lambda:
                 add_joint_to_list(ui.listWidget_btm,
                                   ui.comboBox_btm_joint,
                                   ui.pushButton_btm_add,
                                   ui.pushButton_btm_del,
                                   True))

    ui.pushButton_btm_del.clicked \
        .connect(lambda:
                 del_joint_from_list(
                     ui.listWidget_btm,
                     ui.comboBox_btm_joint,
                     ui.pushButton_btm_add,
                     ui.pushButton_btm_del, False))

    # Forward Kinematics .....................................................

    ui.pushButton_fk_add.clicked \
        .connect(lambda:
                 add_fk_jac(ui.listWidget_fk,
                            ui.comboBox_fk_origin,
                            ui.comboBox_fk_destination,
                            True))

    ui.pushButton_fk_del.clicked \
        .connect(lambda:
                 del_fk_jac(ui.listWidget_fk, True))

    # Jacobians ..............................................................

    ui.pushButton_jac_add.clicked \
        .connect(lambda:
                 add_fk_jac(ui.listWidget_jac,
                            ui.comboBox_jac_origin,
                            ui.comboBox_jac_destination,
                            False))
    ui.pushButton_jac_del.clicked \
        .connect(lambda:
                 del_fk_jac(ui.listWidget_jac,
                            False))

    # Polynomial Trajectories ................................................

    ui.pushButton_poly_del.setEnabled(False)
    ui.pushButton_poly_new_traj.setEnabled(False)
    ui.pushButton_poly_del_condition.setEnabled(False)
    ui.pushButton_poly_new_condition.setEnabled(False)
    ui.listWidget_poly.setEnabled(False)
    ui.tableWidget_poly_conditions.setEnabled(False)
    ui.lineEdit_poly_fname.setEnabled(False)

    ui.pushButton_poly_new_traj.clicked \
        .connect(lambda: new_polynomial_trajectory(ui))

    ui.pushButton_poly_del.clicked \
        .connect(lambda: del_polynomial_trajectory(ui))

    ui.lineEdit_poly_fname.textChanged \
        .connect(lambda: update_trajectory_name(ui))

    ui.listWidget_poly.itemSelectionChanged \
        .connect(lambda: display_trajectory_conditions(ui))

    ui.pushButton_poly_new_condition \
        .clicked.connect(lambda: new_condition_polynomial_trajectory(ui))

    ui.pushButton_poly_del_condition \
        .clicked.connect(lambda: delete_polynomial_trajectory_condition(ui))

    ui.tableWidget_poly_conditions\
        .itemChanged.connect(lambda: edit_polynomial_trajectory_condition(ui))

    # Font ...................................................................

    font_db = QFontDatabase()
    font_db.addApplicationFont("univers-condensed.ttf")

    # Settings apply .........................................................

    ui.pushButton_2.clicked.connect(lambda: update_settings(ui))

    # Generate Button ........................................................

    ui.pushButton_generate.clicked.connect(lambda: generate(ui))
    ui.pushButton_generate.setEnabled(False)

    # setup stylesheet
    file = open(path + "dark.qss")
    app.setStyleSheet(file.read())
    file.close()

    # auto quit after 2s when testing on travis-ci
    if "--travis" in sys.argv:
        QtCore.QTimer.singleShot(2000, app.exit)

    # run
    window.show()
    app.exec_()


if __name__ == "__main__":
    path = ''
    main()
