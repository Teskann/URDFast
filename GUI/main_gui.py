# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 18:20:27 2020

@author: Clément
"""

import logging
import sys

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QUrl
from PyQt5.QtGui import QDesktopServices
from PyQt5.QtGui import QFontDatabase
from PyQt5.QtWidgets import QFileDialog, QTreeWidgetItem

sys.path.insert(1, '../')

import URDF
import create_robot_from_urdf as cr

from code_generator import generate_everything
from Language import Language

# make the example runnable without the need to install

import main_window

# Global variables ___________________________________________________________

ftm_list = []  # Forward transition matrices list
btm_list = []  # Backward transition matrices list
fk_list = []  # Forward Kinematics List
jac_list = []  # Jacobian list
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
    
    robot_obj : create_robot_from_urdf.Robot
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
    
    robot_obj : create_robot_from_urdf.Robot
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
    
    robot_obj : create_robot_from_urdf.Robot
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
    
    robot_obj : create_robot_from_urdf.Robot
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


# Update GUI State from Robot Object _________________________________________

def init_gui_from_urdf(gui, robot):
    """
    Description
    -----------
    
    Updates the GUI with respect to the robot given as parameter
    
    Parameters
    ----------
    
    gui : main_window.Ui_MainWindow
        GUI to update
    
    robot : createRobotFromURDF.robot
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
             '"http://www.w3.org/TR/REC-html40/strict.dtd">\n<html><head><meta' + \
             'name="qrichtext" content="1" /><style type="text/css">\np, li { ' + \
             'white-space: pre-wrap; }\n</style></head><body style=" font-family' + \
             ':\'MS Shell Dlg 2\'; font-size:10pt; font-weight:400; ' + \
             'font-style:normal;">\n'

    html = header + p + "<b>Robot Name</b> : " + f"{robot.name}</p><br/>\n"
    html += p + "<b>Number of Joints</b> : " + f"{robot.njoints()}</p><br/>\n"
    html += p + "<b>Number of Links</b> : " + f"{robot.nlinks()}</p><br/>\n"
    mass = 0;
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
    
    create_robot_from_urdf.Robot
        Robot object of the opened file

    """

    # File dialog
    fname, ftype = QFileDialog.getOpenFileName(caption="Open URDF File",
                                               filter="URDF Files (*.urdf)" + \
                                                      ";;All files (*)",
                                               directory=path + '../Examples')
    if fname == '':
        return
    global robot_obj
    # Open the file
    with open(fname) as file:
        urdf_obj = URDF.URDF(file)
        robot_obj = cr.Robot(urdf_obj, progress_bar)

        init_gui_from_urdf(gui, robot_obj)


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
        Dictionnary of the settings
    
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
    
    robot_obj : create_robot_from_urdf.Robot
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
                        language,
                        path + '../generated/' + settings["filename"])


# UI Main ____________________________________________________________________

def main():
    """
    Application entry point
    """

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

    # Openning button ........................................................

    ui.pushButton.clicked.connect(
        lambda: open_file_dialog(ui, ui.progressBar))

    # Transition Matrices ....................................................

    ui.listWidget_btm.selectionMode()

    ui.pushButton_ftm_add.clicked.connect(lambda:
                                          add_joint_to_list(ui.listWidget_ftm,
                                                            ui.comboBox_ftm_joint,
                                                            ui.pushButton_ftm_add,
                                                            ui.pushButton_ftm_del,
                                                            True))

    ui.pushButton_ftm_del.clicked.connect(lambda:
                                          del_joint_from_list(
                                              ui.listWidget_ftm,
                                              ui.comboBox_ftm_joint,
                                              ui.pushButton_ftm_add,
                                              ui.pushButton_ftm_del, True))

    ui.pushButton_btm_add.clicked.connect(lambda:
                                          add_joint_to_list(ui.listWidget_btm,
                                                            ui.comboBox_btm_joint,
                                                            ui.pushButton_btm_add,
                                                            ui.pushButton_btm_del,
                                                            True))

    ui.pushButton_btm_del.clicked.connect(lambda:
                                          del_joint_from_list(
                                              ui.listWidget_btm,
                                              ui.comboBox_btm_joint,
                                              ui.pushButton_btm_add,
                                              ui.pushButton_btm_del, True))

    # Forward Kinematics .....................................................

    ui.pushButton_fk_add.clicked.connect(lambda:
                                         add_fk_jac(ui.listWidget_fk,
                                                    ui.comboBox_fk_origin,
                                                    ui.comboBox_fk_destination,
                                                    True))

    ui.pushButton_fk_del.clicked.connect(lambda:
                                         del_fk_jac(ui.listWidget_fk, True))

    # Jacobians ..............................................................

    ui.pushButton_jac_add.clicked.connect(lambda:
                                          add_fk_jac(ui.listWidget_jac,
                                                     ui.comboBox_jac_origin,
                                                     ui.comboBox_jac_destination,
                                                     False))
    ui.pushButton_jac_del.clicked.connect(lambda:
                                          del_fk_jac(ui.listWidget_jac,
                                                     False))

    window.setWindowTitle("NYXX - Generate Code from URDF")

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
