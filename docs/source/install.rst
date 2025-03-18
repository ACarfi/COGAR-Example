Installation Instructions for Catkin Workspace
==============================================

To build and install a ROS package, you'll first need to set up a **catkin workspace**. Below are the steps to create the workspace, build it, and install the ROS package.

Step 1: Install ROS Noetic
^^^^^^^^

Follow the official instructions for installing **ROS Noetic** on your system:

- `ROS Noetic Installation Guide <https://wiki.ros.org/noetic/Installation>`_.

Make sure you have all necessary dependencies installed.

Step 2: Create a Catkin Workspace
^^^^^^^^

A **catkin workspace** is a directory where you will keep your ROS packages. Follow the official guide:

- `ROS Catkin Workspace Guide <https://wiki.ros.org/catkin/Tutorials/create_a_workspace>`_.

Step 3: Clone the ROS package
^^^^^^^^

Clone the ROS package into the src folder of the workspace:

.. code-block:: bash
   cd ~/catkin_ws/src
   git clone https://github.com/ACarfi/COGAR-Example.git

Step 4: Build the package
^^^^^^^^

.. code-block:: bash
   cd ~/catkin_ws
   catkin_make

Step 5: Source your workspace:
^^^^^^^^

.. code-block:: bash
   source devel/setup.bash

Now, you should be able to use the package refer to the :ref:`How to Use <use>` page.