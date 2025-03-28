How to Use the COGAR Example Package
====================================
.. _use:

This section explains how to use the `example_pkg` ROS package. The package contains two nodes: `talker` and `listener`, which can be used to communicate via ROS topics.

Running the Talker Node
------------------------

The `talker` node publishes messages to a topic called `chatter`. To run the node, use the following command in your catkin workspace:

.. code-block:: bash

   rosrun example_pkg talker.py

This will start the `talker` node, and it will begin publishing messages to the `chatter` topic.

Running the Listener Node
-------------------------

The `listener` node subscribes to the `chatter` topic and prints the messages received. To run the node, use the following command in your catkin workspace:

.. code-block:: bash

   rosrun example_pkg listener.py

Once the listener node is running, it will print the messages that the talker node is publishing.

Using the Launch File
---------------------

You can also use a launch file to start both the `talker` and `listener` nodes simultaneously. The launch file simplifies the process of starting multiple nodes with a single command.

To launch both nodes at once, use the following command:

.. code-block:: bash

   roslaunch example_pkg example.launch

This will start both the `talker` and `listener` nodes and automatically connect them through the `chatter` topic. Then you should see the following output in your terminal:

.. Sphinx Code to include an image
.. .. image:: /_static/execution_result.png
   :alt: Screenshot of the output
   :align: center
   :width: 70%

.. Sphinx Code to include a video hosted on githu using the `sphinxcontrib-video` extension
.. video:: _static/video.mp4
   :autoplay: 
   :loop: 
   :width: 100%

.. Sphinx Code to include a YouTube video using the `sphinxcontrib-youtube` extension
.. ..  youtube:: dQw4w9WgXcQ
   :width: 100%
   :height: 500px
   

.. raw:: html

   <br><br>
   
This video demonstrates how the result should look when the nodes are running successfully.


Viewing the Topic
-----------------

You can check if the nodes are communicating properly by echoing the `chatter` topic:

.. code-block:: bash

   rostopic echo /chatter

This will display the messages being published by the `talker` node and received by the `listener` node.

Troubleshooting
---------------

If you encounter issues, make sure that:

1. ROS is properly installed and sourced.
2. The nodes are correctly built and no errors appear when running the `talker` or `listener`.
3. Ensure that the correct ROS master URI is set (e.g., `ROS_MASTER_URI=http://localhost:11311`).

For further help, refer to the :ref:`installation instructions <install>`.
