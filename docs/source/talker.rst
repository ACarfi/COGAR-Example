Talker Node
============

The `talker <https://github.com/ACarfi/COGAR-Example/blob/main/scripts/talker.py>`_ node is responsible for publishing messages to a topic named `chatter`. This node continuously sends a message at a fixed rate to other nodes that are subscribed to the `chatter` topic.

Understanding the Talker Node
-----------------------------

The `talker` node in the package is a simple publisher that sends a string message at a fixed frequency `(10Hz) <https://github.com/ACarfi/COGAR-Example/blob/main/scripts/talker.py#L9>`_. By default, it publishes to the `/chatter` topic. Other nodes can listen to this topic and process the incoming messages.

Topic Info

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - **Topic Info**  
     - (spanning across both columns)
   * - **Topic Name**  
     - `/chatter`
   * - **Message Type**  
     - `std_msgs/String <https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html>`_
   * - **Publishing Frequency**  
     - `(10Hz) <https://github.com/ACarfi/COGAR-Example/blob/main/scripts/talker.py#L9>`_


The message type used by the `talker` node is `std_msgs/String <https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html>`_, which is a standard message type in ROS. It is used to send simple text messages between nodes.


