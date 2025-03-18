Talker Node
============

The `talker <https://github.com/ACarfi/COGAR-Example/blob/main/scripts/talker.py>`_ node is responsible for publishing messages to a topic named `chatter`. This node continuously sends a message to other nodes that are subscribed to the `chatter` topic.

**Topic Info**

.. list-table::
   :header-rows: 1
   :widths: 20 80
   * - **Topic Name**  
     - `/chatter`
   * - **Message Type**  
     - `std_msgs/String <https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html>`_
   * - **Publishing Frequency**  
     - `(10Hz) <https://github.com/ACarfi/COGAR-Example/blob/main/scripts/talker.py#L9>`_


