Listener Node
==============

The `listener <https://github.com/ACarfi/COGAR-Example/blob/main/scripts/listener.py>`_ node subscribes to the `/chatter` topic and prints the messages it receives. This node listens for the messages published by the **talker** node and displays them in the terminal.

**Topic Info**

.. list-table:: 
   :header-rows: 1
   :widths: 20 80
   
   * - **Topic Name**
     - `/chatter`
   * - **Message Type**
     - `std_msgs/String <https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html>`_
   * - **Receiving Frequency**
     - On message received
