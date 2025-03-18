.. COGAR example documentation master file, created by
   sphinx-quickstart on Mon Mar 17 12:29:13 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

COGAR example's documentation
=========================================

.. raw:: html

   <p style="text-align: right;">
      <a href="https://github.com/ACarfi/COGAR-Example" target="_blank">
         📌 View on GitHub
      </a>
   </p>

This documentation serves as an example for the Cognitive Architecture for Robotics course at UniGe in 2025 by Alessandro Carfì. It focuses on how to properly document a ROS package using Sphinx and Read the Docs.

The package itself includes:

- A talker node that publishes messages.
- A listener node that subscribes and processes messages.
- A launch file to run both nodes together.

While the package is a simple ROS example, the primary goal is to demonstrate best practices for writing clear, structured, and maintainable documentation.

.. toctree::
   :maxdepth: 2
   :hidden:
   :caption: Content

   install
   listener
   talker
   How to Use <use>