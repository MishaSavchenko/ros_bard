# ROS Bard

I have like this *weird* idea about being able to visualize the ***evolution*** of the node in and their ROS connections through out the bring up procedure but also through out the start up process. 

So what I'm picturing is a slider that allows you to time travel across different points in time of you ROS systems and visualize a ***node graph*** of your ROS system. 
For that you need to be able to monitor and record the state of your system, at different points in time, and visualize it.

While the monitoring is actually not too difficult and you can do it pretty easily in python ( take a look at `ros_bard/scripts/cli_approach.py` ) the much less trivial problem is to be able to visualize the graph, in a coherent manner. 

The vanilla version of RQT node visualization works alright, but I find that its quite limiting for visualization of temporal data, specifically because the graphs that are generated do not have a precistance of where certain nodes are aesthetically. While it seems that aesthetics are not the main concern I find them to be incredibly useful when trying to understand and grasp complex ROS architecture. 
