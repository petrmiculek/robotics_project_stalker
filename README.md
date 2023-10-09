# Project - Welcome Robot
- Assistant robot moving around in a foyer, approaching people to help them before returning to its home station.
- Using ROS, C++, and LiDAR scanners. 
- Perception, Localization, Decision, and Action Nodes
## Authors
- Petr Mičulek
- Farah Maria Majdalani
- Louis Choules
- Vinicius G.A.V Resende
- Youssef Itani

## Presentation Slides
[Google Slides link](https://docs.google.com/presentation/d/e/2PACX-1vRkaz-X0Oz4FooIDIJrjZkoHg-wU4bo78ndhiLwGyLmLtzY28vZ6zEI0qZxggtZY5KK8dI7bHWRsXJX/embed?start=false&loop=false&delayms=60000&slide=id.p)

## Core Files
- action_node.cpp - Handling robot movement (translation, rotation).
- datmo_node.cpp - Processing LiDAR data to detect and track nearby people.
- decision_node.cpp - Robot's behaviour: noticing people, observing them, and offering help when suitable.
- localization_node.cpp - Map-based localization within the environment.
- robot_moving_node.cpp - Determining whether the robot is moving.
- rotation_node.cpp - Rotation-only movement.

## Development

This project was created in the Robotics M1 Course at MOSIG, INP Grenoble, France. As a team, we implemented an assistant robot while extending the provided design. We tested it meticulously and presented our solution, achieving an overall very good grade. The lessons from this project span asynchronous group work management, ROS skills, the importance of detailed testing, and lots of debugging. Lastly, we would like to thank our supervisor, Philip Scales.
