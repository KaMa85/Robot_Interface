
AR Robot Programming Interface README
Project Description
This project presents an innovative Augmented Reality (AR) interface for online robot programming and data visualization, designed to streamline automation in variable manufacturing environments. The interface leverages human interaction to reduce the randomness of robot path planning methods and integrates holographic representations to interact with redundant robot manipulators.

Key features include:

Collision-Free Path Planning: Utilizes the Rapidly Random Tree Star (RRT*) algorithm to determine the shortest path through random sampling.
Smooth Rotation: Achieves smooth end-effector rotation with Spherical Linear Interpolation (SLERP).
Optimal Configuration: Applies Sequential Quadratic Programming (SQP) to compute the robot's configurations for the planned path.
Human-In-The-Loop Interaction: AR users are presented with multiple optimized paths and can select the most suitable one based on context and intuition.
Development Stage
This project has been validated through experimental implementation with a 7 Degree-Of-Freedom (DOF) robot manipulator, demonstrating the value of human-in-the-loop interaction for context-aware robotics. The interface's primary components are functional, but future versions may refine certain features for improved usability and efficiency.

Summary of Workflow
Path Planning: The RRT* algorithm operates in a loop, independently exploring the shortest paths through random sampling in each iteration.
Path Optimization: Paths are presented to the user through the AR interface for context-based selection.
Execution: The selected path is optimized further and executed using SLERP and SQP algorithms, ensuring smooth and efficient robot movement.
Demonstration
This project demonstrates the potential for AR-based interfaces to improve robot programming by combining automation with human intuition. The current implementation provides a solid foundation for further development in this area.

Limitations
The project currently focuses on integrating human-in-the-loop interaction for path selection but does not yet include advanced learning methods for dynamic adaptation.
The interface is validated with a specific 7 DOF manipulator and may require modifications for other robotic platforms.

Demo:
https://github.com/KaMa85/Robot_Interface/blob/main/Untitled%20video%20(1).mp4
