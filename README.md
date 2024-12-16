# Robot Motion Planning Homework Repository

## Overview

This repository contains code and documentation for the **MAE593B – Robot Motion Planning** class (Fall 2024) at West Virginia University. It includes various algorithms and methods related to motion planning, such as visibility graphs, search algorithms, and motion planning strategies for 2D environments and non-holonomic robots.

## Homework Breakdown

### **Homework 1: 2D Configuration Space with Obstacles**
- **Objective**: Create a 2D configuration space to simulate robot motion in an environment with obstacles.
- **Key Concepts**:
  - Defining polygonal obstacles in a 2D space.
  - Checking if a configuration is in a free space or collides with an obstacle.
  - Comparing different space representations, including bitmap grids.

### **Homework 2: Graph Search Algorithms and Motion Planner**
- **Objective**: Implement graph search algorithms like BFS, Dijkstra, and A* to solve pathfinding problems in a 2D environment using a visibility graph.
- **Key Concepts**:
  - Using a visibility graph to represent valid paths.
  - Implementing and comparing BFS, Dijkstra, and A* search algorithms.

### **Homework 3: Motion Planning Algorithms and Pathfinding**
- **Objective**: Implement and test various motion planning algorithms (Wavefront, PRM, RRT) for robot motion in 2D environments.
- **Key Concepts**:
  - Implementing and comparing different motion planning algorithms.
  - Pathfinding using probabilistic roadmaps and random trees.

### **Homework 4: Non-Holonomic Motion Planning and Dubins Paths**
- **Objective**: Implement algorithms for motion planning in non-holonomic systems using Dubins paths.
- **Key Concepts**:
  - Computing Dubins paths for different robot configurations.
  - Finding the shortest path using different Dubins path types.

### **Homework 5: Advanced Motion Planning with Sampling-Based Algorithms**
- **Objective**: Implement and test advanced motion planning algorithms, such as Rapidly-exploring Random Trees (RRT) and RRT*, for complex environments. This includes the implementation of **Informed RRT*** for improved pathfinding.
- **Key Concepts**:
  - Advanced implementation of RRT and RRT* for efficient pathfinding in complex environments.
  - **Informed RRT***: An enhancement to RRT* that focuses the sampling of nodes towards regions that are more likely to lead to an optimal solution, improving the efficiency of the search.
  - Comparison of the performance and quality of solutions with different variants of RRT.

### **Optional Homework: Trajectory Optimization for Mobile Robots**
- **Objective**: Implement trajectory optimization techniques to refine the paths generated by motion planners.
- **Key Concepts**:
  - Trajectory smoothing and optimization using techniques like gradient descent or optimal control.
  - Refining the generated path to minimize energy consumption, time, or other criteria.

## Setup and Requirements

### Prerequisites
- MATLAB (R2020b or later recommended).
- Peter Corke's Robotics Toolbox (available [here](https://petercorke.com/toolboxes/robotics-toolbox/)).

### Note

In the PGraph.m file, make the following changes:

- **Line 303**: Replace with:
  
  ```matlab
  g.vertexlist(:,v) = [];

- **Line 314**: Replace with:	

  ```matlab
  g.edgelist(:,e) = [];

## Acknowledgements

- **Peter Corke’s Robotics Toolbox for MATLAB**: This toolbox was essential in implementing visibility graphs and search algorithms.
- **MAE593B Course Structure**: The homework assignments are based on the teachings and structure provided by Professor Guilherme Pereira at West Virginia University.




