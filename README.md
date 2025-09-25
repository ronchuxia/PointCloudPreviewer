# Message to Claude Code

Hi, I am working on a project that involves 3dgs. I find a problem, but I can't find a proper tool to solve this problem. So I am writing this program myself to find a solution. The problem is here: 

I find that there is something wrong with the input that I feed into the 3dgs trainer. The input camera poses are not aligned with the input point cloud, which serves as the initialization for the 3dgs scene.

The camera poses are extracted from a feedforward slam method called pi3. So I need to investigate what the camera poses look like, with respect to the point cloud (or the trained 3dgs scene). 

However, there are two problems for existing solutions:
1. There is no tool to visualize the camera poses and the point cloud (or the trained 3dgs scene) together. 
2. I find it hard to navigate the scene using existing programs.

So I want you to help me write a program that can solve the two problems above. For a start, you only need to visualize the point cloud, because 3dgs scenes can be hard to render online due to the lack of GPU resources on my macbook. 

We can do this step by step. So let's get started.