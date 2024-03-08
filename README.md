This project builds upon the work from my graduation research, as detailed in the paper [Zhang X, Liu G, Jing L, et al. Automated measurement of heart girth for pigs using two Kinect depth sensors[J]. Sensors, 2020, 20(14): 3848.]. 
The system employs two Kinect devices to estimate the body size of pigs as they pass through, thereby monitoring their growth conditions and guiding feeding strategies. 
The CSC folder contains several images depicting the intermediary stages of the process. Due to their size exceeding 100MB, the files named testPCL2.sdf and testPCL2.pdb were omitted.
The Kinect cameras are positioned at the side and above the farm's pathway. As a pig walks through, depth information is captured in a video format, from which the optimal frame is selected for subsequent image processing. 
The point clouds from the two Kinects undergo feature-based matching and are merged to recreate three-quarters of the pig's body. Through fitting and mirror symmetry techniques, a complete model of the pig's body is generated. 
This allows for the accurate measurement of critical body dimensions, including chest girth, abdominal girth, body length, and body height.
