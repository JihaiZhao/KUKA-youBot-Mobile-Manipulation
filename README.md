## Robot-Manipulation
The propose of this project is writing software that plans a trajectory for the end-effector of the youBot mobile manipulator (a mobile base with four mecanum wheels and a 5R robot arm), performs odometry as the chassis moves, and performs feedback control to drive the youBot to pick up a block at a specified location, carry it to a desired location, and put it down. 

  
____

To achieve the goal of the project I wrote 4 functions. 

* NextState 

        Input: The input of the function includes config, speed, timestep, max_speed 

        config: A 12-vector representing the current configuration of the robot (3 variables for the chassis configuration, 5 variables for the arm configuration, and 4 variables for the wheel angles). 

        speed: A 9-vector of controls indicating the wheel speeds and the arm joint speeds  

        timestep: A timestep  

        max_speed: A positive real value indicating the maximum angular speed of the arm joints and the wheels.  

        For example, if this value is 12.3, the angular speed of the wheels and arm joints is  

        limited to the range [-12.3 radians/s, 12.3 radians/s]. Any speed in the 9-vector of controls  

        that is outside this range will be set to the nearest boundary of the range.  

        
        Output: The return of the function is new_config 

        new_config : A 12-vector representing the configuration of robot. 


        The function NextState is based on a simple first-order Euler step, i.e., new arm joint angles = (old arm joint angles) + (joint speeds) * new wheel angles = (old wheel angles) + (wheel speeds) * new chassis configuration is obtained from odometry. 

 

* TrajectoryGenerator 

        Input: The input of the function includes Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, and k. 

        Tse_initial: The initial configuration of the end-effector in the reference trajectory 

        Tsc_initial: The cube's initial configuration 

        Tsc_goal: The cube's desired final configuration 

        Tce_grasp: The end-effector's configuration relative to the cube when it is grasping the cube 

        Tce_standoff: The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube 

        k: The number of trajectory reference configurations per 0.01 seconds 

        

        Output: The Return of the function is N_final 

        A representation of the N configurations of the end-effector along the entire concatenated eight-segment reference trajectory. Each of these N reference points represents a transformation matrix T_se of the end-effector frame e relative to s at an instant in time, plus the gripper state (0 or 1). 

 

* FeedbackControl 

        Input: The input of the function includes X, Xd, Xd_next, Kp, Ki, delta_t 

        X: The current actual end-effector configuration (also written T_se). 

        Xd: The current end-effector reference configuration (i.e., T_se,d). 

        Xd_next: The end-effector reference configuration at the next timestep in the reference trajectory, (i.e., T_se,d,next, at a time Delta t later. 

        Ki, Kp: the PI gain matrices K_p and K_i. 

        delta_t: The timestep Delta t between reference trajectory configurations. 

        

        Output: 

        V: the commanded end-effector twist expressed in the end-effector frame 

        Xerr: the error twist. 

 

* testJointLimits 	 

        The function is used to help the robot arm avoid singularities. I choose both joint 3 and joint 4 to be always less than -0.3.  
        

        Input: The input of the function includes joint_theta 

        joint_theta: the theta needed to be check whether reach the limits 

        

        Output: 

        res: which joint reaches the limits 