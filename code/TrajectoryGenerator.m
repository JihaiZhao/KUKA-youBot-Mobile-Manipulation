function N_final = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k)
% Takes Tse_initial: The initial configuration of the end-effector in the reference trajectory
%       Tsc_initial: The cube's initial configuration
%       Tsc_goal: The cube's desired final configuration
%       Tce_grasp: The end-effector's configuration relative to the cube when it is grasping the cube
%       Tce_standoff: The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube
%       k: The number of trajectory reference configurations per 0.01 seconds
% Returns N_final: A representation of the N}configurations of the 
%                    end-effector along the entire concatenated eight-segment 
%                    reference trajectory. Each of these N reference points represents a transformation matrix 
%                    T_se of the end-effector frame e relative to s at an instant in time, 
%                    plus the gripper state (0 or 1).
Tf = 4;
N = (Tf*k/0.01);
method = 5;
result = [];
% go to the standoff position
Tse_standoff = Tsc_initial * Tce_standoff;
N_matrix = CartesianTrajectory(Tse_initial, Tse_standoff, Tf, N, method);

for i=1:length(N_matrix)
    matrix_N = cell2mat(N_matrix(i));
    a = reshape(matrix_N(1:3,1:3).',1,[]);
    b = reshape(matrix_N(1:3,4).',1,[]);
    res = [a,b,0]';
    result = [result,res];
end

% go to the initial position
Tse_grasp_initial = Tsc_initial * Tce_grasp;
N_matrix = CartesianTrajectory(Tse_standoff, Tse_grasp_initial, Tf, N, method);

for i=1:length(N_matrix)
    matrix_N = cell2mat(N_matrix(i));
    a = reshape(matrix_N(1:3,1:3).',1,[]);
    b = reshape(matrix_N(1:3,4).',1,[]);
    res = [a,b,0]';
    result = [result,res];
end

% close the gripper
N_matrix = CartesianTrajectory(Tse_grasp_initial, Tse_grasp_initial, Tf/2, N/2, method);

for i=1:length(N_matrix)
    matrix_N = cell2mat(N_matrix(i));
    a = reshape(matrix_N(1:3,1:3).',1,[]);
    b = reshape(matrix_N(1:3,4).',1,[]);
    res = [a,b,1]';
    result = [result,res];
end

% finish pickup and return to standoff position
N_matrix = CartesianTrajectory(Tse_grasp_initial, Tse_standoff, Tf, N, method);

for i=1:length(N_matrix)
    matrix_N = cell2mat(N_matrix(i));
    a = reshape(matrix_N(1:3,1:3).',1,[]);
    b = reshape(matrix_N(1:3,4).',1,[]);
    res = [a,b,1]';
    result = [result,res];
end

% go to final standoff position
Tse_standoff_final = Tsc_goal * Tce_standoff;
N_matrix = CartesianTrajectory(Tse_standoff, Tse_standoff_final, 2*Tf, 2*N, method);

for i=1:length(N_matrix)
    matrix_N = cell2mat(N_matrix(i));
    a = reshape(matrix_N(1:3,1:3).',1,[]);
    b = reshape(matrix_N(1:3,4).',1,[]);
    res = [a,b,1]';
    result = [result,res];
end

% go to the finial position
Tse_grasp = Tsc_goal * Tce_grasp;
N_matrix = CartesianTrajectory(Tse_standoff_final, Tse_grasp, Tf, N, method);

for i=1:length(N_matrix)
    matrix_N = cell2mat(N_matrix(i));
    a = reshape(matrix_N(1:3,1:3).',1,[]);
    b = reshape(matrix_N(1:3,4).',1,[]);
    res = [a,b,1]';
    result = [result,res];
end

% open the gripper
N_matrix = CartesianTrajectory(Tse_grasp, Tse_grasp, Tf/2, N/2, method);

for i=1:length(N_matrix)
    matrix_N = cell2mat(N_matrix(i));
    a = reshape(matrix_N(1:3,1:3).',1,[]);
    b = reshape(matrix_N(1:3,4).',1,[]);
    res = [a,b,0]';
    result = [result,res];
end

% finish test nd return to standoff position
N_matrix = CartesianTrajectory(Tse_grasp, Tse_standoff_final, Tf, N, method);

for i=1:length(N_matrix)
    matrix_N = cell2mat(N_matrix(i));
    a = reshape(matrix_N(1:3,1:3).',1,[]);
    b = reshape(matrix_N(1:3,4).',1,[]);
    res = [a,b,0]';
    result = [result,res];
end
N_final = result';
end