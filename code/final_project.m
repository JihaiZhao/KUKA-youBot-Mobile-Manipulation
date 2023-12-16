%%
addpath('/home/jihai/ME449/ModernRobotics/packages/MATLAB/mr')
%% Final
% how to run this matlab file
% just click run button.
% This is the example input (best)
config = [0.0,0.2,0.2,0.0,0.0,0.0, 0.0,0.0, 0.0, 0.0,0.0, 0.0]';
Tsc_initial = [1,0,0,1;0,1,0,0;0,0,1,0.025;0,0,0,1];
Tsc_goal = [0,1,0,0;-1,0,0,-1;0,0,1,0.025;0,0,0,1];
Kp = 2.5;
Ki = 0;
p = Kp.*eye(6);
Ki = Ki.*eye(6);

%
Tb0 = [1,0,0,0.1662;0,1,0,0;0,0,1,0.0026;0,0,0,1]; % this is a fixed value
M0e = [1,0,0,0.033;0,1,0,0;0,0,1,0.6546;0,0,0,1];

% first generate a reference trajectory 
Tse_d = [0,0,1,0;0,1,0,0;-1,0,0,0.5;0,0,0,1];

% Tsc_initial = [1,0,0,1;0,1,0,0.5;0,0,1,0.025;0,0,0,1];
% Tsc_goal = [0,1,0,0.1;-1,0,0,-1;0,0,1,0.025;0,0,0,1];

Tce_standoff =  [-sqrt(2)/2, 0, sqrt(2)/2, 0; 0, 1, 0, 0;-sqrt(2)/2, 0, -sqrt(2)/2, 0.15; 0, 0, 0, 1];
Tce_grasp = [-sqrt(2)/2, 0, sqrt(2)/2, 0; 0, 1, 0, 0;-sqrt(2)/2, 0, -sqrt(2)/2, 0; 0, 0, 0, 1]; %end-effector's configuration relative to the cube when it is grasping the cube
k = 1;

% go to the grasp stand_off position
trajectory = TrajectoryGenerator(Tse_d, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k);

delta_t = 0.01;

timestep = 0.01;
max_speed = 10;
Blist = [0,         0,         0,         0,        0;
         0,        -1,        -1,        -1,        0;
         1,         0,         0,         0,        1;
         0,   -0.5076,   -0.3526,   -0.2176,        0;
         0.033,     0,         0,         0,        0;
         0,         0,         0,         0,        0];
r = 0.0475;
l = 0.235;                   
w = 0.15;
F = r/4 .* [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w); 1,1,1,1;-1,1,-1,1];
F6 = [0, 0, 0, 0; 0, 0, 0, 0; F; 0, 0, 0, 0];

new_config = [];
Xerr_all = [];
j_all = [];
the_all = [];
for i = 1:length(trajectory)-1
    theta = config(1);
    x = config(2);
    y = config(3);
    thetalist = config(4:8);
    T0e = FKinBody(M0e, Blist, thetalist);
    Tsb = [cos(theta), -sin(theta), 0, x; sin(theta), cos(theta), 0, y; 0, 0, 1, 0.0963; 0, 0, 0, 1];
    Tse = Tsb * Tb0 * T0e;

    % call FeedbackControl
    Xd(1,:) = [trajectory(i,1:3), trajectory(i,10)];
    Xd(2,:) = [trajectory(i,4:6), trajectory(i,11)];
    Xd(3,:) = [trajectory(i,7:9), trajectory(i,12)];
    Xd(4,:) = [0,0,0,1];
    Xd_next(1,:) = [trajectory(i+1,1:3), trajectory(i+1,10)];
    Xd_next(2,:) = [trajectory(i+1,4:6), trajectory(i+1,11)];
    Xd_next(3,:) = [trajectory(i+1,7:9), trajectory(i+1,12)];
    Xd_next(4,:) = [0,0,0,1];
    [V,Xerr] = FeedbackControl(Tse, Xd, Xd_next, Kp, Ki, delta_t);
    Xerr_all = [Xerr_all, Xerr];
    
    J_arm = JacobianBody(Blist, thetalist);
    J_base = Adjoint(TransInv(Tse) * TransInv(Tb0)) * F6;
    J = [J_base, J_arm];
    speed = pinv(J, 1e-2) * V;
    % 
    % res = testJointLimits(thetalist);
    % 
    % if ~isempty(res)
    %     for i = 1:length(res)
    %         J_arm(:,res(i)) = zeros(1,6);
    %     end            
    %     J = [J_base, J_arm];
    %     speed = pinv(J, 1e-2) * V;
    % end
    j_all = [j_all, J];
    % use milesttone 1 to get Xd_next
    config_next = NextState(config, speed, timestep, max_speed);

    gripper_state = trajectory(i,13);
    new_config = [new_config, [config_next;gripper_state]];
    config = config_next;
    the_all = [the_all,thetalist];
end
new_config = new_config';
% Overwrite csv file
csvwrite('/home/jihai/ME449/final_project/final.csv',new_config);
disp('Generating animation csv file.')

x = 0:0.01:31.98;

plot(x, Xerr_all(1,:), x, Xerr_all(2,:), x, Xerr_all(3,:), x, Xerr_all(4,:), x, Xerr_all(5,:), x, Xerr_all(6,:),'LineWidth',2)
grid on
legend(['roll err'],['pitch err'],['yaw err'],['X err'],['Y err'],['Z err'])
xlabel('time(s)')
ylabel('error')
title('Xerr plot')
csvwrite('/home/jihai/ME449/final_project/X_err_bext.csv',Xerr_all');
disp('Writing error plot data.')
disp('DONE')