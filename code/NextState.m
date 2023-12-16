%% This is the function for milestone 1
function new_config = NextState(config, speed, timestep, max_speed)
% Input:
% A 12-vector representing the current configuration of the robot (3 variables for the chassis configuration, 
% 5 variables for the arm configuration, and 4 variables for the wheel angles).
% 
% A 9-vector of controls indicating the wheel speeds and the arm joint speeds 
% A timestep 
% A positive real value indicating the maximum angular speed of the arm joints and the wheels. 
% For example, if this value is 12.3, the angular speed of the wheels and arm joints is 
% limited to the range [-12.3 radians/s, 12.3 radians/s]. Any speed in the 9-vector of controls 
% that is outside this range will be set to the nearest boundary of the range. If you don't want speed limits, 
% just use a very large number. If you prefer, your function can accept separate speed limits for the wheels and arm joints.
% 
% 
% Output: A 12-vector representing the configuration of the robot time 
% The function NextState is based on a simple first-order Euler step, i.e.,
% new arm joint angles = (old arm joint angles) + (joint speeds) * 
% new wheel angles = (old wheel angles) + (wheel speeds) * 
% new chassis configuration is obtained from odometry, as described in Chapter 13.4

%[new_phi, new_x, new_y, new_J1, new_J2, new_J3, new_J4, new_J5, new_W1, new_W2, new_W3, new_W4] = NextState(phi, x, y, J1, J2, J3, J4, J5, W1, W2, W3, W4)

r = 0.0475;
l = 0.235;                   
w = 0.15;

new_config = zeros(12,1);
new_config(4:8) = config(4:8) + timestep .* speed(5:9); %joint_angle
new_config(9:12) = config(9:12) + timestep .* speed(1:4); %wheel_angle
for i=1:9
    if abs(speed(i)) > max_speed
        if speed(i) < 0
            speed(i) = -max_speed;
        else
            speed(i) = max_speed;
        end
    end
end

% h1 = 1/(r*cos(-pi/4)) * [(l*sin(-pi/4) - w*cos(-pi/4)), cos(-pi/4+config(1)), sin(-pi/4+config(1))];
% h2 = 1/(r*cos(pi/4)) * [(l*sin(pi/4) + w*cos(pi/4)), cos(pi/4+config(1)), sin(-pi/4+config(1))];
% h3 = 1/(r*cos(-pi/4)) * [(-l*sin(-pi/4) + w*cos(-pi/4)), cos(-pi/4+config(1)), sin(-pi/4+config(1))];
% h4 = 1/(r*cos(pi/4)) * [(-l*sin(pi/4) - w*cos(pi/4)), cos(pi/4+config(1)), sin(-pi/4+config(1))];
% H = [h1;h2;h3;h4];
% delta_q = pinv(H) * speed(1:4)

Vb = r/4 .* [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w); 1,1,1,1;-1,1,-1,1] * (timestep .* speed(1:4));

w_bz = Vb(1);
v_bx = Vb(2);
v_by = Vb(3);
if w_bz == 0
    delta_qb = [0, v_bx,v_by]';
else
    delta_qb = [w_bz, (v_bx*sin(w_bz) + v_by*(cos(w_bz)-1))/w_bz, (v_by*sin(w_bz) + v_bx*(1-cos(w_bz)))/w_bz]';
end

delta_q = [1,0,0;0,cos(config(1)), -sin(config(1));0,sin(config(1)), cos(config(1))] * delta_qb;
new_config(1:3) = config(1:3) + delta_q;

end