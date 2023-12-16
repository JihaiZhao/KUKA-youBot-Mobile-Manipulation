function res = testJointLimits(joint_theta)
% Input: The input of the function includes joint_theta
%       joint_theta: the theta needed to be check whether reach the limits 
%
% Output: 
%       res: which joint reach the limits
    res = [];
    if joint_theta(3) > -0.3
        res = [res,3];
    end
    if joint_theta(4) > -0.3
        res = [res,4];
    end
end