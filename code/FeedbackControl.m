function [V, Xerr] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, delta_t)
% Input:
% 
% The current actual end-effector configuration X (also written T_se).
% The current end-effector reference configuration X_d (i.e., T_se,d).
% The end-effector reference configuration at the next timestep in the reference trajectory, X_d,next (i.e., ...
%     T_{se,d,next), at a time Delta t later.
% The PI gain matrices K_p and K_i.
% The timestep Delta t between reference trajectory configurations.
% 
% Output:
%   V: the commanded end-effector twist expressed in the end-effector frame
%   Xerr: the error twist.
%   
% The commanded end-effector twist V expressed in the end-effector frame e.
Xerr = MatrixLog6(TransInv(X) * Xd);
Xerr = se3ToVec(Xerr);
% Vd = (1/delta_t) * MatrixLog6(TransInv(Xd) * Xd_next);
Vd = (1/delta_t) * MatrixLog6(inv(Xd) * Xd_next);
Vd = se3ToVec(Vd);

V = Adjoint(TransInv(X) * Xd) * Vd + Kp * Xerr + Ki * (Xerr + Xerr * delta_t);
end