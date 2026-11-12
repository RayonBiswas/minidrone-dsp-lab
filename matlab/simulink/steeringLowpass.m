function eF = steeringLowpass(theta, d, lineFound, alpha)
% MATLAB Function block in the Control System: steering error, low-passed before the PID.
% e = theta (rad) + 0.015*d ; eF = alpha*eF + (1-alpha)*e
%#codegen
persistent ef
if isempty(ef), ef = 0; end
if lineFound
    e = deg2rad(theta) + 0.015*d;
    ef = alpha*ef + (1 - alpha)*e;
end
eF = ef;
end
