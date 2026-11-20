function eF = steeringLowpass(theta, d, lineFound, alpha)
% MATLAB Function block in the Control System.
% Calculates steering error and smooths it before sending it to the PID controller.
% theta: angle of the detected line
% d: distance from the center of the line
% lineFound: checks whether the line is detected
% alpha: controls how much previous error is retained

% e = theta (rad) + 0.015*d
% eF = alpha*eF + (1-alpha)*e

% Enables code generation in MATLAB
%#codegen

% Stores the previous filtered error between function calls
persistent ef

% Initializes the filtered error to zero during the first call
if isempty(ef), ef = 0; end

% Updates the error only when a line is detected
if lineFound

    % Calculates the steering error using angle and distance
    e = deg2rad(theta) + 0.015*d;

    % Applies a low-pass filter to smooth the steering error
    % alpha controls the influence of the previous error
    % (1-alpha) controls the influence of the new error
    ef = alpha*ef + (1 - alpha)*e;

end

% Sends the filtered steering error as the output
eF = ef;

end
