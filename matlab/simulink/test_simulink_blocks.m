%% Test the three Simulink MATLAB Function blocks in plain MATLAB before wiring them into the model
% Tests the three functions together:
% addCameraNoise -> dspVisionBlock -> steeringLowpass
% Uses 120x160 RGB images containing a red track and an optional blue landing pad.
% Compares the detected line with the actual line.

clc; clear; close all;

% Resets the persistent variables in the functions
clear addCameraNoise steeringLowpass

% Sets the random number generator so results are repeatable
rng(7);

% Defines image height, width, and image center
H = 120; W = 160; xc = (W-1)/2; yc = (H-1)/2;

% Creates coordinate matrices for every pixel in the image
[X, Y] = meshgrid(0:W-1, 0:H-1);

%% 1. Accuracy: raw vs DSP over random frames and three noise levels

% Defines three noise levels:
% Column 1 = Gaussian noise standard deviation
% Column 2 = Salt and pepper noise density
levels = [0 0; 0.10 0.08; 0.20 0.15];

% Number of random frames tested for each noise level
N = 60;

% Stores the final results for each noise level
% Columns: raw RMSE, DSP RMSE, DSP success rate, processing time
res = zeros(size(levels,1), 4);

% Loops through all three noise levels
for L = 1:size(levels,1)

    % Stores angle errors for raw and DSP results
    eR = zeros(N,1);
    eF = zeros(N,1);

    % Stores total DSP processing time
    tF = 0;

    % Tests 60 random frames
    for i = 1:N

        % Generates a random line angle and distance
        th = -35 + 70*rand;
        d = -30 + 60*rand;

        % Creates an RGB image containing the red track
        [R, G, B] = makeFrame(th, d, X, Y, xc, yc, false);

        % Adds Gaussian and salt-and-pepper noise to the image
        [Rn, Gn, Bn] = addCameraNoise(R, G, B, levels(L,1), levels(L,2), 1);

        % Runs the vision block without DSP processing
        [t0, d0, f0] = dspVisionBlock(Rn, Gn, Bn, 0);

        % Runs the vision block with DSP processing and measures its time
        tic;
        [t1, d1, f1] = dspVisionBlock(Rn, Gn, Bn, 1);
        tF = tF + toc;

        % Assumes maximum error of 20 degrees if no line is detected
        eR(i) = 20;

        % Calculates raw angle error if a line is found
        if f0
            eR(i) = min(abs(t0 - th), 20);
        end

        % Assumes maximum error of 20 degrees if DSP cannot detect the line
        eF(i) = 20;

        % Calculates DSP angle error if a line is found
        if f1
            eF(i) = min(abs(t1 - th), 20);
        end
    end

    % Stores performance results for the current noise level
    % rms = root mean square error
    % Success = percentage of DSP errors below 3 degrees
    % Time = average processing time in milliseconds per frame
    res(L,:) = [rms(eR) rms(eF) 100*mean(eF < 3) 1000*tF/N];
end

% Displays the test results
fprintf('dspVisionBlock, %d random frames per row\n', N);
fprintf('sigma  s&p    angle RMSE raw   angle RMSE DSP   DSP success   ms/frame\n');

% Prints the results for all noise levels
for L = 1:size(levels,1)
    fprintf('%.2f   %.2f   %8.2f deg     %8.2f deg      %5.0f %%     %6.1f\n', levels(L,1), levels(L,2), res(L,:));
end

%% 2. One frame, checked in detail, with the landing pad

% Creates one image with a line at 12 degrees and distance -8 pixels
% Also includes a blue landing pad
[R, G, B] = makeFrame(12, -8, X, Y, xc, yc, true);

% Adds noise to the generated image
[Rn, Gn, Bn] = addCameraNoise(R, G, B, 0.10, 0.08, 1);

% Detects the line and landing pad using the DSP vision block
% th = detected line angle
% d = detected line distance
% lf = line found
% pf = pad found
% px, py = landing pad position
[th, d, lf, pf, px, py] = dspVisionBlock(Rn, Gn, Bn, 1);

% Displays the detected values and compares them with the actual values
fprintf('\nSingle frame: true 12.0 deg / -8.0 px -> %.1f deg / %.1f px, line %d, pad %d at (%.0f, %.0f) px (true (20, -25))\n', th, d, lf, pf, px, py);

% Checks whether the detected line angle and distance are accurate
assert(lf && abs(th - 12) < 2 && abs(d + 8) < 2, 'line estimate off');

% Checks whether the detected landing pad position is accurate
assert(pf && abs(px - 20) < 3 && abs(py + 25) < 3, 'pad estimate off');

%% 3. Steering low-pass: step response and hold when the line is lost

% Resets the persistent variable of the steering low-pass function
clear steeringLowpass

% Creates an array to store filtered steering errors
e = zeros(30,1);

% Tests the steering low-pass filter for 30 frames
for k = 1:30

    % Gives a 10-degree steering angle for the first 20 frames
    % After frame 20, the line is considered lost
    % alpha = 0.7 controls the amount of filtering
    e(k) = steeringLowpass(10, 0, k <= 20, 0.7);

end

% Displays the filtered error at frame 20 and frame 30
fprintf('steeringLowpass: step to %.3f rad after 20 frames, holds %.3f while line lost\n', e(20), e(30));

% Checks whether the filter reaches the expected value
% Also checks whether it holds the previous value when the line is lost
assert(abs(e(20) - deg2rad(10)) < 0.01 && e(30) == e(20), 'steering low-pass wrong');

%% 4. Display images and performance graphs

% Creates a figure window to display the test results
figure('Name', 'Simulink block test', 'Color', 'w', 'Position', [80 80 1000 380]);

% First subplot displays the noisy RGB image
subplot(1,2,1);
imshow(cat(3, Rn, Gn, Bn));
hold on;

% Creates vertical image coordinates
yy = 0:H-1;

% Calculates the line equation parameter using detected angle and distance
rho = d + xc*cosd(th) + yc*sind(th);

% Draws the detected line in green on the image
plot((rho - yy*sind(th))/cosd(th) + 1, yy + 1, 'g', 'LineWidth', 2);

% Marks the detected landing pad center with a cyan circle
plot(px + xc + 1, py + yc + 1, 'co', 'MarkerSize', 20, 'LineWidth', 2);

% Displays the detected angle and distance in the title
title(sprintf('dspVisionBlock: %.1f%c / %.1f px (true 12%c / -8 px)', th, 176, d, 176));

% Second subplot compares raw and DSP angle errors
subplot(1,2,2);

% Creates a bar graph using raw and DSP RMSE values
bar(res(:,1:2));

% Displays grid lines
grid on;

% Labels the vertical axis
ylabel('Angle RMSE (deg)');

% Names the three noise levels on the horizontal axis
set(gca, 'XTickLabel', {'clean', 'noise', 'heavy noise'});

% Adds a legend to distinguish raw and DSP results
legend('raw', 'DSP');

% Adds the graph title
title('Raw vs DSP chain');

% Displays a message if all tests pass
disp('All Simulink block tests passed.');

%% Local function to generate an artificial camera image

function [R, G, B] = makeFrame(th, d, X, Y, xc, yc, withPad)

% Calculates the line position using its angle and distance
rho = xc*cosd(th) + yc*sind(th) + d;

% Creates a red line with a thickness of 12 pixels
on = abs(X*cosd(th) + Y*sind(th) - rho) < 6;

% Creates a background image with a grey color
img = repmat(reshape([0.55 0.55 0.5], 1, 1, 3), size(X));

% Defines the red color of the track
c = [0.8 0.15 0.1];

% Applies the red color to the track in all three RGB channels
for k = 1:3
    ch = img(:,:,k);
    ch(on) = c(k);
    img(:,:,k) = ch;
end

% Adds a blue landing pad only if withPad is true
if withPad

    % Creates a circular landing pad with radius 14 pixels
    % Its center is 20 pixels right and 25 pixels above the image center
    pad = (X - (xc + 20)).^2 + (Y - (yc - 25)).^2 < 14^2;

    % Defines the blue color of the landing pad
    c = [0.1 0.25 0.85];

    % Applies the blue color to the landing pad in all three RGB channels
    for k = 1:3
        ch = img(:,:,k);
        ch(pad) = c(k);
        img(:,:,k) = ch;
    end
end

% Converts image values from the range 0-1 to uint8 values from 0-255
img = uint8(255*img);

% Separates the RGB image into individual red, green, and blue channels
R = img(:,:,1);
G = img(:,:,2);
B = img(:,:,3);

end
