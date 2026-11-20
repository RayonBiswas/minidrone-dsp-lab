%% Noise-Robust Path Tracking for an Autonomous Mini-Drone - DSP pipeline simulation
% Run section by section (Ctrl+Enter) during the demo, or press Run for everything.
% Runs on plain MATLAB: the toolbox functions (medfilt2, hough, butter, iirnotch, fir1 ...)
% are written out as local functions at the end, so every DSP step is visible.

clc; clear; close all;
rng(1);                     % Keep random values the same every time the code runs

W = 160; H = 120;           % Set the mini-drone camera size

%% 1. Vision path - one noisy frame through every stage

thTrue = 20;                % Actual angle of the path
dTrue = 15;                 % Actual distance of the path from the image centre

% Create an ideal path image and add noise to it
clean = makeFrame(thTrue, dTrue, W, H);
noisy = addNoise(clean, 0.10, 0.06);

% Detect the path without filters and with our filters
[estRaw, ~] = visionPipe(noisy, 'raw');
[est, st] = visionPipe(noisy, 'ours');

% Create a figure to show every image processing stage
lightFig(figure('Name', 'Vision pipeline', 'Color', 'w', 'Position', [50 100 1100 520]));

subplot(2,3,1); imshow(clean); title('Ideal frame');
subplot(2,3,2); imshow(noisy); title('Noisy input');
subplot(2,3,3); imshow(st.raw); title('HSV mask (raw)');
subplot(2,3,4); imshow(st.mask); title('Median + morph');
subplot(2,3,5); imshow(st.edges); title('Edges');

% Display the detected path on the noisy image
subplot(2,3,6); imshow(noisy); hold on;
drawLine(est, W, H, 'g');
drawLine(estRaw, W, H, 'y--');

title(sprintf('Hough: %.1f%c / %.1f px (true %.0f%c / %.0f px)', est(1), 176, est(2), thTrue, 176, dTrue));
legend('filtered', 'raw', 'Location', 'southoutside', 'Orientation', 'horizontal');

%% 2. Vision path - Monte Carlo accuracy, with vs without filters

N = 200;                    % Number of random test images

% Initialize counters and arrays for storing results
okR = 0; okF = 0; eaR = []; eaF = []; edR = []; edF = []; tR = 0; tF = 0;

% Test the vision system on many randomly generated images
for i = 1:N

    % Generate a random path angle and distance
    th = -35 + 70*rand;
    d = -30 + 60*rand;

    % Create the image and add noise
    img = addNoise(makeFrame(th, d, W, H), 0.10, 0.06);

    % Measure the processing time without filters
    tic;
    e = visionPipe(img, 'raw');
    tR = tR + toc;

    % Store the angle and distance errors if a path is detected
    if ~isnan(e(1))
        eaR(end+1) = e(1) - th;
        edR(end+1) = e(2) - d;

        % Count successful detections within the allowed error
        if abs(e(1) - th) < 5 && abs(e(2) - d) < 5
            okR = okR + 1;
        end
    end

    % Measure the processing time with filters
    tic;
    e = visionPipe(img, 'ours');
    tF = tF + toc;

    % Store the angle and distance errors
    if ~isnan(e(1))
        eaF(end+1) = e(1) - th;
        edF(end+1) = e(2) - d;

        % Count successful filtered detections
        if abs(e(1) - th) < 5 && abs(e(2) - d) < 5
            okF = okF + 1;
        end
    end
end

% Calculate accuracy, error and processing time for both methods
vis.accR = 100*okR/N;
vis.accF = 100*okF/N;
vis.angR = rms(eaR);
vis.angF = rms(eaF);
vis.offR = rms(edR);
vis.offF = rms(edF);
vis.msR = 1000*tR/N;
vis.msF = 1000*tF/N;

% Display the comparison results
fprintf('\nVISION (%d random frames)\n', N);
fprintf('             accuracy   angle RMSE   offset RMSE   time/frame\n');
fprintf('raw          %5.1f %%    %5.2f deg    %5.2f px      %5.2f ms\n', vis.accR, vis.angR, vis.offR, vis.msR);
fprintf('filtered     %5.1f %%    %5.2f deg    %5.2f px      %5.2f ms\n', vis.accF, vis.angF, vis.offF, vis.msF);

% Test the vision system at different noise levels
spLevels = 0:0.1:0.6;

% Create arrays to store accuracy and angle error
accR = zeros(size(spLevels));
accF = zeros(size(spLevels));
angR = accR;
angF = accF;

for j = 1:numel(spLevels)

    % Store errors for 60 test images at each noise level
    eR = zeros(60,1);
    eF = zeros(60,1);

    for i = 1:60

        % Generate a random path and add the selected noise level
        th = -35 + 70*rand;
        d = -30 + 60*rand;
        img = addNoise(makeFrame(th, d, W, H), 0.10, spLevels(j));

        % Test without filters
        e = visionPipe(img, 'raw');

        % Give a large error if no path is detected
        if isnan(e(1)), e = [th + 20, d + 20]; end

        % Count successful raw detections
        if abs(e(1) - th) < 5 && abs(e(2) - d) < 5
            accR(j) = accR(j) + 1;
        end

        % Store the angle error
        eR(i) = min(abs(e(1) - th), 20);

        % Test with filters
        e = visionPipe(img, 'ours');

        % Give a large error if no path is detected
        if isnan(e(1)), e = [th + 20, d + 20]; end

        % Count successful filtered detections
        if abs(e(1) - th) < 5 && abs(e(2) - d) < 5
            accF(j) = accF(j) + 1;
        end

        % Store the filtered angle error
        eF(i) = min(abs(e(1) - th), 20);
    end

    % Calculate angle RMSE at the current noise level
    angR(j) = sqrt(mean(eR.^2));
    angF(j) = sqrt(mean(eF.^2));
end

% Plot detection accuracy and angle error against noise level
lightFig(figure('Name', 'Vision accuracy vs noise', 'Color', 'w', 'Position', [60 100 1000 400]));

subplot(1,2,1);
plot(100*spLevels, 100*accR/60, 'o-r', 'LineWidth', 1.5); hold on;
plot(100*spLevels, 100*accF/60, 's-b', 'LineWidth', 1.5);
grid on;
xlabel('Salt-and-pepper density (%)');
ylabel('Detection accuracy (%)');
legend('Raw threshold', 'Median + morphology', 'Location', 'southwest');
title('Track found (within 5 deg / 5 px)');
ylim([0 105]);

subplot(1,2,2);
plot(100*spLevels, angR, 'o-r', 'LineWidth', 1.5); hold on;
plot(100*spLevels, angF, 's-b', 'LineWidth', 1.5);
grid on;
xlabel('Salt-and-pepper density (%)');
ylabel('Angle RMSE (deg)');
legend('Raw threshold', 'Median + morphology', 'Location', 'northwest');
title('Line angle error');

%% 2b. Vision ablation - filter vs filter under three noise types

% Different filtering methods to compare
modes = {'raw', 'median', 'gauss', 'morph', 'gauss+morph', 'ours'};

% Names used when displaying the results
names = {'None (raw)', 'Median 3x3', 'Gaussian 5x5', 'Morphology only', 'Gaussian + morph', 'Median + morph (ours)'};

% Define three different noise conditions
conds = {'Salt & pepper', 0.02, 0.15; 'Gaussian noise', 0.25, 0; 'Mixed', 0.10, 0.08};

Na = 60;                    % Number of test images for each condition
abl = zeros(numel(modes), 3);

% Test each noise condition separately
for c = 1:3

    rng(100 + c);           % Use a fixed random sequence for each condition

    % Initialize success counters, errors and processing times
    succ = zeros(1, numel(modes));
    ea = zeros(Na, numel(modes));
    tm = zeros(1, numel(modes));

    for i = 1:Na

        % Generate a random path and add the selected noise
        th = -35 + 70*rand;
        d = -30 + 60*rand;
        img = addNoise(makeFrame(th, d, W, H), conds{c,2}, conds{c,3});

        % Test every filtering method on the same image
        for k = 1:numel(modes)

            % Measure processing time
            tic;
            e = visionPipe(img, modes{k});
            tm(k) = tm(k) + toc;

            % Assign a large error if the path is not detected
            if isnan(e(1)), e = [th + 20, d + 20]; end

            % Store the angle error
            ea(i,k) = min(abs(e(1) - th), 20);

            % Count detections within the allowed angle and distance error
            if abs(e(1) - th) < 3 && abs(e(2) - d) < 3
                succ(k) = succ(k) + 1;
            end
        end
    end

    % Calculate angle RMSE for each filtering method
    abl(:,c) = sqrt(mean(ea.^2))';

    % Display results for the current noise condition
    fprintf('\nABLATION - %s (%d frames, success = |dtheta|<3 deg and |dd|<3 px)\n', conds{c,1}, Na);
    fprintf('%-24s %9s %12s %10s\n', 'Filter', 'success %', 'angle RMSE', 'ms/frame');

    for k = 1:numel(modes)
        fprintf('%-24s %9.0f %10.2f deg %8.1f\n', names{k}, 100*succ(k)/Na, abl(k,c), 1000*tm(k)/Na);
    end
end

% Plot the angle error for all filters under different noise conditions
lightFig(figure('Name', 'Vision ablation', 'Color', 'w'));
bar(abl);
grid on;
ylabel('Angle RMSE (deg)');
title('Filter vs filter under three noise types');

% Set the graph limit and label unusually large errors
yCap = max(3, 1.3*max(abl(abl < 5)));   % Outliers are clipped and labelled

if max(abl(:)) > yCap
    ylim([0 yCap]);
    off = [-0.22 0 0.22];

    for k = 1:size(abl,1)
        for c = 1:3
            if abl(k,c) > yCap
                text(k + off(c), 0.93*yCap, sprintf('%.1f%c', abl(k,c), 176), 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
            end
        end
    end
end

set(gca, 'XTickLabel', names, 'XTickLabelRotation', 25);
legend(conds(:,1), 'Location', 'northwest');

%% 3. Inertial path - FFT + notch + low-pass, complementary and Kalman fusion

fs = 200;                   % Sampling frequency in Hz
T = 20;                     % Total simulation time in seconds

% Create the time vector and calculate the number of samples
t = (0:1/fs:T-1/fs)';
n = numel(t);
g = 9.81;                   % Acceleration due to gravity

% Generate the actual roll angle using two sine waves
roll = deg2rad(15)*sin(2*pi*0.2*t) + deg2rad(8)*sin(2*pi*0.5*t + 1);

% Calculate the angular velocity from the roll angle
rate = gradient(roll, 1/fs);

% Add bias and random noise to the gyroscope signal
gyro = rate + 0.03 + 0.02*randn(n,1);  % Bias causes drift

% Generate motor and propeller vibrations at 47 Hz and 94 Hz
vib = 3*sin(2*pi*47*t) + 1.5*sin(2*pi*94*t);

% Generate noisy accelerometer readings
ay = g*sin(roll) + vib + 0.5*randn(n,1);
az = g*cos(roll) + 0.6*vib + 0.5*randn(n,1);

% Create the frequency vector and select positive frequencies
f = (0:n-1)'*fs/n;
half = 1:floor(n/2);

% Apply FFT to the Y-axis acceleration after removing its mean
Ya = abs(fft(ay - mean(ay)));
Ya = Ya(half);
f = f(half);

% Ignore low frequencies while searching for vibration peaks
Ys = Ya;
Ys(f < 10) = 0;

% Find the first strongest vibration frequency
[~, i1] = max(Ys);
f1 = f(i1);

% Remove nearby frequencies to find a second peak
Ys(abs(f - f1) < 3) = 0;
[~, i2] = max(Ys);
f2 = f(i2);

fprintf('\nINERTIAL: vibration peaks found by FFT at %.1f Hz and %.1f Hz\n', f1, f2);

% Design two notch filters to remove the vibration frequencies
[bn1, an1] = notchDesign(f1/(fs/2), (f1/(fs/2))/10);
[bn2, an2] = notchDesign(f2/(fs/2), (f2/(fs/2))/10);

% Apply both notch filters followed by a Butterworth low-pass filter
[bl, al] = butter2(5/(fs/2));
ayf = filter(bl, al, filter(bn2, an2, filter(bn1, an1, ay)));
azf = filter(bl, al, filter(bn2, an2, filter(bn1, an1, az)));

% Calculate roll angle from the original and filtered accelerometer readings
accRaw = atan2(ay, az);
accF = atan2(ayf, azf);

% Design a 61-tap FIR low-pass filter
bfir = firLowpass(60, 5/(fs/2));

% Apply the notch filters and FIR low-pass filter
ayFir = filter(bfir, 1, filter(bn2, an2, filter(bn1, an1, ay)));
azFir = filter(bfir, 1, filter(bn2, an2, filter(bn1, an1, az)));

% Calculate roll angle using the FIR-filtered signals
accFir = atan2(ayFir, azFir);

% Integrate the gyroscope readings to estimate roll angle
gyroOnly = cumsum(gyro)/fs;

% Complementary filter combines gyroscope and accelerometer readings
alpha = 0.98;
comp = zeros(n,1);

for k = 2:n
    comp(k) = alpha*(comp(k-1) + gyro(k)/fs) + (1 - alpha)*accF(k);
end

% Initialize the Kalman filter
dt = 1/fs;
x = [0; 0];
P = eye(2);
kal = zeros(n,1);

% Define the Kalman filter matrices
F = [1 -dt; 0 1];
B = [dt; 0];
Hk = [1 0];
Q = diag([1e-5 1e-7]);
R = 0.003;

% Run the Kalman filter
for k = 1:n

    % Predict the angle and error covariance
    x = F*x + B*gyro(k);
    P = F*P*F' + Q;

    % Calculate the Kalman gain
    K = P*Hk'/(Hk*P*Hk' + R);

    % Correct the predicted angle using the accelerometer
    x = x + K*(accF(k) - Hk*x);
    P = (eye(2) - K*Hk)*P;

    % Store the estimated angle
    kal(k) = x(1);
end

% Ignore the first second because the filters are still settling
ok = t > 1;                 % Filters start from zero state

% Calculate roll estimation errors using RMSE
imu.accRaw = rad2deg(rms(accRaw(ok) - roll(ok)));
imu.accF = rad2deg(rms(accF(ok) - roll(ok)));
imu.accFir = rad2deg(rms(accFir(ok) - roll(ok)));
imu.gyro = rad2deg(rms(gyroOnly(ok) - roll(ok)));
imu.comp = rad2deg(rms(comp(ok) - roll(ok)));
imu.kal = rad2deg(rms(kal(ok) - roll(ok)));

% Calculate the group delay of the IIR and FIR filters
gdI = groupDelay(bl, al, [1 2], fs);
gdF = groupDelay(bfir, 1, [1 2], fs);

% Display filter delay and roll estimation errors
fprintf('Phase delay at 1 Hz: IIR Butterworth %.1f ms, FIR (61 taps) %.1f ms\n', 1000*gdI(1)/fs, 1000*gdF(1)/fs);
fprintf('Roll RMSE accel: IIR low-pass %.2f deg vs FIR low-pass %.2f deg (FIR lags more)\n', imu.accF, imu.accFir);
fprintf('Roll RMSE: accel raw %.2f | accel filtered %.2f | gyro only %.2f | complementary %.2f | Kalman %.2f deg\n', ...
    imu.accRaw, imu.accF, imu.gyro, imu.comp, imu.kal);

% Calculate the FFT of the filtered acceleration signal
Yf = abs(fft(ayf - mean(ayf)));
Yf = Yf(half);

% Create a figure to display inertial sensor results
lightFig(figure('Name', 'Inertial path', 'Color', 'w', 'Position', [80 80 1100 650]));

% Compare the frequency spectrum before and after filtering
subplot(2,2,1);
plot(f, 20*log10(Ya/n + 1e-6), 'r');
hold on;
plot(f, 20*log10(Yf/n + 1e-6), 'b');
grid on;
xline(f1, '--k');
xline(f2, '--k');
xlabel('Frequency (Hz)');
ylabel('dB');
title('Accelerometer spectrum');
legend('raw', 'notch + LPF');

% Compare raw, true and filtered roll angles
subplot(2,2,2);
plot(t, rad2deg(accRaw), 'Color', [1 0.7 0.7]);
hold on;
plot(t, rad2deg(roll), 'k', 'LineWidth', 1.5);
plot(t, rad2deg(accF), 'b');
grid on;
xlabel('Time (s)');
ylabel('Roll (deg)');
title('Accelerometer roll');
legend('raw', 'true', 'filtered');
xlim([0 10]);

% Compare gyroscope-only, complementary and Kalman estimates
subplot(2,2,3);
plot(t, rad2deg(roll), 'k', 'LineWidth', 1.5);
hold on;
plot(t, rad2deg(gyroOnly), 'r');
plot(t, rad2deg(comp), 'b');
plot(t, rad2deg(kal), 'g--');
grid on;
xlabel('Time (s)');
ylabel('Roll (deg)');
title('Fusion vs gyro drift');
legend('true', 'gyro only', 'complementary', 'Kalman');

% Compare the RMSE values using a bar graph
subplot(2,2,4);
bar([imu.accRaw imu.accF imu.gyro imu.comp imu.kal]);
set(gca, 'XTickLabel', {'Acc raw', 'Acc filt', 'Gyro', 'Comp', 'Kalman'});
ylabel('RMSE (deg)');
title('Roll error');
grid on;

%% 4. Altitude path - median filter + moving average / 1D Kalman

% Generate the actual altitude profile: take-off, hover and landing
h = interp1([0 3 15 19 20], [0 1.1 1.1 0 0], t);

% Add random sensor noise to the altitude readings
hm = h + 0.03*randn(n,1);

% Add random spikes to simulate sudden sensor errors
spk = rand(n,1) < 0.02;
hm(spk) = hm(spk) + sign(randn(nnz(spk),1)).*(0.3 + 0.3*rand(nnz(spk),1));

% Apply a 7-point median filter to remove sudden spikes
hMed = movmedian(hm, 7);

% Apply a 15-point moving average to smooth the readings
hMa = movmean(hm, 15);

% Initialize the one-dimensional altitude Kalman filter
x = [0; 0];
P = eye(2);
hKal = zeros(n,1);

% Define the Kalman filter matrices
F = [1 dt; 0 1];
Hk = [1 0];
Q = diag([1e-6 1e-4]);
R = 0.03^2;

% Run the Kalman filter using median-filtered altitude readings
for k = 1:n

    % Predict altitude and error covariance
    x = F*x;
    P = F*P*F' + Q;

    % Calculate the Kalman gain
    K = P*Hk'/(Hk*P*Hk' + R);

    % Correct the altitude estimate using the sensor reading
    x = x + K*(hMed(k) - Hk*x);
    P = (eye(2) - K*Hk)*P;

    % Store the estimated altitude
    hKal(k) = x(1);
end

% Define a function to calculate signal-to-noise ratio
snr = @(e) 10*log10(sum(h.^2)/sum(e.^2));

% Calculate RMSE and SNR for each altitude filtering method
alt.rmse = [rms(hm - h) rms(hMa - h) rms(hMed - h) rms(hKal - h)];
alt.snr = [snr(hm - h) snr(hMa - h) snr(hMed - h) snr(hKal - h)];

% Display altitude filtering results
fprintf('\nALTITUDE          raw     mov.avg   median   median+Kalman\n');
fprintf('RMSE (cm)      %6.2f   %6.2f   %6.2f   %6.2f\n', 100*alt.rmse);
fprintf('SNR (dB)       %6.1f   %6.1f   %6.1f   %6.1f\n', alt.snr);

% Plot altitude readings and compare their SNR
lightFig(figure('Name', 'Altitude path', 'Color', 'w', 'Position', [100 100 1000 420]));

subplot(1,2,1);
plot(t, hm, 'Color', [1 0.6 0.6]);
hold on;
plot(t, h, 'k', 'LineWidth', 1.5);
plot(t, hMa, 'm');
plot(t, hKal, 'b', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Height (m)');
title('Altitude: take-off, hover, land');
legend('raw sensor', 'true', 'moving avg only', 'median + Kalman');

subplot(1,2,2);
bar(alt.snr);
set(gca, 'XTickLabel', {'Raw', 'Mov avg', 'Median', 'Med+Kalman'});
ylabel('SNR (dB)');
title('Altitude SNR improvement');
grid on;

%% 5. Closed loop - drone follows a competition-style track using only the camera

showLive = true;            % Enable live display of the drone simulation

% Define the track using its corner coordinates in centimetres
trk.pts = [0 0; 220 0; 360 70; 560 70; 700 -50; 900 -50];

% Set the camera scale and track half-width
trk.s = 0.5;
trk.w = 6;

v = 25;                     % Drone speed in cm/s
dtc = 0.05;                 % Simulation time step
steps = round(36/dtc);      % Total number of simulation steps

% Run the track simulation without filtering
resR = flyTrack(false, trk, v, dtc, steps, W, H, false);

% Run the track simulation with filtering
resF = flyTrack(true, trk, v, dtc, steps, W, H, showLive);

% Display tracking error and steering smoothness
fprintf('\nCLOSED LOOP (40 s flight)\n');
fprintf('raw       : RMS tracking error %5.2f cm, steering jerk %.3f, on track %4.1f s\n', rms(resR.err), rms(diff(resR.w)), numel(resR.err)*dtc);
fprintf('filtered  : RMS tracking error %5.2f cm, steering jerk %.3f, on track %4.1f s\n', rms(resF.err), rms(diff(resF.w)), numel(resF.err)*dtc);

% Compare the actual track and both simulated flight paths
lightFig(figure('Name', 'Closed-loop result', 'Color', 'w', 'Position', [120 120 1100 420]));

subplot(1,2,1);
plot(trk.pts(:,1), trk.pts(:,2), 'k', 'LineWidth', 4);
hold on;
plot(resR.path(:,1), resR.path(:,2), 'r', 'LineWidth', 1.3);
plot(resF.path(:,1), resF.path(:,2), 'b', 'LineWidth', 1.3);
axis equal;
grid on;
xlabel('x (cm)');
ylabel('y (cm)');
legend('track', 'raw pipeline', 'DSP pipeline');
title('Flight path');

% Compare steering commands with and without filtering
subplot(1,2,2);
plot((1:numel(resR.w))*dtc, resR.w, 'r');
hold on;
plot((1:numel(resF.w))*dtc, resF.w, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Yaw-rate command (rad/s)');
title('Steering command: jerky vs smooth');
legend('raw', 'DSP pipeline');

%% 6. Summary

% Display the overall performance improvement from the DSP pipeline
fprintf('\n================ SUMMARY ================\n');
fprintf('Vision accuracy        %5.1f %%  ->  %5.1f %%\n', vis.accR, vis.accF);
fprintf('Line angle RMSE        %5.2f deg -> %5.2f deg\n', vis.angR, vis.angF);
fprintf('Roll RMSE (best)       %5.2f deg -> %5.2f deg\n', imu.accRaw, min([imu.comp imu.kal]));
fprintf('Altitude SNR           %5.1f dB  ->  %5.1f dB\n', alt.snr(1), alt.snr(4));
fprintf('Steering jerk          %5.3f     ->  %5.3f\n', rms(diff(resR.w)), rms(diff(resF.w)));
fprintf('Tracking RMS error     %5.2f cm  ->  %5.2f cm\n', rms(resR.err), rms(resF.err));
fprintf('Processing time        %5.2f ms/frame (filtered vision)\n', vis.msF);

%% ---------------- local functions ----------------

% Create an image containing a straight path at the given angle and distance
function img = makeFrame(th, d, W, H)

% Create coordinate grids for the image
[X, Y] = meshgrid(0:W-1, 0:H-1);

% Find the centre of the image
xc = (W-1)/2;
yc = (H-1)/2;

% Calculate the position of the path
rho = xc*cosd(th) + yc*sind(th) + d;

% Select pixels that lie within the path width
on = abs(X*cosd(th) + Y*sind(th) - rho) < 6;

% Create the final image with the path painted in red
img = paint(on);
end

% Create an image with a red path and a background
function img = paint(on)

% Get the image height and width
[H, W] = size(on);

% Create a blank RGB image
img = zeros(H, W, 3);

% Set the background colour
img(:,:,1) = 0.55;
img(:,:,2) = 0.55;
img(:,:,3) = 0.5;

% Set the red colour of the path
c = [0.8 0.15 0.1];

% Apply the red colour to the selected path pixels
for k = 1:3
    ch = img(:,:,k);
    ch(on) = c(k);
    img(:,:,k) = ch;
end
end

% Add different types of noise to the image
function img = addNoise(img, sig, sp)

% Get the image dimensions
[H, W, ~] = size(img);

% Create a horizontal coordinate grid for uneven lighting
[X, ~] = meshgrid(0:W-1, 0:H-1);

% Simulate uneven lighting across the image
img = img .* (0.5 + 0.7*X/W);

% Apply horizontal motion blur
img = blurH(img, 7);

% Add Gaussian noise
img = img + sig*randn(size(img));

% Keep all pixel values between 0 and 1
img = min(max(img, 0), 1);

% Add salt-and-pepper noise
img = saltPepper(img, sp);
end

% Detect the path using the selected filtering method
function [est, st] = visionPipe(img, mode)

% Get the image dimensions and centre
[H, W, ~] = size(img);
xc = (W-1)/2;
yc = (H-1)/2;

% Convert the image from RGB to HSV colour format
hsv = rgb2hsv(img);

% Detect red pixels using HSV threshold values
st.raw = (hsv(:,:,1) < 0.06 | hsv(:,:,1) > 0.94) & hsv(:,:,2) > 0.45 & hsv(:,:,3) > 0.15;

% Start with the original colour mask
bw = st.raw;

% Decide which filters should be used
useMed = any(strcmp(mode, {'median', 'ours'}));
useGauss = any(strcmp(mode, {'gauss', 'gauss+morph'}));
useMorph = any(strcmp(mode, {'morph', 'gauss+morph', 'ours'}));

% Apply median or Gaussian filtering when selected
if useMed || useGauss

    pre = img;

    % Apply the selected filter to each colour channel
    for k = 1:3
        if useMed
            pre(:,:,k) = median3(pre(:,:,k));
        end

        if useGauss
            pre(:,:,k) = gauss5(pre(:,:,k));
        end
    end

    % Convert the filtered image to HSV and detect red pixels again
    hsv = rgb2hsv(pre);
    bw = (hsv(:,:,1) < 0.06 | hsv(:,:,1) > 0.94) & hsv(:,:,2) > 0.45 & hsv(:,:,3) > 0.15;
end

% Apply morphological operations if selected
if useMorph

    % Create a disk-shaped structuring element
    se = diskSE(2);

    % Closing fills small gaps in the detected path
    bw = erodeB(dilateB(bw, se), se);

    % Opening removes small unwanted regions
    bw = dilateB(erodeB(bw, se), se);
end

% Store the final binary mask
st.mask = bw;

% Detect boundary pixels of the binary mask
ed = bw & ~erodeB(bw, [0 1 0; 1 1 1; 0 1 0]);
st.edges = ed;

% Set the estimated angle and distance to NaN initially
est = [NaN NaN];

% Stop if there are too few edge pixels
if nnz(ed) < 20
    return;
end

% Apply the Hough transform to detect straight lines
[Hh, T, R] = houghLines(ed);

% Find the strongest peaks in the Hough transform
P = houghPeaks(Hh, 6, 0.3*max(Hh(:)));

% Stop if no peaks are found
if isempty(P)
    return;
end

% Get the angle and distance values of the detected lines
tt = T(P(:,2));
rr = R(P(:,1));

% Keep lines whose angles are close to the first detected angle
keep = abs(tt - tt(1)) < 5;

% Calculate the average angle and middle distance
th = mean(tt(keep));
rho = (min(rr(keep)) + max(rr(keep)))/2;

% Calculate the estimated angle and distance from the image centre
est = [th, rho - (xc*cosd(th) + yc*sind(th))];
end

% Draw the detected path on the image
function drawLine(est, W, H, style)

% Stop if no valid line was detected
if isnan(est(1))
    return;
end

% Find the image centre
xc = (W-1)/2;
yc = (H-1)/2;

% Convert the estimated distance back to the Hough line distance
rho = est(2) + xc*cosd(est(1)) + yc*sind(est(1));

% Create vertical image coordinates
y = 0:H-1;

% Calculate the horizontal coordinates of the line
x = (rho - y*sind(est(1)))/cosd(est(1));

% Draw the line on the image
plot(x + 1, y + 1, style, 'LineWidth', 2);
end

% Generate the camera image based on the drone's current position
function img = renderTrack(x, y, psi, trk, W, H)

% Create image coordinate grids
[X, Y] = meshgrid(0:W-1, 0:H-1);

% Find the image centre
xc = (W-1)/2;
yc = (H-1)/2;

% Convert image coordinates into forward and left distances
fwd = (yc - Y)*trk.s;
lft = (xc - X)*trk.s;

% Convert the camera coordinates into world coordinates
xw = x + fwd*cos(psi) - lft*sin(psi);
yw = y + fwd*sin(psi) + lft*cos(psi);

% Calculate the distance from each pixel to the track
dist = segDist(xw, yw, trk.pts);

% Create the camera image and add noise
img = addNoise(paint(dist < trk.w), 0.12, 0.12);
end

% Calculate the distance of each point from the track segments
function [dmin, sgn] = segDist(px, py, pts)

% Initialize the minimum distance and direction arrays
dmin = inf(size(px));
sgn = ones(size(px));

% Check the distance from every point to each track segment
for i = 1:size(pts,1)-1

    % Calculate the direction of the current track segment
    dx = pts(i+1,1) - pts(i,1);
    dy = pts(i+1,2) - pts(i,2);

    % Find the closest position on the segment
    t = ((px - pts(i,1))*dx + (py - pts(i,2))*dy)/(dx^2 + dy^2);
    t = min(max(t, 0), 1);

    % Calculate the distance from each point to the segment
    ex = px - pts(i,1) - t*dx;
    ey = py - pts(i,2) - t*dy;
    d = hypot(ex, ey);

    % Update points that are closer to this segment
    better = d < dmin;
    dmin(better) = d(better);

    % Find which side of the segment each point lies on
    s = sign(dx*ey - dy*ex);
    s(s == 0) = 1;
    sgn(better) = s(better);
end
end

% Simulate the drone moving along the track using camera feedback
function res = flyTrack(filt, trk, v, dt, steps, W, H, live)

% Set the initial position, direction and steering values
x = 0;
y = 5;
psi = 0;
ef = 0;
pe = 0;
prev = [0 0];

% Initialize arrays for storing simulation results
res.err = [];
res.w = [];
res.path = [x y];

% Create the live display if requested
if live

    fig = lightFig(figure('Name', 'Live pipeline', 'Color', 'w', 'Position', [60 60 1000 650]));

    % Show the camera image
    subplot(2,2,1);
    hCam = imshow(zeros(H, W, 3));
    title('Camera (noisy)');

    % Show the cleaned binary mask
    subplot(2,2,2);
    hMask = imshow(false(H, W));
    title('Cleaned mask');

    % Show detected edges and the Hough line
    subplot(2,2,3);
    hEdge = imshow(false(H, W));
    hold on;
    hLine = plot(nan, nan, 'g', 'LineWidth', 2);
    title('Edges + Hough line');

    % Show the drone position and its path from above
    subplot(2,2,4);
    plot(trk.pts(:,1), trk.pts(:,2), 'k', 'LineWidth', 4);
    hold on;
    hPath = plot(x, y, 'b', 'LineWidth', 1.5);
    hDrone = plot(x, y, 'ro', 'MarkerFaceColor', 'r');
    axis equal;
    axis([0 900 -100 100]);
    grid on;
    title('Top view');
end

% Run the drone simulation for the selected number of steps
for i = 1:steps

    % Generate the camera image at the current drone position
    img = renderTrack(x, y, psi, trk, W, H);

    % Select raw or filtered image processing
    if filt
        mode = 'ours';
    else
        mode = 'raw';
    end

    % Detect the track from the camera image
    [est, st] = visionPipe(img, mode);

    % Use the previous valid detection if the current detection fails
    if isnan(est(1))
        est = prev;
    end
    prev = est;

    % Calculate steering error from the detected angle and distance
    e = 0.03*est(2)*trk.s + deg2rad(est(1));

    % Smooth the steering error when filtering is enabled
    if filt
        ef = 0.6*ef + 0.4*e;
    else
        ef = e;
    end

    % Calculate the change in steering error
    de = (ef - pe)/dt;
    pe = ef;

    % Calculate the steering command using a PD controller
    w = -(1.2*ef + 0.1*de);

    % Update the drone direction
    psi = psi + w*dt;

    % Update the drone position using its speed and direction
    x = x + v*cos(psi)*dt;
    y = y + v*sin(psi)*dt;

    % Calculate the drone's distance and side of the track
    [dd, sg] = segDist(x, y, trk.pts);
    err = dd*sg;

    % Store tracking error, steering command and drone position
    res.err(end+1) = err;
    res.w(end+1) = w;
    res.path(end+1,:) = [x y];

    % Update the live display
    if live && isvalid(fig)

        % Update camera, mask and edge images
        set(hCam, 'CData', img);
        set(hMask, 'CData', st.mask);
        set(hEdge, 'CData', st.edges);

        % Draw the detected line if a valid detection is available
        if ~isnan(est(1))
            xc = (W-1)/2;
            yc = (H-1)/2;
            rho = est(2) + xc*cosd(est(1)) + yc*sind(est(1));
            yy = 0:H-1;
            set(hLine, 'XData', (rho - yy*sind(est(1)))/cosd(est(1)) + 1, 'YData', yy + 1);
        end

        % Update the drone path and current position
        set(hPath, 'XData', res.path(:,1), 'YData', res.path(:,2));
        set(hDrone, 'XData', x, 'YData', y);

        % Refresh the live figure
        drawnow limitrate;
    end

    % Stop the simulation if the drone moves too far from the track
    if abs(err) > 40
        break;
    end
end
end

%% ---------- plain-MATLAB versions of the toolbox functions ----------

% Apply horizontal moving average to simulate motion blur
function img = blurH(img, n)

h = (n-1)/2;
W = size(img, 2);

% Keep the index values inside the image boundaries
idx = min(max((1-h:W+h), 1), W);

% Apply the averaging filter to each colour channel
for k = 1:size(img, 3)
    img(:,:,k) = conv2(img(:,idx,k), ones(1,n)/n, 'valid');
end
end

% Add salt-and-pepper noise to an image
function img = saltPepper(img, d)

% Generate random values for every image pixel
r = rand(size(img));

% Set some pixels to black
img(r < d/2) = 0;

% Set some pixels to white
img(r >= d/2 & r < d) = 1;
end

% Apply a 3x3 median filter to remove small noise
function y = median3(x)

% Get image dimensions
[H, W] = size(x);

% Extend the image edges to handle boundary pixels
xp = x([1 1:H H], [1 1:W W]);

% Create an array to store the 3x3 neighbourhood values
S = zeros(H, W, 9);
k = 0;

% Collect the neighbouring pixels
for dy = 0:2
    for dx = 0:2
        k = k + 1;
        S(:,:,k) = xp(1+dy:H+dy, 1+dx:W+dx);
    end
end

% Find the median value of each neighbourhood
y = median(S, 3);
end

% Apply a 5x5 Gaussian filter to smooth the image
function y = gauss5(x)

% Create and normalize the Gaussian filter coefficients
g = exp(-(-2:2).^2/2);
g = g/sum(g);

% Get image dimensions
[H, W] = size(x);

% Extend the image edges
xp = x(min(max(-1:H+2, 1), H), min(max(-1:W+2, 1), W));

% Apply the Gaussian filter
y = conv2(g, g, xp, 'valid');
end

% Create a disk-shaped structuring element for morphology
function se = diskSE(r)

[X, Y] = meshgrid(-r:r);
se = double(X.^2 + Y.^2 <= r^2 + 1);
end

% Apply binary dilation to expand white regions
function bw = dilateB(bw, se)

bw = conv2(double(bw), se, 'same') > 0.5;
end

% Apply binary erosion to shrink white regions
function bw = erodeB(bw, se)

% Calculate padding and image dimensions
r = (size(se,1)-1)/2;
[H, W] = size(bw);

% Pad the image with ones at the boundaries
bp = true(H + 2*r, W + 2*r);
bp(r+1:r+H, r+1:r+W) = bw;

% Keep pixels where all structuring element positions are filled
bw = conv2(double(bp), se, 'valid') > nnz(se) - 0.5;
end

% Perform the Hough transform to detect straight lines
function [Hh, T, R] = houghLines(ed)

% Get the image dimensions
[H, W] = size(ed);

% Define possible line angles and distances
T = -90:89;
D = ceil(hypot(H-1, W-1));
R = -D:D;

% Get the coordinates of all edge pixels
[y, x] = find(ed);
x = x - 1;
y = y - 1;

% Create an empty Hough accumulator matrix
Hh = zeros(numel(R), numel(T));

% Calculate the Hough votes for each angle
for k = 1:numel(T)
    r = round(x*cosd(T(k)) + y*sind(T(k)));
    Hh(:,k) = accumarray(r + D + 1, 1, [numel(R) 1]);
end
end

% Find the strongest peaks in the Hough accumulator
function P = houghPeaks(Hh, n, thr)

% Initialize the array of detected peaks
P = zeros(0, 2);

for i = 1:n

    % Find the strongest remaining value
    [v, idx] = max(Hh(:));

    % Stop if the peak is below the threshold
    if v < thr || v == 0
        break;
    end

    % Convert the peak index into row and column positions
    [r, c] = ind2sub(size(Hh), idx);

    % Store the detected peak
    P(end+1, :) = [r c];

    % Remove nearby values to avoid detecting the same line again
    Hh(max(r-4,1):min(r+4,end), max(c-2,1):min(c+2,end)) = 0;
end
end

% Design a notch filter to remove a selected frequency
function [b, a] = notchDesign(w0, bw)

% Calculate the filter bandwidth parameters
Gb = 10^(-3/20);
beta = sqrt(1 - Gb^2)/Gb * tan(bw*pi/2);
g = 1/(1 + beta);

% Calculate the numerator and denominator coefficients
b = g*[1 -2*cos(w0*pi) 1];
a = [1 -2*g*cos(w0*pi) 2*g-1];
end

% Design a second-order Butterworth low-pass filter
function [b, a] = butter2(wn)

% Calculate the filter coefficients using the bilinear transform
K = tan(pi*wn/2);
nrm = 1/(1 + sqrt(2)*K + K^2);

b = K^2*nrm*[1 2 1];
a = [1 2*(K^2 - 1)*nrm (1 - sqrt(2)*K + K^2)*nrm];
end

% Design a low-pass FIR filter using a Hamming window
function h = firLowpass(N, wn)

% Create the filter sample positions
n = (0:N) - N/2;

% Calculate the ideal low-pass filter coefficients
h = sin(pi*wn*n)./(pi*n);
h(n == 0) = wn;

% Apply the Hamming window
h = h .* (0.54 - 0.46*cos(2*pi*(0:N)/N));

% Normalize the filter coefficients
h = h/sum(h);
end

% Calculate the group delay of a digital filter
function gd = groupDelay(b, a, f, fs)

% Convert the given frequencies into angular frequencies
w = 2*pi*f(:)/fs;

% Calculate the delay from numerator and denominator coefficients
gd = gdPoly(b, w) - gdPoly(a, w);
end

% Calculate the group delay contribution of a polynomial
function g = gdPoly(c, w)

% Create the coefficient index values
k = 0:numel(c)-1;

% Calculate the complex frequency response
z = exp(-1i*w*k);

% Calculate the group delay
g = real((z*(k.*c).') ./ (z*c.'));
end

% Set the figure to a light theme for better readability
function fig = lightFig(fig)

% Check whether the theme function is available
if exist('theme', 'file') || exist('theme', 'builtin')
    try
        theme(fig, 'light');
    catch
    end
end
end
