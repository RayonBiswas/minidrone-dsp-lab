%% Mini-drone DSP pipeline on REAL phone sensors (run inside MATLAB Mobile)
% MATLAB Mobile app -> Sensors: turn on Acceleration + Angular Velocity, Stream to: MATLAB
% Then run this script from MATLAB Mobile (or MATLAB Online with the same account).
% Needs Signal Processing + Image Processing Toolboxes.

clc; clear; close all;

%% 1. Record accelerometer + gyroscope

logTime = 15;                 % Recording time in seconds
m = mobiledev;                % Connect to the phone sensors
m.SampleRate = 100;           % Set the sampling rate to 100 Hz

% Turn on the accelerometer and gyroscope
m.AccelerationSensorEnabled = 1;
m.AngularVelocitySensorEnabled = 1;

discardlogs(m);               % Clear old sensor data

disp('Recording... tilt the phone left/right. Put it on a fan or vibrating motor for vibration.');

m.Logging = 1;                % Start recording sensor data
pause(logTime);               % Wait for the recording time
m.Logging = 0;                % Stop recording

% Get accelerometer and gyroscope readings with their time values
[a, ta] = accellog(m);
[w, tw] = angvellog(m);

fprintf('Got %d accel and %d gyro samples\n', size(a,1), size(w,1));

%% 2. Resample to a uniform time grid

% Find the common time range of both sensors
t0 = max(ta(1), tw(1));
t1 = min(ta(end), tw(end));

% Calculate the sampling frequency
fs = round(numel(ta)/(ta(end) - ta(1)));

% Create a new time vector with equal time intervals
t = (t0:1/fs:t1)';
n = numel(t);

% Interpolate accelerometer readings onto the new time vector
ax = interp1(ta, a(:,1), t);
ay = interp1(ta, a(:,2), t);
az = interp1(ta, a(:,3), t);

% Interpolate gyroscope readings onto the same time vector
gy = interp1(tw, w(:,2), t);  % Rotation about the phone's long axis (rad/s)

% Display sampling frequency and Nyquist frequency
fprintf('Uniform sample rate: %d Hz (Nyquist %.0f Hz)\n', fs, fs/2);

%% 3. FFT -> find vibration peaks automatically

% Calculate the total acceleration magnitude
amag = sqrt(ax.^2 + ay.^2 + az.^2);

% Create the frequency values for the FFT
f = (0:n-1)'*fs/n;
half = 1:floor(n/2);

% Apply FFT after removing the average acceleration
Y = abs(fft(amag - mean(amag)));

% Keep only the positive frequency values
Y = Y(half);
f = f(half);

% Copy the FFT values for finding vibration peaks
Ys = Y;
Ys(f < 2) = 0;                % Ignore frequencies below 2 Hz

% Find the first highest frequency peak
[pk1, i1] = max(Ys);
f1 = f(i1);

% Remove nearby frequencies so the second peak is different
Ys(abs(f - f1) < 1.5) = 0;

% Find the second highest frequency peak
[pk2, i2] = max(Ys);
f2 = f(i2);

% Compare peaks with the noise level
noiseFloor = median(Y);
useN1 = pk1 > 8*noiseFloor;
useN2 = pk2 > 8*noiseFloor;

% Display the detected vibration frequencies
fprintf('FFT peaks: %.1f Hz (%s), %.1f Hz (%s)\n', f1, string(useN1), f2, string(useN2));

%% 4. Notch + low-pass on each accel axis, then roll

fc = 3;                       % Low-pass filter cutoff frequency

% Design a second-order Butterworth low-pass filter
[bl, al] = butter(2, fc/(fs/2));

% Design notch filters to remove the first and second vibration peaks
[bn1, an1] = iirnotch(f1/(fs/2), (f1/(fs/2))/8);
[bn2, an2] = iirnotch(f2/(fs/2), (f2/(fs/2))/8);

% Make copies of the original acceleration signals
axf = ax;
ayf = ay;
azf = az;

% Apply the first notch filter if a strong peak is found
if useN1
    axf = filter(bn1, an1, axf);
    ayf = filter(bn1, an1, ayf);
    azf = filter(bn1, an1, azf);
end

% Apply the second notch filter if a strong peak is found
if useN2
    axf = filter(bn2, an2, axf);
    ayf = filter(bn2, an2, ayf);
    azf = filter(bn2, an2, azf);
end

% Apply the Butterworth low-pass filter to remove high-frequency noise
axf = filter(bl, al, axf);
ayf = filter(bl, al, ayf);
azf = filter(bl, al, azf);

% Calculate roll angle from the original acceleration readings
rollRaw = rad2deg(atan2(ax, sqrt(ay.^2 + az.^2)));

% Calculate roll angle from the filtered acceleration readings
rollF = rad2deg(atan2(axf, sqrt(ayf.^2 + azf.^2)));

%% 5. Gyro integration, complementary and Kalman fusion

% Convert gyroscope readings from radians per second to degrees per second
gyroDeg = rad2deg(gy);

% Calculate the change in roll angle from the accelerometer
dRoll = [0; diff(rollF)];

% Match the gyroscope direction with the accelerometer direction
if sum(gyroDeg.*dRoll) < 0
    gyroDeg = -gyroDeg;
end

% Add a small bias to make gyroscope drift visible
gyroDeg = gyroDeg + 1.0;

dt = 1/fs;                    % Time between two samples

% Calculate roll angle using only gyroscope readings
gyroOnly = rollF(1) + cumsum(gyroDeg)*dt;

% Complementary filter combines gyroscope and accelerometer readings
alpha = 0.98;
comp = zeros(n,1);
comp(1) = rollF(1);

% Update the roll estimate at every sample
for k = 2:n
    comp(k) = alpha*(comp(k-1) + gyroDeg(k)*dt) + (1 - alpha)*rollF(k);
end

% Set initial angle, bias and error covariance for the Kalman filter
x = [rollF(1); 0];
P = eye(2);
kal = zeros(n,1);

% Define the Kalman filter matrices
F = [1 -dt; 0 1];
B = [dt; 0];
Hk = [1 0];
Q = diag([0.002 0.0003*dt]);
R = 3;

% Run the Kalman filter for all samples
for k = 1:n

    % Predict the next angle and error covariance
    x = F*x + B*gyroDeg(k);
    P = F*P*F' + Q;

    % Calculate the Kalman gain
    K = P*Hk'/(Hk*P*Hk' + R);

    % Correct the prediction using the accelerometer measurement
    x = x + K*(rollF(k) - Hk*x);
    P = (eye(2) - K*Hk)*P;

    % Store the estimated roll angle
    kal(k) = x(1);
end

% Calculate the noise before and after filtering
nzRaw = std(diff(rollRaw))/sqrt(2);
nzF = std(diff(rollF))/sqrt(2);

fprintf('Roll noise (sample-to-sample): raw %.2f deg -> filtered %.2f deg\n', nzRaw, nzF);

% Display the difference between gyroscope-only and complementary estimates
fprintf('Gyro-only drift after %.0f s: %.1f deg\n', t(end) - t(1), gyroOnly(end) - comp(end));

% Calculate the FFT of the filtered acceleration magnitude
Yf = abs(fft(sqrt(axf.^2 + ayf.^2 + azf.^2) - mean(sqrt(axf.^2 + ayf.^2 + azf.^2))));
Yf = Yf(half);

% Reset the time vector to start from zero
tt = t - t(1);

% Create a figure to display the sensor results
figure('Name', 'Phone IMU', 'Color', 'w');

% Plot the frequency spectrum before and after filtering
subplot(3,1,1);
plot(f, 20*log10(Y + 1e-6), 'r');
hold on;
plot(f, 20*log10(Yf + 1e-6), 'b');
grid on;

% Mark the detected vibration frequencies
if useN1, xline(f1, '--k'); end
if useN2, xline(f2, '--k'); end

xlabel('Hz');
ylabel('dB');
title('Accelerometer spectrum: before vs after notch + LPF');
legend('raw', 'filtered');

% Plot the roll angle before and after filtering
subplot(3,1,2);
plot(tt, rollRaw, 'Color', [1 0.6 0.6]);
hold on;
plot(tt, rollF, 'b', 'LineWidth', 1.3);
grid on;
ylabel('deg');
title('Accelerometer roll');
legend('raw', 'notch + LPF');

% Compare gyroscope-only, complementary and Kalman estimates
subplot(3,1,3);
plot(tt, gyroOnly, 'Color', [1 0.6 0]);
hold on;
plot(tt, comp, 'g', 'LineWidth', 1.5);
plot(tt, kal, 'm--');
grid on;
xlabel('Time (s)');
ylabel('deg');
title('Fusion fixes gyro drift');
legend('gyro only', 'complementary', 'Kalman');

%% 6. Camera: photo of a red tape line through the vision path

% Stick red electrical tape on the floor/desk. Tap the shutter in MATLAB Mobile when asked.
cam = camera(m, 'back');

% Capture a photo using the phone's back camera
img = snapshot(cam, 'manual');

% Resize the image and convert its values to the range 0 to 1
img = imresize(im2double(img), [120 160]);

% Add salt-and-pepper noise to test the image filter
noisy = imnoise(img, 'salt & pepper', 0.06);

% Process the image without filtering
[estR, stR] = visionPipe(noisy, false);

% Process the image with filtering
[estF, stF] = visionPipe(noisy, true);

% Display the different image processing stages
figure('Name', 'Phone camera', 'Color', 'w');

subplot(2,3,1);
imshow(img);
title('Phone photo');

subplot(2,3,2);
imshow(noisy);
title('+ salt & pepper');

subplot(2,3,3);
imshow(stR.raw);
title('Raw HSV threshold');

subplot(2,3,4);
imshow(stF.mask);
title('Median + close/open');

subplot(2,3,5);
imshow(stF.edges);
title('Edges');

% Show the detected line on the original noisy image
subplot(2,3,6);
imshow(noisy);
hold on;
drawLine(estF, 'g');
drawLine(estR, 'y--');

title(sprintf('DSP: %.1f%c, %.1f px', estF(1), 176, estF(2)));

% Clear the camera and phone objects
clear cam m

%% ---------------- local functions ----------------

% This function detects the red line and estimates its position and angle
function [est, st] = visionPipe(img, filt)

% Get the image height and width
[H, W, ~] = size(img);

% Find the centre of the image
xc = (W-1)/2;
yc = (H-1)/2;

% Convert the RGB image into HSV colour format
hsv = rgb2hsv(img);

% Detect red colour using hue, saturation and brightness values
st.raw = (hsv(:,:,1) < 0.06 | hsv(:,:,1) > 0.94) & hsv(:,:,2) > 0.35 & hsv(:,:,3) > 0.15;

% Copy the detected red region
bw = st.raw;

% Apply image filtering only when filt is true
if filt

    % Apply a median filter to each colour channel
    med = img;
    for k = 1:3
        med(:,:,k) = medfilt2(img(:,:,k), [3 3]);
    end

    % Convert the filtered image into HSV format
    hsv = rgb2hsv(med);

    % Detect the red region again after filtering
    bw = (hsv(:,:,1) < 0.06 | hsv(:,:,1) > 0.94) & hsv(:,:,2) > 0.35 & hsv(:,:,3) > 0.15;

    % Close small gaps and remove small unwanted regions
    bw = imopen(imclose(bw, strel('disk', 2)), strel('disk', 2));
end

% Store the final binary mask
st.mask = bw;

% Detect edges in the binary image using the Canny method
ed = edge(bw, 'canny');
st.edges = ed;

% Set the estimated line values to NaN initially
est = [NaN NaN];

% Stop if there are too few detected edge pixels
if nnz(ed) < 20
    return;
end

% Apply the Hough transform to detect straight lines
[Hh, T, R] = hough(ed);

% Find the strongest peaks in the Hough transform
P = houghpeaks(Hh, 6, 'Threshold', 0.3*max(Hh(:)));

% Stop if no line peaks are found
if isempty(P)
    return;
end

% Get the angle and distance values of the detected peaks
tt = T(P(:,2));
rr = R(P(:,1));

% Keep lines whose angles are close to the first detected angle
keep = abs(tt - tt(1)) < 5;

% Calculate the average angle and middle distance of the selected lines
th = mean(tt(keep));
rho = (min(rr(keep)) + max(rr(keep)))/2;

% Calculate the line angle and its position relative to the image centre
est = [th, rho - (xc*cosd(th) + yc*sind(th))];

end

% This function draws the detected line on the image
function drawLine(est, style)

% Do not draw anything if no line was detected
if isnan(est(1))
    return;
end

% Set the image dimensions and centre
W = 160;
H = 120;
xc = (W-1)/2;
yc = (H-1)/2;

% Convert the estimated position back to the Hough line distance
rho = est(2) + xc*cosd(est(1)) + yc*sind(est(1));

% Create vertical image coordinates
y = 0:H-1;

% Calculate and plot the line
plot((rho - y*sind(est(1)))/cosd(est(1)) + 1, y + 1, style, 'LineWidth', 2);

end
