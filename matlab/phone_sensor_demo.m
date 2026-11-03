%% Mini-drone DSP pipeline on REAL phone sensors (run inside MATLAB Mobile)
% MATLAB Mobile app -> Sensors: turn on Acceleration + Angular Velocity, Stream to: MATLAB
% Then run this script from MATLAB Mobile (or MATLAB Online with the same account).
% Needs Signal Processing + Image Processing Toolboxes.
clc; clear; close all;

%% 1. Record accelerometer + gyroscope
logTime = 15;                 % seconds
m = mobiledev;
m.SampleRate = 100;
m.AccelerationSensorEnabled = 1;
m.AngularVelocitySensorEnabled = 1;
discardlogs(m);
disp('Recording... tilt the phone left/right. Put it on a fan or vibrating motor for vibration.');
m.Logging = 1;
pause(logTime);
m.Logging = 0;
[a, ta] = accellog(m);
[w, tw] = angvellog(m);
fprintf('Got %d accel and %d gyro samples\n', size(a,1), size(w,1));

%% 2. Resample to a uniform time grid
t0 = max(ta(1), tw(1)); t1 = min(ta(end), tw(end));
fs = round(numel(ta)/(ta(end) - ta(1)));
t = (t0:1/fs:t1)'; n = numel(t);
ax = interp1(ta, a(:,1), t); ay = interp1(ta, a(:,2), t); az = interp1(ta, a(:,3), t);
gy = interp1(tw, w(:,2), t);               % rotation about the phone's long axis (rad/s)
fprintf('Uniform sample rate: %d Hz (Nyquist %.0f Hz)\n', fs, fs/2);

%% 3. FFT -> find vibration peaks automatically
amag = sqrt(ax.^2 + ay.^2 + az.^2);
f = (0:n-1)'*fs/n; half = 1:floor(n/2);
Y = abs(fft(amag - mean(amag))); Y = Y(half); f = f(half);
Ys = Y; Ys(f < 2) = 0;
[pk1, i1] = max(Ys); f1 = f(i1);
Ys(abs(f - f1) < 1.5) = 0;
[pk2, i2] = max(Ys); f2 = f(i2);
noiseFloor = median(Y);
useN1 = pk1 > 8*noiseFloor; useN2 = pk2 > 8*noiseFloor;
fprintf('FFT peaks: %.1f Hz (%s), %.1f Hz (%s)\n', f1, string(useN1), f2, string(useN2));

%% 4. Notch + low-pass on each accel axis, then roll
fc = 3;
[bl, al] = butter(2, fc/(fs/2));
[bn1, an1] = iirnotch(f1/(fs/2), (f1/(fs/2))/8);
[bn2, an2] = iirnotch(f2/(fs/2), (f2/(fs/2))/8);
axf = ax; ayf = ay; azf = az;
if useN1
    axf = filter(bn1, an1, axf); ayf = filter(bn1, an1, ayf); azf = filter(bn1, an1, azf);
end
if useN2
    axf = filter(bn2, an2, axf); ayf = filter(bn2, an2, ayf); azf = filter(bn2, an2, azf);
end
axf = filter(bl, al, axf); ayf = filter(bl, al, ayf); azf = filter(bl, al, azf);

rollRaw = rad2deg(atan2(ax, sqrt(ay.^2 + az.^2)));
rollF = rad2deg(atan2(axf, sqrt(ayf.^2 + azf.^2)));

%% 5. Gyro integration, complementary and Kalman fusion
gyroDeg = rad2deg(gy);
dRoll = [0; diff(rollF)];
if sum(gyroDeg.*dRoll) < 0, gyroDeg = -gyroDeg; end       % match gyro sign to accel roll
gyroDeg = gyroDeg + 1.0;                                   % add a 1 deg/s bias so drift is visible

dt = 1/fs;
gyroOnly = rollF(1) + cumsum(gyroDeg)*dt;

alpha = 0.98; comp = zeros(n,1); comp(1) = rollF(1);
for k = 2:n
    comp(k) = alpha*(comp(k-1) + gyroDeg(k)*dt) + (1 - alpha)*rollF(k);
end

x = [rollF(1); 0]; P = eye(2); kal = zeros(n,1);
F = [1 -dt; 0 1]; B = [dt; 0]; Hk = [1 0]; Q = diag([0.002 0.0003*dt]); R = 3;
for k = 1:n
    x = F*x + B*gyroDeg(k); P = F*P*F' + Q;
    K = P*Hk'/(Hk*P*Hk' + R);
    x = x + K*(rollF(k) - Hk*x); P = (eye(2) - K*Hk)*P;
    kal(k) = x(1);
end

nzRaw = std(diff(rollRaw))/sqrt(2); nzF = std(diff(rollF))/sqrt(2);
fprintf('Roll noise (sample-to-sample): raw %.2f deg -> filtered %.2f deg\n', nzRaw, nzF);
fprintf('Gyro-only drift after %.0f s: %.1f deg\n', t(end) - t(1), gyroOnly(end) - comp(end));

Yf = abs(fft(sqrt(axf.^2 + ayf.^2 + azf.^2) - mean(sqrt(axf.^2 + ayf.^2 + azf.^2)))); Yf = Yf(half);
tt = t - t(1);
figure('Name', 'Phone IMU', 'Color', 'w');
subplot(3,1,1); plot(f, 20*log10(Y + 1e-6), 'r'); hold on; plot(f, 20*log10(Yf + 1e-6), 'b'); grid on;
if useN1, xline(f1, '--k'); end
if useN2, xline(f2, '--k'); end
xlabel('Hz'); ylabel('dB'); title('Accelerometer spectrum: before vs after notch + LPF'); legend('raw', 'filtered');
subplot(3,1,2); plot(tt, rollRaw, 'Color', [1 0.6 0.6]); hold on; plot(tt, rollF, 'b', 'LineWidth', 1.3); grid on;
ylabel('deg'); title('Accelerometer roll'); legend('raw', 'notch + LPF');
subplot(3,1,3); plot(tt, gyroOnly, 'Color', [1 0.6 0]); hold on; plot(tt, comp, 'g', 'LineWidth', 1.5); plot(tt, kal, 'm--');
grid on; xlabel('Time (s)'); ylabel('deg'); title('Fusion fixes gyro drift'); legend('gyro only', 'complementary', 'Kalman');

%% 6. Camera: photo of a red tape line through the vision path
% Stick red electrical tape on the floor/desk. Tap the shutter in MATLAB Mobile when asked.
cam = camera(m, 'back');
img = snapshot(cam, 'manual');
img = imresize(im2double(img), [120 160]);
noisy = imnoise(img, 'salt & pepper', 0.06);              % add noise on purpose

[estR, stR] = visionPipe(noisy, false);
[estF, stF] = visionPipe(noisy, true);
figure('Name', 'Phone camera', 'Color', 'w');
subplot(2,3,1); imshow(img); title('Phone photo');
subplot(2,3,2); imshow(noisy); title('+ salt & pepper');
subplot(2,3,3); imshow(stR.raw); title('Raw HSV threshold');
subplot(2,3,4); imshow(stF.mask); title('Median + close/open');
subplot(2,3,5); imshow(stF.edges); title('Edges');
subplot(2,3,6); imshow(noisy); hold on; drawLine(estF, 'g'); drawLine(estR, 'y--');
title(sprintf('DSP: %.1f%c, %.1f px', estF(1), 176, estF(2)));
clear cam m

%% ---------------- local functions ----------------
function [est, st] = visionPipe(img, filt)
[H, W, ~] = size(img);
xc = (W-1)/2; yc = (H-1)/2;
hsv = rgb2hsv(img);
st.raw = (hsv(:,:,1) < 0.06 | hsv(:,:,1) > 0.94) & hsv(:,:,2) > 0.35 & hsv(:,:,3) > 0.15;
bw = st.raw;
if filt
    med = img;
    for k = 1:3, med(:,:,k) = medfilt2(img(:,:,k), [3 3]); end
    hsv = rgb2hsv(med);
    bw = (hsv(:,:,1) < 0.06 | hsv(:,:,1) > 0.94) & hsv(:,:,2) > 0.35 & hsv(:,:,3) > 0.15;
    bw = imopen(imclose(bw, strel('disk', 2)), strel('disk', 2));
end
st.mask = bw;
ed = edge(bw, 'canny');
st.edges = ed;
est = [NaN NaN];
if nnz(ed) < 20, return; end
[Hh, T, R] = hough(ed);
P = houghpeaks(Hh, 6, 'Threshold', 0.3*max(Hh(:)));
if isempty(P), return; end
tt = T(P(:,2)); rr = R(P(:,1));
keep = abs(tt - tt(1)) < 5;
th = mean(tt(keep));
rho = (min(rr(keep)) + max(rr(keep)))/2;
est = [th, rho - (xc*cosd(th) + yc*sind(th))];
end

function drawLine(est, style)
if isnan(est(1)), return; end
W = 160; H = 120; xc = (W-1)/2; yc = (H-1)/2;
rho = est(2) + xc*cosd(est(1)) + yc*sind(est(1));
y = 0:H-1;
plot((rho - y*sind(est(1)))/cosd(est(1)) + 1, y + 1, style, 'LineWidth', 2);
end
