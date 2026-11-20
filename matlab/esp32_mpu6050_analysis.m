%% ESP32 + MPU6050 vibration rig - offline DSP analysis
% This program analyzes vibration data recorded using ESP32 and MPU6050.
% Data can be recorded using the app or Serial Monitor.
% Columns: t_us, ax, ay, az (m/s^2), gx, gy, gz (deg/s)
% Test protocol: keep board flat for 5 seconds, tilt it to a known angle
% for 5 seconds, then bring it back to flat while the motor is running.
% Set knownAngle below to calculate the angle measurement error.
% The program runs on plain MATLAB without requiring additional toolboxes.
% If no CSV file is found, simulated data is generated for testing.
% Simulated results must not be treated as actual hardware results.

clc;
clear;
close all;

fileName = 'mpu6050_log.csv';   % Name of the CSV file containing sensor data
knownAngle = 30;                % Known tilt angle in degrees

%% 1. Load and resample to a uniform grid

simulated = false;              % Initially assume that real sensor data is available

if ~isfile(fileName)             % Check whether the given CSV file exists
    L = dir('mpu6050_log*.csv'); % Search for other MPU6050 log files

    if ~isempty(L)
        [~, iNew] = max([L.datenum]); % Find the newest CSV file
        fileName = L(iNew).name;
    else
        simulated = true;        % Use simulated data if no file is found
    end
end

if simulated
    D = simulateRig(knownAngle); % Generate artificial sensor readings
    fprintf('No mpu6050_log*.csv found: using a SIMULATED 500 Hz rig log.\n');
else
    D = readmatrix(fileName);    % Read sensor data from the CSV file
    fprintf('Loaded %s\n', fileName);
end

D = D(all(isfinite(D), 2), :);   % Remove rows containing invalid values

t = (D(:,1) - D(1,1)) / 1e6;    % Convert timestamps from microseconds to seconds

[t, iu] = unique(t);             % Remove repeated timestamps
D = D(iu, :);

fs = round(1 / median(diff(t))); % Calculate the sampling frequency

tu = (0:1/fs:t(end))';           % Create a uniform time grid

% Interpolate sensor readings so that all samples have equal time spacing
ax = interp1(t, D(:,2), tu);
ay = interp1(t, D(:,3), tu);
az = interp1(t, D(:,4), tu);
gy = interp1(t, D(:,6), tu);

n = numel(tu);                   % Total number of samples
t = tu;

tag = '';
if simulated
    tag = ' (simulated)';        % Add a label if data is simulated
end

fprintf('%d samples, fs = %d Hz, %.1f s\n', n, fs, t(end));

%% 2. FFT - find the motor vibration peaks

% Calculate the total acceleration magnitude from three axes
amag = sqrt(ax.^2 + ay.^2 + az.^2);

% Create the frequency axis for the FFT
f = (0:n-1)'*fs/n;
half = 1:floor(n/2);             % Use only the positive frequency half

% Apply a Hann window before FFT to reduce spectral leakage
win = 0.5 - 0.5*cos(2*pi*(0:n-1)'/(n-1));

% Calculate the FFT and normalize its magnitude
Y = abs(fft((amag - mean(amag)) .* win)) / sum(win);
Y = Y(half);
f = f(half);

Ys = Y;
Ys(f < 5) = 0;                   % Ignore frequencies below 5 Hz

[~, i1] = max(Ys);               % Find the strongest vibration frequency
f1 = f(i1);

Ys(abs(f - f1) < 3) = 0;         % Ignore frequencies close to the first peak

[~, i2] = max(Ys);               % Find the second strongest vibration frequency
f2 = f(i2);

fprintf('Vibration peaks: %.1f Hz and %.1f Hz\n', f1, f2);

%% 3. Notch + low-pass (IIR) and low-pass only (FIR) for comparison

fc = 3;                          % Low-pass cutoff frequency in Hz

% Design two notch filters to remove the detected motor vibrations
[bn1, an1] = notchDesign(f1/(fs/2), (f1/(fs/2))/10);
[bn2, an2] = notchDesign(f2/(fs/2), (f2/(fs/2))/10);

% Design a second-order Butterworth low-pass filter
[bl, al] = butter2(fc/(fs/2));

% Design an FIR low-pass filter for comparison
bfir = firLowpass(2*round(0.6*fs/fc), fc/(fs/2));

% Apply both notch filters to the accelerometer signals
axN = filter(bn2, an2, filter(bn1, an1, ax));
azN = filter(bn2, an2, filter(bn1, an1, az));
ayN = filter(bn2, an2, filter(bn1, an1, ay));

% Apply the Butterworth low-pass filter after removing vibration peaks
axI = filter(bl, al, axN);
ayI = filter(bl, al, ayN);
azI = filter(bl, al, azN);

% Apply only the FIR low-pass filter to the original signals
axF = filter(bfir, 1, ax);
ayF = filter(bfir, 1, ay);
azF = filter(bfir, 1, az);

% Calculate roll angle from the accelerometer readings
% atan2d gives the angle in degrees
rollRaw = atan2d(ax, sqrt(ay.^2 + az.^2));

% Roll angle after notch and IIR low-pass filtering
rollIIR = atan2d(axI, sqrt(ayI.^2 + azI.^2));

% Roll angle after FIR low-pass filtering
rollFIR = atan2d(axF, sqrt(ayF.^2 + azF.^2));

%% 4. Gyro integration, complementary and Kalman

dt = 1/fs;                       % Time between two consecutive samples
g = gy;                          % Gyroscope angular velocity

if sum(g .* gradient(rollIIR)) < 0
    g = -g;                      % Match gyro direction with accelerometer roll
end

bias = mean(g(t < 2));           % Estimate gyro bias during the first 2 seconds

% Integrate angular velocity to calculate angle
gyroOnly = cumsum(g) * dt;

% Complementary filter combines gyro and accelerometer measurements
alpha = 0.98;
comp = zeros(n,1);
comp(1) = rollIIR(1);

for k = 2:n
    % Gyroscope predicts the angle, while accelerometer corrects it
    comp(k) = alpha*(comp(k-1) + (g(k) - bias)*dt) + (1 - alpha)*rollIIR(k);
end

% Initialize the Kalman filter
% x contains the estimated angle and gyro bias
x = [rollIIR(1); bias];

P = eye(2);                      % Initial estimation uncertainty
kal = zeros(n,1);                % Store the estimated angle

% State transition and measurement matrices
F = [1 -dt; 0 1];
B = [dt; 0];
Hk = [1 0];

% Process noise and measurement noise
Q = diag([0.002 0.0003*dt]);
R = 3;

for k = 1:n

    % Predict the next angle and gyro bias
    x = F*x + B*g(k);
    P = F*P*F' + Q;

    % Calculate Kalman gain
    K = P*Hk'/(Hk*P*Hk' + R);

    % Correct the predicted estimate using accelerometer measurement
    x = x + K*(rollIIR(k) - Hk*x);
    P = (eye(2) - K*Hk)*P;

    % Store the estimated roll angle
    kal(k) = x(1);
end

%% 5. Metrics

% Extract the high-frequency part of the signal to estimate noise
hp = @(s) s - filtfiltZ(bl, al, s);

ok = t > 1;                      % Ignore the first second of filter warm-up

% Calculate how much the filtering reduces high-frequency noise
snrGain = 10*log10(var(hp(rollRaw(ok))) / var(hp(rollIIR(ok))));

fprintf('Roll noise std: raw %.2f deg -> notch+IIR %.2f deg (%.1f dB less noise)\n', ...
    std(hp(rollRaw(ok))), std(hp(rollIIR(ok))), snrGain);

% Calculate group delay of IIR and FIR filters
gdI = groupDelay(bl, al, [1 2], fs);
gdF = groupDelay(bfir, 1, [1 2], fs);

fprintf('Low-pass delay at 1 Hz: IIR %.0f ms, FIR (%d taps) %.0f ms\n', ...
    1000*gdI(1)/fs, numel(bfir), 1000*gdF(1)/fs);

% Compare gyro-only integration with the complementary filter
fprintf('Gyro-only drift over %.0f s: %.1f deg\n', ...
    t(end), gyroOnly(end) - comp(end));

% Calculate RMSE if the known tilt angle is available
if ~isempty(knownAngle)

    % Select samples where the estimated angle is close to the known angle
    seg = abs(comp - knownAngle) < 10 & t > 2;

    if any(seg)
        fprintf('RMSE at the %d deg hold: raw %.2f | IIR %.2f | FIR %.2f | comp %.2f | Kalman %.2f deg\n', knownAngle, ...
            rms(rollRaw(seg) - knownAngle), rms(rollIIR(seg) - knownAngle), ...
            rms(rollFIR(seg) - knownAngle), rms(comp(seg) - knownAngle), ...
            rms(kal(seg) - knownAngle));
    end
end

%% 6. Plots - signals

% Calculate total acceleration magnitude after IIR filtering
amagI = sqrt(axI.^2 + ayI.^2 + azI.^2);

% Calculate FFT of the filtered acceleration magnitude
Yf = abs(fft((amagI - mean(amagI)) .* win)) / sum(win);
Yf = Yf(half);

% Create the first figure
lightFig(figure('Name', 'MPU6050 rig', 'Color', 'w', 'Position', [60 60 1100 700]));

% First subplot: compare vibration frequency before and after filtering
subplot(3,1,1);
plot(f, 20*log10(Y + 1e-6), 'r');
hold on;
plot(f, 20*log10(Yf + 1e-6), 'b');
grid on;

xline(f1, '--k');
xline(f2, '--k');

xlabel('Hz');
ylabel('dB');
title(['Accelerometer spectrum: motor vibration before / after notch + LPF' tag]);
legend('raw', 'filtered');

% Second subplot: compare roll angles from different filters
subplot(3,1,2);
plot(t, rollRaw, 'Color', [1 0.6 0.6]);
hold on;
plot(t, rollFIR, 'm');
plot(t, rollIIR, 'b', 'LineWidth', 1.3);

grid on;
ylabel('deg');
title(['Accelerometer roll' tag]);
legend('raw', 'FIR LPF', 'notch + IIR LPF');

xlim([1 t(end)]);                % Ignore the first second of filter warm-up

% Third subplot: compare different angle estimation methods
subplot(3,1,3);
plot(t, gyroOnly, 'Color', [1 0.6 0]);
hold on;
plot(t, comp, 'g', 'LineWidth', 1.5);
plot(t, kal, 'k--');

if ~isempty(knownAngle)
    yline(knownAngle, ':', 'known angle'); % Show the actual tilt angle
end

grid on;
xlabel('Time (s)');
ylabel('deg');
title(['Fusion' tag]);
legend('gyro only', 'complementary', 'Kalman', 'Location', 'northwest');

xlim([1 t(end)]);

%% 7. Plots - frequency response of every filter

% Calculate the frequency response of each filter
[Hn1, w] = freqResp(bn1, an1, 4096, fs);
Hn2 = freqResp(bn2, an2, 4096, fs);
Hl = freqResp(bl, al, 4096, fs);
Hf = freqResp(bfir, 1, 4096, fs);

% Combine the responses to represent the complete IIR filter chain
Hc = Hn1 .* Hn2 .* Hl;

% Convert the magnitude response into decibels
dB = @(H) 20*log10(abs(H) + 1e-12);

lightFig(figure('Name', 'Filter frequency response', 'Color', 'w', 'Position', [80 80 1100 700]));

% First subplot: magnitude response of all filters
subplot(2,1,1);

plot(w, dB(Hn1), 'Color', [0.85 0.4 0]);
hold on;
plot(w, dB(Hn2), 'Color', [0.95 0.7 0.1]);
plot(w, dB(Hl), 'b');
plot(w, dB(Hf), 'm');
plot(w, dB(Hc), 'k', 'LineWidth', 1.5);

% Mark the vibration frequencies and low-pass cutoff frequency
xline(f1, '--k');
xline(f2, '--k');
xline(fc, ':b');

grid on;
xlim([0 fs/2]);
ylim([-100 5]);

xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');

title(sprintf('Magnitude: notches at %.1f / %.1f Hz, low-pass fc = %g Hz%s', ...
    f1, f2, fc, tag));

legend('notch 1', 'notch 2', 'IIR Butterworth LPF', ...
    sprintf('FIR LPF (%d taps)', numel(bfir)), ...
    'notch + notch + IIR', 'Location', 'southwest');

% Second subplot: compare group delay of IIR and FIR filters
subplot(2,1,2);

fz = linspace(0.05, 10, 400);    % Frequency range from 0.05 to 10 Hz

plot(fz, 1000*(groupDelay(bn1, an1, fz, fs) + ...
    groupDelay(bn2, an2, fz, fs) + groupDelay(bl, al, fz, fs))/fs, ...
    'k', 'LineWidth', 1.5);

hold on;

plot(fz, 1000*groupDelay(bfir, 1, fz, fs)/fs, ...
    'm', 'LineWidth', 1.5);

grid on;
xlabel('Frequency (Hz)');
ylabel('Group delay (ms)');

title('Delay in the control band (0-10 Hz): IIR chain vs linear-phase FIR');
legend('notch + notch + IIR', 'FIR');

%% 8. Plots - pole-zero maps

% Display the poles and zeros of each filter
lightFig(figure('Name', 'Pole-zero', 'Color', 'w', 'Position', [100 100 1100 420]));

% Pole-zero plot of the combined notch filters
subplot(1,3,1);
pzPlot(conv(bn1, bn2), conv(an1, an2));
title(sprintf('Notches %.1f / %.1f Hz', f1, f2));

% Pole-zero plot of the Butterworth low-pass filter
subplot(1,3,2);
pzPlot(bl, al);
title(sprintf('Butterworth LPF %g Hz', fc));

% Pole-zero plot of the FIR low-pass filter
subplot(1,3,3);
pzPlot(bfir, 1);
title(sprintf('FIR LPF (%d taps)', numel(bfir)));

sgtitle(['Pole-zero plots: notch zeros sit on the unit circle at the vibration frequencies' tag]);

%% 9. Plots - spectrogram, raw vs filtered

% Choose the window size for the spectrogram
nw = 2^nextpow2(round(0.5*fs));  % Approximately 0.5 second window with 75% overlap

% Calculate the time-frequency representation of raw acceleration
[Sr, fsg, tsg] = stftPower(amag - mean(amag), fs, nw, nw/4);

% Calculate total acceleration magnitude after filtering
amagF = sqrt(axI.^2 + ayI.^2 + azI.^2);

% Calculate the spectrogram of filtered acceleration
Sf = stftPower(amagF - mean(amagF), fs, nw, nw/4);

% Set the same colour range for both spectrograms
cl = max(10*log10(Sr(:))) + [-80 0];

lightFig(figure('Name', 'Spectrogram', 'Color', 'w', 'Position', [120 120 1100 600]));

% First subplot: raw signal spectrogram
subplot(2,1,1);
imagesc(tsg, fsg, 10*log10(Sr));
axis xy;
clim(cl);
colorbar;
colormap(turbo);

ylabel('Hz');
title(['|a| spectrogram, raw: motor lines stay on through the tilt' tag]);

% Second subplot: filtered signal spectrogram
subplot(2,1,2);
imagesc(tsg, fsg, 10*log10(Sf));
axis xy;
clim(cl);
colorbar;

xlabel('Time (s)');
ylabel('Hz');
title('|a| spectrogram after notch + IIR low-pass');

%% ---------------- local functions ----------------

function D = simulateRig(ang)
% Generate simulated 500 Hz sensor data.
% The board stays flat for 5 seconds, tilts, and returns to flat.
% Motor vibration and random noise are added to the sensor readings.

if isempty(ang), ang = 30; end

fs = 500;
t = (0:1/fs:20)';
n = numel(t);
g = 9.81;

% Create a smooth transition instead of an instant angle change
s = @(t0) 0.5 - 0.5*cos(pi*min(max(t - t0, 0), 1));

th = ang*(s(5) - s(13));         % Simulated board tilt angle

fm = 37.3;                       % Motor vibration frequency
vib = 2.5*sin(2*pi*fm*t) + 1.2*sin(2*pi*2*fm*t + 0.4);

% Generate acceleration readings with vibration and random noise
ax = g*sind(th) + vib + 0.08*randn(n,1);
ay = 0.7*vib + 0.08*randn(n,1);
az = g*cosd(th) + 0.5*vib + 0.08*randn(n,1);

% Generate gyroscope readings with bias, vibration and noise
gy = gradient(th, 1/fs) + 1.5 + 0.3*randn(n,1) + 0.8*sin(2*pi*fm*t);

% Combine all readings into the same format as the CSV file
D = [round(t*1e6) ax ay az 0.2*randn(n,1) gy 0.2*randn(n,1)];

end

function [b, a] = notchDesign(w0, bw)
% Design a notch filter to remove a particular frequency.
% The filter reduces the signal around the selected frequency.

Gb = 10^(-3/20);
beta = sqrt(1 - Gb^2)/Gb * tan(bw*pi/2);
gn = 1/(1 + beta);

b = gn*[1 -2*cos(w0*pi) 1];
a = [1 -2*gn*cos(w0*pi) 2*gn-1];

end

function [b, a] = butter2(wn)
% Design a second-order Butterworth low-pass filter.
% It allows low frequencies to pass and reduces high frequencies.

K = tan(pi*wn/2);
nrm = 1/(1 + sqrt(2)*K + K^2);

b = K^2*nrm*[1 2 1];
a = [1 2*(K^2 - 1)*nrm (1 - sqrt(2)*K + K^2)*nrm];

end

function h = firLowpass(N, wn)
% Design an FIR low-pass filter using a Hamming window.
% The filter reduces high-frequency components of the signal.

k = (0:N) - N/2;

h = sin(pi*wn*k)./(pi*k);
h(k == 0) = wn;

% Apply the Hamming window to the filter coefficients
h = h .* (0.54 - 0.46*cos(2*pi*(0:N)/N));

% Normalize the coefficients
h = h/sum(h);

end

function gd = groupDelay(b, a, f, fs)
% Calculate the group delay of a digital filter.
% Group delay tells us how much the filter delays different frequencies.

w = 2*pi*f(:)/fs;
gd = gdPoly(b, w) - gdPoly(a, w);

end

function gd = gdPoly(c, w)
% Calculate the group delay contribution of a polynomial.

k = 0:numel(c)-1;
z = exp(-1i*w*k);

gd = real((z*(k.*c).') ./ (z*c.'));

end

function [H, f] = freqResp(b, a, N, fs)
% Calculate the frequency response of a digital filter.
% H contains the response and f contains the corresponding frequencies.

f = (0:N-1)'*fs/(2*N);
z = exp(-1i*2*pi*f/fs);

H = polyval(fliplr(b), z) ./ polyval(fliplr(a), z);

end

function y = filtfiltZ(b, a, x)
% Apply forward and backward filtering.
% This removes phase delay while filtering the signal.

nb = max(numel(a), numel(b));

a(end+1:nb) = 0;
b(end+1:nb) = 0;

nf = 3*(nb - 1);

% Calculate initial filter conditions
zi = (eye(nb-1) - [-a(2:nb).' [eye(nb-2); zeros(1, nb-2)]]) \ ...
    (b(2:nb).' - a(2:nb).'*b(1));

% Extend the signal at both ends to reduce edge effects
xp = [2*x(1) - x(nf+1:-1:2); x; 2*x(end) - x(end-1:-1:end-nf)];

% Filter in the forward direction
y = filter(b, a, xp, zi*xp(1));

% Reverse the signal and filter again
y = flipud(y);
y = filter(b, a, y, zi*y(1));
y = flipud(y);

% Remove the extra samples added at the beginning and end
y = y(nf+1:end-nf);

end

function pzPlot(b, a)
% Plot the poles and zeros of a digital filter.
% Zeros are shown as circles and poles are shown as crosses.

z = roots(b);
p = roots(a);

% FIR filters have their extra poles at the origin
p = [p; zeros(numel(b) - numel(a), 1)];

% Generate points for drawing the unit circle
th = linspace(0, 2*pi, 400);

plot(cos(th), sin(th), ':', 'Color', [0.5 0.5 0.5]);
hold on;

% Plot zeros and poles
plot(real(z), imag(z), 'bo', 'MarkerSize', 7, 'LineWidth', 1.2);
plot(real(p), imag(p), 'rx', 'MarkerSize', 9, 'LineWidth', 1.5);

axis equal;
grid on;

xlim([-1.3 1.3]);
ylim([-1.3 1.3]);

xlabel('Real');
ylabel('Imag');

legend('unit circle', 'zeros', 'poles', ...
    'Location', 'southoutside', 'Orientation', 'horizontal');

end

function [S, f, tc] = stftPower(x, fs, nw, hop)
% Calculate the Short-Time Fourier Transform (STFT).
% It shows how the frequency content of a signal changes with time.

% Create a Hann window
w = 0.5 - 0.5*cos(2*pi*(0:nw-1)'/(nw-1));

% Find the starting position of each window
starts = 1:hop:numel(x) - nw + 1;

% Create storage for the spectrogram values
S = zeros(nw/2 + 1, numel(starts));

for k = 1:numel(starts)

    % Select a small portion of the signal and apply the window
    X = fft(x(starts(k):starts(k)+nw-1) .* w);

    % Calculate the power for positive frequencies
    S(:,k) = abs(X(1:nw/2+1)).^2 / (fs*sum(w.^2));

end

% Create the frequency and time axes
f = (0:nw/2)'*fs/nw;
tc = (starts - 1 + nw/2)/fs;

end

function fig = lightFig(fig)
% Set the figure to light mode so that labels and titles remain readable.
% This is useful when MATLAB is running in dark mode.

if exist('theme', 'file') || exist('theme', 'builtin')
    try
        theme(fig, 'light');
    catch
    end
end

end
