%% ESP32 + MPU6050 vibration rig - offline DSP analysis
% Log from the app (IMU tab -> ESP32 + MPU6050 -> Record 20 s CSV) or from Serial Monitor.
% Columns: t_us, ax, ay, az (m/s^2), gx, gy, gz (deg/s)
% Test protocol: board flat for 5 s, tilt to a known angle (e.g. 30 deg on a protractor) for 5 s,
% back to flat, all with the motor running. Set knownAngle below for the RMSE.
clc; clear; close all;

fileName = 'mpu6050_log.csv';
knownAngle = 30;            % deg, the tilt you held during the middle segment (set [] if unknown)

%% 1. Load and resample to a uniform grid
D = readmatrix(fileName);
D = D(all(isfinite(D), 2), :);
t = (D(:,1) - D(1,1)) / 1e6;
fs = round(1 / median(diff(t)));
tu = (0:1/fs:t(end))';
ax = interp1(t, D(:,2), tu); ay = interp1(t, D(:,3), tu); az = interp1(t, D(:,4), tu);
gy = interp1(t, D(:,6), tu);
n = numel(tu); t = tu;
fprintf('%d samples, fs = %d Hz, %.1f s\n', n, fs, t(end));

%% 2. FFT - find the motor vibration peaks
amag = sqrt(ax.^2 + ay.^2 + az.^2);
f = (0:n-1)'*fs/n; half = 1:floor(n/2);
Y = abs(fft(amag - mean(amag))) / n; Y = Y(half); f = f(half);
Ys = Y; Ys(f < 5) = 0;
[~, i1] = max(Ys); f1 = f(i1);
Ys(abs(f - f1) < 3) = 0;
[~, i2] = max(Ys); f2 = f(i2);
fprintf('Vibration peaks: %.1f Hz and %.1f Hz\n', f1, f2);

%% 3. Notch + low-pass (IIR) and low-pass only (FIR) for comparison
fc = 3;
[bn1, an1] = iirnotch(f1/(fs/2), (f1/(fs/2))/10);
[bn2, an2] = iirnotch(f2/(fs/2), (f2/(fs/2))/10);
[bl, al] = butter(2, fc/(fs/2));
bfir = fir1(2*round(0.6*fs/fc), fc/(fs/2));

axN = filter(bn2, an2, filter(bn1, an1, ax)); azN = filter(bn2, an2, filter(bn1, an1, az)); ayN = filter(bn2, an2, filter(bn1, an1, ay));
axI = filter(bl, al, axN); ayI = filter(bl, al, ayN); azI = filter(bl, al, azN);
axF = filter(bfir, 1, ax);  ayF = filter(bfir, 1, ay);  azF = filter(bfir, 1, az);

rollRaw = atan2d(ax, sqrt(ay.^2 + az.^2));
rollIIR = atan2d(axI, sqrt(ayI.^2 + azI.^2));
rollFIR = atan2d(axF, sqrt(ayF.^2 + azF.^2));

%% 4. Gyro integration, complementary and Kalman
dt = 1/fs;
g = gy;
if sum(g .* gradient(rollIIR)) < 0, g = -g; end      % match gyro sign to accel roll
bias = mean(g(t < 2));                                % board still for the first 2 s
gyroOnly = cumsum(g) * dt;

alpha = 0.98; comp = zeros(n,1); comp(1) = rollIIR(1);
for k = 2:n
    comp(k) = alpha*(comp(k-1) + (g(k) - bias)*dt) + (1 - alpha)*rollIIR(k);
end

x = [rollIIR(1); bias]; P = eye(2); kal = zeros(n,1);
F = [1 -dt; 0 1]; B = [dt; 0]; Hk = [1 0]; Q = diag([0.002 0.0003*dt]); R = 3;
for k = 1:n
    x = F*x + B*g(k); P = F*P*F' + Q;
    K = P*Hk'/(Hk*P*Hk' + R);
    x = x + K*(rollIIR(k) - Hk*x); P = (eye(2) - K*Hk)*P;
    kal(k) = x(1);
end

%% 5. Metrics
hp = @(s) s - filtfilt(bl, al, s);                    % high-frequency part = noise
snrGain = 10*log10(var(hp(rollRaw)) / var(hp(rollIIR)));
fprintf('Roll noise std: raw %.2f deg -> notch+IIR %.2f deg (%.1f dB less noise)\n', std(hp(rollRaw)), std(hp(rollIIR)), snrGain);
gdI = grpdelay(bl, al, [1 2], fs); gdF = grpdelay(bfir, 1, [1 2], fs);
fprintf('Low-pass delay at 1 Hz: IIR %.0f ms, FIR (%d taps) %.0f ms\n', 1000*gdI(1)/fs, numel(bfir), 1000*gdF(1)/fs);
fprintf('Gyro-only drift over %.0f s: %.1f deg\n', t(end), gyroOnly(end) - comp(end));
if ~isempty(knownAngle)
    seg = abs(comp - knownAngle) < 10 & t > 2;
    if any(seg)
        fprintf('RMSE at the %d deg hold: raw %.2f | IIR %.2f | FIR %.2f | comp %.2f | Kalman %.2f deg\n', knownAngle, ...
            rms(rollRaw(seg) - knownAngle), rms(rollIIR(seg) - knownAngle), rms(rollFIR(seg) - knownAngle), ...
            rms(comp(seg) - knownAngle), rms(kal(seg) - knownAngle));
    end
end

%% 6. Plots
Yf = abs(fft(sqrt(axI.^2 + ayI.^2 + azI.^2) - mean(sqrt(axI.^2 + ayI.^2 + azI.^2)))) / n; Yf = Yf(half);
figure('Name', 'MPU6050 rig', 'Color', 'w', 'Position', [60 60 1100 700]);
subplot(3,1,1); plot(f, 20*log10(Y + 1e-6), 'r'); hold on; plot(f, 20*log10(Yf + 1e-6), 'b'); grid on;
xline(f1, '--k'); xline(f2, '--k'); xlabel('Hz'); ylabel('dB');
title('Accelerometer spectrum: real motor vibration before / after notch + LPF'); legend('raw', 'filtered');
subplot(3,1,2); plot(t, rollRaw, 'Color', [1 0.6 0.6]); hold on; plot(t, rollFIR, 'm'); plot(t, rollIIR, 'b', 'LineWidth', 1.3);
grid on; ylabel('deg'); title('Accelerometer roll'); legend('raw', 'FIR LPF', 'notch + IIR LPF');
subplot(3,1,3); plot(t, gyroOnly, 'Color', [1 0.6 0]); hold on; plot(t, comp, 'g', 'LineWidth', 1.5); plot(t, kal, 'k--');
if ~isempty(knownAngle), yline(knownAngle, ':', 'known angle'); end
grid on; xlabel('Time (s)'); ylabel('deg'); title('Fusion'); legend('gyro only', 'complementary', 'Kalman');
