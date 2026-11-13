%% ESP32 + MPU6050 vibration rig - offline DSP analysis
% Log from the app (IMU tab -> ESP32 + MPU6050 -> Record 20 s CSV) or from Serial Monitor.
% Columns: t_us, ax, ay, az (m/s^2), gx, gy, gz (deg/s)
% Test protocol: board flat for 5 s, tilt to a known angle (e.g. 30 deg on a protractor) for 5 s,
% back to flat, all with the motor running. Set knownAngle below for the RMSE.
% Runs on plain MATLAB (filter design, filtfilt, freqz, zplane, spectrogram written out at the end).
% If no log is found, a SIMULATED rig log is generated so the plots can be checked; every figure
% title then says "(simulated)". Never present those numbers as rig results.
clc; clear; close all;

fileName = 'mpu6050_log.csv';   % the app saves mpu6050_log_<time>.csv; the newest one is used if this is missing
knownAngle = 30;                 % deg, the tilt you held during the middle segment (set [] if unknown)

%% 1. Load and resample to a uniform grid
simulated = false;
if ~isfile(fileName)
    L = dir('mpu6050_log*.csv');
    if ~isempty(L)
        [~, iNew] = max([L.datenum]); fileName = L(iNew).name;
    else
        simulated = true;
    end
end
if simulated
    D = simulateRig(knownAngle);
    fprintf('No mpu6050_log*.csv found: using a SIMULATED 500 Hz rig log.\n');
else
    D = readmatrix(fileName);
    fprintf('Loaded %s\n', fileName);
end
D = D(all(isfinite(D), 2), :);
t = (D(:,1) - D(1,1)) / 1e6;
[t, iu] = unique(t); D = D(iu, :);                    % drop repeated timestamps before interp1
fs = round(1 / median(diff(t)));
tu = (0:1/fs:t(end))';
ax = interp1(t, D(:,2), tu); ay = interp1(t, D(:,3), tu); az = interp1(t, D(:,4), tu);
gy = interp1(t, D(:,6), tu);
n = numel(tu); t = tu;
tag = ''; if simulated, tag = ' (simulated)'; end
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
[bn1, an1] = notchDesign(f1/(fs/2), (f1/(fs/2))/10);
[bn2, an2] = notchDesign(f2/(fs/2), (f2/(fs/2))/10);
[bl, al] = butter2(fc/(fs/2));
bfir = firLowpass(2*round(0.6*fs/fc), fc/(fs/2));

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
hp = @(s) s - filtfiltZ(bl, al, s);                   % high-frequency part = noise
ok = t > 1;                                           % skip filter warm-up
snrGain = 10*log10(var(hp(rollRaw(ok))) / var(hp(rollIIR(ok))));
fprintf('Roll noise std: raw %.2f deg -> notch+IIR %.2f deg (%.1f dB less noise)\n', std(hp(rollRaw(ok))), std(hp(rollIIR(ok))), snrGain);
gdI = groupDelay(bl, al, [1 2], fs); gdF = groupDelay(bfir, 1, [1 2], fs);
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

%% 6. Plots - signals
Yf = abs(fft(sqrt(axI.^2 + ayI.^2 + azI.^2) - mean(sqrt(axI.^2 + ayI.^2 + azI.^2)))) / n; Yf = Yf(half);
figure('Name', 'MPU6050 rig', 'Color', 'w', 'Position', [60 60 1100 700]);
subplot(3,1,1); plot(f, 20*log10(Y + 1e-6), 'r'); hold on; plot(f, 20*log10(Yf + 1e-6), 'b'); grid on;
xline(f1, '--k'); xline(f2, '--k'); xlabel('Hz'); ylabel('dB');
title(['Accelerometer spectrum: motor vibration before / after notch + LPF' tag]); legend('raw', 'filtered');
subplot(3,1,2); plot(t, rollRaw, 'Color', [1 0.6 0.6]); hold on; plot(t, rollFIR, 'm'); plot(t, rollIIR, 'b', 'LineWidth', 1.3);
grid on; ylabel('deg'); title(['Accelerometer roll' tag]); legend('raw', 'FIR LPF', 'notch + IIR LPF');
subplot(3,1,3); plot(t, gyroOnly, 'Color', [1 0.6 0]); hold on; plot(t, comp, 'g', 'LineWidth', 1.5); plot(t, kal, 'k--');
if ~isempty(knownAngle), yline(knownAngle, ':', 'known angle'); end
grid on; xlabel('Time (s)'); ylabel('deg'); title(['Fusion' tag]); legend('gyro only', 'complementary', 'Kalman');

%% 7. Plots - frequency response of every filter
[Hn1, w] = freqResp(bn1, an1, 4096, fs); Hn2 = freqResp(bn2, an2, 4096, fs);
Hl = freqResp(bl, al, 4096, fs); Hf = freqResp(bfir, 1, 4096, fs);
Hc = Hn1 .* Hn2 .* Hl;                                % full IIR chain: notch -> notch -> LPF
dB = @(H) 20*log10(abs(H) + 1e-12);
figure('Name', 'Filter frequency response', 'Color', 'w', 'Position', [80 80 1100 700]);
subplot(2,1,1);
plot(w, dB(Hn1), 'Color', [0.85 0.4 0]); hold on; plot(w, dB(Hn2), 'Color', [0.95 0.7 0.1]);
plot(w, dB(Hl), 'b'); plot(w, dB(Hf), 'm'); plot(w, dB(Hc), 'k', 'LineWidth', 1.5);
xline(f1, '--k'); xline(f2, '--k'); xline(fc, ':b');
grid on; xlim([0 fs/2]); ylim([-100 5]); xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)');
title(sprintf('Magnitude: notches at %.1f / %.1f Hz, low-pass fc = %g Hz%s', f1, f2, fc, tag));
legend('notch 1', 'notch 2', 'IIR Butterworth LPF', sprintf('FIR LPF (%d taps)', numel(bfir)), 'notch + notch + IIR', 'Location', 'southwest');
subplot(2,1,2);
fz = linspace(0.05, 10, 400);
plot(fz, 1000*(groupDelay(bn1, an1, fz, fs) + groupDelay(bn2, an2, fz, fs) + groupDelay(bl, al, fz, fs))/fs, 'k', 'LineWidth', 1.5); hold on;
plot(fz, 1000*groupDelay(bfir, 1, fz, fs)/fs, 'm', 'LineWidth', 1.5);
grid on; xlabel('Frequency (Hz)'); ylabel('Group delay (ms)');
title('Delay in the control band (0-10 Hz): IIR chain vs linear-phase FIR'); legend('notch + notch + IIR', 'FIR');

%% 8. Plots - pole-zero maps
figure('Name', 'Pole-zero', 'Color', 'w', 'Position', [100 100 1100 420]);
subplot(1,3,1); pzPlot(conv(bn1, bn2), conv(an1, an2)); title(sprintf('Notches %.1f / %.1f Hz', f1, f2));
subplot(1,3,2); pzPlot(bl, al); title(sprintf('Butterworth LPF %g Hz', fc));
subplot(1,3,3); pzPlot(bfir, 1); title(sprintf('FIR LPF (%d taps)', numel(bfir)));
sgtitle(['Pole-zero plots: notch zeros sit on the unit circle at the vibration frequencies' tag]);

%% 9. Plots - spectrogram, raw vs filtered
nw = 2^nextpow2(round(0.5*fs));                       % ~0.5 s Hann window, 75% overlap
[Sr, fsg, tsg] = stftPower(amag - mean(amag), fs, nw, nw/4);
amagF = sqrt(axI.^2 + ayI.^2 + azI.^2);
Sf = stftPower(amagF - mean(amagF), fs, nw, nw/4);
cl = max(10*log10(Sr(:))) + [-80 0];
figure('Name', 'Spectrogram', 'Color', 'w', 'Position', [120 120 1100 600]);
subplot(2,1,1); imagesc(tsg, fsg, 10*log10(Sr)); axis xy; clim(cl); colorbar; colormap(turbo);
ylabel('Hz'); title(['|a| spectrogram, raw: motor lines stay on through the tilt' tag]);
subplot(2,1,2); imagesc(tsg, fsg, 10*log10(Sf)); axis xy; clim(cl); colorbar;
xlabel('Time (s)'); ylabel('Hz'); title('|a| spectrogram after notch + IIR low-pass');

%% ---------------- local functions ----------------
function D = simulateRig(ang)                     % 500 Hz log: flat 5 s, tilt to ang, back to flat, motor on
if isempty(ang), ang = 30; end
fs = 500; t = (0:1/fs:20)'; n = numel(t); g = 9.81;
s = @(t0) 0.5 - 0.5*cos(pi*min(max(t - t0, 0), 1));  % smooth 1 s ramp
th = ang*(s(5) - s(13));                              % deg
fm = 37.3;                                            % motor speed (Hz) + harmonic
vib = 2.5*sin(2*pi*fm*t) + 1.2*sin(2*pi*2*fm*t + 0.4);
ax = g*sind(th) + vib + 0.08*randn(n,1);
ay = 0.7*vib + 0.08*randn(n,1);
az = g*cosd(th) + 0.5*vib + 0.08*randn(n,1);
gy = gradient(th, 1/fs) + 1.5 + 0.3*randn(n,1) + 0.8*sin(2*pi*fm*t);   % deg/s with bias
D = [round(t*1e6) ax ay az 0.2*randn(n,1) gy 0.2*randn(n,1)];
end

function [b, a] = notchDesign(w0, bw)             % same design as iirnotch (-3 dB bandwidth bw)
Gb = 10^(-3/20);
beta = sqrt(1 - Gb^2)/Gb * tan(bw*pi/2);
gn = 1/(1 + beta);
b = gn*[1 -2*cos(w0*pi) 1];
a = [1 -2*gn*cos(w0*pi) 2*gn-1];
end

function [b, a] = butter2(wn)                     % 2nd-order Butterworth low-pass, bilinear transform
K = tan(pi*wn/2); nrm = 1/(1 + sqrt(2)*K + K^2);
b = K^2*nrm*[1 2 1];
a = [1 2*(K^2 - 1)*nrm (1 - sqrt(2)*K + K^2)*nrm];
end

function h = firLowpass(N, wn)                    % windowed-sinc (Hamming), like fir1(N, wn)
k = (0:N) - N/2;
h = sin(pi*wn*k)./(pi*k); h(k == 0) = wn;
h = h .* (0.54 - 0.46*cos(2*pi*(0:N)/N));
h = h/sum(h);
end

function gd = groupDelay(b, a, f, fs)             % group delay in samples, like grpdelay(b,a,f,fs)
w = 2*pi*f(:)/fs;
gd = gdPoly(b, w) - gdPoly(a, w);
end

function gd = gdPoly(c, w)
k = 0:numel(c)-1; z = exp(-1i*w*k);
gd = real((z*(k.*c).') ./ (z*c.'));
end

function [H, f] = freqResp(b, a, N, fs)           % like freqz(b, a, N, fs)
f = (0:N-1)'*fs/(2*N); z = exp(-1i*2*pi*f/fs);
H = polyval(fliplr(b), z) ./ polyval(fliplr(a), z);
end

function y = filtfiltZ(b, a, x)                   % zero-phase filtering, like filtfilt
nb = max(numel(a), numel(b)); a(end+1:nb) = 0; b(end+1:nb) = 0;
nf = 3*(nb - 1);
zi = (eye(nb-1) - [-a(2:nb).' [eye(nb-2); zeros(1, nb-2)]]) \ (b(2:nb).' - a(2:nb).'*b(1));
xp = [2*x(1) - x(nf+1:-1:2); x; 2*x(end) - x(end-1:-1:end-nf)];
y = filter(b, a, xp, zi*xp(1));
y = flipud(y); y = filter(b, a, y, zi*y(1)); y = flipud(y);
y = y(nf+1:end-nf);
end

function pzPlot(b, a)                             % like zplane(b, a)
z = roots(b); p = roots(a);
p = [p; zeros(numel(b) - numel(a), 1)];          % an FIR filter has all its poles at z = 0
th = linspace(0, 2*pi, 400);
plot(cos(th), sin(th), ':', 'Color', [0.5 0.5 0.5]); hold on;
plot(real(z), imag(z), 'bo', 'MarkerSize', 7, 'LineWidth', 1.2);
plot(real(p), imag(p), 'rx', 'MarkerSize', 9, 'LineWidth', 1.5);
axis equal; grid on; xlim([-1.3 1.3]); ylim([-1.3 1.3]);
xlabel('Real'); ylabel('Imag'); legend('unit circle', 'zeros', 'poles', 'Location', 'southoutside', 'Orientation', 'horizontal');
end

function [S, f, tc] = stftPower(x, fs, nw, hop)   % Hann-window STFT power, like spectrogram(...)
w = 0.5 - 0.5*cos(2*pi*(0:nw-1)'/(nw-1));
starts = 1:hop:numel(x) - nw + 1;
S = zeros(nw/2 + 1, numel(starts));
for k = 1:numel(starts)
    X = fft(x(starts(k):starts(k)+nw-1) .* w);
    S(:,k) = abs(X(1:nw/2+1)).^2 / (fs*sum(w.^2));
end
f = (0:nw/2)'*fs/nw; tc = (starts - 1 + nw/2)/fs;
end
