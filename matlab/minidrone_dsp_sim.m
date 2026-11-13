%% Noise-Robust Path Tracking for an Autonomous Mini-Drone - DSP pipeline simulation
% Run section by section (Ctrl+Enter) during the demo, or press Run for everything.
% Runs on plain MATLAB: the toolbox functions (medfilt2, hough, butter, iirnotch, fir1 ...)
% are written out as local functions at the end, so every DSP step is visible.
clc; clear; close all;
rng(1);
W = 160; H = 120;           % minidrone camera size

%% 1. Vision path - one noisy frame through every stage
thTrue = 20; dTrue = 15;
clean = makeFrame(thTrue, dTrue, W, H);
noisy = addNoise(clean, 0.10, 0.06);
[estRaw, ~] = visionPipe(noisy, 'raw');
[est, st] = visionPipe(noisy, 'ours');

figure('Name', 'Vision pipeline', 'Color', 'w', 'Position', [50 100 1100 600]);
subplot(2,3,1); imshow(clean); title('Ideal camera frame');
subplot(2,3,2); imshow(noisy); title('Noise + lighting + blur');
subplot(2,3,3); imshow(st.raw); title('HSV threshold (raw)');
subplot(2,3,4); imshow(st.mask); title('Median 3x3 + close/open');
subplot(2,3,5); imshow(st.edges); title('Boundary edges');
subplot(2,3,6); imshow(noisy); hold on;
drawLine(est, W, H, 'g'); drawLine(estRaw, W, H, 'y--');
title(sprintf('Hough: true %.0f%c / %.0fpx, est %.1f%c / %.1fpx', thTrue, 176, dTrue, est(1), 176, est(2)));
legend('filtered', 'raw', 'Location', 'southoutside', 'Orientation', 'horizontal');

%% 2. Vision path - Monte Carlo accuracy, with vs without filters
N = 200;
okR = 0; okF = 0; eaR = []; eaF = []; edR = []; edF = []; tR = 0; tF = 0;
for i = 1:N
    th = -35 + 70*rand; d = -30 + 60*rand;
    img = addNoise(makeFrame(th, d, W, H), 0.10, 0.06);

    tic; e = visionPipe(img, 'raw'); tR = tR + toc;
    if ~isnan(e(1))
        eaR(end+1) = e(1) - th; edR(end+1) = e(2) - d;
        if abs(e(1) - th) < 5 && abs(e(2) - d) < 5, okR = okR + 1; end
    end

    tic; e = visionPipe(img, 'ours'); tF = tF + toc;
    if ~isnan(e(1))
        eaF(end+1) = e(1) - th; edF(end+1) = e(2) - d;
        if abs(e(1) - th) < 5 && abs(e(2) - d) < 5, okF = okF + 1; end
    end
end
vis.accR = 100*okR/N;  vis.accF = 100*okF/N;
vis.angR = rms(eaR);   vis.angF = rms(eaF);
vis.offR = rms(edR);   vis.offF = rms(edF);
vis.msR = 1000*tR/N;   vis.msF = 1000*tF/N;
fprintf('\nVISION (%d random frames)\n', N);
fprintf('             accuracy   angle RMSE   offset RMSE   time/frame\n');
fprintf('raw          %5.1f %%    %5.2f deg    %5.2f px      %5.2f ms\n', vis.accR, vis.angR, vis.offR, vis.msR);
fprintf('filtered     %5.1f %%    %5.2f deg    %5.2f px      %5.2f ms\n', vis.accF, vis.angF, vis.offF, vis.msF);

spLevels = [0 0.02 0.04 0.06 0.08 0.10 0.12];
accR = zeros(size(spLevels)); accF = zeros(size(spLevels));
for j = 1:numel(spLevels)
    for i = 1:60
        th = -35 + 70*rand; d = -30 + 60*rand;
        img = addNoise(makeFrame(th, d, W, H), 0.10, spLevels(j));
        e = visionPipe(img, 'raw');
        if abs(e(1) - th) < 5 && abs(e(2) - d) < 5, accR(j) = accR(j) + 1; end
        e = visionPipe(img, 'ours');
        if abs(e(1) - th) < 5 && abs(e(2) - d) < 5, accF(j) = accF(j) + 1; end
    end
end
figure('Name', 'Vision accuracy vs noise', 'Color', 'w');
plot(100*spLevels, 100*accR/60, 'o-r', 'LineWidth', 1.5); hold on;
plot(100*spLevels, 100*accF/60, 's-b', 'LineWidth', 1.5); grid on;
xlabel('Salt-and-pepper density (%)'); ylabel('Detection accuracy (%)');
legend('Raw threshold', 'Median + morphology', 'Location', 'southwest');
title('Track detection under increasing noise'); ylim([0 105]);

%% 2b. Vision ablation - filter vs filter under three noise types
modes = {'raw', 'median', 'gauss', 'morph', 'gauss+morph', 'ours'};
names = {'None (raw)', 'Median 3x3', 'Gaussian 5x5', 'Morphology only', 'Gaussian + morph', 'Median + morph (ours)'};
conds = {'Salt & pepper', 0.02, 0.15; 'Gaussian noise', 0.25, 0; 'Mixed', 0.10, 0.08};
Na = 60; abl = zeros(numel(modes), 3);
for c = 1:3
    rng(100 + c);
    succ = zeros(1, numel(modes)); ea = zeros(Na, numel(modes)); tm = zeros(1, numel(modes));
    for i = 1:Na
        th = -35 + 70*rand; d = -30 + 60*rand;
        img = addNoise(makeFrame(th, d, W, H), conds{c,2}, conds{c,3});
        for k = 1:numel(modes)
            tic; e = visionPipe(img, modes{k}); tm(k) = tm(k) + toc;
            if isnan(e(1)), e = [th + 20, d + 20]; end
            ea(i,k) = min(abs(e(1) - th), 20);
            if abs(e(1) - th) < 3 && abs(e(2) - d) < 3, succ(k) = succ(k) + 1; end
        end
    end
    abl(:,c) = sqrt(mean(ea.^2))';
    fprintf('\nABLATION - %s (%d frames, success = |dtheta|<3 deg and |dd|<3 px)\n', conds{c,1}, Na);
    fprintf('%-24s %9s %12s %10s\n', 'Filter', 'success %', 'angle RMSE', 'ms/frame');
    for k = 1:numel(modes)
        fprintf('%-24s %9.0f %10.2f deg %8.1f\n', names{k}, 100*succ(k)/Na, abl(k,c), 1000*tm(k)/Na);
    end
end
figure('Name', 'Vision ablation', 'Color', 'w');
bar(abl); grid on; ylabel('Angle RMSE (deg)'); title('Filter vs filter under three noise types');
set(gca, 'XTickLabel', names, 'XTickLabelRotation', 25); legend(conds(:,1), 'Location', 'northeast');

%% 3. Inertial path - FFT + notch + low-pass, complementary and Kalman fusion
fs = 200; T = 20; t = (0:1/fs:T-1/fs)'; n = numel(t); g = 9.81;
roll = deg2rad(15)*sin(2*pi*0.2*t) + deg2rad(8)*sin(2*pi*0.5*t + 1);
rate = gradient(roll, 1/fs);
gyro = rate + 0.03 + 0.02*randn(n,1);                 % bias -> drift
vib = 3*sin(2*pi*47*t) + 1.5*sin(2*pi*94*t);          % motor/propeller vibration
ay = g*sin(roll) + vib + 0.5*randn(n,1);
az = g*cos(roll) + 0.6*vib + 0.5*randn(n,1);

f = (0:n-1)'*fs/n; half = 1:floor(n/2);
Ya = abs(fft(ay - mean(ay))); Ya = Ya(half); f = f(half);
Ys = Ya; Ys(f < 10) = 0;
[~, i1] = max(Ys); f1 = f(i1);
Ys(abs(f - f1) < 3) = 0;
[~, i2] = max(Ys); f2 = f(i2);
fprintf('\nINERTIAL: vibration peaks found by FFT at %.1f Hz and %.1f Hz\n', f1, f2);

[bn1, an1] = notchDesign(f1/(fs/2), (f1/(fs/2))/10);
[bn2, an2] = notchDesign(f2/(fs/2), (f2/(fs/2))/10);
[bl, al] = butter2(5/(fs/2));
ayf = filter(bl, al, filter(bn2, an2, filter(bn1, an1, ay)));
azf = filter(bl, al, filter(bn2, an2, filter(bn1, an1, az)));

accRaw = atan2(ay, az);
accF = atan2(ayf, azf);
bfir = firLowpass(60, 5/(fs/2));
ayFir = filter(bfir, 1, filter(bn2, an2, filter(bn1, an1, ay)));
azFir = filter(bfir, 1, filter(bn2, an2, filter(bn1, an1, az)));
accFir = atan2(ayFir, azFir);
gyroOnly = cumsum(gyro)/fs;

alpha = 0.98; comp = zeros(n,1);
for k = 2:n
    comp(k) = alpha*(comp(k-1) + gyro(k)/fs) + (1 - alpha)*accF(k);
end

dt = 1/fs; x = [0; 0]; P = eye(2); kal = zeros(n,1);
F = [1 -dt; 0 1]; B = [dt; 0]; Hk = [1 0]; Q = diag([1e-5 1e-7]); R = 0.003;
for k = 1:n
    x = F*x + B*gyro(k); P = F*P*F' + Q;
    K = P*Hk'/(Hk*P*Hk' + R);
    x = x + K*(accF(k) - Hk*x); P = (eye(2) - K*Hk)*P;
    kal(k) = x(1);
end

ok = t > 1;                     % skip 1 s warm-up: filters start from zero state (atan2 swings wildly)
imu.accRaw = rad2deg(rms(accRaw(ok) - roll(ok)));
imu.accF = rad2deg(rms(accF(ok) - roll(ok)));
imu.accFir = rad2deg(rms(accFir(ok) - roll(ok)));
imu.gyro = rad2deg(rms(gyroOnly(ok) - roll(ok)));
imu.comp = rad2deg(rms(comp(ok) - roll(ok)));
imu.kal = rad2deg(rms(kal(ok) - roll(ok)));

gdI = groupDelay(bl, al, [1 2], fs); gdF = groupDelay(bfir, 1, [1 2], fs);
fprintf('Phase delay at 1 Hz: IIR Butterworth %.1f ms, FIR (61 taps) %.1f ms\n', 1000*gdI(1)/fs, 1000*gdF(1)/fs);
fprintf('Roll RMSE accel: IIR low-pass %.2f deg vs FIR low-pass %.2f deg (FIR lags more)\n', imu.accF, imu.accFir);
fprintf('Roll RMSE: accel raw %.2f | accel filtered %.2f | gyro only %.2f | complementary %.2f | Kalman %.2f deg\n', ...
    imu.accRaw, imu.accF, imu.gyro, imu.comp, imu.kal);

Yf = abs(fft(ayf - mean(ayf))); Yf = Yf(half);
figure('Name', 'Inertial path', 'Color', 'w', 'Position', [80 80 1100 650]);
subplot(2,2,1); plot(f, Ya, 'r'); hold on; plot(f, Yf, 'b'); grid on;
xline(f1, '--k'); xline(f2, '--k');
xlabel('Frequency (Hz)'); ylabel('|Y(f)|'); title('Accelerometer spectrum');
legend('raw', 'notch + LPF');
subplot(2,2,2); plot(t, rad2deg(accRaw), 'Color', [1 0.7 0.7]); hold on;
plot(t, rad2deg(roll), 'k', 'LineWidth', 1.5); plot(t, rad2deg(accF), 'b');
grid on; xlabel('Time (s)'); ylabel('Roll (deg)'); title('Accelerometer roll');
legend('raw', 'true', 'filtered'); xlim([0 10]);
subplot(2,2,3); plot(t, rad2deg(roll), 'k', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(gyroOnly), 'r'); plot(t, rad2deg(comp), 'b'); plot(t, rad2deg(kal), 'g--');
grid on; xlabel('Time (s)'); ylabel('Roll (deg)'); title('Fusion vs gyro drift');
legend('true', 'gyro only', 'complementary', 'Kalman');
subplot(2,2,4);
bar([imu.accRaw imu.accF imu.gyro imu.comp imu.kal]);
set(gca, 'XTickLabel', {'Acc raw', 'Acc filt', 'Gyro', 'Comp', 'Kalman'});
ylabel('RMSE (deg)'); title('Roll error'); grid on;

%% 4. Altitude path - median filter + moving average / 1D Kalman
h = interp1([0 3 15 19 20], [0 1.1 1.1 0 0], t);
hm = h + 0.03*randn(n,1);
spk = rand(n,1) < 0.02;
hm(spk) = hm(spk) + sign(randn(nnz(spk),1)).*(0.3 + 0.3*rand(nnz(spk),1));

hMed = movmedian(hm, 7);                      % 7-point median filter
hMa = movmean(hm, 15);
x = [0; 0]; P = eye(2); hKal = zeros(n,1);
F = [1 dt; 0 1]; Hk = [1 0]; Q = diag([1e-6 1e-4]); R = 0.03^2;
for k = 1:n
    x = F*x; P = F*P*F' + Q;
    K = P*Hk'/(Hk*P*Hk' + R);
    x = x + K*(hMed(k) - Hk*x); P = (eye(2) - K*Hk)*P;
    hKal(k) = x(1);
end

snr = @(e) 10*log10(sum(h.^2)/sum(e.^2));
alt.rmse = [rms(hm - h) rms(hMa - h) rms(hMed - h) rms(hKal - h)];
alt.snr = [snr(hm - h) snr(hMa - h) snr(hMed - h) snr(hKal - h)];
fprintf('\nALTITUDE          raw     mov.avg   median   median+Kalman\n');
fprintf('RMSE (cm)      %6.2f   %6.2f   %6.2f   %6.2f\n', 100*alt.rmse);
fprintf('SNR (dB)       %6.1f   %6.1f   %6.1f   %6.1f\n', alt.snr);

figure('Name', 'Altitude path', 'Color', 'w', 'Position', [100 100 1000 420]);
subplot(1,2,1); plot(t, hm, 'Color', [1 0.6 0.6]); hold on;
plot(t, h, 'k', 'LineWidth', 1.5); plot(t, hMa, 'm'); plot(t, hKal, 'b', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('Height (m)'); title('Altitude: take-off, hover, land');
legend('raw sensor', 'true', 'moving avg only', 'median + Kalman');
subplot(1,2,2); bar(alt.snr);
set(gca, 'XTickLabel', {'Raw', 'Mov avg', 'Median', 'Med+Kalman'});
ylabel('SNR (dB)'); title('Altitude SNR improvement'); grid on;

%% 5. Closed loop - drone follows a competition-style track using only the camera
showLive = true;
trk.pts = [0 0; 220 0; 360 70; 560 70; 700 -50; 900 -50];   % straight segments + corners (cm)
trk.s = 0.5; trk.w = 6;                                      % cm per pixel, half width (cm)
v = 25; dtc = 0.05; steps = round(36/dtc);

resR = flyTrack(false, trk, v, dtc, steps, W, H, false);
resF = flyTrack(true, trk, v, dtc, steps, W, H, showLive);

fprintf('\nCLOSED LOOP (40 s flight)\n');
fprintf('raw       : RMS tracking error %5.2f cm, steering jerk %.3f, on track %4.1f s\n', rms(resR.err), rms(diff(resR.w)), numel(resR.err)*dtc);
fprintf('filtered  : RMS tracking error %5.2f cm, steering jerk %.3f, on track %4.1f s\n', rms(resF.err), rms(diff(resF.w)), numel(resF.err)*dtc);

figure('Name', 'Closed-loop result', 'Color', 'w', 'Position', [120 120 1100 420]);
subplot(1,2,1);
plot(trk.pts(:,1), trk.pts(:,2), 'k', 'LineWidth', 4); hold on;
plot(resR.path(:,1), resR.path(:,2), 'r', 'LineWidth', 1.3);
plot(resF.path(:,1), resF.path(:,2), 'b', 'LineWidth', 1.3);
axis equal; grid on; xlabel('x (cm)'); ylabel('y (cm)');
legend('track', 'raw pipeline', 'DSP pipeline'); title('Flight path');
subplot(1,2,2);
plot((1:numel(resR.w))*dtc, resR.w, 'r'); hold on;
plot((1:numel(resF.w))*dtc, resF.w, 'b', 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('Yaw-rate command (rad/s)'); title('Steering command: jerky vs smooth');
legend('raw', 'DSP pipeline');

%% 6. Summary
fprintf('\n================ SUMMARY ================\n');
fprintf('Vision accuracy        %5.1f %%  ->  %5.1f %%\n', vis.accR, vis.accF);
fprintf('Line angle RMSE        %5.2f deg -> %5.2f deg\n', vis.angR, vis.angF);
fprintf('Roll RMSE (best)       %5.2f deg -> %5.2f deg\n', imu.accRaw, min([imu.comp imu.kal]));
fprintf('Altitude SNR           %5.1f dB  ->  %5.1f dB\n', alt.snr(1), alt.snr(4));
fprintf('Steering jerk          %5.3f     ->  %5.3f\n', rms(diff(resR.w)), rms(diff(resF.w)));
fprintf('Tracking RMS error     %5.2f cm  ->  %5.2f cm\n', rms(resR.err), rms(resF.err));
fprintf('Processing time        %5.2f ms/frame (filtered vision)\n', vis.msF);

%% ---------------- local functions ----------------
function img = makeFrame(th, d, W, H)
[X, Y] = meshgrid(0:W-1, 0:H-1);
xc = (W-1)/2; yc = (H-1)/2;
rho = xc*cosd(th) + yc*sind(th) + d;
on = abs(X*cosd(th) + Y*sind(th) - rho) < 6;
img = paint(on);
end

function img = paint(on)
[H, W] = size(on);
img = zeros(H, W, 3);
img(:,:,1) = 0.55; img(:,:,2) = 0.55; img(:,:,3) = 0.5;
c = [0.8 0.15 0.1];
for k = 1:3
    ch = img(:,:,k); ch(on) = c(k); img(:,:,k) = ch;
end
end

function img = addNoise(img, sig, sp)
[H, W, ~] = size(img);
[X, ~] = meshgrid(0:W-1, 0:H-1);
img = img .* (0.5 + 0.7*X/W);                     % uneven lighting
img = blurH(img, 7);                              % motion blur
img = img + sig*randn(size(img));                 % gaussian noise
img = min(max(img, 0), 1);
img = saltPepper(img, sp);                        % per-channel impulses, like a real sensor
end

function [est, st] = visionPipe(img, mode)
[H, W, ~] = size(img);
xc = (W-1)/2; yc = (H-1)/2;
hsv = rgb2hsv(img);
st.raw = (hsv(:,:,1) < 0.06 | hsv(:,:,1) > 0.94) & hsv(:,:,2) > 0.45 & hsv(:,:,3) > 0.15;
bw = st.raw;
useMed = any(strcmp(mode, {'median', 'ours'}));
useGauss = any(strcmp(mode, {'gauss', 'gauss+morph'}));
useMorph = any(strcmp(mode, {'morph', 'gauss+morph', 'ours'}));
if useMed || useGauss
    pre = img;
    for k = 1:3
        if useMed, pre(:,:,k) = median3(pre(:,:,k)); end          % median on the image first
        if useGauss, pre(:,:,k) = gauss5(pre(:,:,k)); end
    end
    hsv = rgb2hsv(pre);
    bw = (hsv(:,:,1) < 0.06 | hsv(:,:,1) > 0.94) & hsv(:,:,2) > 0.45 & hsv(:,:,3) > 0.15;
end
if useMorph
    se = diskSE(2);
    bw = erodeB(dilateB(bw, se), se);             % close
    bw = dilateB(erodeB(bw, se), se);             % open
end
st.mask = bw;
ed = bw & ~erodeB(bw, [0 1 0; 1 1 1; 0 1 0]);     % boundary pixels of the mask
st.edges = ed;
est = [NaN NaN];
if nnz(ed) < 20, return; end
[Hh, T, R] = houghLines(ed);
P = houghPeaks(Hh, 6, 0.3*max(Hh(:)));
if isempty(P), return; end
tt = T(P(:,2)); rr = R(P(:,1));
keep = abs(tt - tt(1)) < 5;
th = mean(tt(keep));
rho = (min(rr(keep)) + max(rr(keep)))/2;        % centre between the two track edges
est = [th, rho - (xc*cosd(th) + yc*sind(th))];
end

function drawLine(est, W, H, style)
if isnan(est(1)), return; end
xc = (W-1)/2; yc = (H-1)/2;
rho = est(2) + xc*cosd(est(1)) + yc*sind(est(1));
y = 0:H-1;
x = (rho - y*sind(est(1)))/cosd(est(1));
plot(x + 1, y + 1, style, 'LineWidth', 2);
end

function img = renderTrack(x, y, psi, trk, W, H)
[X, Y] = meshgrid(0:W-1, 0:H-1);
xc = (W-1)/2; yc = (H-1)/2;
fwd = (yc - Y)*trk.s; lft = (xc - X)*trk.s;
xw = x + fwd*cos(psi) - lft*sin(psi);
yw = y + fwd*sin(psi) + lft*cos(psi);
dist = segDist(xw, yw, trk.pts);
img = addNoise(paint(dist < trk.w), 0.12, 0.12);
end

function [dmin, sgn] = segDist(px, py, pts)
dmin = inf(size(px)); sgn = ones(size(px));
for i = 1:size(pts,1)-1
    dx = pts(i+1,1) - pts(i,1); dy = pts(i+1,2) - pts(i,2);
    t = ((px - pts(i,1))*dx + (py - pts(i,2))*dy)/(dx^2 + dy^2);
    t = min(max(t, 0), 1);
    ex = px - pts(i,1) - t*dx; ey = py - pts(i,2) - t*dy;
    d = hypot(ex, ey);
    better = d < dmin;
    dmin(better) = d(better);
    s = sign(dx*ey - dy*ex); s(s == 0) = 1;
    sgn(better) = s(better);
end
end

function res = flyTrack(filt, trk, v, dt, steps, W, H, live)
x = 0; y = 5; psi = 0; ef = 0; pe = 0; prev = [0 0];
res.err = []; res.w = []; res.path = [x y];
if live
    fig = figure('Name', 'Live pipeline', 'Color', 'w', 'Position', [60 60 1000 650]);
    subplot(2,2,1); hCam = imshow(zeros(H, W, 3)); title('Camera (noisy)');
    subplot(2,2,2); hMask = imshow(false(H, W)); title('Cleaned mask');
    subplot(2,2,3); hEdge = imshow(false(H, W)); hold on;
    hLine = plot(nan, nan, 'g', 'LineWidth', 2); title('Edges + Hough line');
    subplot(2,2,4);
    plot(trk.pts(:,1), trk.pts(:,2), 'k', 'LineWidth', 4); hold on;
    hPath = plot(x, y, 'b', 'LineWidth', 1.5); hDrone = plot(x, y, 'ro', 'MarkerFaceColor', 'r');
    axis equal; axis([0 900 -100 100]); grid on; title('Top view');
end
for i = 1:steps
    img = renderTrack(x, y, psi, trk, W, H);
    if filt, mode = 'ours'; else, mode = 'raw'; end
    [est, st] = visionPipe(img, mode);
    if isnan(est(1)), est = prev; end
    prev = est;
    e = 0.03*est(2)*trk.s + deg2rad(est(1));      % steering error from offset + angle
    if filt
        ef = 0.6*ef + 0.4*e;                      % low-pass before PID
    else
        ef = e;
    end
    de = (ef - pe)/dt; pe = ef;
    w = -(1.2*ef + 0.1*de);                       % PD on steering error
    psi = psi + w*dt;
    x = x + v*cos(psi)*dt; y = y + v*sin(psi)*dt;
    [dd, sg] = segDist(x, y, trk.pts); err = dd*sg;
    res.err(end+1) = err; res.w(end+1) = w; res.path(end+1,:) = [x y];
    if live && isvalid(fig)
        set(hCam, 'CData', img); set(hMask, 'CData', st.mask); set(hEdge, 'CData', st.edges);
        if ~isnan(est(1))
            xc = (W-1)/2; yc = (H-1)/2;
            rho = est(2) + xc*cosd(est(1)) + yc*sind(est(1));
            yy = 0:H-1; set(hLine, 'XData', (rho - yy*sind(est(1)))/cosd(est(1)) + 1, 'YData', yy + 1);
        end
        set(hPath, 'XData', res.path(:,1), 'YData', res.path(:,2));
        set(hDrone, 'XData', x, 'YData', y);
        drawnow limitrate;
    end
    if abs(err) > 40, break; end                  % left the track
end
end

%% ---------- plain-MATLAB versions of the toolbox functions ----------
function img = blurH(img, n)                      % horizontal moving average, replicate edges
h = (n-1)/2; W = size(img, 2);
idx = min(max((1-h:W+h), 1), W);
for k = 1:size(img, 3)
    img(:,:,k) = conv2(img(:,idx,k), ones(1,n)/n, 'valid');
end
end

function img = saltPepper(img, d)                 % same rule as imnoise(...,'salt & pepper', d)
r = rand(size(img));
img(r < d/2) = 0;
img(r >= d/2 & r < d) = 1;
end

function y = median3(x)                           % 3x3 median filter
[H, W] = size(x);
xp = x([1 1:H H], [1 1:W W]);
S = zeros(H, W, 9); k = 0;
for dy = 0:2
    for dx = 0:2
        k = k + 1; S(:,:,k) = xp(1+dy:H+dy, 1+dx:W+dx);
    end
end
y = median(S, 3);
end

function y = gauss5(x)                            % 5x5 Gaussian, sigma = 1
g = exp(-(-2:2).^2/2); g = g/sum(g);
[H, W] = size(x);
xp = x(min(max(-1:H+2, 1), H), min(max(-1:W+2, 1), W));
y = conv2(g, g, xp, 'valid');
end

function se = diskSE(r)                           % same shape as strel('disk', 2)
[X, Y] = meshgrid(-r:r);
se = double(X.^2 + Y.^2 <= r^2 + 1);
end

function bw = dilateB(bw, se)
bw = conv2(double(bw), se, 'same') > 0.5;
end

function bw = erodeB(bw, se)                      % pad with 1s like imerode
r = (size(se,1)-1)/2; [H, W] = size(bw);
bp = true(H + 2*r, W + 2*r); bp(r+1:r+H, r+1:r+W) = bw;
bw = conv2(double(bp), se, 'valid') > nnz(se) - 0.5;
end

function [Hh, T, R] = houghLines(ed)              % rho = x*cos(theta) + y*sin(theta), 1 deg / 1 px bins
[H, W] = size(ed);
T = -90:89; D = ceil(hypot(H-1, W-1)); R = -D:D;
[y, x] = find(ed); x = x - 1; y = y - 1;
Hh = zeros(numel(R), numel(T));
for k = 1:numel(T)
    r = round(x*cosd(T(k)) + y*sind(T(k)));
    Hh(:,k) = accumarray(r + D + 1, 1, [numel(R) 1]);
end
end

function P = houghPeaks(Hh, n, thr)               % strongest cells, suppress a 9x5 neighbourhood
P = zeros(0, 2);
for i = 1:n
    [v, idx] = max(Hh(:));
    if v < thr || v == 0, break; end
    [r, c] = ind2sub(size(Hh), idx);
    P(end+1, :) = [r c];
    Hh(max(r-4,1):min(r+4,end), max(c-2,1):min(c+2,end)) = 0;
end
end

function [b, a] = notchDesign(w0, bw)             % same design as iirnotch (-3 dB bandwidth bw)
Gb = 10^(-3/20);
beta = sqrt(1 - Gb^2)/Gb * tan(bw*pi/2);
g = 1/(1 + beta);
b = g*[1 -2*cos(w0*pi) 1];
a = [1 -2*g*cos(w0*pi) 2*g-1];
end

function [b, a] = butter2(wn)                     % 2nd-order Butterworth low-pass, bilinear transform
K = tan(pi*wn/2); nrm = 1/(1 + sqrt(2)*K + K^2);
b = K^2*nrm*[1 2 1];
a = [1 2*(K^2 - 1)*nrm (1 - sqrt(2)*K + K^2)*nrm];
end

function h = firLowpass(N, wn)                    % windowed-sinc (Hamming), like fir1(N, wn)
n = (0:N) - N/2;
h = sin(pi*wn*n)./(pi*n); h(n == 0) = wn;
h = h .* (0.54 - 0.46*cos(2*pi*(0:N)/N));
h = h/sum(h);
end

function gd = groupDelay(b, a, f, fs)             % group delay in samples, like grpdelay(b,a,f,fs)
w = 2*pi*f(:)/fs;
gd = gdPoly(b, w) - gdPoly(a, w);
end

function g = gdPoly(c, w)
k = 0:numel(c)-1; z = exp(-1i*w*k);
g = real((z*(k.*c).') ./ (z*c.'));
end
