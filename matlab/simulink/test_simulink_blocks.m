%% Test the three Simulink MATLAB Function blocks in plain MATLAB before wiring them into the model
% Feeds 120x160 uint8 R,G,B frames (red track, optional blue pad) through
% addCameraNoise -> dspVisionBlock -> steeringLowpass and compares with the true line.
clc; clear; close all;
clear addCameraNoise steeringLowpass               % reset persistent state
rng(7);
H = 120; W = 160; xc = (W-1)/2; yc = (H-1)/2;
[X, Y] = meshgrid(0:W-1, 0:H-1);

%% 1. Accuracy: raw vs DSP over random frames and three noise levels
levels = [0 0; 0.10 0.08; 0.20 0.15];             % [sigma spDensity], same as the README table
N = 60; res = zeros(size(levels,1), 4);
for L = 1:size(levels,1)
    eR = zeros(N,1); eF = zeros(N,1); tF = 0;
    for i = 1:N
        th = -35 + 70*rand; d = -30 + 60*rand;
        [R, G, B] = makeFrame(th, d, X, Y, xc, yc, false);
        [Rn, Gn, Bn] = addCameraNoise(R, G, B, levels(L,1), levels(L,2), 1);
        [t0, d0, f0] = dspVisionBlock(Rn, Gn, Bn, 0);
        tic; [t1, d1, f1] = dspVisionBlock(Rn, Gn, Bn, 1); tF = tF + toc;
        eR(i) = 20; if f0, eR(i) = min(abs(t0 - th), 20); end
        eF(i) = 20; if f1, eF(i) = min(abs(t1 - th), 20); end
    end
    res(L,:) = [rms(eR) rms(eF) 100*mean(eF < 3) 1000*tF/N];
end
fprintf('dspVisionBlock, %d random frames per row\n', N);
fprintf('sigma  s&p    angle RMSE raw   angle RMSE DSP   DSP success   ms/frame\n');
for L = 1:size(levels,1)
    fprintf('%.2f   %.2f   %8.2f deg     %8.2f deg      %5.0f %%     %6.1f\n', levels(L,1), levels(L,2), res(L,:));
end

%% 2. One frame, checked in detail, with the landing pad
[R, G, B] = makeFrame(12, -8, X, Y, xc, yc, true);
[Rn, Gn, Bn] = addCameraNoise(R, G, B, 0.10, 0.08, 1);
[th, d, lf, pf, px, py] = dspVisionBlock(Rn, Gn, Bn, 1);
fprintf('\nSingle frame: true 12.0 deg / -8.0 px -> %.1f deg / %.1f px, line %d, pad %d at (%.0f, %.0f) px (true (20, -25))\n', th, d, lf, pf, px, py);
assert(lf && abs(th - 12) < 2 && abs(d + 8) < 2, 'line estimate off');
assert(pf && abs(px - 20) < 3 && abs(py + 25) < 3, 'pad estimate off');

%% 3. Steering low-pass: step response and hold when the line is lost
clear steeringLowpass
e = zeros(30,1);
for k = 1:30
    e(k) = steeringLowpass(10, 0, k <= 20, 0.7);  % line lost after step 20
end
fprintf('steeringLowpass: step to %.3f rad after 20 frames, holds %.3f while line lost\n', e(20), e(30));
assert(abs(e(20) - deg2rad(10)) < 0.01 && e(30) == e(20), 'steering low-pass wrong');

figure('Name', 'Simulink block test', 'Color', 'w', 'Position', [80 80 1000 380]);
subplot(1,2,1); imshow(cat(3, Rn, Gn, Bn)); hold on;
yy = 0:H-1; rho = d + xc*cosd(th) + yc*sind(th);
plot((rho - yy*sind(th))/cosd(th) + 1, yy + 1, 'g', 'LineWidth', 2);
plot(px + xc + 1, py + yc + 1, 'co', 'MarkerSize', 20, 'LineWidth', 2);
title(sprintf('dspVisionBlock: %.1f%c / %.1f px (true 12%c / -8 px)', th, 176, d, 176));
subplot(1,2,2); bar(res(:,1:2)); grid on; ylabel('Angle RMSE (deg)');
set(gca, 'XTickLabel', {'clean', 'noise', 'heavy noise'}); legend('raw', 'DSP'); title('Raw vs DSP chain');
disp('All Simulink block tests passed.');

function [R, G, B] = makeFrame(th, d, X, Y, xc, yc, withPad)
rho = xc*cosd(th) + yc*sind(th) + d;
on = abs(X*cosd(th) + Y*sind(th) - rho) < 6;
img = repmat(reshape([0.55 0.55 0.5], 1, 1, 3), size(X));
c = [0.8 0.15 0.1];
for k = 1:3, ch = img(:,:,k); ch(on) = c(k); img(:,:,k) = ch; end
if withPad
    pad = (X - (xc + 20)).^2 + (Y - (yc - 25)).^2 < 14^2;
    c = [0.1 0.25 0.85];
    for k = 1:3, ch = img(:,:,k); ch(pad) = c(k); img(:,:,k) = ch; end
end
img = uint8(255*img);
R = img(:,:,1); G = img(:,:,2); B = img(:,:,3);
end
