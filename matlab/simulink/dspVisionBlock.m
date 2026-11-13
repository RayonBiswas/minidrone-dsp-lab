function [theta, d, lineFound, padFound, padX, padY] = dspVisionBlock(R, G, B, useFilters)
% MATLAB Function block for the Image Processing System of parrotMinidroneCompetition.
% Inputs: R, G, B camera planes (uint8, 120x160), useFilters (1 = our DSP chain, 0 = raw threshold).
% Outputs: line angle theta (deg), offset d (px), line/pad flags, pad centroid (px from centre).
% Written with plain loops only (no Image Processing Toolbox calls, no variable-size arrays),
% so it compiles inside a MATLAB Function block on any licence.
% DSP chain: median 3x3 -> HSV threshold -> close + open (disk r=2) -> bottom-half ROI ->
% biggest blob -> boundary edges -> Hough (top 6 peaks, same-angle group) -> least-squares refine.
%#codegen
H = size(R, 1); W = size(R, 2);
xc = (W-1)/2; yc = (H-1)/2;
img = zeros(H, W, 3);
img(:,:,1) = double(R)/255; img(:,:,2) = double(G)/255; img(:,:,3) = double(B)/255;

if useFilters
    for k = 1:3
        img(:,:,k) = median3(img(:,:,k));
    end
end
[h, s, v] = hsvPlanes(img);

% track: red line (set hueC / hueW to the track colour, e.g. picked in the app)
hueC = 0; hueW = 0.06; satMin = 0.45;
dh = abs(h - hueC); dh = min(dh, 1 - dh);
bw = dh < hueW & s > satMin & v > 0.15;
if useFilters
    se = diskSE2();
    bw = openB(closeB(bw, se), se);
    bw(1:floor(H/2), :) = false;                  % bottom-half ROI: only the floor just ahead
    bw = biggestBlob(bw);
end
ed = bw & ~erodeB(bw, [0 1 0; 1 1 1; 0 1 0]);      % boundary pixels
theta = 0; d = 0; lineFound = false;
[ok, th, rho] = houghLine(ed);
if ok
    if useFilters
        [th, rho] = refineLine(bw, th, rho);
    end
    theta = th;
    d = rho - (xc*cosd(th) + yc*sind(th));
    lineFound = true;
end

% landing pad: blue blob
pad = abs(h - 0.62) < 0.09 & s > 0.45 & v > 0.15;
if useFilters
    se = diskSE2();
    pad = openB(closeB(pad, se), se);
end
n = 0; sx = 0; sy = 0;
for c = 1:W
    for r = 1:H
        if pad(r, c), n = n + 1; sx = sx + c; sy = sy + r; end
    end
end
padFound = n > 0.015*H*W;
padX = 0; padY = 0;
if padFound
    padX = sx/n - 1 - xc; padY = sy/n - 1 - yc;
end
end

%% ---------- local functions (all fixed-size, code-generation friendly) ----------
function y = median3(x)
[H, W] = size(x); y = x; win = zeros(9, 1);
for c = 1:W
    for r = 1:H
        k = 0;
        for dc = -1:1
            for dr = -1:1
                k = k + 1;
                win(k) = x(min(max(r+dr, 1), H), min(max(c+dc, 1), W));
            end
        end
        sw = sort(win); y(r, c) = sw(5);
    end
end
end

function [h, s, v] = hsvPlanes(img)
r = img(:,:,1); g = img(:,:,2); b = img(:,:,3);
v = max(max(r, g), b); mn = min(min(r, g), b); dd = v - mn;
s = zeros(size(v)); s(v > 0) = dd(v > 0) ./ v(v > 0);
h = zeros(size(v));
for i = 1:numel(v)
    if dd(i) > 0
        if v(i) == r(i)
            hh = mod((g(i) - b(i))/dd(i), 6);
        elseif v(i) == g(i)
            hh = (b(i) - r(i))/dd(i) + 2;
        else
            hh = (r(i) - g(i))/dd(i) + 4;
        end
        h(i) = hh/6;
    end
end
end

function se = diskSE2()                            % same shape as strel('disk', 2)
se = [0 1 1 1 0; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 0 1 1 1 0];
end

function o = dilateB(bw, se)
o = conv2(double(bw), se, 'same') > 0.5;
end

function o = erodeB(bw, se)                        % pad with 1s like imerode
[H, W] = size(bw); r = (size(se, 1) - 1)/2;
bp = true(H + 2*r, W + 2*r); bp(r+1:r+H, r+1:r+W) = bw;
o = conv2(double(bp), se, 'valid') > sum(se(:)) - 0.5;
end

function o = closeB(bw, se)
o = erodeB(dilateB(bw, se), se);
end

function o = openB(bw, se)
o = dilateB(erodeB(bw, se), se);
end

function o = biggestBlob(bw)                       % 8-connected labelling, keep the largest component
[H, W] = size(bw); N = H*W;
lab = zeros(H, W); stk = zeros(N, 1);
nl = 0; best = 0; bestN = 0;
for p0 = 1:N
    if bw(p0) && lab(p0) == 0
        nl = nl + 1; lab(p0) = nl; sp = 1; stk(1) = p0; cnt = 0;
        while sp > 0
            p = stk(sp); sp = sp - 1; cnt = cnt + 1;
            r = mod(p - 1, H) + 1; c = floor((p - 1)/H) + 1;
            for dc = -1:1
                for dr = -1:1
                    rr = r + dr; cc = c + dc;
                    if rr >= 1 && rr <= H && cc >= 1 && cc <= W
                        q = (cc - 1)*H + rr;
                        if bw(q) && lab(q) == 0
                            lab(q) = nl; sp = sp + 1; stk(sp) = q;
                        end
                    end
                end
            end
        end
        if cnt > bestN, bestN = cnt; best = nl; end
    end
end
o = lab == best & best > 0;
end

function [ok, th, rho] = houghLine(ed)             % rho = x*cos(theta) + y*sin(theta), x,y from 0
[H, W] = size(ed);
D = ceil(sqrt((H-1)^2 + (W-1)^2)); nR = 2*D + 1;
T = -90:89; nT = numel(T); cs = cosd(T); sn = sind(T);
acc = zeros(nR, nT);
nE = 0;
for c = 1:W
    for r = 1:H
        if ed(r, c)
            nE = nE + 1;
            for k = 1:nT
                ri = round((c-1)*cs(k) + (r-1)*sn(k)) + D + 1;
                acc(ri, k) = acc(ri, k) + 1;
            end
        end
    end
end
ok = false; th = 0; rho = 0;
if nE < 20, return; end
pt = zeros(6, 1); pr = zeros(6, 1); np = 0; first = 0;
for i = 1:6                                         % top 6 peaks, suppress a 9x5 neighbourhood
    [bv, idx] = max(acc(:));
    if i == 1, first = bv; end
    if bv < 0.3*first || bv < 8, break; end
    ri = mod(idx - 1, nR) + 1; ti = floor((idx - 1)/nR) + 1;
    np = np + 1; pt(np) = T(ti); pr(np) = ri - D - 1;
    acc(max(ri-4, 1):min(ri+4, nR), max(ti-2, 1):min(ti+2, nT)) = 0;
end
if np == 0, return; end
t0 = pt(1); sT = 0; nK = 0; rMin = inf; rMax = -inf;
for i = 1:np                                        % keep peaks with the same angle (both track edges)
    t = pt(i); r = pr(i);
    if t - t0 > 90, t = t - 180; r = -r; elseif t0 - t > 90, t = t + 180; r = -r; end
    if abs(t - t0) < 5
        sT = sT + t; nK = nK + 1; rMin = min(rMin, r); rMax = max(rMax, r);
    end
end
th = sT/nK; rho = (rMin + rMax)/2;                  % centre between the two track edges
if th >= 90, th = th - 180; rho = -rho; end
if th < -90, th = th + 180; rho = -rho; end
ok = true;
end

function [th, rho] = refineLine(bw, th0, rho0)     % principal-axis fit on mask pixels near the Hough line
[H, W] = size(bw);
c0 = cosd(th0); s0 = sind(th0);
n = 0; sx = 0; sy = 0; sxx = 0; syy = 0; sxy = 0;
for c = 1:W
    for r = 1:H
        x = c - 1; y = r - 1;
        if bw(r, c) && abs(x*c0 + y*s0 - rho0) < 10
            n = n + 1; sx = sx + x; sy = sy + y; sxx = sxx + x*x; syy = syy + y*y; sxy = sxy + x*y;
        end
    end
end
th = th0; rho = rho0;
if n < 20, return; end
mx = sx/n; my = sy/n;
cxx = sxx/n - mx^2; cyy = syy/n - my^2; cxy = sxy/n - mx*my;
t = 0.5*atan2d(2*cxy, cxx - cyy) - 90;              % normal = principal direction - 90 deg
while t - th0 > 90, t = t - 180; end
while th0 - t > 90, t = t + 180; end
if abs(t - th0) > 6, return; end                    % fit disagrees: keep the Hough answer
r = mx*cosd(t) + my*sind(t);
if t >= 90, t = t - 180; r = -r; end
if t < -90, t = t + 180; r = -r; end
th = t; rho = r;
end
