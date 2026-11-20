function [theta, d, lineFound, padFound, padX, padY] = dspVisionBlock(R, G, B, useFilters)

% This function processes camera images to detect a track line and landing pad.
% It is used in the Image Processing System of parrotMinidroneCompetition.
%
% Inputs:
% R, G, B  = Red, Green and Blue camera image channels (120x160).
% useFilters = 1 to apply our DSP processing, 0 to use raw thresholding.
%
% Outputs:
% theta     = Detected line angle in degrees.
% d         = Distance of the line from the image centre in pixels.
% lineFound = Indicates whether a track line is detected.
% padFound  = Indicates whether a landing pad is detected.
% padX      = Horizontal position of the pad from image centre.
% padY      = Vertical position of the pad from image centre.
%
% Processing steps:
% Median filter -> HSV conversion -> Colour thresholding ->
% Morphological operations -> ROI -> Largest blob ->
% Edge detection -> Hough transform -> Line refinement.
%
%#codegen

% Get image height and width
H = size(R, 1);
W = size(R, 2);

% Calculate the centre coordinates of the image
xc = (W-1)/2;
yc = (H-1)/2;

% Create an empty RGB image
img = zeros(H, W, 3);

% Convert RGB pixel values from 0-255 to 0-1
img(:,:,1) = double(R)/255;
img(:,:,2) = double(G)/255;
img(:,:,3) = double(B)/255;

% Apply a 3x3 median filter to remove noise from each colour channel
if useFilters
    for k = 1:3
        img(:,:,k) = median3(img(:,:,k));
    end
end

% Convert the RGB image into Hue, Saturation and Value components
[h, s, v] = hsvPlanes(img);

% Track detection: identify the red colour using HSV thresholding
% Change hueC and hueW if the track has a different colour
hueC = 0;
hueW = 0.06;
satMin = 0.45;

% Calculate the difference between actual hue and target hue
dh = abs(h - hueC);

% Account for the circular nature of hue values
dh = min(dh, 1 - dh);

% Create a binary image: true for pixels matching the red colour
bw = dh < hueW & s > satMin & v > 0.15;

if useFilters

    % Create a disk-shaped structuring element
    se = diskSE2();

    % Apply closing to fill small gaps and opening to remove small noise
    bw = openB(closeB(bw, se), se);

    % Keep only the bottom half of the image where the track is expected
    bw(1:floor(H/2), :) = false;

    % Keep only the largest connected region
    bw = biggestBlob(bw);
end

% Detect boundary pixels by subtracting the eroded image
ed = bw & ~erodeB(bw, [0 1 0; 1 1 1; 0 1 0]);

% Initialize line detection outputs
theta = 0;
d = 0;
lineFound = false;

% Detect a straight line using the Hough transform
[ok, th, rho] = houghLine(ed);

if ok

    % Improve the detected line angle using least-squares refinement
    if useFilters
        [th, rho] = refineLine(bw, th, rho);
    end

    theta = th;

    % Calculate the line's distance from the image centre
    d = rho - (xc*cosd(th) + yc*sind(th));

    % Mark that a track line has been detected
    lineFound = true;
end

% Landing pad detection: identify the blue region using HSV thresholding
pad = abs(h - 0.62) < 0.09 & s > 0.45 & v > 0.15;

if useFilters

    % Apply morphological closing and opening to clean the blue region
    se = diskSE2();
    pad = openB(closeB(pad, se), se);
end

% Initialize variables for calculating the pad centre
n = 0;
sx = 0;
sy = 0;

% Visit every pixel in the binary pad image
for c = 1:W
    for r = 1:H

        if pad(r, c)

            % Count the detected pixels and add their coordinates
            n = n + 1;
            sx = sx + c;
            sy = sy + r;

        end
    end
end

% Declare a pad detected if enough blue pixels are present
padFound = n > 0.015*H*W;

% Initialize pad position
padX = 0;
padY = 0;

if padFound

    % Calculate the average pixel position of the landing pad
    % Subtract image centre to get position relative to the centre
    padX = sx/n - 1 - xc;
    padY = sy/n - 1 - yc;

end
end

%% ---------- local functions (all fixed-size, code-generation friendly) ----------

function y = median3(x)
% Apply a 3x3 median filter to remove small noise from an image.
% Each pixel is replaced by the middle value of its 3x3 neighbourhood.

[H, W] = size(x);

y = x;
win = zeros(9, 1);               % Store the 9 neighbouring pixel values

for c = 1:W
    for r = 1:H

        k = 0;

        % Visit the 3x3 neighbourhood around the current pixel
        for dc = -1:1
            for dr = -1:1

                k = k + 1;

                % Keep coordinates inside the image boundaries
                win(k) = x(min(max(r+dr, 1), H), min(max(c+dc, 1), W));

            end
        end

        % Sort the 9 values and select the middle value
        sw = sort(win);
        y(r, c) = sw(5);

    end
end
end

function [h, s, v] = hsvPlanes(img)
% Convert an RGB image into separate HSV colour components.
% Hue represents colour, saturation represents colour strength,
% and value represents brightness.

r = img(:,:,1);
g = img(:,:,2);
b = img(:,:,3);

% Find the maximum and minimum colour values for each pixel
v = max(max(r, g), b);
mn = min(min(r, g), b);

% Calculate the difference between maximum and minimum values
dd = v - mn;

% Calculate saturation
s = zeros(size(v));
s(v > 0) = dd(v > 0) ./ v(v > 0);

% Initialize hue values
h = zeros(size(v));

% Calculate hue for every pixel
for i = 1:numel(v)

    if dd(i) > 0

        % Calculate hue depending on which RGB channel is maximum
        if v(i) == r(i)
            hh = mod((g(i) - b(i))/dd(i), 6);

        elseif v(i) == g(i)
            hh = (b(i) - r(i))/dd(i) + 2;

        else
            hh = (r(i) - g(i))/dd(i) + 4;

        end

        % Normalize hue between 0 and 1
        h(i) = hh/6;

    end
end
end

function se = diskSE2()
% Create a disk-shaped structuring element of radius 2.
% It is used for morphological opening and closing.

se = [0 1 1 1 0;
      1 1 1 1 1;
      1 1 1 1 1;
      1 1 1 1 1;
      0 1 1 1 0];

end

function o = dilateB(bw, se)
% Dilation expands white regions in a binary image.
% It helps fill small gaps and connect nearby regions.

o = conv2(double(bw), se, 'same') > 0.5;

end

function o = erodeB(bw, se)
% Erosion shrinks white regions in a binary image.
% It helps remove small unwanted regions.
% The image is padded with ones to handle its boundaries.

[H, W] = size(bw);
r = (size(se, 1) - 1)/2;

% Add padding around the binary image
bp = true(H + 2*r, W + 2*r);
bp(r+1:r+H, r+1:r+W) = bw;

% Keep pixels where the structuring element completely fits
o = conv2(double(bp), se, 'valid') > sum(se(:)) - 0.5;

end

function o = closeB(bw, se)
% Closing = dilation followed by erosion.
% It fills small holes and gaps in white regions.

o = erodeB(dilateB(bw, se), se);

end

function o = openB(bw, se)
% Opening = erosion followed by dilation.
% It removes small unwanted white regions.

o = dilateB(erodeB(bw, se), se);

end

function o = biggestBlob(bw)
% Find all connected white regions and keep only the largest one.
% 8-connected means diagonal neighbouring pixels are also connected.

[H, W] = size(bw);
N = H*W;

lab = zeros(H, W);               % Store the label of each connected region
stk = zeros(N, 1);               % Stack used to visit connected pixels

nl = 0;                           % Number of connected regions found
best = 0;                         % Label of the largest region
bestN = 0;                        % Pixel count of the largest region

% Check every pixel in the binary image
for p0 = 1:N

    % Start a new region if the pixel is white and not already visited
    if bw(p0) && lab(p0) == 0

        nl = nl + 1;
        lab(p0) = nl;

        sp = 1;
        stk(1) = p0;
        cnt = 0;

        % Visit all connected pixels using a stack
        while sp > 0

            p = stk(sp);
            sp = sp - 1;
            cnt = cnt + 1;

            % Convert the linear index into row and column
            r = mod(p - 1, H) + 1;
            c = floor((p - 1)/H) + 1;

            % Check all 8 neighbouring pixels
            for dc = -1:1
                for dr = -1:1

                    rr = r + dr;
                    cc = c + dc;

                    % Make sure the neighbour is inside the image
                    if rr >= 1 && rr <= H && cc >= 1 && cc <= W

                        q = (cc - 1)*H + rr;

                        % Add an unvisited white neighbour to the stack
                        if bw(q) && lab(q) == 0

                            lab(q) = nl;
                            sp = sp + 1;
                            stk(sp) = q;

                        end
                    end
                end
            end
        end

        % Update the largest region if the current one is bigger
        if cnt > bestN
            bestN = cnt;
            best = nl;
        end
    end
end

% Create an output containing only the largest connected region
o = lab == best & best > 0;

end

function [ok, th, rho] = houghLine(ed)
% Detect a straight line using the Hough transform.
% A line is represented by rho = x*cos(theta) + y*sin(theta).
% The accumulator counts how many edge pixels support each line.

[H, W] = size(ed);

% Calculate the maximum possible distance from the image origin
D = ceil(sqrt((H-1)^2 + (W-1)^2));

nR = 2*D + 1;

% Test line angles from -90 to 89 degrees
T = -90:89;
nT = numel(T);

% Precalculate sine and cosine values
cs = cosd(T);
sn = sind(T);

% Create the Hough accumulator matrix
acc = zeros(nR, nT);

nE = 0;                           % Count the total number of edge pixels

% Visit every pixel in the edge image
for c = 1:W
    for r = 1:H

        if ed(r, c)

            nE = nE + 1;

            % Vote for possible lines at every tested angle
            for k = 1:nT

                ri = round((c-1)*cs(k) + (r-1)*sn(k)) + D + 1;

                % Increase the accumulator value for this line
                acc(ri, k) = acc(ri, k) + 1;

            end
        end
    end
end

% Initialize line detection outputs
ok = false;
th = 0;
rho = 0;

% Stop if there are too few edge pixels
if nE < 20
    return;
end

% Store the top 6 detected peaks
pt = zeros(6, 1);
pr = zeros(6, 1);

np = 0;
first = 0;

% Find the strongest peaks in the accumulator
for i = 1:6

    % Find the maximum accumulator value
    [bv, idx] = max(acc(:));

    if i == 1
        first = bv;
    end

    % Stop if the peak is too weak
    if bv < 0.3*first || bv < 8
        break;
    end

    % Convert the linear index into row and column
    ri = mod(idx - 1, nR) + 1;
    ti = floor((idx - 1)/nR) + 1;

    % Store the detected angle and distance
    np = np + 1;
    pt(np) = T(ti);
    pr(np) = ri - D - 1;

    % Suppress nearby peaks so that the same line is not detected again
    acc(max(ri-4, 1):min(ri+4, nR), ...
        max(ti-2, 1):min(ti+2, nT)) = 0;

end

% Stop if no valid peak was found
if np == 0
    return;
end

% Initialize variables for grouping similar line angles
t0 = pt(1);
sT = 0;
nK = 0;
rMin = inf;
rMax = -inf;

% Group peaks that have approximately the same angle
% These peaks usually represent the two edges of the track
for i = 1:np

    t = pt(i);
    r = pr(i);

    % Adjust angles that differ by more than 90 degrees
    if t - t0 > 90
        t = t - 180;
        r = -r;

    elseif t0 - t > 90
        t = t + 180;
        r = -r;
    end

    % Keep peaks whose angles are within 5 degrees
    if abs(t - t0) < 5

        sT = sT + t;
        nK = nK + 1;

        % Find the minimum and maximum distances
        rMin = min(rMin, r);
        rMax = max(rMax, r);

    end
end

% Calculate the average angle and the centre between track edges
th = sT/nK;
rho = (rMin + rMax)/2;

% Keep the final angle within the range -90 to 90 degrees
if th >= 90
    th = th - 180;
    rho = -rho;
end

if th < -90
    th = th + 180;
    rho = -rho;
end

ok = true;

end

function [th, rho] = refineLine(bw, th0, rho0)
% Improve the Hough line estimate using the pixels of the detected track.
% Principal component analysis is used to find the main direction
% of the track pixels near the initially detected line.

[H, W] = size(bw);

% Calculate sine and cosine of the initial line angle
c0 = cosd(th0);
s0 = sind(th0);

% Initialize sums used to calculate the mean and covariance
n = 0;
sx = 0;
sy = 0;
sxx = 0;
syy = 0;
sxy = 0;

% Visit all pixels in the binary track image
for c = 1:W
    for r = 1:H

        x = c - 1;
        y = r - 1;

        % Select track pixels close to the Hough-detected line
        if bw(r, c) && abs(x*c0 + y*s0 - rho0) < 10

            n = n + 1;

            % Calculate sums of coordinates and their products
            sx = sx + x;
            sy = sy + y;
            sxx = sxx + x*x;
            syy = syy + y*y;
            sxy = sxy + x*y;

        end
    end
end

% Keep the original Hough result if there are too few pixels
th = th0;
rho = rho0;

if n < 20
    return;
end

% Calculate the mean coordinates of the selected pixels
mx = sx/n;
my = sy/n;

% Calculate covariance values
cxx = sxx/n - mx^2;
cyy = syy/n - my^2;
cxy = sxy/n - mx*my;

% Calculate the refined line angle
% The normal direction is 90 degrees from the principal direction
t = 0.5*atan2d(2*cxy, cxx - cyy) - 90;

% Adjust the angle to stay close to the original Hough angle
while t - th0 > 90
    t = t - 180;
end

while th0 - t > 90
    t = t + 180;
end

% Reject the refined result if it differs too much from the Hough result
if abs(t - th0) > 6
    return;
end

% Calculate the distance of the refined line from the origin
r = mx*cosd(t) + my*sind(t);

% Keep the angle within -90 to 90 degrees
if t >= 90
    t = t - 180;
    r = -r;
end

if t < -90
    t = t + 180;
    r = -r;
end

% Return the refined angle and distance
th = t;
rho = r;

end
