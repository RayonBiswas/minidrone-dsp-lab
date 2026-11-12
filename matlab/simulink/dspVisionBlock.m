function [theta, d, lineFound, padFound, padX, padY] = dspVisionBlock(R, G, B, useFilters)
% MATLAB Function block for the Image Processing System of parrotMinidroneCompetition.
% Inputs: R, G, B camera planes (uint8, 120x160), useFilters (1 = our DSP chain, 0 = raw threshold).
% Outputs: line angle theta (deg), offset d (px), line/pad flags, pad centroid (px from centre).
%#codegen
img = cat(3, double(R), double(G), double(B)) / 255;
[H, W, ~] = size(img);
xc = (W-1)/2; yc = (H-1)/2;

if useFilters
    for k = 1:3
        img(:,:,k) = medfilt2(img(:,:,k), [3 3]);
    end
end
hsv = rgb2hsv(img);
h = hsv(:,:,1); s = hsv(:,:,2); v = hsv(:,:,3);

% track: red line
bw = (h < 0.06 | h > 0.94) & s > 0.45 & v > 0.15;
if useFilters
    se = strel('disk', 2);
    bw = imopen(imclose(bw, se), se);
end
ed = edge(bw, 'canny');
theta = 0; d = 0; lineFound = false;
if nnz(ed) >= 20
    [Hh, T, Rr] = hough(ed, 'Theta', -90:1:89);
    P = houghpeaks(Hh, 6, 'Threshold', 0.3*max(Hh(:)));
    if ~isempty(P)
        tt = T(P(:,2)); rr = Rr(P(:,1));
        keep = abs(tt - tt(1)) < 5;
        theta = mean(tt(keep));
        rho = (min(rr(keep)) + max(rr(keep)))/2;
        d = rho - (xc*cosd(theta) + yc*sind(theta));
        lineFound = true;
    end
end

% landing pad: blue blob
pad = abs(h - 0.62) < 0.09 & s > 0.45 & v > 0.15;
if useFilters
    pad = imopen(imclose(pad, strel('disk', 2)), strel('disk', 2));
end
[py, px] = find(pad);
padFound = numel(px) > 0.015*H*W;
padX = 0; padY = 0;
if padFound
    padX = mean(px) - 1 - xc; padY = mean(py) - 1 - yc;
end
end
