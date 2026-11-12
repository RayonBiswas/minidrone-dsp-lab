function [Rn, Gn, Bn] = addCameraNoise(R, G, B, sigma, spDensity, seed)
% MATLAB Function block placed BEFORE dspVisionBlock to inject noise on purpose.
% sigma: Gaussian noise std (0..0.3), spDensity: salt & pepper density per channel (0..0.25).
%#codegen
persistent k
if isempty(k), k = seed; end
k = k + 1; rng(k);
img = cat(3, double(R), double(G), double(B)) / 255;
img = img + sigma*randn(size(img));
mask = rand(size(img));
img(mask < spDensity/2) = 0;
img(mask > 1 - spDensity/2) = 1;
img = min(max(img, 0), 1);
Rn = uint8(255*img(:,:,1)); Gn = uint8(255*img(:,:,2)); Bn = uint8(255*img(:,:,3));
end
