function [Rn, Gn, Bn] = addCameraNoise(R, G, B, sigma, spDensity, seed)
% MATLAB Function block placed BEFORE dspVisionBlock to inject noise on purpose.
% sigma: Gaussian noise std (0..0.3), spDensity: salt & pepper density per channel (0..0.25).
% seed: seeds the generator once at the first call, so a run is repeatable.
%#codegen
persistent seeded
if isempty(seeded)
    rng(seed); seeded = true;
end
H = size(R, 1); W = size(R, 2);
img = zeros(H, W, 3);
img(:,:,1) = double(R)/255; img(:,:,2) = double(G)/255; img(:,:,3) = double(B)/255;
img = img + sigma*randn(H, W, 3);
mask = rand(H, W, 3);
img(mask < spDensity/2) = 0;
img(mask > 1 - spDensity/2) = 1;
img = min(max(img, 0), 1);
Rn = uint8(255*img(:,:,1)); Gn = uint8(255*img(:,:,2)); Bn = uint8(255*img(:,:,3));
end
