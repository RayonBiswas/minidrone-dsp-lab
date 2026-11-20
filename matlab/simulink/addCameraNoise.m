function [Rn, Gn, Bn] = addCameraNoise(R, G, B, sigma, spDensity, seed)

% This function adds artificial noise to an RGB camera image.
% It is placed BEFORE dspVisionBlock to test how well the vision system
% works when the camera image contains noise.
%
% sigma: Controls the amount of Gaussian noise (0 to 0.3).
% spDensity: Controls the amount of salt-and-pepper noise (0 to 0.25).
% seed: Used to generate repeatable random noise.
%
%#codegen

persistent seeded

% Initialize the random number generator only during the first call
if isempty(seeded)
    rng(seed);
    seeded = true;
end

% Get the height and width of the input image
H = size(R, 1);
W = size(R, 2);

% Create an empty RGB image
img = zeros(H, W, 3);

% Convert the RGB image values from uint8 (0 to 255) to double (0 to 1)
img(:,:,1) = double(R)/255;
img(:,:,2) = double(G)/255;
img(:,:,3) = double(B)/255;

% Add Gaussian noise to all three colour channels
img = img + sigma*randn(H, W, 3);

% Generate random values to decide where salt-and-pepper noise is added
mask = rand(H, W, 3);

% Set selected pixels to 0 (black pixels)
img(mask < spDensity/2) = 0;

% Set selected pixels to 1 (white pixels)
img(mask > 1 - spDensity/2) = 1;

% Keep all pixel values within the valid range of 0 to 1
img = min(max(img, 0), 1);

% Convert the noisy image back to uint8 (0 to 255)
% Separate the red, green and blue channels
Rn = uint8(255*img(:,:,1));
Gn = uint8(255*img(:,:,2));
Bn = uint8(255*img(:,:,3));

end
