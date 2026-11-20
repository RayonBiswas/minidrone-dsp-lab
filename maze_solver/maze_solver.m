%% Maze solver: shortest path from a photo of a maze
% Photo -> walls -> maze outline -> entrance/exit (gaps in the outer wall)
% -> Dijkstra shortest path through the corridor middles -> drawn in red.
% Runs on plain MATLAB (no toolboxes). Test images are in test_images/.

clear; clc; close all;

files = {'maze_generated.png', 'maze_generated_photo.jpg', ...
         'maze_no_path.png',   'maze_red_path.png'};
folder = 'test_images';                      % works from the repo root ...
if ~isfolder(folder), folder = fullfile('..', 'test_images'); end   % ... or from a subfolder

figure('Color', 'w', 'Position', [60 60 1100 1000]);
fprintf('%-26s %8s %12s %12s %10s %8s\n', 'Maze', 'Size', 'Start', 'End', 'Path [px]', 'Time [s]');

for i = 1:numel(files)
    img = readImage(fullfile(folder, files{i}));
    tic;
    [path, startXY, endXY] = solveMaze(img);
    t = toc;

    subplot(2, 2, i);
    imshowPlain(img); hold on;
    if isempty(path)
        title([files{i} ': no path found'], 'Interpreter', 'none');
        fprintf('%-26s %8s %12s %12s %10s %8.2f\n', files{i}, sizeStr(img), '-', '-', '-', t);
        continue;
    end
    plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2.5);
    plot(startXY(1), startXY(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0 0.7 0], 'MarkerEdgeColor', 'k');
    plot(endXY(1),   endXY(2),   's', 'MarkerSize', 10, 'MarkerFaceColor', [0 0.3 1], 'MarkerEdgeColor', 'k');
    text(startXY(1), startXY(2), '  Start', 'Color', [0 0.5 0], 'FontWeight', 'bold');
    text(endXY(1),   endXY(2),   '  End',   'Color', [0 0.2 0.8], 'FontWeight', 'bold');
    title(files{i}, 'Interpreter', 'none');

    len = sum(vecnorm(diff(path), 2, 2));
    fprintf('%-26s %8s %12s %12s %10.0f %8.2f\n', files{i}, sizeStr(img), ...
        sprintf('(%d, %d)', startXY), sprintf('(%d, %d)', endXY), len, t);
end

%% Check against the solution drawn in maze_red_path.png
% maze_no_path.png and maze_red_path.png are the same maze; the second one
% has the correct route drawn in red. Compare our path with it.
noPath = readImage(fullfile(folder, 'maze_no_path.png'));
solved = readImage(fullfile(folder, 'maze_red_path.png'));
myPath = solveMaze(noPath);

red = solved(:,:,1) > 0.6 & solved(:,:,2) < 0.45 & solved(:,:,3) < 0.45;
tol = 6;                                           % allowed distance [px]
nearRed = conv2(double(red), ones(2*tol+1), 'same') > 0;
idx = sub2ind(size(red), round(myPath(:,2)), round(myPath(:,1)));
match = 100 * mean(nearRed(idx));
fprintf('\nmaze_no_path.png: %.1f %% of the path lies within %d px of the drawn solution\n', match, tol);


%% ---------------- local functions ----------------
function [path, startXY, endXY, nGaps] = solveMaze(img)
    path = [];  startXY = [];  endXY = [];
    [h, w, ~] = size(img);

    % 1. Walls: dark in every colour channel (a red line drawn on the maze is not a wall)
    v = max(img, [], 3);
    wall = v < otsu(v);

    % 2. Thicken walls a little: closes small breaks and keeps the path off the walls
    r = max(2, round(min(h, w)/150));
    wall = dilate(wall, r);

    % 3. Drop dark regions touching the image edge (background, black bars)
    [lab, n] = components(wall);
    edgeLabels = unique([lab(1,:) lab(end,:) lab(:,1)' lab(:,end)']);
    edgeLabels(edgeLabels == 0) = [];
    wall(ismember(lab, edgeLabels)) = false;

    % 4. Maze outline = convex hull of all wall pieces (tiny specks ignored)
    sizes = accumarray(lab(lab > 0), 1, [n 1]);
    sizes(edgeLabels) = 0;
    keep = find(sizes >= 0.02 * max(sizes));
    [yy, xx] = find(ismember(lab, keep));
    k = convhull(xx, yy, 'Simplify', true);
    [X, Y] = meshgrid(1:w, 1:h);
    inside = inpolygon(X, Y, xx(k), yy(k));

    % 5. Entrance and exit: free pixels on the outline (gaps in the outer wall)
    free = inside & ~wall;
    rim  = inside & ~erode(inside, r + 1);
    [gapLab, nGaps] = components(free & rim);
    if nGaps < 2, return; end
    gapSize = accumarray(gapLab(gapLab > 0), 1, [nGaps 1]);
    [~, order] = sort(gapSize, 'descend');
    [sy, sx] = gapPoint(gapLab == order(1));
    [ey, ex] = gapPoint(gapLab == order(2));

    % Start = the gap nearer the top-left corner
    if sx + sy > ex + ey
        [sx, sy, ex, ey] = deal(ex, ey, sx, sy);
    end

    % 6. Cost: cheap in the middle of a corridor, expensive next to a wall
    d = distToWall(free);
    cost = 1 + 30 ./ d.^2;

    % 7. Dijkstra on the pixel grid (8 neighbours)
    G = pixelGraph(free, cost);
    node = zeros(h, w);  node(free) = 1:nnz(free);
    p = shortestpath(G, node(sy, sx), node(ey, ex));
    if isempty(p), return; end
    [py, px] = ind2sub([h w], find(free));
    path = smoothPath([px(p) py(p)], 5);
    startXY = [sx sy];  endXY = [ex ey];
end

function img = readImage(file)
    [img, map] = imread(file);
    if ~isempty(map), img = ind2rgb(img, map); end
    img = double(img);
    if max(img(:)) > 1, img = img / 255; end
    if size(img, 3) == 1, img = repmat(img, 1, 1, 3); end
    img = img(:, :, 1:3);
end

function t = otsu(v)
    % Otsu threshold: splits pixels into dark and bright with the best separation
    counts = histcounts(v(:), linspace(0, 1, 257));
    p = counts / sum(counts);
    levels = (0.5:255.5) / 256;
    w0 = cumsum(p);  mu = cumsum(p .* levels);  muT = mu(end);
    between = (muT*w0 - mu).^2 ./ (w0 .* (1 - w0) + eps);
    [~, k] = max(between);
    t = levels(k);
end

function out = dilate(mask, r)
    [x, y] = meshgrid(-r:r);
    out = conv2(double(mask), double(x.^2 + y.^2 <= r^2), 'same') > 0.5;
end

function out = erode(mask, r)
    out = ~dilate(~mask, r);
end

function [lab, n] = components(mask)
    % Label 8-connected regions of a binary mask
    [h, w] = size(mask);
    idx = find(mask);
    node = zeros(h, w);  node(idx) = 1:numel(idx);
    s = [];  t = [];
    for off = [0 1; 1 0; 1 1; 1 -1]'
        [a, b] = neighbourPairs(mask, node, off);
        s = [s; a]; t = [t; b]; %#ok<AGROW>
    end
    G = graph(s, t, [], numel(idx));
    c = conncomp(G);
    lab = zeros(h, w);  lab(idx) = c;
    n = max([c 0]);
end

function [a, b] = neighbourPairs(mask, node, off)
    % Node pairs (a, b) where pixel b = pixel a shifted by off = [dy dx], both in mask
    [h, w] = size(mask);
    dy = off(1);  dx = off(2);
    rows = max(1, 1-dy):min(h, h-dy);
    cols = max(1, 1-dx):min(w, w-dx);
    m1 = mask(rows, cols);  m2 = mask(rows+dy, cols+dx);
    both = m1 & m2;
    n1 = node(rows, cols);  n2 = node(rows+dy, cols+dx);
    a = n1(both);  b = n2(both);
end

function G = pixelGraph(free, cost)
    % Graph of free pixels; edge weight = step length x average cost
    node = zeros(size(free));  node(free) = 1:nnz(free);
    c = cost(free);
    s = [];  t = [];  wt = [];
    for off = [0 1; 1 0; 1 1; 1 -1]'
        [a, b] = neighbourPairs(free, node, off);
        s = [s; a]; t = [t; b]; %#ok<AGROW>
        wt = [wt; norm(off) * (c(a) + c(b)) / 2]; %#ok<AGROW>
    end
    G = graph(s, t, wt, nnz(free));
end

function d = distToWall(free)
    % Distance (in pixels) from each free pixel to the nearest wall, by repeated erosion
    d = zeros(size(free));
    cur = free;  k = 0;
    while any(cur(:)) && k < 200
        k = k + 1;
        d(cur) = k;
        cur = cur & ~(conv2(double(~cur), ones(3), 'same') > 0);
    end
    d(~free) = 1;
end

function [y, x] = gapPoint(mask)
    % Pixel of a gap closest to its centre
    [yy, xx] = find(mask);
    [~, k] = min((yy - mean(yy)).^2 + (xx - mean(xx)).^2);
    y = yy(k);  x = xx(k);
end

function p = smoothPath(p, n)
    % Moving average to remove the pixel staircase (ends kept fixed)
    if size(p, 1) <= 2*n, return; end
    k = ones(2*n+1, 1) / (2*n+1);
    q = [repmat(p(1,:), n, 1); p; repmat(p(end,:), n, 1)];
    p = [conv(q(:,1), k, 'valid') conv(q(:,2), k, 'valid')];
end

function imshowPlain(img)
    image(img);  axis image off;
end

function s = sizeStr(img)
    s = sprintf('%dx%d', size(img, 2), size(img, 1));
end
