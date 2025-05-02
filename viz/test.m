close all;
clc;
clear all;

% Parameterss
% radius = 0.05;
radius = .1;
alpha = 0.9;
xGoal = [.10, .10, 0.1];
% xGoal = [80, 95, 90];
STEP_SIZE = 0.1;
% File paths
obstacleFilePath = '/home/nicolas/dev/research/KGMT/include/config/obstacles/empty/obstacles.csv';
controlPath = '/home/nicolas/dev/research/KGMT/build/Data/ControlPathsToGoal/ControlPathsToGoal0/controlPathsToGoal.csv';
% 
stateSize = 6;
sampleSize = 10;
% stateSize = 12;
% sampleSize = 17;
model = 1; % Choose model (1: Double Integrator, 2: Dubins Airplane, 3: Quadcopter)

% Read and flip control data
controls = flipud(readmatrix(controlPath));

% Load obstacles
obstacles = readmatrix(obstacleFilePath);

% Color palette
colors = [0 .1 0.8];  % Orange

fig = figure('Position', [100, 100, 1000, 1000]);
hold on;
axis equal;
axis off;
% title('Trajectory Visualization');

% Plot obstacles
for j = 1:size(obstacles, 1)
    x_min = obstacles(j, 1);
    y_min = obstacles(j, 2);
    z_min = obstacles(j, 3);
    x_max = obstacles(j, 4);
    y_max = obstacles(j, 5);
    z_max = obstacles(j, 6);
    vertices = [
        x_min, y_min, z_min;
        x_max, y_min, z_min;
        x_max, y_max, z_min;
        x_min, y_max, z_min;
        x_min, y_min, z_max;
        x_max, y_min, z_max;
        x_max, y_max, z_max;
        x_min, y_max, z_max];
    faces = [
        1, 2, 6, 5;
        2, 3, 7, 6;
        3, 4, 8, 7;
        4, 1, 5, 8;
        1, 2, 3, 4;
        5, 6, 7, 8];
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'r', 'EdgeColor', 'k', 'FaceAlpha', alpha);
end

cubeSize = 1;
cubeOrigin = [0.5, 0.5, 0.5];
vertices = cubeSize * [-0.5, -0.5, -0.5; 0.5, -0.5, -0.5; 0.5, 0.5, -0.5; -0.5, 0.5, -0.5; -0.5, -0.5, 0.5; 0.5, -0.5, 0.5; 0.5, 0.5, 0.5; -0.5, 0.5, 0.5] + cubeOrigin;
faces = [1, 2, 6, 5; 2, 3, 7, 6; 3, 4, 8, 7; 4, 1, 5, 8; 1, 2, 3, 4; 5, 6, 7, 8];
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'none', 'EdgeColor', 'k', 'LineWidth', 0.5);

% Plot goal
[X, Y, Z] = sphere(20);
surf(radius * X + xGoal(1), radius * Y + xGoal(2), radius * Z + xGoal(3), ...
     'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

% Separate trajectories based on rows of zeros
zeroRows = all(controls == 0, 2);
trajectoryIndices = find(zeroRows);
startIndices = [1; trajectoryIndices + 1];
endIndices = [trajectoryIndices - 1; size(controls, 1)];

% Plot each trajectory using propagation functions
for i = 1:length(startIndices)
    if startIndices(i) > endIndices(i)
        continue;
    end

    trajectory = controls(startIndices(i):endIndices(i), :);
    x0 = trajectory(1, :); % Initial state
    color = colors(mod(i - 1, size(colors, 1)) + 1, :); % Assign color

    for j = 2:size(trajectory, 1)
        sample = trajectory(j, :);

        % Propagate segment based on the model
        if model == 1
            [segmentX, segmentY, segmentZ] = propDoubleIntegrator(x0, sample, STEP_SIZE, stateSize, sampleSize);
        elseif model == 2
            [segmentX, segmentY, segmentZ] = propDubinsAirplane(x0, sample, STEP_SIZE, stateSize, sampleSize);
        elseif model == 3
            [segmentX, segmentY, segmentZ] = propQuad(x0, sample, STEP_SIZE, stateSize, sampleSize);
        end

        % Plot the propagated segment
        plot3(segmentX, segmentY, segmentZ, 'LineWidth', 1.5, 'Color', color);
        % plot3(gather(controls(j, 1)), gather(controls(j, 2)), gather(controls(j, 3)), 'o', 'Color', 'k', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
        % radius = 0.01;
        % for k = 1:length(segmentX)
        %     surf(radius * X + segmentX(k), radius * Y + segmentY(k), radius * Z + segmentZ(k), ...
        %          'FaceColor', 'b', 'EdgeColor', 'none');
        % end


        % Update the initial state for the next segment
        x0 = sample;
    end
end

% Lighting and initial view
camlight('headlight');
camlight('right');
lighting phong;

% Define views
views = {...
    % [0, 90], 'top'; ...     % Top view
    % [90, 0], 'side'; ...    % Side view
    [20, 30], 'isometric'; ... % Isometric view
    % [180, 0], 'reverse'; ...   % Reverse side view
};

% Save different views
for v = 1:size(views, 1)
    view(views{v, 1});
    drawnow;
    saveas(fig, sprintf('figs/trajectory_visualization_%s.jpg', views{v, 2}));
    print(sprintf('figs/trajectory_visualization_%s.jpg', views{v, 2}), '-djpeg', '-r300');
end
close(fig);


function [segmentX, segmentY, segmentZ] = propDoubleIntegrator(x0, sample, STEP_SIZE, stateSize, sampleSize)
    segmentX = gpuArray(x0(1));
    segmentY = gpuArray(x0(2));
    segmentZ = gpuArray(x0(3));
    u = gpuArray(sample(stateSize+1:sampleSize-1));
    duration = gpuArray(sample(sampleSize));
    numDisc = gpuArray(duration / STEP_SIZE);
    x = gpuArray(x0(1));
    y = gpuArray(x0(2));
    z = gpuArray(x0(3));
    vx = gpuArray(x0(4));
    vy = gpuArray(x0(5));
    vz = gpuArray(x0(6));
    ax = u(1);
    ay = u(2);
    az = u(3);
    for k = 1:numDisc
        x = x + (vx + (vx + 2 * (vx + ax * STEP_SIZE / 2) + (vx + ax * STEP_SIZE))) * STEP_SIZE / 6;
        y = y + (vy + (vy + 2 * (vy + ay * STEP_SIZE / 2) + (vy + ay * STEP_SIZE))) * STEP_SIZE / 6;
        z = z + (vz + (vz + 2 * (vz + az * STEP_SIZE / 2) + (vz + az * STEP_SIZE))) * STEP_SIZE / 6;
        vx = vx + (ax + 2 * ax + 2 * ax + ax) * STEP_SIZE / 6;
        vy = vy + (ay + 2 * ay + 2 * ay + ay) * STEP_SIZE / 6;
        vz = vz + (az + 2 * az + 2 * az + az) * STEP_SIZE / 6;
        segmentX = [segmentX, x];
        segmentY = [segmentY, y];
        segmentZ = [segmentZ, z];
    end
    segmentX = [segmentX, gpuArray(sample(1))];
    segmentY = [segmentY, gpuArray(sample(2))];
    segmentZ = [segmentZ, gpuArray(sample(3))];
end

function [segmentX, segmentY, segmentZ] = propDubinsAirplane(x0, sample, STEP_SIZE, stateSize, sampleSize)
    segmentX = gpuArray(x0(1));
    segmentY = gpuArray(x0(2));
    segmentZ = gpuArray(x0(3));
    u = gpuArray(sample(stateSize+1:sampleSize-1));
    duration = gpuArray(sample(sampleSize));
    numDisc = gpuArray(duration / STEP_SIZE);
    x = gpuArray(x0(1));
    y = gpuArray(x0(2));
    z = gpuArray(x0(3));
    yaw = gpuArray(x0(4));
    pitch = gpuArray(x0(5));
    v = gpuArray(x0(6));
    yawRate = u(1);
    pitchRate = u(2);
    a = u(3);
    for k = 1:numDisc
        x = x + (STEP_SIZE / 6.0) * ...
            (v * cos(pitch) * cos(yaw) + ...
             2.0 * ((v + 0.5 * STEP_SIZE * a) * cos(pitch + 0.5 * STEP_SIZE * pitchRate) * cos(yaw + 0.5 * STEP_SIZE * yawRate) + ...
                    (v + 0.5 * STEP_SIZE * a) * cos(pitch + 0.5 * STEP_SIZE * pitchRate) * cos(yaw + 0.5 * STEP_SIZE * yawRate)) + ...
             (v + STEP_SIZE * a) * cos(pitch + STEP_SIZE * pitchRate) * cos(yaw + STEP_SIZE * yawRate));
        
        y = y + (STEP_SIZE / 6.0) * ...
            (v * cos(pitch) * sin(yaw) + ...
             2.0 * ((v + 0.5 * STEP_SIZE * a) * cos(pitch + 0.5 * STEP_SIZE * pitchRate) * sin(yaw + 0.5 * STEP_SIZE * yawRate) + ...
                    (v + 0.5 * STEP_SIZE * a) * cos(pitch + 0.5 * STEP_SIZE * pitchRate) * sin(yaw + 0.5 * STEP_SIZE * yawRate)) + ...
             (v + STEP_SIZE * a) * cos(pitch + STEP_SIZE * pitchRate) * sin(yaw + STEP_SIZE * yawRate));
        
        z = z + (STEP_SIZE / 6.0) * ...
            (v * sin(pitch) + ...
             2.0 * ((v + 0.5 * STEP_SIZE * a) * sin(pitch + 0.5 * STEP_SIZE * pitchRate) + ...
                    (v + 0.5 * STEP_SIZE * a) * sin(pitch + 0.5 * STEP_SIZE * pitchRate)) + ...
             (v + STEP_SIZE * a) * sin(pitch + STEP_SIZE * pitchRate));
        
        yaw = yaw + STEP_SIZE * yawRate;
        pitch = pitch + STEP_SIZE * pitchRate;
        v = v + (STEP_SIZE / 6.0) * (a + 2.0 * (a + a) + a);
        segmentX = [segmentX, x];
        segmentY = [segmentY, y];
        segmentZ = [segmentZ, z];
    end
    segmentX = [segmentX, gpuArray(sample(1))];
    segmentY = [segmentY, gpuArray(sample(2))];
    segmentZ = [segmentZ, gpuArray(sample(3))];
end

function [segmentX, segmentY, segmentZ] = propQuad(x0, sample, STEP_SIZE, stateSize, sampleSize)
    segmentX = gpuArray(x0(1));
    segmentY = gpuArray(x0(2));
    segmentZ = gpuArray(x0(3));
    u = gpuArray(sample(stateSize+1:sampleSize-1));
    duration = gpuArray(sample(sampleSize));
    numDisc = gpuArray(duration / STEP_SIZE);
    Zc = u(1);
    Lc = u(2);
    Mc = u(3);
    Nc = u(4);
    h0 = x0(1:12);
    x = x0(1);
    y = x0(2);
    z = x0(3);

    for k = 1:numDisc
        h1 = ode(h0, Zc, Lc, Mc, Nc);
        h2 = ode(h0 + 0.5 * STEP_SIZE * h1, Zc, Lc, Mc, Nc);
        h3 = ode(h0 + 0.5 * STEP_SIZE * h2, Zc, Lc, Mc, Nc);
        h4 = ode(h0 + STEP_SIZE * h3, Zc, Lc, Mc, Nc);
        
        h0 = h0 + (STEP_SIZE / 6) * (h1 + 2 * h2 + 2 * h3 + h4);
        
        x = h0(1);
        y = h0(2);
        z = h0(3);
        
        segmentX = [segmentX, x];
        segmentY = [segmentY, y];
        segmentZ = [segmentZ, z];
    end
    
    segmentX = [segmentX, gpuArray(sample(1))];
    segmentY = [segmentY, gpuArray(sample(2))];
    segmentZ = [segmentZ, gpuArray(sample(3))];
end

function x0dot = ode(x0, Zc, Lc, Mc, Nc)
        
    NU = 10e-3;
    MU = 2e-6;
    IX = 1.0;
    IY = 1.0;
    IZ = 2.0;
    GRAVITY = -9.81;
    MASS = 1.0;
    MASS_INV = 1.0 / MASS;

    
    phi   = x0(4);
    theta = x0(5);
    psi   = x0(6);
    u     = x0(7);
    v     = x0(8);
    w     = x0(9);
    p     = x0(10);
    q     = x0(11);
    r     = x0(12);

    x0dot = zeros(1, 12);

    x0dot(1) = cos(theta) * cos(psi) * u + (sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi)) * v + ...
               (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) * w;

    x0dot(2) = cos(theta) * sin(psi) * u + (sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi)) * v + ...
               (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) * w;

    x0dot(3) = -sin(theta) * u + sin(phi) * cos(theta) * v + cos(phi) * cos(theta) * w;

    x0dot(4) = p + (q * sin(phi) + r * cos(phi)) * tan(theta);

    x0dot(5) = q * cos(phi) - r * sin(phi);

    x0dot(6) = (q * sin(phi) + r * cos(phi)) / cos(theta);

    XYZ = -NU * sqrt(u^2 + v^2 + w^2);
    X   = XYZ * u;
    x0dot(7)  = (r * v - q * w) - GRAVITY * sin(theta) + MASS_INV * X;

    Y  = XYZ * v;
    x0dot(8) = (p * w - r * u) + GRAVITY * cos(theta) * sin(phi) + MASS_INV * Y;

    Z  = XYZ * w;
    x0dot(9) = (q * u - p * v) + GRAVITY * cos(theta) * cos(phi) + MASS_INV * Z + MASS_INV * Zc;

    LMN = -MU * sqrt(p^2 + q^2 + r^2);
    L   = LMN * p;
    x0dot(10) = (IY - IZ) / IX * q * r + (1 / IX) * L + (1 / IX) * Lc;

    M   = LMN * q;
    x0dot(11) = (IZ - IX) / IY * p * r + (1 / IY) * M + (1 / IY) * Mc;

    N   = LMN * r;
    x0dot(12) = (IX - IY) / IZ * p * q + (1 / IZ) * N + (1 / IZ) * Nc;
end