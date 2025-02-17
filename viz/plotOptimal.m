close all
clc
clear

% Constants
STEP_SIZE = .1;
model = 1;
sampleSize = 10;
stateSize = 6;
controlSize = 3;
alpha = 1.0;
xGoal = [.75, .95, .9];
radius = 0.05;
numR1 = 5;
numR2 = 1;
regionSize = 1 / numR1;
subregionSize = regionSize / numR2;

% Load data
folderPath = '/home/nicolas/dev/research/KGMT/build/Data/Samples';
files = dir(folderPath);
dirFlags = [files.isdir];
subFolders = files(dirFlags);
subFolders = subFolders(~ismember({subFolders.name}, {'.', '..'}));
numIterations = numel(subFolders);
numIterations = 100;
obstaclesPath = '/home/nicolas/dev/research/KGMT/include/config/obstacles/zigZag/obstacles.csv';
obstacles = readmatrix(obstaclesPath);

% Plot settings
camlight('headlight'); 
camlight('right');
lighting phong;


for i = 1:numIterations

    % Load data
    samplesPath = "/home/nicolas/dev/research/KGMT/build/Data/Samples/Samples" + i + "/samples" + i + ".csv";
    parentsPath = "/home/nicolas/dev/research/KGMT/build/Data/Parents/Parents" + i + "/parents" + i + ".csv";
    frontierPath = "/home/nicolas/dev/research/KGMT/build/Data/Frontier/Frontier" + i + "/frontier.csv";
    samples = readmatrix(samplesPath);
    parents = readmatrix(parentsPath);
    frontier = readmatrix(frontierPath);

    % Begin Plot
    fig = figure('Position', [100, 100, 1000, 1000]); 
    hold on;
    axis equal;
    camlight('headlight'); 
    camlight('right');
    lighting phong;
    title(sprintf('Iteration %d', i));

    % Plot Starting Point
    plot3(samples(1,1), samples(1,2), samples(1,3), 'ko', 'MarkerFaceColor', 'b', 'MarkerSize', 10);

    % Plot Goal Point
    [X, Y, Z] = sphere(20);
    surf(radius * X + xGoal(1), radius * Y + xGoal(2), radius * Z + xGoal(3), ...
         'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

    % Plot Obstacles
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

    % Plot Regions:
    for rx = 0:numR1-1
        for ry = 0:numR1-1
            for rz = 0:numR1-1
                % Coordinates of the region
                x_min = rx * regionSize;
                y_min = ry * regionSize;
                z_min = rz * regionSize;
                x_max = x_min + regionSize;
                y_max = y_min + regionSize;
                z_max = z_min + regionSize;
    
                % Loop through each subregion within the current region
                for sx = 0:numR2-1
                    for sy = 0:numR2-1
                        for sz = 0:numR2-1
                            % Coordinates of the subregion
                            x_min_s = x_min + sx * subregionSize;
                            y_min_s = y_min + sy * subregionSize;
                            z_min_s = z_min + sz * subregionSize;
                            x_max_s = x_min_s + subregionSize;
                            y_max_s = y_min_s + subregionSize;
                            z_max_s = z_min_s + subregionSize;
    
                            % Vertices of the subregion cube
                            vertices = [
                                x_min_s, y_min_s, z_min_s;
                                x_max_s, y_min_s, z_min_s;
                                x_max_s, y_max_s, z_min_s;
                                x_min_s, y_max_s, z_min_s;
                                x_min_s, y_min_s, z_max_s;
                                x_max_s, y_min_s, z_max_s;
                                x_max_s, y_max_s, z_max_s;
                                x_min_s, y_max_s, z_max_s
                            ];
    
                            % Faces index
                            faces = [
                                1, 2, 6, 5;
                                2, 3, 7, 6;
                                3, 4, 8, 7;
                                4, 1, 5, 8;
                                1, 2, 3, 4;
                                5, 6, 7, 8
                            ];
    
                            % Drawing the subregion wireframe
                            patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'none', 'EdgeColor', 'k');
                        end
                    end
                end
            end
        end
    end

    
    for j = 1:size(frontier,2)
        if frontier(1,j) == 1
            
            % Get Branch:
            x = [samples(j,:)];
            x0 = parents(j, 1);
            while x0 ~= -1
                x0 = x0 + 1;
                x = [samples(x0, :); x];
                x0 = parents(x0, 1);
            end

            % Plot Branch:
            segmentX = [];
            segmentY = [];
            segmentZ = [];
            for k = 1:size(x,1)-1
                x0 = x(k,:);
                sample = x(k+1,:);
                if model == 1
                    [segmentX, segmentY, segmentZ] = propDoubleIntegrator(x0, sample, STEP_SIZE, stateSize, sampleSize);
                elseif model == 2
                    [segmentX, segmentY, segmentZ] = propDubinsAirplane(x0, sample, STEP_SIZE, stateSize, sampleSize);
                end
            end
            plot3(segmentX, segmentY, segmentZ, '-.', 'Color', 'k', 'LineWidth', 0.01);
            plot3(samples(j, 1), samples(j, 2), samples(j, 3), 'o', 'Color', 'blue', 'MarkerFaceColor', 'blue', 'MarkerSize', 2);
        end
    end

    drawnow;
    saveas(gcf, sprintf('figs/top_KGMT_Iteration_%d.jpg', i));
    print(sprintf('figs/top_KGMT_Iteration_%d.jpg', i), '-djpeg', '-r300');
    close(gcf);
end

function [segmentX, segmentY, segmentZ] = propDoubleIntegrator(x0, sample, STEP_SIZE, stateSize, sampleSize)
    segmentX = [x0(1)];
    segmentY = [x0(2)];
    segmentZ = [x0(3)];
    u = sample(stateSize+1:sampleSize-1);
    duration = sample(sampleSize);
    numDisc = duration/STEP_SIZE;
    x = x0(1);
    y = x0(2);
    z = x0(3);
    vx = x0(4);
    vy = x0(5);
    vz = x0(6);
    ax = u(1);
    ay = u(2);
    az = u(3);
    for k = 1:(numDisc)
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
    segmentX = [segmentX, sample(1)];
    segmentY = [segmentY, sample(2)];
    segmentZ = [segmentZ, sample(3)];
end

function [segmentX, segmentY, segmentZ] = propDubinsAirplane(x0, sample, STEP_SIZE, stateSize, sampleSize)
    segmentX = [x0(1)];
    segmentY = [x0(2)];
    segmentZ = [x0(3)];
    u = sample(stateSize+1:sampleSize-1);
    duration = sample(sampleSize);
    numDisc = duration/STEP_SIZE;
    x = x0(1);
    y = x0(2);
    z = x0(3);
    yaw = x0(4);
    pitch = x0(5);
    v = x0(6);
    yawRate = u(1);
    pitchRate = u(2);
    a = u(3);
    for k = 1:(numDisc)
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
    segmentX = [segmentX, sample(1)];
    segmentY = [segmentY, sample(2)];
    segmentZ = [segmentZ, sample(3)];
end