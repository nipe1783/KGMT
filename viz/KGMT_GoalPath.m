close all
clc
clear all

try
    opengl hardware
    disp('Using hardware OpenGL.');
catch
    opengl software
    disp('Using software OpenGL.');
end

% Parameters
numFiles = 1;
radius = 0.05;
N = 8;
n = 4;
sampleSize = 10;
stateSize = 6;
controlSize = 3;
xGoal = [.7, .95, .9];
alphaValue = 0.8;
STEP_SIZE = .1;

% Obstacle file path
obstacleFilePath = '/home/nicolas/dev/research/KGMT/include/config/obstacles/pillars/obstacles.csv';
obstacles = readmatrix(obstacleFilePath);

controlPath = '/home/nicolas/dev/research/KGMT/build/Data/ControlPathToGoal/ControlPathToGoal0/controlPathToGoal.csv';
controls = readmatrix(controlPath);
controls = flipud(controls);

for i = 1:numFiles

    % Create figure and set to full screen
    fig = figure('Position', [100, 100, 1000, 1000]); % Set figure size
    hold on;
    axis equal;
    title(sprintf('Iteration %d', i));
    xlabel('X Position');
    ylabel('Y Position');
    zlabel('Z Position');
    plot3(controls(1,1), controls(1,2), controls(1,3), 'ko', 'MarkerFaceColor', 'k');

    % Plot goal region
    [X, Y, Z] = sphere(20);
    surf(radius * X + xGoal(1), radius * Y + xGoal(2), radius * Z + xGoal(3), ...
         'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

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
        patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'r', 'EdgeColor', 'k', 'FaceAlpha', 0.6);
    end

    % Add light source
    camlight('headlight'); 
    lighting gouraud;

    % Plot paths
    for j = 2:size(controls, 1)
        x0 = controls(j-1, 1:stateSize);
        segmentX = [x0(1)];
        segmentY = [x0(2)];
        segmentZ = [x0(3)];
        u = controls(j, stateSize+1:sampleSize-1);
        duration = controls(j, sampleSize);
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
        segmentX = [segmentX, controls(j, 1)];
        segmentY = [segmentY, controls(j, 2)];
        segmentZ = [segmentZ, controls(j, 3)];

        plot3(segmentX, segmentY, segmentZ, '-.', 'Color', 'k', 'LineWidth', 0.01);
        plot3(controls(j, 1), controls(j, 2), controls(j, 3), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 2);
    end

    % Save original view with high resolution
    % view(3);
    % drawnow;
    % saveas(gcf, sprintf('figs/KGMT_Iteration_%d.jpg', i));
    % print(sprintf('figs/KGMT_Iteration_%d.jpg', i), '-djpeg', '-r300'); % Save with 300 DPI

    % Save top-down view with high resolution
    view(2);
    drawnow;
    saveas(gcf, sprintf('figs/top_KGMT_Iteration_%d.jpg', i));
    print(sprintf('figs/top_KGMT_Iteration_%d.jpg', i), '-djpeg', '-r300'); % Save with 300 DPI

    % Save view looking along the x-axis from above
    % view([-1, .2, 0.6]); % Look along the x-axis
    % drawnow;
    % saveas(gcf, sprintf('figs/xAxis_KGMT_Iteration_%d.jpg', i));
    % print(sprintf('figs/xAxis_KGMT_Iteration_%d.jpg', i), '-djpeg', '-r300'); % Save with 300 DPI
    % 
    % % Close the figure
    % close(gcf);

end