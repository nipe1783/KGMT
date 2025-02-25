close all;
clc;
clear;

% Load time data
timeData = load('/home/nicolas/dev/research/KGMT/build/Data/IterationTime/IterationTime1/IterationTime.csv');

% Load path cost data
pathCostData = readmatrix('/home/nicolas/dev/research/KGMT/build/Data/pathCosts/pathCosts1/pathCosts.csv');
pathCostData = pathCostData(1:size(timeData),:);

% Extract indexing information and costs
indices = pathCostData(:, 3);   % Indices to access timeData
costs = pathCostData(:, 2);     % Costs as y-values

% Ensure indices are valid
validIndices = indices > 0 & indices <= length(timeData);
indices = indices(validIndices);
costs = costs(validIndices);

% Map indices to times
times = timeData(indices);

% Create the plot
figure;
scatter(times, costs, '.');  % 'o-' for line with circle markers
title('Path Costs over Time');
xlabel('Time (ms)');
ylabel('Path Cost');
grid on;
