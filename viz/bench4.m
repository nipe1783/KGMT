close all; clc; clear;

% Load data from two runs
load('OKPAX_20mill_50_3_3_3_DubinsAirplane_trees.mat', 'bestCostMatrix', 'timeGrid');
bestCostMatrix1 = bestCostMatrix;
timeGrid1 = timeGrid;

load('KPAX_20mill_50_3_3_3_DubinsAirplane_trees.mat', 'bestCostMatrix', 'timeGrid');
bestCostMatrix2 = bestCostMatrix;
timeGrid2 = timeGrid;

% Overlay the box plots
figure;
hold on;
boxplot1 = boxplot(bestCostMatrix1', 'positions', timeGrid1, 'colors', [0, 0, 1], 'symbol', '.', 'OutlierSize', 4);
boxplot2 = boxplot(bestCostMatrix2', 'positions', timeGrid2, 'colors', [0.5, 0.5, 0], 'symbol', '.', 'OutlierSize', 4);
hold off;


xlabel('Time (ms)');
ylabel('Solution Cost');
% legend([boxplot1(1), boxplot2(1)], {'Run 1', 'Run 2'}, 'Location', 'best');
grid on;

set(gca, 'XTick', timeGrid);  % Set the x-axis ticks to correspond to your time grid
set(gca, 'XTickLabel', arrayfun(@num2str, timeGrid, 'UniformOutput', false));  % Convert numbers to strings for labels

% Determine the range of the cost data to set y-limits
allData = [bestCostMatrix1(:); bestCostMatrix2(:)];  % Combine all data points
allData = allData(~isnan(allData));  % Remove NaN values which represent missing data
if ~isempty(allData)  % Check if there is any data
    minY = min(allData);
    maxY = max(allData);
    ylim([minY maxY]);  % Set y-limits based on actual data range
else
    disp('No valid data to plot.');
end

% Set figure size
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);  % Fixed size in inches (e.g., Letter size)
set(gcf, 'PaperPosition', [0 0 8.5 11]);  % Position on the paper

% Save the figure
fileName = 'DubinsAirplane_trees.pdf';  % Define your filename here
print(gcf, fileName, '-dpdf');  % Save as PDF