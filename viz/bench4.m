close all; clc; clear;

% Load data from two runs
load('OKPAX_0.1mill_10_3_3_3_DubinsAirplane_narrowPassage.mat', 'bestCostMatrix', 'timeGrid');
bestCostMatrix1 = bestCostMatrix;
timeGrid1 = timeGrid;

load('KPAX_0.1mill_10_3_3_3_DubinsAirplane_narrowPassage.mat', 'bestCostMatrix', 'timeGrid');
bestCostMatrix2 = bestCostMatrix;
timeGrid2 = timeGrid;

% Overlay the box plots
figure;
hold on;
boxplot1 = boxplot(bestCostMatrix1', 'positions', timeGrid1, 'colors', [0, 0, 1], 'symbol', '.', 'OutlierSize', 4);
boxplot2 = boxplot(bestCostMatrix2', 'positions', timeGrid2, 'colors', [1, 0, 0], 'symbol', '.', 'OutlierSize', 4);

% Fill boxes with specific colors
h1 = findobj(gca, 'tag', 'Box', 'Color', [0, 0, 1]);
h2 = findobj(gca, 'tag', 'Box', 'Color', [1, 0, 0]);

for j = 1:length(h1)
    patch(get(h1(j), 'XData'), get(h1(j), 'YData'), 'b', 'FaceAlpha', .5); % Blue for KPAX
end
for j = 1:length(h2)
    patch(get(h2(j), 'XData'), get(h2(j), 'YData'), 'r', 'FaceAlpha', .5); % Yellow for OKPAX
end

hold off;

% Additional code for displaying statistics and formatting the plot continues...
% Display statistics, set axis labels, legends, and grid as before.

xlabel('Time (ms)');
ylabel('Solution Cost');
legend([boxplot1(5), boxplot2(5)], {'KPAX*', 'KPAX'}, 'Location', 'best');
grid on;
set(gca, 'XTick', timeGrid);
set(gca, 'XTickLabel', arrayfun(@num2str, timeGrid, 'UniformOutput', false));

ylim([1.4 2]);  % Set y-limits from 2.0 to 3.0

% Set figure size and save the figure as before.
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPosition', [0 0 8.5 11]);
fileName = 'KPAX_OKPAX_DubinsAirplane_narrowPassage_fast.pdf';
print(gcf, fileName, '-dpdf');
