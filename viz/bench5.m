close all; clc; clear;

% Load data from three runs
load('KPAX_0.1mill_10_3_3_3_6DDI_house.mat', 'bestCostMatrix', 'timeGrid');
bestCostMatrix1 = bestCostMatrix;
timeGrid1 = timeGrid;

load('SST_6DDI_house_results.mat', 'bestCostMatrix', 'timeGrid');
bestCostMatrix2 = bestCostMatrix;
timeGrid2 = timeGrid;

load('OKPAX_0.1mill_10_3_3_3_6DDI_house.mat', 'bestCostMatrix', 'timeGrid');
bestCostMatrix3 = bestCostMatrix;
timeGrid3 = timeGrid;

figure;
hold on;
boxplot1 = boxplot(bestCostMatrix1', 'positions', timeGrid1/1000, 'colors', [0, 0, 1], 'symbol', '.', 'OutlierSize', 4);
boxplot2 = boxplot(bestCostMatrix2', 'positions', timeGrid2/1000, 'colors', [1, 0, 0], 'symbol', '.', 'OutlierSize', 4);
boxplot3 = boxplot(bestCostMatrix3', 'positions', timeGrid3/1000, 'colors', [0, 1, 0], 'symbol', '.', 'OutlierSize', 4); % Green for third run

% Fill boxes with specific colors
h1 = findobj(gca, 'tag', 'Box', 'Color', [0, 0, 1]);
h2 = findobj(gca, 'tag', 'Box', 'Color', [1, 0, 0]);
h3 = findobj(gca, 'tag', 'Box', 'Color', [0, 1, 0]); % Find the boxes for the third run

for j = 1:length(h1)
    patch(get(h1(j), 'XData'), get(h1(j), 'YData'), 'b', 'FaceAlpha', .5); % Blue for first run
end
for j = 1:length(h2)
    patch(get(h2(j), 'XData'), get(h2(j), 'YData'), 'r', 'FaceAlpha', .5); % Red for second run
end
for j = 1:length(h3)
    patch(get(h3(j), 'XData'), get(h3(j), 'YData'), 'g', 'FaceAlpha', .5); % Green for third run
end

hold off;

xlabel('Time (s)');
ylabel('Solution Cost');
legend([boxplot1(5), boxplot2(5), boxplot3(5)], {'KPAX', 'SST', 'KPAX*'}, 'Location', 'best');
grid on;

% Convert timeGrid values from milliseconds to seconds for x-ticks
allTimeGrids = [timeGrid1; timeGrid2; timeGrid3]/1000;  % Convert all to seconds
uniqueTimeGrids = unique(allTimeGrids);  % Find unique positions for ticks

set(gca, 'XTick', uniqueTimeGrids);
set(gca, 'XTickLabel', arrayfun(@(x) num2str(x, '%g'), uniqueTimeGrids, 'UniformOutput', false));

ylim([2.0 6]);  % Set y-limits from 2.0 to 6.0

% Set figure size and save the figure as before.
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPosition', [0 0 8.5 11]);
fileName = 'KPAX_SST_OKPAX_6DDI_house.pdf';
print(gcf, fileName, '-dpdf');