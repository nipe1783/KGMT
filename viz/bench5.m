close all; clc; clear;

% Load data from three runs
load('OKPAX_100mill_Quad_zigZag_long.mat', 'bestCostMatrix', 'timeGrid');
bestCostMatrix1 = bestCostMatrix;
timeGrid1 = timeGrid / 1000;  % Convert to seconds
bestCostMatrix1 = bestCostMatrix1 ./ 100;

load('OKPAX_3mill_Quad_zigZag_fast.mat', 'bestCostMatrix', 'timeGrid');
bestCostMatrix2 = bestCostMatrix;
timeGrid2 = timeGrid / 1000;
bestCostMatrix2 = bestCostMatrix2 ./ 100;

load('KPAX_3mill_Quad_zigZag_fast.mat', 'bestCostMatrix', 'timeGrid');
bestCostMatrix3 = bestCostMatrix;
timeGrid3 = timeGrid / 1000;
bestCostMatrix3 = bestCostMatrix3 ./ 100;

% Custom colors
color1 = [1, 0, 1];   % Blue for KPAX
color2 = [1, 0, 0];   % Red for KPAX*-Large-delta
color3 = [0, 0, 1]; % Green for SST (slightly darker for better visibility)

figure;
hold on;

% Draw boxplots
boxplot(bestCostMatrix1', 'positions', timeGrid1, 'colors', color1, 'symbol', '.', 'OutlierSize', 4);
boxplot(bestCostMatrix2', 'positions', timeGrid2, 'colors', color2, 'symbol', '.', 'OutlierSize', 4);
boxplot(bestCostMatrix3', 'positions', timeGrid3, 'colors', color3, 'symbol', '.', 'OutlierSize', 4);

% Fill boxes with the corresponding color
h1 = findobj(gca, 'tag', 'Box', 'Color', color1);
h2 = findobj(gca, 'tag', 'Box', 'Color', color2);
h3 = findobj(gca, 'tag', 'Box', 'Color', color3);

for j = 1:length(h1)
    patch(get(h1(j), 'XData'), get(h1(j), 'YData'), color1, 'FaceAlpha', .5);
end
for j = 1:length(h2)
    patch(get(h2(j), 'XData'), get(h2(j), 'YData'), color2, 'FaceAlpha', .5);
end
for j = 1:length(h3)
    patch(get(h3(j), 'XData'), get(h3(j), 'YData'), color3, 'FaceAlpha', .5);
end

% Dummy handles for legend
hLegend1 = plot(nan, nan, 's', 'MarkerFaceColor', color1, 'MarkerEdgeColor', color1);
hLegend2 = plot(nan, nan, 's', 'MarkerFaceColor', color2, 'MarkerEdgeColor', color2);
hLegend3 = plot(nan, nan, 's', 'MarkerFaceColor', color3, 'MarkerEdgeColor', color3);

legend([hLegend1, hLegend2, hLegend3], ...
    {'\textbf{KPAX*}-Small-$\delta$', '\textbf{KPAX*}-Large-$\delta$', '\textbf{KPAX}'}, ...
    'Interpreter', 'latex', 'Location', 'northeast');

xlabel('Time (s)');
ylabel('Solution Cost');
grid on;

% Set X-ticks using all unique time points
allTimeGrids = unique([timeGrid1, timeGrid2, timeGrid3]);
set(gca, 'XTick', allTimeGrids);
set(gca, 'XTickLabel', arrayfun(@(x) num2str(x, '%g'), allTimeGrids, 'UniformOutput', false));

ylim([4.5 7]);  % Set y-limits

% Set figure size and export
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 8.5]);
set(gcf, 'PaperPosition', [0 0 8.5 8.5]);
fileName = 'KPAX_KPAX*_KPAX*_Quad_zigZag.pdf';
print(gcf, fileName, '-dpdf');
