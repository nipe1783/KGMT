close all; clc; clear;

% Load data from two runs
load('KPAX_3mill_Quad_zigZag_fast.mat', 'bestCostMatrix', 'timeGrid');
bestCostMatrix1 = bestCostMatrix;
timeGrid1 = timeGrid;
bestCostMatrix1 = bestCostMatrix1 ./ 100;

load('OKPAX_3mill_Quad_zigZag_fast.mat', 'bestCostMatrix', 'timeGrid');
bestCostMatrix2 = bestCostMatrix;
timeGrid2 = timeGrid;
bestCostMatrix2 = bestCostMatrix2 ./ 100;

% Custom colors
color1 = [0, 0, 1];  % Magenta
color2 = [1, 0, 0];  % Red

% Overlay the box plots
figure;
hold on;
boxplot(bestCostMatrix1', 'positions', timeGrid1, 'colors', color1, 'symbol', '.', 'OutlierSize', 4);
boxplot(bestCostMatrix2', 'positions', timeGrid2, 'colors', color2, 'symbol', '.', 'OutlierSize', 4);

% Fill boxes with corresponding colors
h1 = findobj(gca, 'tag', 'Box', 'Color', color1);
h2 = findobj(gca, 'tag', 'Box', 'Color', color2);

for j = 1:length(h1)
    patch(get(h1(j), 'XData'), get(h1(j), 'YData'), color1, 'FaceAlpha', .5);
end
for j = 1:length(h2)
    patch(get(h2(j), 'XData'), get(h2(j), 'YData'), color2, 'FaceAlpha', .5);
end

% Dummy legend handles
hLegend1 = plot(nan, nan, 's', 'MarkerFaceColor', color1, 'MarkerEdgeColor', color1);
hLegend2 = plot(nan, nan, 's', 'MarkerFaceColor', color2, 'MarkerEdgeColor', color2);

legend([hLegend1, hLegend2], ...
    {'\textbf{KPAX}', '\textbf{KPAX*}-Large-$\delta$'}, ...
    'Interpreter', 'latex', 'Location', 'northeast');

xlabel('Time (ms)');
ylabel('Solution Cost');
grid on;

set(gca, 'XTick', timeGrid);
set(gca, 'XTickLabel', arrayfun(@num2str, timeGrid, 'UniformOutput', false));
ylim([5 7]);

% Set figure size and export
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 8.5]);
set(gcf, 'PaperPosition', [0 0 8.5 8.5]);
fileName = 'KPAX_KPAX*_Quad_zigZag.pdf';
print(gcf, fileName, '-dpdf');
