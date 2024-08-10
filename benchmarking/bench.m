clc;
clear all;
close all;

%% consts:
titleSize = 10;
algorithmSize = 9;
figureSize = [100, 100, 1000, 1000];

%% Dubins Airplane:

% -- File Paths --
kgmtExecutionTimePath = '/home/nicolas/dev/research/KGMT/benchmarking/kgmt/DubinsAirplane/executionTime.csv';
kgmtExpandedNodesPath = '/home/nicolas/dev/research/KGMT/benchmarking/kgmt/DubinsAirplane/Data/ExpandedNodes/';
kgmtTreeSizePath = '/home/nicolas/dev/research/KGMT/benchmarking/kgmt/DubinsAirplane/Data/TreeSize/';

kgmtStateGridExecutionTimePath = '/home/nicolas/dev/research/KGMT/benchmarking/kgmtStateGrid/DubinsAirplane/executionTime.csv';
kgmtStateGridExpandedNodesPath = '/home/nicolas/dev/research/KGMT/benchmarking/kgmtStateGrid/DubinsAirplane/Data/ExpandedNodes/';
kgmtStateGridTreeSizePath = '/home/nicolas/dev/research/KGMT/benchmarking/kgmtStateGrid/DubinsAirplane/Data/TreeSize/';

rrtParallelExecutionTimePath = '/home/nicolas/dev/research/KGMT/benchmarking/rrtParallel/DubinsAirplane/Data/ExecutionTime/executionTime.csv';
rrtParallelTreeSize = '/home/nicolas/dev/research/KGMT/benchmarking/rrtParallel/DubinsAirplane/Data/Vertices/vertices.csv';
rrtParallelExpandedNodesPath = '/home/nicolas/dev/research/KGMT/benchmarking/rrtParallel/DubinsAirplane/Data/Iterations/iterations.csv';

estParallelExecutionTimePath = '/home/nicolas/dev/research/KGMT/benchmarking/estParallel/DubinsAirplane/Data/ExecutionTime/executionTime.csv';
estParallelExpandedNodesPath = '/home/nicolas/dev/research/KGMT/benchmarking/estParallel/DubinsAirplane/Data/Iterations/iterations.csv';
estParallelTreeSize = '/home/nicolas/dev/research/KGMT/benchmarking/estParallel/DubinsAirplane/Data/Vertices/vertices.csv';

pdstParallelExecutionTimePath = '/home/nicolas/dev/research/KGMT/benchmarking/pdstParallel/DubinsAirplane/Data/ExecutionTime/executionTime.csv';
pdstParallelExpandedNodesPath = '/home/nicolas/dev/research/KGMT/benchmarking/pdstParallel/DubinsAirplane/Data/Iterations/iterations.csv';
pdstParallelTreeSize = '/home/nicolas/dev/research/KGMT/benchmarking/pdstParallel/DubinsAirplane/Data/Vertices/vertices.csv';

% -- Execution Time Data --
kgmtExecutionTime = readmatrix(kgmtExecutionTimePath) * 1000;
kgmtStateGridExecutionTime = readmatrix(kgmtStateGridExecutionTimePath) * 1000;
rrtParallelExecutionTime = readmatrix(rrtParallelExecutionTimePath) * 1000;
estParallelExecutionTime = readmatrix(estParallelExecutionTimePath) * 1000;
pdstParallelExecutionTime = readmatrix(pdstParallelExecutionTimePath) * 1000;

% -- Node Expansion / Tree Size Data --
N = length(dir(kgmtExpandedNodesPath))-2;
kgmtExpandedNodes = zeros(N, 1);
kgmtTreeSize = zeros(N, 1);
kgmtFrontierSize = zeros(N, 1);

kgmtStateGridExpandedNodes = zeros(N, 1);
kgmtStateGridTreeSize = zeros(N, 1);
kgmtStateGridFrontierSize = zeros(N, 1);
for i = 1:N
    expandedNodesPath = append(kgmtExpandedNodesPath, 'ExpandedNodes', num2str(i-1), '/expandedNodes.csv');
    treeSizePath = append(kgmtTreeSizePath, 'TreeSize', num2str(i-1), '/treeSize.csv');
    kgmtExpandedNodes(i) = sum(readmatrix(expandedNodesPath));
    treeSize = readmatrix(treeSizePath);
    kgmtTreeSize(i) = treeSize(end);

    expandedNodesStateGridPath = append(kgmtStateGridExpandedNodesPath, 'ExpandedNodes', num2str(i-1), '/expandedNodes.csv');
    treeSizeStateGridPath = append(kgmtStateGridTreeSizePath, 'TreeSize', num2str(i-1), '/treeSize.csv');
    kgmtStateGridExpandedNodes(i) = sum(readmatrix(expandedNodesStateGridPath));
    treeSizeStateGrid = readmatrix(treeSizeStateGridPath);
    kgmtStateGridTreeSize(i) = treeSizeStateGrid(end);
end

rrtParallelExpandedNodes = readmatrix(rrtParallelExpandedNodesPath);
rrtParallelTreeSize = readmatrix(rrtParallelTreeSize);

estParallelExpandedNodes = readmatrix(estParallelExpandedNodesPath);
estParallelTreeSize = readmatrix(estParallelTreeSize);

pdstParallelExpandedNodes = readmatrix(pdstParallelExpandedNodesPath);
pdstParallelTreeSize = readmatrix(pdstParallelTreeSize);

plotBenchmarkResultsDA(kgmtExecutionTime, kgmtStateGridExecutionTime, rrtParallelExecutionTime, estParallelExecutionTime, pdstParallelExecutionTime, ...
                     kgmtExpandedNodes, kgmtStateGridExpandedNodes, rrtParallelExpandedNodes, estParallelExpandedNodes, pdstParallelExpandedNodes, ...
                     kgmtTreeSize, kgmtStateGridTreeSize, rrtParallelTreeSize, estParallelTreeSize, pdstParallelTreeSize, 'Dubins Airplane', ...
                     titleSize, algorithmSize, figureSize,  '/home/nicolas/dev/research/KGMT/viz/figs/bench/dubinsAirplane');

function plotBenchmarkResultsDA(kgmtExecutionTime, kgmtStateGridExecutionTime, rrtParallelExecutionTime, estParallelExecutionTime, pdstParallelExecutionTime, ...
                              kgmtExpandedNodes, kgmtStateGridExpandedNodes, rrtParallelExpandedNodes, estParallelExpandedNodes, pdstParallelExpandedNodes, ...
                              kgmtTreeSize, kgmtStateGridTreeSize, rrtParallelTreeSize, estParallelTreeSize, pdstParallelTreeSize, dynamics, ...
                              titleSize, algorithmSize, figureSize, output_dir)

    %% Execution Time
    kgmt_mean = mean(kgmtExecutionTime);
    kgmt_std = std(kgmtExecutionTime);
    kgmt_min = min(kgmtExecutionTime);
    kgmt_max = max(kgmtExecutionTime);

    kgmtStateGrid_mean = mean(kgmtStateGridExecutionTime);
    kgmtStateGrid_std = std(kgmtStateGridExecutionTime);
    kgmtStateGrid_min = min(kgmtStateGridExecutionTime);
    kgmtStateGrid_max = max(kgmtStateGridExecutionTime);

    rrtParallel_mean = mean(rrtParallelExecutionTime);
    rrtParallel_std = std(rrtParallelExecutionTime);
    rrtParallel_min = min(rrtParallelExecutionTime);
    rrtParallel_max = max(rrtParallelExecutionTime);

    estParallel_mean = mean(estParallelExecutionTime);
    estParallel_std = std(estParallelExecutionTime);
    estParallel_min = min(estParallelExecutionTime);
    estParallel_max = max(estParallelExecutionTime);

    pdstParallel_mean = mean(pdstParallelExecutionTime);
    pdstParallel_std = std(pdstParallelExecutionTime);
    pdstParallel_min = min(pdstParallelExecutionTime);
    pdstParallel_max = max(pdstParallelExecutionTime);

    fprintf('/* KGMT Execution Time */\n');
    fprintf('Mean: %.2f ms\n', kgmt_mean);
    fprintf('Standard Deviation: %.2f ms\n', kgmt_std);
    fprintf('Minimum: %.2f ms\n', kgmt_min);
    fprintf('Maximum: %.2f ms\n', kgmt_max);
    fprintf('/***************************/\n\n');

    fprintf('/* KGMT State Grid Execution Time */\n');
    fprintf('Mean: %.2f ms\n', kgmtStateGrid_mean);
    fprintf('Standard Deviation: %.2f ms\n', kgmtStateGrid_std);
    fprintf('Minimum: %.2f ms\n', kgmtStateGrid_min);
    fprintf('Maximum: %.2f ms\n', kgmtStateGrid_max);
    fprintf('/***************************/\n\n');

    data = [kgmtExecutionTime; kgmtStateGridExecutionTime];
    group = [ones(length(kgmtExecutionTime), 1); 2 * ones(length(kgmtStateGridExecutionTime), 1)];

    figure;
    b = boxchart(group, data);
    b.JitterOutliers = 'on';
    b.MarkerStyle = '.';

    title(sprintf('Execution Time (KGMT vs KGMT State Grid) - %s', dynamics), 'FontSize', titleSize, 'FontWeight', 'Bold');
    ylabel('Execution Time (ms)', 'FontSize', 14, 'FontWeight', 'Bold');
    xticks([1, 2]);
    xticklabels({'KGMT', 'KGMT State Grid'});
    set(gca, 'FontSize', algorithmSize);
    set(gcf, 'Position', figureSize);

    saveas(gcf, fullfile(output_dir, 'ExecutionTime_KGMT_vs_KGMTStateGrid.jpg'));
    print(fullfile(output_dir, 'ExecutionTime_KGMT_vs_KGMTStateGrid.jpg'), '-djpeg', '-r300');

    %% Nodes Expanded
    data = [kgmtExpandedNodes; kgmtStateGridExpandedNodes];
    group = [ones(length(kgmtExpandedNodes), 1); 2 * ones(length(kgmtStateGridExpandedNodes), 1)];

    figure;
    b = boxchart(group, data);
    b.JitterOutliers = 'on';
    b.MarkerStyle = '.';

    title(sprintf('Number Of Nodes Expanded (KGMT vs KGMT State Grid) - %s', dynamics), 'FontSize', titleSize, 'FontWeight', 'Bold');
    ylabel('Number Of Expanded Nodes', 'FontSize', 14, 'FontWeight', 'Bold');
    xticks([1, 2]);
    xticklabels({'KGMT', 'KGMT State Grid'});
    set(gca, 'FontSize', algorithmSize);
    set(gcf, 'Position', figureSize);

    saveas(gcf, fullfile(output_dir, 'NodesExpanded_KGMT_vs_KGMTStateGrid.jpg'));
    print(fullfile(output_dir, 'NodesExpanded_KGMT_vs_KGMTStateGrid.jpg'), '-djpeg', '-r300');

    %% Tree Size
    data = [kgmtTreeSize; kgmtStateGridTreeSize];
    group = [ones(length(kgmtTreeSize), 1); 2 * ones(length(kgmtStateGridTreeSize), 1)];

    figure;
    b = boxchart(group, data);
    b.JitterOutliers = 'on';
    b.MarkerStyle = '.';

    title(sprintf('Tree Size (KGMT vs KGMT State Grid) - %s', dynamics), 'FontSize', titleSize, 'FontWeight', 'Bold');
    ylabel('Number Of Nodes In Tree', 'FontSize', 14, 'FontWeight', 'Bold');
    xticks([1, 2]);
    xticklabels({'KGMT', 'KGMT State Grid'});
    set(gca, 'FontSize', algorithmSize);
    set(gcf, 'Position', figureSize);

    saveas(gcf, fullfile(output_dir, 'TreeSize_KGMT_vs_KGMTStateGrid.jpg'));
    print(fullfile(output_dir, 'TreeSize_KGMT_vs_KGMTStateGrid.jpg'), '-djpeg', '-r300');

    %% Full Comparison (Original Code)
    data = [kgmtExecutionTime; kgmtStateGridExecutionTime; rrtParallelExecutionTime; estParallelExecutionTime; pdstParallelExecutionTime];
    group = [ones(length(kgmtExecutionTime), 1); 2 * ones(length(kgmtStateGridExecutionTime), 1); 3 * ones(length(rrtParallelExecutionTime), 1); 4 * ones(length(estParallelExecutionTime), 1); 5 * ones(length(pdstParallelExecutionTime), 1)];

    figure;
    b = boxchart(group, data);
    b.JitterOutliers = 'on';
    b.MarkerStyle = '.';

    title(sprintf('Execution Time (%s)', dynamics), 'FontSize', titleSize, 'FontWeight', 'Bold');
    ylabel('Execution Time (ms)', 'FontSize', 14, 'FontWeight', 'Bold');
    xticks([1, 2, 3, 4, 5]);
    xticklabels({'KGMT', 'KGMT State Grid', 'RRT (CPU Parallelization)', 'EST (CPU Parallelization)', 'PDST (CPU Parallelization)'});
    set(gca, 'FontSize', algorithmSize);
    set(gcf, 'Position', figureSize);

    saveas(gcf, fullfile(output_dir, 'ExecutionTime_Comparison.jpg'));
    print(fullfile(output_dir, 'ExecutionTime_Comparison.jpg'), '-djpeg', '-r300');

    %% Nodes Expanded (Original Code)
    data = [kgmtExpandedNodes; kgmtStateGridExpandedNodes; rrtParallelExpandedNodes; estParallelExpandedNodes; pdstParallelExpandedNodes];
    group = [ones(length(kgmtExpandedNodes), 1); 2 * ones(length(kgmtStateGridExpandedNodes), 1); 3 * ones(length(rrtParallelExpandedNodes), 1); 4 * ones(length(estParallelExpandedNodes), 1); 5 * ones(length(pdstParallelExpandedNodes), 1)];

    figure;
    b = boxchart(group, data);
    b.JitterOutliers = 'on';
    b.MarkerStyle = '.';

    title(sprintf('Number Of Nodes Expanded (%s)', dynamics), 'FontSize', titleSize, 'FontWeight', 'Bold');
    ylabel('Number Of Expanded Nodes', 'FontSize', 14, 'FontWeight', 'Bold');
    xticks([1, 2, 3, 4, 5]);
    xticklabels({'KGMT', 'KGMT State Grid', 'RRT (CPU Parallelization)', 'EST (CPU Parallelization)', 'PDST (CPU Parallelization)'});
    set(gca, 'FontSize', algorithmSize);
    set(gcf, 'Position', figureSize);

    saveas(gcf, fullfile(output_dir, 'NodesExpanded_Comparison.jpg'));
    print(fullfile(output_dir, 'NodesExpanded_Comparison.jpg'), '-djpeg', '-r300');

    %% Tree Size (Original Code)
    data = [kgmtTreeSize; kgmtStateGridTreeSize; rrtParallelTreeSize; estParallelTreeSize; pdstParallelTreeSize];
    group = [ones(length(kgmtTreeSize), 1); 2 * ones(length(kgmtStateGridTreeSize), 1); 3 * ones(length(rrtParallelTreeSize), 1); 4 * ones(length(estParallelTreeSize), 1); 5 * ones(length(pdstParallelTreeSize), 1)];

    figure;
    b = boxchart(group, data);
    b.JitterOutliers = 'on';
    b.MarkerStyle = '.';

    title(sprintf('Tree Size (%s)', dynamics), 'FontSize', titleSize, 'FontWeight', 'Bold');
    ylabel('Number Of Nodes In Tree', 'FontSize', 14, 'FontWeight', 'Bold');
    xticks([1, 2, 3, 4, 5]);
    xticklabels({'KGMT', 'KGMT State Grid', 'RRT (CPU Parallelization)', 'EST (CPU Parallelization)', 'PDST (CPU Parallelization)'});
    set(gca, 'FontSize', algorithmSize);
    set(gcf, 'Position', figureSize);

    saveas(gcf, fullfile(output_dir, 'TreeSize_Comparison.jpg'));
    print(fullfile(output_dir, 'TreeSize_Comparison.jpg'), '-djpeg', '-r300');
end

