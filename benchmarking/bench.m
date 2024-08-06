clc;
clear all;
close all;

%% consts:
titleSize = 10;
algorithmSize = 9;
figureSize = [100, 100, 1000, 1000];

%% Data Import

% -- File Paths --
kgmtExecutionTimePath = '/home/nicolas/dev/research/KGMT/benchmarking/kgmt/DoubleIntegrator/executionTime.csv';
kgmtExpandedNodesPath = '/home/nicolas/dev/research/KGMT/benchmarking/kgmt/DoubleIntegrator/Data/ExpandedNodes/';
kgmtTreeSizePath = '/home/nicolas/dev/research/KGMT/benchmarking/kgmt/DoubleIntegrator/Data/TreeSize/';


rrtParallelExecutionTimePath = '/home/nicolas/dev/research/KGMT/benchmarking/rrtParallel/DoubleIntegrator/Data/ExecutionTime/executionTime.csv';
rrtParallelTreeSize = '/home/nicolas/dev/research/KGMT/benchmarking/rrtParallel/DoubleIntegrator/Data/Vertices/vertices.csv';
rrtParallelExpandedNodesPath = '/home/nicolas/dev/research/KGMT/benchmarking/rrtParallel/DoubleIntegrator/Data/Iterations/iterations.csv';

estParallelExecutionTimePath = '/home/nicolas/dev/research/KGMT/benchmarking/estParallel/DoubleIntegrator/Data/ExecutionTime/executionTime.csv';
estParallelExpandedNodesPath = '/home/nicolas/dev/research/KGMT/benchmarking/estParallel/DoubleIntegrator/Data/Iterations/iterations.csv';
estParallelTreeSize = '/home/nicolas/dev/research/KGMT/benchmarking/estParallel/DoubleIntegrator/Data/Vertices/vertices.csv';

pdstParallelExecutionTimePath = '/home/nicolas/dev/research/KGMT/benchmarking/pdstParallel/DoubleIntegrator/Data/ExecutionTime/executionTime.csv';
pdstParallelExpandedNodesPath = '/home/nicolas/dev/research/KGMT/benchmarking/pdstParallel/DoubleIntegrator/Data/Iterations/iterations.csv';
pdstParallelTreeSize = '/home/nicolas/dev/research/KGMT/benchmarking/pdstParallel/DoubleIntegrator/Data/Vertices/vertices.csv';

% -- Execution Time Data --
kgmtExecutionTime = readmatrix(kgmtExecutionTimePath) * 1000;
rrtParallelExecutionTime = readmatrix(rrtParallelExecutionTimePath) * 1000;
estParallelExecutionTime = readmatrix(estParallelExecutionTimePath) * 1000;
pdstParallelExecutionTime = readmatrix(pdstParallelExecutionTimePath) * 1000;

% -- Node Expansion / Tree Size Data --
N = length(dir(kgmtExpandedNodesPath))-2;
kgmtExpandedNodes = zeros(N, 1);
kgmtTreeSize = zeros(N, 1);
kgmtFrontierSize = zeros(N, 1);
for i = 1:N
    expandedNodesPath = append(kgmtExpandedNodesPath, 'ExpandedNodes', num2str(i-1), '/expandedNodes.csv');
    treeSizePath = append(kgmtTreeSizePath, 'TreeSize', num2str(i-1), '/treeSize.csv');
    kgmtExpandedNodes(i) = sum(readmatrix(expandedNodesPath));
    treeSize = readmatrix(treeSizePath);
    kgmtTreeSize(i) = treeSize(end);
end

rrtParallelExpandedNodes = readmatrix(rrtParallelExpandedNodesPath);
rrtParallelTreeSize = readmatrix(rrtParallelTreeSize);

estParallelExpandedNodes = readmatrix(estParallelExpandedNodesPath);
estParallelTreeSize = readmatrix(estParallelTreeSize);

pdstParallelExpandedNodes = readmatrix(pdstParallelExpandedNodesPath);
pdstParallelTreeSize = readmatrix(pdstParallelTreeSize);



plotBenchmarkResults(kgmtExecutionTime, rrtParallelExecutionTime, estParallelExecutionTime, pdstParallelExecutionTime, ...
                     kgmtExpandedNodes, rrtParallelExpandedNodes, estParallelExpandedNodes, pdstParallelExpandedNodes, ...
                     kgmtTreeSize, rrtParallelTreeSize, estParallelTreeSize, pdstParallelTreeSize, 'Double Integrator', ...
                     titleSize, algorithmSize, figureSize,  '/home/nicolas/dev/research/KGMT/viz/figs/bench/doubleIntegrator');


function plotBenchmarkResults(kgmtExecutionTime, rrtParallelExecutionTime, estParallelExecutionTime, pdstParallelExecutionTime, ...
                              kgmtExpandedNodes, rrtParallelExpandedNodes, estParallelExpandedNodes, pdstParallelExpandedNodes, ...
                              kgmtTreeSize, rrtParallelTreeSize, estParallelTreeSize, pdstParallelTreeSize, dynamics, ...
                              titleSize, algorithmSize, figureSize, output_dir)

    %% Execution Time
    kgmt_mean = mean(kgmtExecutionTime);
    kgmt_std = std(kgmtExecutionTime);
    kgmt_min = min(kgmtExecutionTime);
    kgmt_max = max(kgmtExecutionTime);

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

    fprintf('/* RRT Parallel Execution Time */\n');
    fprintf('Mean: %.2f ms\n', rrtParallel_mean);
    fprintf('Standard Deviation: %.2f ms\n', rrtParallel_std);
    fprintf('Minimum: %.2f ms\n', rrtParallel_min);
    fprintf('Maximum: %.2f ms\n', rrtParallel_max);
    fprintf('/***************************/\n\n');

    fprintf('/* PDST Parallel Execution Time */\n');
    fprintf('Mean: %.2f ms\n', pdstParallel_mean);
    fprintf('Standard Deviation: %.2f ms\n', pdstParallel_std);
    fprintf('Minimum: %.2f ms\n', pdstParallel_min);
    fprintf('Maximum: %.2f ms\n', pdstParallel_max);
    fprintf('/***************************/\n\n');

    data = [kgmtExecutionTime; rrtParallelExecutionTime; estParallelExecutionTime; pdstParallelExecutionTime];
    group = [ones(length(kgmtExecutionTime), 1); 2 * ones(length(rrtParallelExecutionTime), 1); 3 * ones(length(estParallelExecutionTime), 1); 4 * ones(length(pdstParallelExecutionTime), 1)];

    figure;
    b = boxchart(group, data);
    b.JitterOutliers = 'on';
    b.MarkerStyle = '.';

    title(sprintf('Execution Time (%s)', dynamics), 'FontSize', titleSize, 'FontWeight', 'Bold');
    ylabel('Execution Time (ms)', 'FontSize', 14, 'FontWeight', 'Bold');
    xticks([1, 2, 3, 4]);
    xticklabels({'KGMT', 'RRT (CPU Parallelization)', 'EST (CPU Parallelization)', 'PDST (CPU Parallelization)'});
    set(gca, 'FontSize', algorithmSize);
    set(gcf, 'Position', figureSize);

    saveas(gcf, fullfile(output_dir, 'ExecutionTime_Comparison.jpg'));
    print(fullfile(output_dir, 'ExecutionTime_Comparison.jpg'), '-djpeg', '-r300');

    data = [kgmtExecutionTime];
    group = [ones(length(kgmtExecutionTime), 1)];

    figure;
    b = boxchart(group, data);
    b.JitterOutliers = 'on';
    b.MarkerStyle = '.';

    title(sprintf('Execution Time (%s)', dynamics), 'FontSize', titleSize, 'FontWeight', 'Bold');
    ylabel('Execution Time (ms)', 'FontSize', 14, 'FontWeight', 'Bold');
    xticks([1]);
    xticklabels({'KGMT'});
    set(gca, 'FontSize', algorithmSize);
    set(gcf, 'Position', figureSize);

    saveas(gcf, fullfile(output_dir, 'ExecutionTime_KGMT.jpg'));
    print(fullfile(output_dir, 'ExecutionTime_KGMT.jpg'), '-djpeg', '-r300');

    %% Nodes Expanded
    data = [kgmtExpandedNodes; rrtParallelExpandedNodes; estParallelExpandedNodes; pdstParallelExpandedNodes];
    group = [ones(length(kgmtExpandedNodes), 1); 2 * ones(length(rrtParallelExpandedNodes), 1); 3 * ones(length(estParallelExpandedNodes), 1); 4 * ones(length(pdstParallelExpandedNodes), 1)];

    figure;
    b = boxchart(group, data);
    b.JitterOutliers = 'on';
    b.MarkerStyle = '.';

    title(sprintf('Number Of Nodes Expanded (%s)', dynamics), 'FontSize', titleSize, 'FontWeight', 'Bold');
    ylabel('Number Of Expanded Nodes', 'FontSize', 14, 'FontWeight', 'Bold');
    xticks([1, 2, 3, 4]);
    xticklabels({'KGMT', 'RRT (CPU Parallelization)', 'EST (CPU Parallelization)', 'PDST (CPU Parallelization)'});
    set(gca, 'FontSize', algorithmSize);
    set(gcf, 'Position', figureSize);

    saveas(gcf, fullfile(output_dir, 'NodesExpanded_Comparison.jpg'));
    print(fullfile(output_dir, 'NodesExpanded_Comparison.jpg'), '-djpeg', '-r300');

    %% Tree Size
    data = [kgmtTreeSize; rrtParallelTreeSize; estParallelTreeSize; pdstParallelTreeSize];
    group = [ones(length(kgmtTreeSize), 1); 2 * ones(length(rrtParallelTreeSize), 1); 3 * ones(length(estParallelTreeSize), 1); 4 * ones(length(pdstParallelTreeSize), 1)];

    figure;
    b = boxchart(group, data);
    b.JitterOutliers = 'on';
    b.MarkerStyle = '.';

    title(sprintf('Tree Size (%s)', dynamics), 'FontSize', titleSize, 'FontWeight', 'Bold');
    ylabel('Number Of Nodes In Tree', 'FontSize', 14, 'FontWeight', 'Bold');
    xticks([1, 2, 3, 4]);
    xticklabels({'KGMT', 'RRT (CPU Parallelization)', 'EST (CPU Parallelization)', 'PDST (CPU Parallelization)'});
    set(gca, 'FontSize', algorithmSize);
    set(gcf, 'Position', figureSize);

    saveas(gcf, fullfile(output_dir, 'TreeSize_Comparison.jpg'));
    print(fullfile(output_dir, 'TreeSize_Comparison.jpg'), '-djpeg', '-r300');
end
