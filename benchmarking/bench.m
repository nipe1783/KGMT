clc;
clear all;
close all;

%% Data Import

% -- File Paths --s
kgmtExecutionTimePath = '/home/nicolas/dev/research/KGMT/benchmarking/kgmt/executionTime.csv';
kgmtExpandedNodesPath = '/home/nicolas/dev/research/KGMT/benchmarking/kgmt/Data/ExpandedNodes/';
kgmtTreeSizePath = '/home/nicolas/dev/research/KGMT/benchmarking/kgmt/Data/TreeSize/';
rrtParallelExecutionTimePath = '/home/nicolas/dev/research/KGMT/benchmarking/rrtParallel/executionTime.csv';
estParallelExecutionTimePath = '/home/nicolas/dev/research/KGMT/benchmarking/estParallel/executionTime.csv';


% -- Execution Time Data --
kgmtExecutionTime = readmatrix(kgmtExecutionTimePath) * 1000;
rrtParallelExecutionTime = readmatrix(rrtParallelExecutionTimePath) * 1000;
estParallelExecutionTime = readmatrix(estParallelExecutionTimePath) * 1000;

% -- Node Expansion Data --
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


%% Execution Time

% kgmt_mean = mean(kgmtExecutionTime);
% kgmt_std = std(kgmtExecutionTime);
% kgmt_min = min(kgmtExecutionTime);
% kgmt_max = max(kgmtExecutionTime);
% 
% rrtParallel_mean = mean(rrtParallelExecutionTime);
% rrtParallel_std = std(rrtParallelExecutionTime);
% rrtParallel_min = min(rrtParallelExecutionTime);
% rrtParallel_max = max(rrtParallelExecutionTime);
% 
% estParallel_mean = mean(estParallelExecutionTime);
% estParallel_std = std(estParallelExecutionTime);
% estParallel_min = min(estParallelExecutionTime);
% estParallel_max = max(estParallelExecutionTime);
% 
% fprintf('/* KGMT Execution Time */\n');
% fprintf('Mean: %.2f ms\n', kgmt_mean);
% fprintf('Standard Deviation: %.2f ms\n', kgmt_std);
% fprintf('Minimum: %.2f ms\n', kgmt_min);
% fprintf('Maximum: %.2f ms\n', kgmt_max);
% fprintf('/***************************/\n\n');
% 
% fprintf('/* RRT Parallel Execution Time */\n');
% fprintf('Mean: %.2f ms\n', rrtParallel_mean);
% fprintf('Standard Deviation: %.2f ms\n', rrtParallel_std);
% fprintf('Minimum: %.2f ms\n', rrtParallel_min);
% fprintf('Maximum: %.2f ms\n', rrtParallel_max);
% fprintf('/***************************/\n\n');
% 
% fprintf('/* EST Parallel Execution Time */\n');
% fprintf('Mean: %.2f ms\n', estParallel_mean);
% fprintf('Standard Deviation: %.2f ms\n', estParallel_std);
% fprintf('Minimum: %.2f ms\n', estParallel_min);
% fprintf('Maximum: %.2f ms\n', estParallel_max);
% fprintf('/***************************/\n\n');
% 
% 
% data = [kgmtExecutionTime; rrtParallelExecutionTime; estParallelExecutionTime];
% group = [ones(length(kgmtExecutionTime), 1); 2 * ones(length(rrtParallelExecutionTime), 1); 3 * ones(length(estParallelExecutionTime), 1)];
% 
% figure;
% boxchart(group, data);
% 
% 
% title('Double Integrator', 'FontSize', 16, 'FontWeight', 'Bold');
% xlabel('Method', 'FontSize', 14, 'FontWeight', 'Bold');
% ylabel('Execution Time (ms)', 'FontSize', 14, 'FontWeight', 'Bold');
% xticks([1 2 3]);
% xticklabels({'KGMT', 'RRT Parallel', 'EST Parallel'});

%% Nodes Expanded

data = [kgmtExpandedNodes];
group = [ones(length(kgmtExpandedNodes), 1)];

figure;
boxchart(group, data);

title('Double Integrator', 'FontSize', 16, 'FontWeight', 'Bold');
xlabel('Method', 'FontSize', 14, 'FontWeight', 'Bold');
ylabel('Nodes Expanded', 'FontSize', 14, 'FontWeight', 'Bold');
xticks([1]);
xticklabels({'KGMT'});

%% Tree Size

data = [kgmtTreeSize];
group = [ones(length(kgmtTreeSize), 1)];

figure;
boxchart(group, data);

title('Double Integrator', 'FontSize', 16, 'FontWeight', 'Bold');
xlabel('Method', 'FontSize', 14, 'FontWeight', 'Bold');
ylabel('Tree Size', 'FontSize', 14, 'FontWeight', 'Bold');
xticks([1]);
xticklabels({'KGMT'});

