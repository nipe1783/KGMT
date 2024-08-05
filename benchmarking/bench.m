clc;
clear all;
close all;

%% Execution Time

% -- File Paths --
kgmtExecutionTimePath = '/home/nicolas/dev/research/KGMT/benchmarking/kgmt/executionTime.csv';
rrtParallelExecutionTimePath = '/home/nicolas/dev/research/KGMT/benchmarking/rrtParallel/executionTime.csv';
estParallelExecutionTimePath = '/home/nicolas/dev/research/KGMT/benchmarking/estParallel/executionTime.csv';

% -- Data --
kgmtExecutionTime = readmatrix(kgmtExecutionTimePath) * 1000;
rrtParallelExecutionTime = readmatrix(rrtParallelExecutionTimePath) * 1000;
estParallelExecutionTime = readmatrix(estParallelExecutionTimePath) * 1000;

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

fprintf('/* EST Parallel Execution Time */\n');
fprintf('Mean: %.2f ms\n', estParallel_mean);
fprintf('Standard Deviation: %.2f ms\n', estParallel_std);
fprintf('Minimum: %.2f ms\n', estParallel_min);
fprintf('Maximum: %.2f ms\n', estParallel_max);
fprintf('/***************************/\n\n');


data = [kgmtExecutionTime; rrtParallelExecutionTime; estParallelExecutionTime];
group = [ones(length(kgmtExecutionTime), 1); 2 * ones(length(rrtParallelExecutionTime), 1); 3 * ones(length(estParallelExecutionTime), 1)];

figure;
boxchart(group, data);


title('Double Integrator', 'FontSize', 16, 'FontWeight', 'Bold');
xlabel('Method', 'FontSize', 14, 'FontWeight', 'Bold');
ylabel('Execution Time (ms)', 'FontSize', 14, 'FontWeight', 'Bold');
xticks([1 2 3]);
xticklabels({'KGMT', 'RRT Parallel', 'EST Parallel'});

%% Number of Expansions


