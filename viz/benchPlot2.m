close all; clc; clear;

% --------------------------
% 1) Define time grid
% --------------------------
k = 0.25;   % step (ms)
T = 11;  % final time (make sure this matches your real max time)
timeGrid = 0:k:T;

numTimes = length(timeGrid);
numSims  = 100;

% We will store the best-cost-over-time for each simulation in a matrix:
% Rows = each discrete time in timeGrid
% Cols = each simulation
bestCostMatrix = NaN(numTimes, numSims);

% If a simulation has no solution before interval t, we use cost = V
V = NaN;  % define a default cost larger than any real solution

% --------------------------
% 2) Loop over each simulation
% --------------------------
for simIdx = 0:numSims-1
    
    % (a) Read the iteration time data: iteration index -> time (in ms)
    iterationTimeFile = sprintf( ...
        '/home/nicolas/dev/research/KGMT/NSight/OKPAX_20mill_50_3_3_3_6DDI_zigZag/IterationTime/IterationTime%d.csv', ...
        simIdx);
    iterationTimeData = load(iterationTimeFile); % or readmatrix
    
    % (b) Read the path cost data
    pathCostFile = sprintf( ...
        '/home/nicolas/dev/research/KGMT/NSight/OKPAX_20mill_50_3_3_3_6DDI_zigZag/PathCosts/pathCosts%d.csv', ...
        simIdx);
    pathCostData = readmatrix(pathCostFile);
    
    % pathCostData has columns: [unusedTime, cost, iteration]
    costs        = pathCostData(:,2);
    iterations   = pathCostData(:,3);
    
    % (c) Filter out invalid iteration indices
    validMask    = iterations > 0 & iterations <= length(iterationTimeData);
    costs        = costs(validMask);
    iterations   = iterations(validMask);
    
    % (d) Convert iteration indices to times
    solutionTimes = iterationTimeData(iterations);
    
    % (e) For each t in timeGrid, find the last solution <= t
    for tIdx = 1:numTimes
        tVal = timeGrid(tIdx);
        
        % Start with the default (meaning no solution yet)
        bestSolutionCost = V;
        
        % Because solutionTimes is sorted ascending, we can do a simple
        % forward search or a binary search. Here we do a simple approach:
        for i = 1:length(solutionTimes)
            if solutionTimes(i) <= tVal
                bestSolutionCost = costs(i);
            else
                % Since solutionTimes(i) > tVal, no need to check further
                break;
            end
        end
        
        % Store the best solution for this sim at this time
        bestCostMatrix(tIdx, simIdx+1) = bestSolutionCost;
    end
    
end

validRows = any(bestCostMatrix ~= V, 2);  % Logical index of rows not entirely V
bestCostMatrix = bestCostMatrix(validRows, :);  % Apply filter to matrix
timeGrid = timeGrid(validRows);  % Also filter timeGrid accordingly

% --------------------------
% 3) Plot results
% --------------------------
figure;
hBoxPlot = boxplot(bestCostMatrix', ...
        'positions', timeGrid, ...          % place each box at the correct time
        'symbol', '');                      % optional: hide individual outlier symbols

xlabel('Time (ms)');
ylabel('Solution Cost');
xlim([0, T + k]);  % slightly expand x-axis beyond the last time interval

% Manually setting x-axis ticks and labels
set(gca, 'XTick', timeGrid);  % Set the x-axis ticks to correspond to your time grid
set(gca, 'XTickLabel', arrayfun(@num2str, timeGrid, 'UniformOutput', false));  % Convert numbers to strings for labels

grid on;

% --------------------------
% 4) Calculate and print box plot statistics
% --------------------------
disp('Time (ms) | Min Cost | Median Cost | Max Cost');
for tIdx = 1:length(timeGrid)
    tVal = timeGrid(tIdx);
    dataAtT = bestCostMatrix(tIdx, :);
    
    % Calculating statistics
    minCost = min(dataAtT);
    medianCost = median(dataAtT);
    maxCost = max(dataAtT);
    
    % Displaying results
    fprintf('%9d | %8.2f | %11.2f | %8.2f\n', tVal, minCost, medianCost, maxCost);
end

% Set figure size
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);  % Fixed size in inches (e.g., Letter size)
set(gcf, 'PaperPosition', [0 0 8.5 11]);  % Position on the paper

% Save the figure
fileName = 'KPruning_5_20mill_50_3_3_3_6DDI_zigZag.pdf';  % Define your filename here
print(gcf, fileName, '-dpdf');  % Save as PDF