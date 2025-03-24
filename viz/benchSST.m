close all; clc; clear;

% --------------------------
% Define time grid
% --------------------------
k = 10000.0;   % step (ms)
T = 300000;    % final time (make sure this matches your real max time)
timeGrid = 0:k:T;

numTimes = length(timeGrid);
numSims  = 100;

% Matrix to store the best-cost-over-time for each simulation
bestCostMatrix = NaN(numTimes, numSims);

% Default cost if no solution is found before interval t
V = NaN;

% --------------------------
% Loop over each simulation
% --------------------------
for simIdx = 0:numSims-1
    % Read the solution times and costs
    solutionFile = sprintf('/home/nicolas/dev/research/KGMT/NSight/SST_6DDI_trees/trees_solution_times_and_costs%d.csv', simIdx);
    solutionData = readmatrix(solutionFile);

    solutionTimes = solutionData(:,1);
    costs = solutionData(:,2);
    
    % Sort data by solution time to ensure the times are in increasing order
    [sortedTimes, sortIdx] = sort(solutionTimes);
    sortedCosts = costs(sortIdx);
    
    % For each time in the time grid, find the best cost that is valid up to that time
    for tIdx = 1:numTimes
        tVal = timeGrid(tIdx);
        bestSolutionCost = V;
        
        % Find the last solution cost that fits within the current time grid value
        validCosts = sortedCosts(sortedTimes <= tVal);
        if ~isempty(validCosts)
            bestSolutionCost = validCosts(end);
        end
        
        bestCostMatrix(tIdx, simIdx+1) = bestSolutionCost;
    end
end

% Remove rows where all entries are NaN (no solutions found in time)
validRows = any(~isnan(bestCostMatrix), 2);
bestCostMatrix = bestCostMatrix(validRows, :);
timeGrid = timeGrid(validRows);

% Save the data for later use
save('SST_6DDI_trees_results.mat', 'bestCostMatrix', 'timeGrid');
