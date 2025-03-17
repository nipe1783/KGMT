close all; clc; clear;

% --------------------------
% Define time grid
% --------------------------
k = 100;   % step (ms)
T = 3000;  % final time (make sure this matches your real max time)
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
    % Read the iteration time data
    iterationTimeFile = sprintf('/home/nicolas/dev/research/KGMT/NSight/KPAX_20mill_50_3_3_3_DubinsAirplane_trees/IterationTime/IterationTime%d.csv', simIdx);
    iterationTimeData = load(iterationTimeFile);  % or use readmatrix if .csv includes non-numeric data
    
    % Read the path cost data
    pathCostFile = sprintf('/home/nicolas/dev/research/KGMT/NSight/KPAX_20mill_50_3_3_3_DubinsAirplane_trees/PathCosts/pathCosts%d.csv', simIdx);
    pathCostData = readmatrix(pathCostFile);
    
    costs = pathCostData(:,2);
    iterations = pathCostData(:,3);
    
    validMask = iterations > 0 & iterations <= length(iterationTimeData);
    costs = costs(validMask);
    iterations = iterations(validMask);
    
    solutionTimes = iterationTimeData(iterations);
    
    for tIdx = 1:numTimes
        tVal = timeGrid(tIdx);
        bestSolutionCost = V;
        
        for i = 1:length(solutionTimes)
            if solutionTimes(i) <= tVal
                bestSolutionCost = costs(i);
            else
                break;
            end
        end
        
        bestCostMatrix(tIdx, simIdx+1) = bestSolutionCost;
    end
end

validRows = any(~isnan(bestCostMatrix), 2);
bestCostMatrix = bestCostMatrix(validRows, :);
timeGrid = timeGrid(validRows);

% Save the data for later use
save('KPAX_20mill_50_3_3_3_DubinsAirplane_trees.mat', 'bestCostMatrix', 'timeGrid');
