close all; clc; clear;

% --------------------------
% 1) Define Time Grid
% --------------------------
k = 200;   % step size in ms
T = 5000;    % final cutoff time in ms
timeGrid = 0:k:T;

numTimes = length(timeGrid);
numSims  = 100;  % number of simulation runs

% Default cost if no solution is found by time t
V = NaN;

% --------------------------
% 2) Prepare Data Structures
% --------------------------
% We'll store the best cost over time for each simulation:
bestCostMatrix = NaN(numTimes, numSims);

% Also store solution stats to compute medians, success, etc.
initialSolutionTimes = [];
initialSolutionCosts = [];
finalSolutionTimes   = [];
finalSolutionCosts   = [];

% Track how many runs produced at least one solution
foundSolutionsCount = 0;
fileName = "OKPAX_3mill_Quad_zigZag_fast";

% --------------------------
% 3) Main Loop Over Simulations
% --------------------------
for simIdx = 0 : numSims-1

    % ---------------------------------------------------------------------
    % (A) Load the iteration time data: "IterationTimeX.csv"
    % ---------------------------------------------------------------------
    iterationTimeFile = sprintf( ...
        '/home/nicolas/dev/research/KGMT/NSight/' + fileName + '/IterationTime/IterationTime%d.csv', ...
        simIdx);
    iterationTimeData = [];
    if isfile(iterationTimeFile)
        fileInfo = dir(iterationTimeFile);
        if fileInfo.bytes > 0
            % If purely numeric, you can use load. If it's CSV with headers, use readmatrix
            iterationTimeData = load(iterationTimeFile); 
        end
    end
    
    % ---------------------------------------------------------------------
    % (B) Load the path cost data: "pathCostsX.csv"
    % ---------------------------------------------------------------------
    pathCostFile = sprintf( ...
        '/home/nicolas/dev/research/KGMT/NSight/' + fileName + '/PathCosts/pathCosts%d.csv', ...
        simIdx);
    pathCostData = [];
    if isfile(pathCostFile)
        fileInfo = dir(pathCostFile);
        if fileInfo.bytes > 0
            pathCostData = readmatrix(pathCostFile);
        end
    end
    
    % If we have no valid iterationTimeData or pathCostData, skip
    if isempty(iterationTimeData) || isempty(pathCostData)
        bestCostMatrix(:, simIdx+1) = NaN(numTimes, 1);
        continue
    end
    
    % ---------------------------------------------------------------------
    % (C) Parse path cost / iteration arrays
    % ---------------------------------------------------------------------
    % pathCostData columns: [??, cost, iteration]
    costs      = pathCostData(:,2);  % cost
    iterations = pathCostData(:,3);  % iteration index
    
    % Filter out invalid iterations
    validMask = (iterations > 0) & (iterations <= length(iterationTimeData));
    costs      = costs(validMask);
    iterations = iterations(validMask);
    
    if isempty(costs)
        % No valid solutions for this run
        bestCostMatrix(:, simIdx+1) = NaN(numTimes, 1);
        continue
    end
    
    % For each (cost, iteration) pair, find the solution time from iterationTimeData
    solutionTimes = iterationTimeData(iterations);
    
    % Sort solutions by ascending time
    [sortedTimes, sortIdx] = sort(solutionTimes);
    sortedCosts            = costs(sortIdx);
    
    % ---------------------------------------------------------------------
    % (D) Exclude solutions that exceed 300000 ms
    % ---------------------------------------------------------------------
    timeThreshold = 300000;
    validMask     = (sortedTimes <= timeThreshold);
    sortedTimes   = sortedTimes(validMask);
    sortedCosts   = sortedCosts(validMask);
    
    if isempty(sortedTimes)
        % All solutions were after 300k ms => treat as no solution
        bestCostMatrix(:, simIdx+1) = NaN(numTimes, 1);
        continue
    end
    
    % ---------------------------------------------------------------------
    % (E) Mark that we found solutions for this run
    % ---------------------------------------------------------------------
    foundSolutionsCount = foundSolutionsCount + 1;
    
    % ---------------------------------------------------------------------
    % (F) Record initial/final solutions
    % ---------------------------------------------------------------------
    % The earliest solution
    initialSolutionTimes(end+1) = sortedTimes(1);
    initialSolutionCosts(end+1) = sortedCosts(1);
    
    % The latest solution
    finalSolutionTimes(end+1)   = sortedTimes(end);
    finalSolutionCosts(end+1)   = sortedCosts(end);
    
    % ---------------------------------------------------------------------
    % (G) Fill bestCostMatrix over the time grid
    % ---------------------------------------------------------------------
    for tIdx = 1:numTimes
        tVal = timeGrid(tIdx);
        bestSolutionCost = NaN;  % Default if no solution by tVal
        
        % A simple linear approach: for each solution in ascending time
        for iSol = 1:length(sortedTimes)
            if sortedTimes(iSol) <= tVal
                bestSolutionCost = sortedCosts(iSol);
            else
                break;
            end
        end
        
        bestCostMatrix(tIdx, simIdx+1) = bestSolutionCost;
    end
    
end  % end for simIdx

% --------------------------
% 4) Remove time rows that are all NaN
% (just to shrink data; optional)
% --------------------------
validRows = any(~isnan(bestCostMatrix), 2);
bestCostMatrix = bestCostMatrix(validRows, :);
timeGrid = timeGrid(validRows);

% --------------------------
% 5) Compute Success Statistics
% --------------------------
successRate = (foundSolutionsCount / numSims) * 100;

if foundSolutionsCount > 0
    medianInitialTime = median(initialSolutionTimes);
    medianInitialCost = median(initialSolutionCosts);
    medianFinalTime   = median(finalSolutionTimes);
    medianFinalCost   = median(finalSolutionCosts);
else
    % If zero solutions in all runs
    medianInitialTime = NaN;
    medianInitialCost = NaN;
    medianFinalTime   = NaN;
    medianFinalCost   = NaN;
end

% --------------------------
% 6) Print Results
% --------------------------
fprintf('Number of simulations: %d\n', numSims);
fprintf('Success rate: %.2f %%\n', successRate);
fprintf('Median initial solution time (ms): %.1f\n', medianInitialTime);
fprintf('Median initial solution cost: %.4f\n', medianInitialCost);
fprintf('Median final solution time (ms): %.1f\n', medianFinalTime);
fprintf('Median final solution cost: %.4f\n', medianFinalCost);

% --------------------------
% 7) Save Results to .mat
% --------------------------
save(fileName + '.mat', ...
     'bestCostMatrix', 'timeGrid', ...
     'initialSolutionTimes', 'initialSolutionCosts', ...
     'finalSolutionTimes',   'finalSolutionCosts', ...
     'successRate', ...
     'medianInitialTime', 'medianInitialCost', ...
     'medianFinalTime',   'medianFinalCost');
