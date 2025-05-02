close all; clc; clear;

% --------------------------
% Define time grid
% --------------------------
k = 10000.0;   % step (ms)
T = 300000;    % final time (make sure this matches your real max time)
timeGrid = 0:k:T;

numTimes = length(timeGrid);
numSims  = 100;

% Prepare arrays for storing initial/final solution data across runs
initialSolutionTimes = [];
initialSolutionCosts = [];
finalSolutionTimes   = [];
finalSolutionCosts   = [];

% Track how many runs actually found a solution
foundSolutionsCount = 0;

% Matrix to store the best-cost-over-time for each simulation
bestCostMatrix = NaN(numTimes, numSims);

% Default cost if no solution is found before interval t
V = NaN;

% --------------------------
% Loop over each simulationQuad
% -------------------------- 
for simIdx = 0:numSims-1

    % 1) Read the solution times and costs
    solutionFile = sprintf('/home/nicolas/dev/research/KGMT/NSight/SST_6DDI_house/house_solution_times_and_costs%d.csv', simIdx);

    fileInfo = dir(solutionFile);
    if ~isempty(fileInfo) && fileInfo.bytes > 0
        solutionData = readmatrix(solutionFile);
        if ~isempty(solutionData)
            % We have at least one solution for this run
            foundSolutionsCount = foundSolutionsCount + 1;

            solutionTimes = solutionData(:,1);
            costs = solutionData(:,2);

            % Sort data by solution time to ensure times are in increasing order
            [sortedTimes, sortIdx] = sort(solutionTimes);
            sortedCosts = costs(sortIdx);

            % ----------------------
            % Store initial/final solution data
            % ----------------------
            % The "initial" solution is the earliest (first in sorted order)
            initialSolutionTimes(end+1) = sortedTimes(1);
            initialSolutionCosts(end+1) = sortedCosts(1);

            % The "final" solution is the last one found
            finalSolutionTimes(end+1)   = sortedTimes(end);
            finalSolutionCosts(end+1)   = sortedCosts(end);

            % ----------------------
            % Build bestCostMatrix over time
            % ----------------------
            for tIdx = 1:numTimes
                tVal = timeGrid(tIdx);
                bestSolutionCost = V;  % default is NaN if no solution by tVal

                % Find the last solution cost that occurred on or before tVal
                validCosts = sortedCosts(sortedTimes <= tVal);
                if ~isempty(validCosts)
                    bestSolutionCost = validCosts(end);
                end

                bestCostMatrix(tIdx, simIdx+1) = bestSolutionCost;
            end

        else
            % Empty file data
            bestCostMatrix(:, simIdx+1) = NaN(numTimes, 1);
        end
    else
        % File doesn't exist or is empty
        bestCostMatrix(:, simIdx+1) = NaN(numTimes, 1);
    end
end

% --------------------------
% Compute success rate
% --------------------------
successRate = (foundSolutionsCount / numSims) * 100;

% --------------------------
% Compute medians (only if we have at least one success)
% --------------------------
if foundSolutionsCount > 0
    medianInitialTime = median(initialSolutionTimes);
    medianInitialCost = median(initialSolutionCosts);
    medianFinalTime   = median(finalSolutionTimes);
    medianFinalCost   = median(finalSolutionCosts);
else
    % If no solutions in any run, set everything to NaN
    medianInitialTime = NaN;
    medianInitialCost = NaN;
    medianFinalTime   = NaN;
    medianFinalCost   = NaN;
end

% --------------------------
% Remove rows where all entries are NaN (no solutions at that time)
% (Optional: keeps bestCostMatrix/timeGrid smaller)
% --------------------------
validRows = any(~isnan(bestCostMatrix), 2);
bestCostMatrix = bestCostMatrix(validRows, :);
timeGrid = timeGrid(validRows);

% --------------------------
% Print results
% --------------------------
fprintf('Success rate: %.1f %%\n', successRate);
fprintf('Median time of initial solution: %.1f ms\n', medianInitialTime);
fprintf('Median cost of initial solution: %.4f\n', medianInitialCost);
fprintf('Median time of final solution: %.1f ms\n', medianFinalTime);
fprintf('Median cost of final solution: %.4f\n', medianFinalCost);

% --------------------------
% Save the data for later use
% --------------------------
save('SST_6DDI_house_results.mat', ...
     'bestCostMatrix', 'timeGrid', ...
     'initialSolutionTimes', 'initialSolutionCosts', ...
     'finalSolutionTimes', 'finalSolutionCosts', ...
     'successRate', ...
     'medianInitialTime', 'medianInitialCost', ...
     'medianFinalTime', 'medianFinalCost');
