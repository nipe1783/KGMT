close all
clc
clear

% Read the CSV file
data = readmatrix('/home/nicolas/dev/research/KGMT/build/Data/pathCosts/pathCosts0/pathCosts.csv'); % Replace 'data.csv' with your actual file name

% Extract integers and costs
integers = data(1:2:end); % Every first value in each pair (x-axis)
costs = data(2:2:end);    % Every second value in each pair (y-axis)

% Find the index where costs start being zero
zero_idx = find(costs == 0, 1); % Find the first occurrence of zero

% If there are zeros, truncate the data
if ~isempty(zero_idx)
    integers = integers(1:zero_idx-1);
    costs = costs(1:zero_idx-1);
end

% Scatter plot
figure;
scatter(integers, costs, 'filled'); 
xlabel('Iterations');
ylabel('Cost');
grid on;
