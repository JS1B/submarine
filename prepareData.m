function data = prepareData(filename, columns_to_split_on)
% DATA = PREPAREDATA(FILENAME) ertracts and prepares the data from robot
% model csv file
%
% DATA = PREPAREDATA(FILENAME) ertracts and prepares the data from robot
% model csv file
%
% Example
% data = prepareData("model_data.csv")
%
% See also IMPORTCSV, READTABLE

T = importCsv(filename, [2, Inf]);
T(1:3,:) = [];

% Initialize a cell array to store the subset s
data = cell(length(unique_values), 1);
subset = cell(length(columns_to_split_on), 1);
for j = length(columns_to_split_on)
    % Unique values in the column to split on
    unique_values = unique(T.(columns_to_split_on(j)));

    % Loop through the unique values and create a new table for each value
    
    for i = 1:length(unique_values)
        % Subset the original table based on the current unique value
        current_value = unique_values(i);
        subset = T(T.(columns_to_split_on(j)) == current_value,:);
        
        % Save the subset in the data cell array
        data{i} = subset;
    end
end

end

