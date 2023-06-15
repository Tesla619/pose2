close all; clear all; clc;

% create a tcpip object
t = tcpip('localhost', 12345); clc;

% open the connection
fopen(t); 

% main code
while true
    % read some data (read until terminator)
    data = fscanf(t);

    % check if data is not empty
    if ~isempty(data)
        % use data here, maybe call your function
        fprintf('Received data: %s\n', data);
        % your_function(str2double(data));
    end

    if isempty(data)
        fprintf('Closing Connection...');
        fclose(t);
    end
end

% close the connection
fclose(t);