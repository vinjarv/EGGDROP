%% Clear workspace and list available ports
% Experiment runs on arduino, this file starts and logs the output (u) and
% response (y) at each timestep (t)
clear; clc; close all;
ports = serialportlist
%% Open serial and start writing to file
s = serial(ports(1), 'BaudRate',115200, 'Terminator',"LF");
fopen(s);
fname = "experiment1.csv";
f = fopen(fname, 'w');

fprintf(f, "%s\r\n", "u, y, t");
pause(1);
try
    % print line to start experiment
    fprintf(s, "%s", " ");
    data = "\r\n";
    % read until empty string and append to file
    while (data ~= "")
        data = fgetl(s);
        fprintf(f, "%s", data);
    end
catch
    % Catch errors to ensure file gets closed properly
end

try
fclose(f);
catch
end
fclose(s);
clear s
%% Read data from file to workspace
data = readtable(fname);
plot(data.t, data.y)
