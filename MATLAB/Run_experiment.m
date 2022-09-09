%% Clear workspace and list available ports
% Experiment runs on arduino, this file starts and logs the output (u) and
% response (y) at each timestep (t)
clear; clc; close all;
ports = serialportlist

%% Open serial and start writing to file
s = serialport(ports(1), 115200);
fname = "experiment1.csv";

f = fopen(fname, 'w');
fprintf(f, "%s\r\n", "u, y, t");
pause(2)
try
    % print line to start experiment
    writeline(s, "")
    configureTerminator(s, "CR/LF")
    data = "";
    % read until empty string and append to file
    while (data ~= "\r\n")
        data = readline(s);
        fprintf(f, "%s\r\n", data);
    end
catch
    % Catch errors to ensure file gets closed properly
end
fclose(f);

%% Read data from file to workspace
data = readtable(fname)
