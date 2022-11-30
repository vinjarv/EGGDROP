%% Read data from file to workspace
clear; clc; close all

%% Read data from file to workspace
fname = "experiment1.csv";
data = readtable(fname);
data.y = data.y * 2*pi*4 / 2048;
figure(1)
plot(data.t, data.y, data.t, data.u);
legend(["Motor position (mm)", "PWM output (\pm255)"], 'Location', 'best');
title("Experiment data");
xlabel("Sample");
ylabel("Value");

dt = (data.t(2)-data.t(1))*1e-3
testdata = iddata(data.y, data.u, dt)
