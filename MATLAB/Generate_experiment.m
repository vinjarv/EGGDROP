clear; clc; close all;

%% Generate data
fs = 200; % Samples/s

% Automatically generate C++ array
startStr = "const int PROGMEM testdata[] = {";
endStr = "};";

% Pause - 0V
v0 = zeros(1, 0.5*fs);

% 1G acceleration 0.25s, -4G acc. 0.1s x4
% 5s range
t1 = linspace(0, 5, fs*5);

% 1 Hz with voltage ramp 1V -> 9V, 5s
v1 = 1 + 8/5*t1;

% 9V frequency sweep from 0.1Hz -> 5Hz, 5s
v2 = 9*chirp(t1, 0.1, 5, 5);

% 0.25s range
t2 = linspace(0, 0.25, 0.25*fs);
% 1) Ramp 4 -> 7V over 0.25s
v31 = 4 + (7-4)/0.25 * t2;
% 2) -12V brake for 0.1s
v32 = -12 * ones(1, 0.1*fs);
% Create drop/brake sequence
v3 = [v31, v32, v0];

% Assemble output
out = [v1, -v1, v0, v2, v0];
out = [out, v3, v3, -v3];
out = out * 255/12; % Convert voltage to PWM value
out = round(out, 0); % To integer
out = cast(out, 'int16');
plot(out)

%% Write to file
f = fopen("testdata.h", 'w');
fprintf(f, '%s', startStr);
for i = 1:length(out)
    if i < length(out)
        fprintf(f, '%d,', out(i));
    else
        fprintf(f, '%d', out(i));
    end
end
fprintf(f, '%s', endStr);
fclose(f);
