clc; clear; close all
syms t1 t2

% Init cond.
s0 = 0.205;
v0 = 0;
a0 = -1.35*9.81;

% Accel
a1 = 10*9.81;

% End cond.
s2 = 0.005;
v2 = -0.150;

% Equations of motion
% s1 = s0 + v0*t1 + 1/2*a0*t1^2
% s2 = s1 + v1*t2 + 1/2*a1*t2^2
% v1 = v0 + a0*t1
% v2 = v1 + a1*t2

eqs = [
0 == v0 - v2 + a0*t1 + a1*t2; %eq1
0 == s0 - s2 + v0*t1 + 1/2*a0*t1^2 + (v0 + a0*t1)*t2 + 1/2*a1*t2^2 %eq2
]

a = vpasolve(eqs);
t1 = max(a.t1)
t2 = max(a.t2)
t_tot = t1 + t2

v1 = v0 + a0*t1
v2 = v1 + a1*t2

t = linspace(0, t_tot+abs(s2/v2 * 1.1), 100);
y = zeros(100,1);
for i = 1:length(t)
    y(i) = 0;
    if (0 <= t(i) && t(i) < t1)
        y(i) = s0  + v0*t(i) + 1/2*a0*t(i)^2;
    elseif (t1 <= t(i) && t(i) <= t_tot)
        y(i) = (s0 + v0*t1 + 1/2*a0*t1^2) + a0*t1*(t(i)-t1) + 1/2*a1*(t(i)-t1)^2;
    else
        y(i) = (s0 + v0*t1 + 1/2*a0*t1^2) + a0*t1*(t2) + 1/2*a1*(t2)^2 + v2*(t(i)-t_tot);
    end
end

plot(t, y)
grid()
title("Eggdrop motion profile")
xlabel("Time (s)")
ylabel("Height (m)")
gnd = refline(0, 0);
gnd.Color = "k";
tl0 = xline(0, "-", "Freefall");
tl1 = xline(double(t1), "-", "Braking");
tl2 = xline(double(t1+t2), "-", "Constant vel.");
