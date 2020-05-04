%% Initialise Workspace
clc;
clear all;
%%

controller;
x_init = [0 0.5 0 0]';
tspan = [0:0.01:10];


[t, x] = ode45(@nl_eq_function, tspan, x_init);
plot(t, x);