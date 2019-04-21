clear;clc;
file = load("data6.mat");
data = file.data;
real_range = 221:500;
phi_real = data(:,2)/180*pi;
plot(phi_real)
file = load("sim5v.mat");
sim_range = 1:280;
phi_sim = file.out.phi_sim(sim_range,2);

%plot([phi_real,phi_sim]),grid;