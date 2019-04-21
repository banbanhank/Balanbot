clear;clc;
file = load("data8.mat");
data = file.data;
real_range = 160:439;
phi_real = data(real_range,2)/180*pi;
file = load("sim1v.mat");
sim_range = 1:280;
phi_sim = file.out.phi_sim(sim_range,2);

plot([phi_real,phi_sim]),grid;