eclc 
clear all
close all

% load joints_xtraj.mat
% load joints_straj.mat
% load joints_vtraj.mat

load xtraj.mat
load straj.mat
load vtraj.mat

axis equal;

x_vec = 1:1:length(xtraj);

plot(x_vec, xtraj');
xlabel('x'); ylabel('y');
title('Position');
legend('1', '2', '3', '4', '5', '6', '7');

figure;
plot(x_vec, vtraj');

figure;
plot(x_vec, straj');


xtraj(:,1)


