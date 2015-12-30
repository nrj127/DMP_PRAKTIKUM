clc 
clear all
close all

load xtraj.mat
load straj.mat
load vtraj.mat

axis equal
plot(xtraj(1,:), xtraj(2,:),'*')
xlabel('x'); ylabel('y');
title('Position')
%xlim([-0.5, 0.5]); ylim([-0.5, 0.5])

figure
xx = 1:length(vtraj);
plot(xx, vtraj(1,:),xx, vtraj(2,:),xx, vtraj(3,:))
title('Velocity')
legend('x', 'y', 'theta');
xlabel('timestep'); ylabel('m/s or rad/s')

figure
plot(xx, straj(1,:),xx, straj(2,:),xx, straj(3,:))
title('Clock Signal s')
xlabel('timestep'); ylabel('s')