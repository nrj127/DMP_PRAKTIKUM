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


figure


load  /home/hwadong/cotesys-lwr4+/dmp_praktikum/data/rec_taskparms.mat
load  /home/hwadong/cotesys-lwr4+/dmp_praktikum/data/rec_end.mat

plot( rec_taskparms(1,:), rec_taskparms(2,:),'*','Color','r');
hold on
plot( rec_end(1,:), rec_end(2,:),'b','LineWidth',2);
axis equal

plot(xtraj(1,:), xtraj(2,:),'g','LineWidth',2);

ax = gca

plot(rec_taskparms(1,1), rec_taskparms(2,1), 'd','MarkerSize',10,'MarkerFaceColor','y')

%plot(rec_taskparms(1,size(rec_taskparms)), rec_taskparms(2,size(rec_taskparms)), 'o','MarkerSize',10,'MarkerFaceColor','b');
legend('tracked task parameters','recorded motion on kuka','output of motion primitives','starting point');

for i=200:40:size(rec_end,2)
    plotline( [rec_end(1,i) rec_end(2,i)],rec_end(3,i)+pi/4,0.04,1,[0 0 1] , ax); 
end

for i=200:40:size(xtraj,2)
    plotline( [xtraj(1,i) xtraj(2,i)],xtraj(3,i)+pi/4,0.04,1,[0 1 0] , ax); 
end

xlabel('x')
ylabel('y')

title('GMR badly moved box by Frederik :-(')
