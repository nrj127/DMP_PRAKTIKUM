clc 
clear all
close all

load xtraj.mat
load straj.mat
load vtraj.mat


plot(xtraj(1,:), xtraj(2,:),'*')
axis equal

figure
xx = 1:length(vtraj);
plot(xx, vtraj(1,:),xx, vtraj(2,:),xx, vtraj(3,:))
title vtraj

figure
plot(xx, straj(1,:),xx, straj(2,:),xx, straj(3,:))
title straj