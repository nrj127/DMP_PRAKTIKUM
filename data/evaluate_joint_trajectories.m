clear all;
close all;


for n=1:1:16
    tp = importdata(['affan_trajectories/DemonstratedTaskParams' num2str(n) '.txt']);
    c = importdata(['joint_trajectories/DemonstratedTrajectory_Cart' num2str(n) '.txt']);
    plot(c(:,4), c(:,8));
    axis equal;
    hold on;
    plot(tp(1), tp(2), 'd');
    title(['Trajectory No:' num2str(n)]);
    xlim([-0.9 -0.1]);
    %hold off;
    %k = waitforbuttonpress;
end


