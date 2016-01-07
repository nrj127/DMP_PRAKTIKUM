function plotline( center,angle,linelength,width,shade, ax )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% example:
% for i=200:400:size(Data,2)
%     plotline( [Data(1,i) Data(2,i)],Data(3,i)+pi/4,0.04,3,[0 0 1] ) 
% end

   
    r = linelength/2;
    
    x1 = center(1) + r * cos(angle);
    y1 = center(2) + r * sin(angle);
    
    x2 = center(1) + r * cos(angle+pi);
    y2 = center(2) + r * sin(angle+pi);
    
    plot(ax, [x1 x2],[y1 y2],'color',shade,'linewidth',width)

end

