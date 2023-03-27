function [Range, Time] = interpolatePoints(ranges, times, step)

indx = 1;
indy = 1;

for i = 2:1:length(ranges)
    
    m = (ranges(1,i)-ranges(1,i-1))/(times(1,i)-times(1,i-1));
    c = ranges(i-1);
    
    for x = 0:step:(times(1,i)-times(1,i-1))
        lineFunction = m*x+c;
        Range(indy) = lineFunction;
        Time(indx) = times(i-1)+x;
        indx = indx+1;
        indy = indy+1;
    end
end