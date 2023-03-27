function [Range] = NewRange(sensor,index)

 Range = sum(sensor(1,index))/length(index);