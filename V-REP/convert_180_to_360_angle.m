function [ output_angle ] = convert_180_to_360_angle( angle )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
if angle < 0
    output_angle = 360 - abs(angle);
else 
    output_angle = angle;

end

