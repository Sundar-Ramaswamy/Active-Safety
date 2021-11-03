% This is a minimal set of checks for speed quality, you are welcome to add
% more tests or refine the one below

function [ok] = isGoodQuality(speed)

ok = true;

% if  all(speed==0) % vehicle is standing still
%     ok = false;
% end

if  (numel(speed(speed==0)) / numel(speed))>0.7  % proportion of speed is zero. Vehicle standing still or sensor error
    ok = false;
end 

if  (numel(speed(speed==-1)) / numel(speed))>0.7  % proportion of speed is minus 1. sensor error?
    ok = false;
end 

if  all(diff(speed)==0) % the vehicle travels at perfectly constant speed. Sensor error?
    ok = false;
end

end
