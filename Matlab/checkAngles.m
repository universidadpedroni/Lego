function theta_arc = checkAngles(theta_arc, type)
if(theta_arc < 0 && type == 'L')
    theta_arc = theta_arc + 2*pi;
elseif(theta_arc > 0 && type == 'R')
    theta_arc = theta_arc - 2*pi;
end
