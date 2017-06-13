function [cx,cy,Radius] = linearuncertainty(t,Rtm,Rinit,Radius,dr,del_closing)
%outputs:
% cx - Circle xs
% cy - Circle ys

%Inputs:
% t   - Target instance
% Rtm - Range between real target and missile
% m   - slope of the line

%Assuming Rmin = 0 for simplification
ang = 0:pi/20:2*pi;

R = Rinit-dr*del_closing;
cx = R*cos(ang)+t.x;
cy = R*sin(ang)+t.y;

end

