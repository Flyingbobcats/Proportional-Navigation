function [Rtm,Vcl,lam,lamd,mw] = PN(m,t)

N = 10;
%X and Y Ranges between missile and target
Rtmx = t.x-m.x;
Rtmy = t.y-m.y;
Rtm = sqrt(Rtmx^2+Rtmy^2);

%Relative velocity components
Vtmx = t.vx - m.vx;
Vtmy = t.vy - m.vy;

%Closing Velocity
Vcl  = -(Rtmx*Vtmx+Rtmy*Vtmy)/Rtm;

%LOS angle between missile and pseudo target
lam  = atan2(Rtmy,Rtmx);

%LOS rate between missile and pseudo target
lamd = (Rtmx*Vtmy-Rtmy*Vtmx)/(Rtm^2);

%Calculate commanded acceleration
mw = N*lamd;
end

