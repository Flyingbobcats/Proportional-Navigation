%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PNwithLinearUncertainty.m
%
% Program runs many intercept and follow scenarios for varrying changing
% uncertainty rate (as a function of closing distance) and for various
% initial uncertainty radii. 
%
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
close all
close all

dr = 0:0.005:0.4;
RinitRatio = [0.1,0.2,0.3,0.4,0.5];

for q = 1:length(RinitRatio)
parfor k = 1:length(dr)
     
%=================
mx   = [];
my   = [];
tx   = [];
ty   = [];
ptx  = [];
pty  = [];
mw   = [];
X    = [];
Y    = [];
Mx   = [];
My   = [];
RTM  = [];
VTM  = [];
%=================

%================= Simulation Constants
ang = 0:pi/100:2*pi;     %Angle array for uncertainty circle
dt = .01;                %Time Step

%Initalize the vehicles
m  = uav;        
t  = uav;

%Inital Target Setup
t.angle = deg2rad(180);
t.x     = 500;
t.y     = t.x;
t.v     = 10;
t.vx    = -t.v*cos(t.angle);
t.vy    =  t.v*sin(t.angle);

%Initial Pseudo Target Setup
pt     = t;
pt.x   = 0;
pt.y   = 0;
pt.v   = 0;
pt.vx  = 0;
pt.vy  = 0;

%Initial Missile Setup
m.angle = deg2rad(90);
m.x  = 0;
m.y  = 0;
m.v  = t.v;
m.vx = m.v*cos(m.angle);
m.vy = m.v*sin(m.angle);
m.w  = 0;

%Determine state from initial conditions. Equations are described in more
%detail in the while loop
[Rtm,Vcl,lam,lamd,m.w] = PN(m,pt);
R = sqrt((t.y-m.y)^2+(t.x-m.x)^2);

%Store initial state in non volatile memory
mx  = [mx,m.x];
my  = [my,m.y];
tx  = [tx,t.x];
ty  = [ty,t.y];
ptx = [ptx,pt.x];
pty = [pty,pt.y];
mw  = [mw,m.w];
VTM = [VTM,Vcl];
RTM = [RTM,R];


%Uncertainty Parameters
% dr = 0.1;
 Rinit = t.x*RinitRatio(q);
b = Rinit-(R*dr(k));

T = 0;

while T<60
    
    %Update target position
    t.x = t.x + t.v*cos(t.angle)*dt;
    t.y = t.y + t.v*sin(t.angle)*dt;
    
    %Determine Range from target
    R = sqrt((t.y-m.y)^2+(t.x-m.x)^2);
    Radius = dr(k)*R+b;
    if Radius < 0
        Radius =0;
    end
    cx = Radius*cos(ang)+t.x;
    cy = Radius*sin(ang)+t.y;
    cxs = [];
    cys = [];
    
    for j = 1:length(cx)
        if cx(j)>=m.x
            if cy(j)<t.y
                cxs = [cxs,cx(j)];
                cys = [cys,cy(j)];
            end
        end
    end
    
    
    if m.mode == 'w'
       
        if length(cys)>1
            if min(cys)>0
                pt.y = min(cys);
                pt.x = pt.x;
            end
            if m.y<pt.y
                m.mode = 'i';
            else
                m.mode = 'w';
            end
        else
            m.mode = 'f';
        end
        
    elseif m.mode =='i'
        if length(cys)>1
            if min(cys)>0
                pt.y = min(cys);
                pt.x = pt.x;
            end
            if m.y<pt.y
                m.mode = 'i';
            else
                m.mode = 'w';
            end
        else
            m.mode = 'f';
        end
     
        
        %Set the pt to intercept the appropriate intercept point
        [Rtm,Vcl,lam,lamd,m.w] = PN(m,pt);
        if Rtm>0.5
        m.angleold = m.angle;
        m.angle = 0.5*(m.angleold+m.angle+m.w*dt);
        m.vx = m.v*cos(m.angle);
        m.vy = m.v*sin(m.angle);
        m.xold = m.x;
        m.yold = m.y;
        m.x = m.x+m.vx*dt;
        m.y = m.y+m.vy*dt;
        m.x = 0.5*(m.xold+m.x+m.vx*dt);
        m.y = 0.5*(m.yold+m.y+m.vy*dt);
        end
        
        
    elseif m.mode =='f'
        %overlay the pt on top of the target
        if t.x<0
        pt.x = t.x;
        pt.y = t.y;
        
        else
            pt.x = pt.x;
            pt.y = t.y;
        end
        [Rtm,Vcl,lam,lamd,m.w] = PN(m,pt);
        m.angleold = m.angle;
        m.angle = 0.5*(m.angleold+m.angle+m.w*dt);
        m.vx = m.v*cos(m.angle);
        m.vy = m.v*sin(m.angle);
        m.xold = m.x;
        m.yold = m.y;
        m.x = m.x+m.vx*dt;
        m.y = m.y+m.vy*dt;
        m.x = 0.5*(m.xold+m.x+m.vx*dt);
        m.y = 0.5*(m.yold+m.y+m.vy*dt);
        
    else
        error('Some kind of mode error');
    end
  
    
    
    T = T+dt; %Update Current Time 
    mx = [mx,m.x];
    my = [my,m.y];
    tx = [tx,t.x];
    ty = [ty,t.y];
    ptx = [ptx,pt.x];
    pty = [pty,pt.y];
    VTM = [VTM,Vcl];
 
%     plot(cxs,cys,'*g',m.x,m.y,'r*',t.x,t.y,'*b',[m.x,t.x],[m.y,t.y],'g',pt.x,pt.y,'*m',[m.x,pt.x],[m.y,pt.y],'--m',cx,cy,'b','Linewidth',1);
%     grid on
%     xlabel('x axis');
%     ylabel('y axis');
%     title('PN Guidance With PseudoTarget');
%     axis('equal');
%     axis([-500,500,0,600]);
%     str = num2str(sqrt((t.x-m.x)^2+(t.y-m.y)^2));
%     text(200,100,str);
%     pause(0.01);
end

followdist(q,k) = sqrt((t.x-m.x)^2+(t.y-m.y)^2);


end
    %Show completed percentage
    str = strcat(num2str(q/length(RinitRatio)*100),' % Complete');
    disp(str);
end

[a,b] = size(followdist);
figure
led = {};
for q=1:a
hold on
plot(dr,followdist(q,:),'Linewidth',2)
led{q} = num2str(RinitRatio(q));
end

xlabel('dr/dc');
ylabel('Following Distance [meters]');
title('Minimum Following Distances for Variable Initial Uncertainty Radius');
grid on
axis([0,0.5,0,300]);
legend(led)


