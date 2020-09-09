clc;close;clear;


theta_1=16;
theta_2=0;
theta_3=0;
theta_4=0;
theta_5=0;
theta_6=0;
syms a alpha theta d
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tx=[1 0 0 a;0 1 0 0;0 0 1 0;0 0 0 1];
tz=[1 0 0 0;0 1 0 0;0 0 1 d;0 0 0 1];
rx=[1 0 0 0;0 cosd(alpha) -sind(alpha) 0;0 sind(alpha) cosd(alpha) 0 ;0 0 0 1];
rz=[cosd(theta)  -sind(theta) 0 0;sind(theta) cosd(theta) 0 0 ;0 0 1 0 ;0 0 0 1];
A= simplify(rz*tz*tx*rx);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% 1
alpha=-90;
a=0;
d=1120;
theta=theta_1;

A1=...
[ cosd(theta), -cosd(alpha)*sind(theta),  sind(alpha)*sind(theta), a*cosd(theta);
 sind(theta),  cosd(alpha)*cosd(theta), -sind(alpha)*cosd(theta), a*sind(theta);
          0,             sind(alpha),             cosd(alpha),            d;
          0,                      0,                      0,            1];


%% 2      
alpha=0;
a=650;
d=0;
theta=theta_2;

A2=...
[ cosd(theta), -cosd(alpha)*sind(theta),  sind(alpha)*sind(theta), a*cosd(theta);
 sind(theta),  cosd(alpha)*cosd(theta), -sind(alpha)*cosd(theta), a*sind(theta);
          0,             sind(alpha),             cosd(alpha),            d;
          0,                      0,                      0,            1];

      
%% 3      
alpha=90;
a=600;
d=0;
theta=theta_3;

A3=...
[ cosd(theta), -cosd(alpha)*sind(theta),  sind(alpha)*sind(theta), a*cosd(theta);
 sind(theta),  cosd(alpha)*cosd(theta), -sind(alpha)*cosd(theta), a*sind(theta);
          0,             sind(alpha),             cosd(alpha),            d;
          0,                      0,                      0,            1];      
   
      
      
%% 4      
alpha=-90;
a=0;
d=0;
theta=theta_4;

A4=...
[ cosd(theta), -cosd(alpha)*sind(theta),  sind(alpha)*sind(theta), a*cosd(theta);
 sind(theta),  cosd(alpha)*cosd(theta), -sind(alpha)*cosd(theta), a*sind(theta);
          0,             sind(alpha),             cosd(alpha),            d;
          0,                      0,                      0,            1];

%% 5 
alpha=90;
a=0;
d=0;
theta=theta_5;

A5=...
[ cosd(theta), -cosd(alpha)*sind(theta),  sind(alpha)*sind(theta), a*cosd(theta);
 sind(theta),  cosd(alpha)*cosd(theta), -sind(alpha)*cosd(theta), a*sind(theta);
          0,             sind(alpha),             cosd(alpha),            d;
          0,                      0,                      0,            1];      

      
%% 6
alpha=0;
a=0;
d=190;
theta=theta_6;

A6=...
[ cosd(theta), -cosd(alpha)*sind(theta),  sind(alpha)*sind(theta), a*cosd(theta);
 sind(theta),  cosd(alpha)*cosd(theta), -sind(alpha)*cosd(theta), a*sind(theta);
          0,             sind(alpha),             cosd(alpha),            d;
          0,                      0,                      0,            1];       

%%

A1*A2*A3*A4*A5*A6

