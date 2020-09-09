
clc;close;clear;
syms theta d a alpha theta_4 theta_5 theta_6 d6


tx=[1 0 0 a;0 1 0 0;0 0 1 0;0 0 0 1];
tz=[1 0 0 0;0 1 0 0;0 0 1 d;0 0 0 1];

rx=[1 0 0 0;0 cos(alpha) -sin(alpha) 0;0 sin(alpha) cos(alpha) 0 ;0 0 0 1];

rz=[cos(theta)  -sin(theta) 0 0;sin(theta) cos(theta) 0 0 ;0 0 1 0 ;0 0 0 1];

A= simplify(rz*tz*tx*rx);

alpha=-90;
a=0;
d=0;
theta=theta_4;

A4=...
[ cosd(theta), -cosd(alpha)*sind(theta),  sind(alpha)*sind(theta), a*cosd(theta);
 sind(theta),  cosd(alpha)*cosd(theta), -sind(alpha)*cosd(theta), a*sind(theta);
          0,             sind(alpha),             cosd(alpha),            d;
          0,                      0,                      0,            1];
      
alpha=90;
a=0;
d=0;
theta=theta_5;

A5=...
[ cosd(theta), -cosd(alpha)*sind(theta),  sind(alpha)*sind(theta), a*cosd(theta);
 sind(theta),  cosd(alpha)*cosd(theta), -sind(alpha)*cosd(theta), a*sind(theta);
          0,             sind(alpha),             cosd(alpha),            d;
          0,                      0,                      0,            1];
      
alpha=0;
a=0;
d=d6;
theta=theta_6;

A6=...
[ cos(theta), -cos(alpha)*sin(theta),  sin(alpha)*sin(theta), a*cos(theta);
 sin(theta),  cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta);
          0,             sin(alpha),             cos(alpha),            d;
          0,                      0,                      0,            1]
