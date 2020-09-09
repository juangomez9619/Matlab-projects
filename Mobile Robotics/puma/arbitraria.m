clear; 
close;
clc;

div=100;
%longitudes
d1=1120;
d2=355;
d4=185;
L3=650;
L5=600;
d8=250;

%ángulos

alfa=[-90;0;0;-90;0;-90;90;0];
theta_1=[-160:320/div:160]';
theta_2=[-115:230/div:115]';
theta_3=[-135:270/div:135]';
theta_4=[-266:532/div:266]';
%theta_5=[-200:200/div:0]';
theta_5=[-100:200/div:100]';
theta_6=[-266:532/div:266]';
base=[0;0;0;1];

%% matrices 
syms a d t alpha theta  phi
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tx=[1 0 0 a;0 1 0 0;0 0 1 0;0 0 0 1];
ty=[1 0 0 0;0 1 0 t;0 0 1 0;0 0 0 1];
tz=[1 0 0 0;0 1 0 0;0 0 1 d;0 0 0 1];
rx=[1 0 0 0;0 cosd(alpha) -sind(alpha) 0;0 sind(alpha) cosd(alpha) 0 ;0 0 0 1];
ry=[cosd(phi) 0 sind(phi) 0;0 1 0 0;-sind(phi) 0 cosd(phi) 0 ;0 0 0 1];
rz=[cosd(theta)  -sind(theta) 0 0;sind(theta) cosd(theta) 0 0 ;0 0 1 0 ;0 0 0 1];
%%
for i=1:div+1
    load('linksdata2.mat');
%% 1
d=1120; 
theta=180;
rz_a=[cosd(theta)  -sind(theta) 0 0;sind(theta) cosd(theta) 0 0 ;0 0 1 0 ;0 0 0 1];
tz=[1 0 0 0;0 1 0 0;0 0 1 d;0 0 0 1];
a=-355;
tx=[1 0 0 a;0 1 0 0;0 0 1 0;0 0 0 1];
theta=theta_1(i).*0;
rz=[cosd(theta)  -sind(theta) 0 0;sind(theta) cosd(theta) 0 0 ;0 0 1 0 ;0 0 0 1];
T1=rz*tx*tz*rz_a;
o1=T1*base;

%% 2
a=-185;
tx=[1 0 0 a;0 1 0 0;0 0 1 0;0 0 0 1];
t=-650;
ty=[1 0 0 0;0 1 0 t;0 0 1 0;0 0 0 1];
alpha=theta_2(i);
rx=[1 0 0 0;0 cosd(alpha) -sind(alpha) 0;0 sind(alpha) cosd(alpha) 0 ;0 0 0 1];

T2=rx*ty*tx;
o2=T1*T2*base;

 %% 3
 t=-600;
 ty=[1 0 0 0;0 1 0 t;0 0 1 0;0 0 0 1];
 alpha=theta_3(i).*0-45;
 rx=[1 0 0 0;0 cosd(alpha) -sind(alpha) 0;0 sind(alpha) cosd(alpha) 0 ;0 0 0 1];
 T3=rx*ty;
 o3=T1*T2*T3*base;

 %% 4
 phi=theta_4(i)*0;
 ry=[cosd(phi) 0 sind(phi) 0;0 1 0 0;-sind(phi) 0 cosd(phi) 0 ;0 0 0 1];
 T4=ry;
 o4=T1*T2*T3*T4*base;
 
 %% 5 
 alpha=theta_5(i)*0;
 rx=[1 0 0 0;0 cosd(alpha) -sind(alpha) 0;0 sind(alpha) cosd(alpha) 0 ;0 0 0 1];
 T5=rx;
 
 o5=T1*T2*T3*T4*T5*base;
 
 %% 6
 phi=theta_6(i)*0;
 ry=[cosd(phi) 0 sind(phi) 0;0 1 0 0;-sind(phi) 0 cosd(phi) 0 ;0 0 0 1];
 t=-250;
 ty=[1 0 0 0;0 1 0 t;0 0 1 0;0 0 0 1];

 T6=ry*ty;
 o6=T1*T2*T3*T4*T5*T6*base;
 
 
 
%% gráfica
clf
 %o3(1) o4(1) o5(1)
 xp=[0 o1(1) o2(1) o3(1) o4(1) o5(1) o6(1)];
 yp=[0 o1(2) o2(2) o3(2) o4(2) o5(2) o6(2)];
 zp=[0 o1(3) o2(3) o3(3) o4(3) o5(3) o6(3)];
   plot3(0,0,0,'o','MarkerSize',10,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
  hold on; grid on;  xlabel('X'); ylabel('Y'); zlabel('Z');
%
  plot3(xp,yp,zp,'-','LineWidth',3,'Color','r');

plot3(o1(1),o1(2),o1(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
plot3(o2(1),o2(2),o2(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
plot3(o3(1),o3(2),o3(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
plot3(o4(1),o4(2),o4(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
plot3(o5(1),o5(2),o5(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
plot3(o6(1),o6(2),o6(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
    set(gca,...
             'Xlim',[-1500 1500],...
             'Ylim',[-1500 1500],...
             'Zlim',[-250 2700])


 pause(0.02);
end