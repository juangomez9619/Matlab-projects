clc;clear all; close all;
% div=25;
% d1=7;
% d2=[5:(10-5)/div:10]';
% d3=[5:(20-5)/div:20]';
% alfa=[0;-90;0];
% theta1=[0:360/div:360]';
% a=[0;0;0];
% base=[0;0;0;1];
% figure
% for i=1:div+1
% A1=matrizDH(alfa(1),theta1(i),a(1),d1);
% o1=A1*base;
% A2=matrizDH(alfa(2),0,a(2),d2(i));
% T=A1*A2;
% o2=T*base;
% A3=matrizDH(alfa(3),0,a(3),d3(i));
% T=T*A3;
% o3=T*base;
% grafica3d(o1,o2,o3)
% end
div=25;
d1=7;
d2=7;
L1=10;
L2=5;

alfa=[-90;0;90];
theta1=[0:360/div:360]';
theta2=0*[270:(360-270)/div:360]';
theta3=0*[270:(360-270)/div:360]';
base=[0;0;0;1];
figure

for i=1:div+1
A1=matrizDH(alfa(1),theta1(i),0,d1);
o1=A1*base;
A2=matrizDH(alfa(2),theta2(i)*0,L1,0);
T=A1*A2;
o2=T*base;
A3=matrizDH(alfa(3),theta3(i),L2,d2);
T=T*A3;
o3=T*base;
grafica3d(o1,o2,o3)
end
