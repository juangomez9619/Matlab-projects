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
div=100;
d1=1120;
d2=355;
d4=185;
L3=650;
L5=750;
d8=425;
alfa=[-90;0;0;-90;0;-90;90;0];
theta1=[-160:320/div:160]';
theta2=[-115:230/div:115]';
theta3=[-135:270/div:135]';
theta4=[-266:532/div:266]';
theta5=[-200:200/div:0]';
theta6=[-266:532/div:266]';
base=[0;0;0;1];
figure

for i=1:div+1
A1=matrizDH(alfa(1),theta1(i)*0,0,d1);
o1=A1*base;
A2=matrizDH(alfa(2),0,0,d2);
T=A1*A2;
o2=T*base;
A3=matrizDH(alfa(3),theta2(i)*0.,L3,0);
T=T*A3;
o3=T*base;
A4=matrizDH(alfa(4),theta3(i)*0-90,0,-d4);
T=T*A4;
o4=T*base;
A5=matrizDH(alfa(5),0,0,L5);
T=T*A5;
o5=T*base;
A6=matrizDH(alfa(6),theta4(i)*0,0,0);
T=T*A6;
o6=T*base;
A7=matrizDH(alfa(7),theta5(i)*0.8,0,0);
T=T*A7;
o7=T*base;
A8=matrizDH(alfa(8),theta6(i)*0,0,d8);
T=T*A8;
o8=T*base;
grafica3d(o1,o2,o3,o4,o5,o6,o7,o8,A1)
pos_fin(i,:)=o8;
end
% 
% for i=1:div+1
% A1=matrizDH(-90,theta1(i)*0.,L3,0);
% o1=A1*base;
% A2=matrizDH(0,theta2(i)*0,0,L3);
% T=A1*A2;
% o2=T*base;
% A3=matrizDH(alfa(5),theta3(i)*0,L5,0);
% T=T*A3;
% o3=T*base;
% A4=matrizDH(alfa(6),theta4(i)*0,0,0);
% T=T*A4;
% o4=T*base;
% A5=matrizDH(alfa(7),theta5(i)*0.,0,0);
% T=T*A5;
% o5=T*base;
% A6=matrizDH(alfa(8),theta6(i)*0,0,d8);
% T=T*A6;
% o6=T*base;
% grafica3d2(o1,o2,o3,o4,o5,o6)
% end