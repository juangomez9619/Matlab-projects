clc;clear all; close all;
L1=20;
L2=15;
div=50;
alfa=[0;0];
theta=[0:45/div:45;0:45/div:45];
a=[L1;L2];
d=[0;0];
base=[0;0;0;1];
figure
for i=1:div+1
A1=matrizDH(alfa(1),theta(1,i),a(1),d(1));
o1=A1*base;
A2=matrizDH(alfa(2),theta(2,i),a(2),d(2));
T=A1*A2;
o2=T*base;
grafica2d(o1,o2)
end

