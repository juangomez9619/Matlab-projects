%parámetros iniciales
clear all;close all;clc;
R = 0.03;
L = 0.15;

x(1) = 0;
y(1) = 0;
theta(1) = 0;
dt = 0.114/10;

figure(1),axis ([-1 1 -1 1])
carrito(x(1),y(1),theta(1));

% cinemática directa
%velocidad de cada motor
wL = 0*ones(1,100);
wR = 30*ones(1,100);

vP = (R/2)*(wR + wL);
wP = (R/L)*(wR - wL);

% cinemática inversa
% % 
% % R = 3;
% % L = 15;
 vP = 0*ones(1,10);
 wP = 10*ones(1,10);
% 
wR = 1/R*vP + L/(2*R)*wP;
wL = 1/R*vP - L/(2*R)*wP;

%movimiento del carro
for i = 1:length(wL)
 d_x(i) = vP(i)*cosd(theta(i));
 d_y(i) = vP(i)*sind(theta(i));
 d_theta(i) = wP(i)*180/pi;

 x(i+1) = x(i) + dt*d_x(i);
 y(i+1) = y(i) + dt*d_y(i);
 theta(i+1) = theta(i) + dt*d_theta(i);
 carrito(x(i+1),y(i+1),theta(i+1));

end

