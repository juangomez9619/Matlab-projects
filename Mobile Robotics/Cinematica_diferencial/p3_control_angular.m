%clear all;close all;clc;
clc;
R = 0.03;
L = 0.15;
global grafica_carro;
global trazado_x;
global trazado_y;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%posición inicial del carro
x(1) = 0;
trazado_x = x(1);
y(1) = 0;
trazado_y = y(1);

dt = 0.114/10;
T= dt; % periodo de muestreo.



%inicialización de variables para la implementación del control y motor
%control motor derecho
error_motor_der_km1 = 0;
u_motor_der_km1 = 0;
%motor derecho
u_motor_der_km2 = 0;
wd_km1 = 0; %salida del motor
wd_km2 = 0; %salida del motor

%control motor izquierdo
error_motor_izq_km1 = 0;
u_motor_izq_km1 = 0;
%control motor izquierdo
u_motor_izq_km2 = 0;
wi_km1 = 0; %salida del motor
wi_km2 = 0; %salida del motor

% control angular
error_angular_km1 = 0;
error_angular_km2 = 0;
wp_km1 = 0;
%orientación inicial
theta(1) = -150;
% constantes del controlador angular
Ki = 1.4;%integral
Kd = 0.01;%derivativa
Kp = 0.3;%proporcional

b0 = (Kp*T+Ki*T^2+Kd)/(T);
b1 = -(Kp*T+2*Kd)/(T);
b2 =  Kd/T;


%punto deseado
x_goal = .5;
y_goal = -.9;


% cinemática inversa
 t=0:T:170*T;
 vP_ref = 1*ones(1,length(t));
vP_ref(length(t)-25:length(t)) = 0;



figure(1), hold on;
figure(1), grid on;
figure(1),plot(x_goal,y_goal,'*m','LineWidth',3);
figure(1),axis([-2 2 -2 2]);
figure(1),carrito(x(1),y(1),theta(1));



for i = 1:length(t)
    if(i==1)
    set(grafica_carro,'Visible','off');
    end
 
    
 theta_goal(i) =(180/pi)*atan2((y_goal - y(i)),(x_goal - x(i))) ; %theta goal   
 %implementación del control angular
 %error angular
 
 error_angular(i) = theta_goal(i) - theta(i);
 if(error_angular(i)<180)
 error_angular(i) = error_angular(i)+360;
 end
  if(error_angular(i)>180)
 error_angular(i) = error_angular(i)-360;
 end
 
 wP(i) = b0*error_angular(i)+b1*error_angular_km1+b2*error_angular_km2+wp_km1;
  %cota superior para velocidad angular
 if(wP(i)) >= 5
 wP(i) = 5;
 end
 %cota inferior para velocidad angular
 if(wP(i))<= -5
 wP(i) = -5;
 end
 
 
 
 %actualizaciones
 error_angular_km2 = error_angular_km1;
 error_angular_km1 =  error_angular(i);
 wp_km1 = wP(i);
 
 % velocidad angulares de referencia para el controlador
 Wd_ref(i) = 1/R*vP_ref(i) + L/(2*R)*wP(i);
 Wi_ref(i) = 1/R*vP_ref(i) - L/(2*R)*wP(i);
 
 % implementanción de control y motor
 %velocidad del motor derecho:
 wd(i) = 0.1257*u_motor_der_km1+0.04716*u_motor_der_km2+...
         0.854*wd_km1-0.04605*wd_km2;
 %señal de error motor derecho:
 error_motor_der(i) =  Wd_ref(i) -  wd(i);
 %controlador motor derecho
 u_motor_der(i) = 0.76595*error_motor_der(i)-0.5551*error_motor_der_km1+...
                  u_motor_der_km1;
 
 %actualizaciones
 u_motor_der_km2 = u_motor_der_km1;
 u_motor_der_km1 = u_motor_der(i);
 wd_km2 = wd_km1;
 wd_km1 = wd(i);
 error_motor_der_km1 = error_motor_der(i);
 
 %velocidad del motor izquierdo
 
  wi(i) = 0.08311*u_motor_izq_km1+0.04198*u_motor_izq_km2+...
         0.9765*wi_km1-0.1237*wi_km2;
 % señal de error motor izquierdo
 error_motor_izq(i) =  Wi_ref(i) -  wi(i);
 u_motor_izq(i) = 1.024*error_motor_izq(i)-0.8005*error_motor_izq_km1+...
                  u_motor_izq_km1;
 
 % actualizaciones
 
 u_motor_izq_km2 = u_motor_izq_km1;
 u_motor_izq_km1 = u_motor_izq(i);
 wi_km2 = wi_km1;
 wi_km1 = wi(i);
 error_motor_izq_km1 = error_motor_izq(i);
 
 % 
 
 
 %cálculo velocidad lineal y angular 
 
 vP(i) = (R/2)*(wi(i)+wd(i));
 wP(i) = (R/L)*(wd(i)-wi(i));

 
 d_x(i) = vP(i)*cosd(theta(i));
 d_y(i) = vP(i)*sind(theta(i));
 d_theta(i) = wP(i)*180/pi;

 x(i+1) = x(i) + dt*d_x(i);
 y(i+1) = y(i) + dt*d_y(i);
 theta(i+1) = theta(i) + dt*d_theta(i);

carrito(x(i+1),y(i+1),theta(i+1));
plot(trazado_x,trazado_y,'b','LineWidth',2)
if(i<length(t))
set(grafica_carro,'Visible','off');
end
end
figure(2), hold on;
figure(2), plot(t,wP,'r','LineWidth',2);
figure(2), plot(t,vP,'k','LineWidth',2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function carrito(x,y,theta)
global grafica_carro;
global trazado_x;
global trazado_y;
 px = 0.01*[0   0    10  15   15  10  0  0];
 py = 0.01*[0 -7.5 -7.5 -2.5 2.5 7.5 7.5 0];

 rot = [cosd(theta) -sind(theta);
       sind(theta)  cosd(theta)];
   
 p_rot = rot*[px;py];

 px_rot = x + p_rot(1,:);
 trazado_x = [trazado_x px_rot(1)];
 py_rot = y + p_rot(2,:);
  trazado_y = [trazado_y py_rot(1)];
 grafica_carro = plot([px_rot(1) px_rot(2) px_rot(3) px_rot(4) px_rot(5) px_rot(6) px_rot(7) px_rot(8)],...
                [py_rot(1) py_rot(2) py_rot(3) py_rot(4) py_rot(5) py_rot(6) py_rot(7) py_rot(8)],...
                'r','LineWidth',2);
 figure(1),grafica_carro;

end

