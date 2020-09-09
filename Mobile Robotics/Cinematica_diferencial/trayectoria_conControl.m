clear all;close all;clc;
clc;
R = 0.03;
L = 0.15;
global grafica_carro;
global trazado_x;
global trazado_y;


x_dibujo = [0  0.2    0.4 0.6 0.65 0.55 0.62 0.65 0.62 0.55 0.42 0.25 0.1  0.1 0.1 -0.1 -0.1 -0.32 -0.5 -0.6 -0.625 -0.61 -0.59 -0.57 -0.57 -0.62 -0.7 -0.71 -0.69 -0.62 -0.51 -0.4 -0.2 0];
y_dibujo = [-1 -0.98 -0.91  -0.6 -0.3 0 0.14 0.3 0.44 0.49 0.41 0.44 0.5  0.7 0.8 0.8 0.7 0.8 0.84 0.83 0.75 0.62 0.4 0.24 0.105 -0.05 -0.18 -0.3 -0.5 -0.65 -0.79 -0.9 -0.98 -1];
punto = 2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%posición inicial del carro
x(1) = 0;
trazado_x = x(1);
y(1) = -1;
trazado_y = y(1);

dt = 0.114/10;
T= dt; % periodo de muestreo.



%%inicialización de variables para la implementación del control y motor
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


% control de velocidad
vp_km1 = 0;
d_km1 = 0;
d_km2 = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%
%orientación inicial
theta(1) = 0;
% constantes del controlador angular
Ki = 1.4;%integral
Kd = 0.01;%derivativa
Kp = 0.3;%proporcional

b0 = (Kp*T+Ki*T^2+Kd)/(T);
b1 = -(Kp*T+2*Kd)/(T);
b2 =  Kd/T;

% constantes del controlador velocidad
Ki_v = 0.0;%integral
Kd_v = 0.00;%derivativa
Kp_v = 2;%proporcional

f0 = (Kp_v*T+Ki_v*T^2+Kd_v)/(T);
f1 = -(Kp_v*T+2*Kd_v)/(T);
f2 =  Kd_v/T;

%punto deseado
x_goal = x_dibujo(2);
y_goal = y_dibujo(2);


% cinemática inversa
 t=0:T:22000*T;

figure(1), hold on;
figure(1), grid on;
figure(1),axis([-1 1 -1 1]);
figure(1),carrito(x(1),y(1),theta(1));
figure(1), plot(x_dibujo,y_dibujo,'ok');
figure(1), plot(x_dibujo,y_dibujo,'k');


for i = 1:length(t)
    if(i==1)
    set(grafica_carro,'Visible','off');
    end
 
   
 theta_goal(i) =(180/pi)*atan2((y_goal - y(i)),(x_goal - x(i))) ; %theta goal
 distancia_goal(i) = sqrt((x_goal-x(i))^2+(y_goal-y(i))^2);

 
 
 
 %%implementación del control angular
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
 
 %%implementación control de velocidad
 
 vP(i) = vp_km1+f0*distancia_goal(i)+f1*d_km1+f2*d_km2;
 if vP(i) >= 2
     vP(i) = 2;
 end
 if vP(i) <= -2
     vP(i) = -2;
 end
 
 %actualizaciones
 d_km2 = d_km1;
 d_km1 = distancia_goal(i);
 vp_km1 = vP(i);
 
 
 % velocidad angulares de referencia para el controlador
 Wd_ref(i) = 1/R*vP(i) + L/(2*R)*wP(i);
 Wi_ref(i) = 1/R*vP(i) - L/(2*R)*wP(i);
 
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
figure(1),plot(x_goal,y_goal,'*m','LineWidth',3); 
fprintf('distancia goal: %f \n',distancia_goal(i))
if(distancia_goal(i)<=0.01)
    punto=punto+1;
    if(punto == 35)
        break
    end
    x_goal =x_dibujo(punto);
y_goal =y_dibujo(punto);
   tiempo = i;
  %   break;
end




if(i<length(t))
set(grafica_carro,'Visible','off');
end
 



end
figure(2), hold on;
figure(2), plot(t(1:i),wP,'r','LineWidth',2);
figure(2), plot(t(1:i),vP,'k','LineWidth',2);
figure(2), plot(t(1:i),distancia_goal,'b','LineWidth',2);
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

