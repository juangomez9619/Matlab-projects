clc;close;clear;

global codo
%CONFIGURACIÓN DE LA ANIMACIÓN
codo=1; % arriba = 0 (1  codo abajo):
cinematica=1; %1 inversa, 0 directa

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%dimensiones del brazo y antebrazo
global l1 l2 ;
l1 =20;
l2=15;

%lectura de coordenadas para ambos tipos de cinemática
if codo ==0 %codo arriba
[x,y,theta1,theta2] = lectura(xlsread('data_codoArriba_2Art.xlsx'));
else  % codo abajo
[x,y,theta1,theta2] = lectura(xlsread('data_codoAbajo_2Art.xlsx'));
end


%configuración de la gráfica
figure 
hold on;
axis([-35 35 -35 35]);
title('Animacion 2D manipulador de 2 articulaciones','interpreter','latex');
xlabel('x','interpreter','latex','FontSize',16);
ylabel('y','interpreter','latex','FontSize',16);

set(gca,'FontSize',15);

%zona de trabajo del manipulador
[circulo_x, circulo_y]=circulo(35);%circulo de radio 35 centrado en el origen
patch(circulo_x ,circulo_y, [1 1 0.5]); %coloreado amarillo
plot(circulo_x,circulo_y,'k'); %contorno del circulo
[circulo_x ,circulo_y]=circulo(5);%circulo de radio 5 centrado en el origen
patch(circulo_x ,circulo_y,'w')%coloreado blanco
plot(circulo_x,circulo_y,'k');%contorno del circulo


for i=1:length(theta1)
 if cinematica == 0 % DIRECTA no requiere validar cuadrantes
 [x, y] = cinematica_directa(theta1(i),theta2(i));
 J2x=l1*cosd(theta1(i));
 J2y=l1*sind(theta1(i));
 %gráfica del brazo L2 
 brazo_l2=plot([J2x x], [J2y y],'k','LineWidth',2);
 %gráfica punto del efector
 plot(x,y,'xr','LineWidth',2);
 else % inversa si requiere evaluar en qué cuadrante está ubicado el manipulador
 [theta1(i), theta2(i)] = cinematica_inversa(x(i),y(i));
 J2x=l1*cosd(theta1(i));
 J2y=l1*sind(theta1(i));
 %gráfica del brazo L2 
 brazo_l2= plot([J2x x(i)], [J2y y(i)],'k','LineWidth',2);
 %gráfica punto del efector
 plot(x(i),y(i),'xr','LineWidth',2);
 end

%gráfica del brazo L1 
brazo_l1=plot([0 J2x], [0 J2y],'k','LineWidth',2);

%gráfica articulación J2
grafica_j2=plot(J2x,J2y,'xr','LineWidth',2);
%articulación J1 
plot(0,0,'xr','LineWidth',2);
pause(0.01);

 if i <= (length(x)-1) || (i <= length(theta1)-1)
set(brazo_l1,'Visible','off');
set(brazo_l2,'Visible','off');
set(grafica_j2,'Visible','off'); 
 end


end


% funciones 

function [x,y,theta1,theta2] = lectura(coordenadas)
x=coordenadas(:,1)';%columna A excel
y=coordenadas(:,2)';%columna B excel
theta1=coordenadas(:,3)';%columna C excel
theta2=coordenadas(:,4)';%columna D excel
end

function [x,y] = cinematica_directa(theta1,theta2)
global l1 l2
x=l1*cosd(theta1)+l2*cosd(theta1+theta2);
y=l1*sind(theta1)+l2*sind(theta1+theta2);
end

function [theta1,theta2] = cinematica_inversa(x,y)
global l1 l2 codo
if codo ==1
if x >= 0 %cuadrante 1 y 4
theta2=(180/pi)*acos((x^2+y^2-l1^2-l2^2)/(2*l1*l2));
else %cuadrante 2 y 3
theta2=(-180/pi)*acos((x^2+y^2-l1^2-l2^2)/(2*l1*l2)); 
end
else %cuadrante 2 y 3
if x >= 0 %cuadrante 1 y 4
theta2=(-180/pi)*acos((x^2+y^2-l1^2-l2^2)/(2*l1*l2));
else 
theta2=(180/pi)*acos((x^2+y^2-l1^2-l2^2)/(2*l1*l2)); 
end
    
end
theta1=(180/pi)*atan2((y*(l1+l2*cosd(theta2))-x*l2*sind(theta2)),(x*(l1+l2*cosd(theta2))+y*l2*sind(theta2)));
end

function [x,y]=circulo(radio)
x=radio*cos(0:0.01:2*pi);
y=radio*sin(0:0.01:2*pi);
end


