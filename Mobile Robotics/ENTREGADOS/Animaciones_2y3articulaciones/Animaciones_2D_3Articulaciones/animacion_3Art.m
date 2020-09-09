clc;close;clear;

global codo
%CONFIGURACIÓN DE LA ANIMACIÓN
codo=1; % arriba = 0 (1  codo abajo):
cinematica=1; %1 inversa, 0 directa


%dimensiones del brazo y antebrazo
global l1 l2 l3 ;
l1 =20;
l2=15;
l3=5;

%lecura de coordenadas para ambos tipos de cinemática
if codo ==0 %codo arriba
[x,y,phi,theta1,theta2,theta3] = lectura(xlsread('data_codoArriba_3Art.xlsx'));
else  % codo abajo
[x,y,phi,theta1,theta2,theta3] = lectura(xlsread('data_codoAbajo_3Art.xlsx'));
end

%configuración de la gráfica
figure 
hold on;
axis([-40 40 -40 40]);
title('Animacion 2D manipulador de 3 articulaciones','interpreter','latex');
xlabel('x','interpreter','latex','FontSize',16);
ylabel('y','interpreter','latex','FontSize',16);

set(gca,'FontSize',15);
%zona de trabajo del manipulador
[circulo_x, circulo_y]=circulo(40);
patch(circulo_x ,circulo_y, [1 1 0.5]);


for i=1:length(theta1)
if cinematica == 0 % DIRECTA no requiere validar cuadrantes
[x, y] = cinematica_directa(theta1(i),theta2(i),theta3(i));
 J2x=l1*cosd(theta1(i));
 J2y=l1*sind(theta1(i));
 J3x= x -l3*cosd(theta1(i)+theta2(i)+theta3(i));
 J3y= y -l3*sind(theta1(i)+theta2(i)+theta3(i));
  %gráfica del brazo L2 
 brazo_l2=plot([J2x J3x], [J2y J3y],'k','LineWidth',2);
 %gráfica del brazo L3 
 brazo_l3=plot( [J3x x], [J3y y],'k','LineWidth',2);
 %gráfica punto del efector
 plot(x,y,'xr','LineWidth',2);
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
else %cinemática inversa
  [theta1(i), theta2(i),theta3(i)] = cinematica_inversa(x(i),y(i),phi(i));
  J2x=l1*cosd(theta1(i));
  J2y=l1*sind(theta1(i));
   J3x= x(i) -l3*cosd(theta1(i)+theta2(i)+theta3(i));
   J3y= y(i) -l3*sind(theta1(i)+theta2(i)+theta3(i));
   %gráfica del brazo L2 
 brazo_l2=plot([J2x J3x], [J2y J3y],'k','LineWidth',2);
  %gráfica del brazo L3 
 brazo_l3=plot( [J3x x(i)], [J3y y(i)],'k','LineWidth',2);
 %gráfica punto del efector
 plot(x(i),y(i),'xr','LineWidth',2);

    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%gráfica del brazo L1 
brazo_l1=plot([0 J2x], [0 J2y],'k','LineWidth',2);
%articulación J1 
plot(0,0,'xr','LineWidth',2);
%gráfica articulación J2
grafica_j2=plot(J2x,J2y,'xr','LineWidth',2);
%gráfica articulación J3
grafica_j3=plot(J3x,J3y,'xr','LineWidth',2);

pause(0.01);

 if i <= (length(x)-1) || (i <= length(theta1)-1)
 set(brazo_l1,'Visible','off');
set(brazo_l2,'Visible','off');
set(brazo_l3,'Visible','off');
set(grafica_j2,'Visible','off'); 
set(grafica_j3,'Visible','off'); 
 end

end

function [theta1,theta2,theta3] = cinematica_inversa(x,y,phi)
global l1 l2 l3 codo
x3=x-l3*cosd(phi);
y3=y-l3*sind(phi);

if codo ==1
if x >= 0 %cuadrante 1 y 4
theta2=(180/pi)*acos((x3^2+y3^2-l1^2-l2^2)/(2*l1*l2));
else 
theta2=(-180/pi)*acos((x3^2+y3^2-l1^2-l2^2)/(2*l1*l2));
end
else
if x >= 0 %cuadrante 1 y 4
theta2=(-180/pi)*acos((x3^2+y3^2-l1^2-l2^2)/(2*l1*l2));
else 
theta2=(180/pi)*acos((x3^2+y3^2-l1^2-l2^2)/(2*l1*l2));
end
end
theta1=(180/pi)*atan2((y3*(l1+l2*cosd(theta2))-x3*l2*sind(theta2)),(x3*(l1+l2*cosd(theta2))+y3*l2*sind(theta2)));
theta3=phi-theta1-theta2;
end

function [x,y] = cinematica_directa(theta1,theta2,theta3)
global l1 l2 l3
x=l1*cosd(theta1)+l2*cosd(theta1+theta2)+l3*cosd(theta1+theta2+theta3);
y=l1*sind(theta1)+l2*sind(theta1+theta2)+l3*sind(theta1+theta2+theta3);
end


function [x,y,phi,theta1,theta2,theta3] = lectura(coordenadas)
x=coordenadas(:,1)'; %columna A excel
y=coordenadas(:,2)';%columna B excel
phi=coordenadas(:,3)';%columna C excel
theta1=coordenadas(:,4)';%columna D excel
theta2=coordenadas(:,5)';%columna E excel
theta3=coordenadas(:,6)';%columna F excel
end



function [x,y]=circulo(radio)
x=radio*cos(0:0.01:2*pi);
y=radio*sin(0:0.01:2*pi);
end
