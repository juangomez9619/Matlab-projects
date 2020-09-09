function [] = transformacion(x,v)
clear;close;clc
scatter3(0,0,0,'filled','.k','LineWidth',0.2);
title('Ubicacion de puntos clave','interpreter','latex');
xlabel('x','interpreter','latex','FontSize',16);
ylabel('y','interpreter','latex','FontSize',16);
zlabel('z','interpreter','latex','FontSize',16);
hold on;
vector=v;
%
%ejes
 x=0:0.1:10;
 y=0:0.1:10;
 z=0:0.1:10;
 ceros=0*ones(1,101);
 %gráfica del punto
 graf=scatter3(vector(1),vector(2),vector(3),'filled','k');
% graf.LineWidth=5;
% 
% resaltado ejes sistema de referencia inicial 
% scatter3(x,ceros,ceros,'filled','k','LineWidth',0.2);
% scatter3(ceros,y,ceros,'filled','k','LineWidth',0.2);
% scatter3(ceros,ceros,z,'filled','k','LineWidth',0.2);




end

