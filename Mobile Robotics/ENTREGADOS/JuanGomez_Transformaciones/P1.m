clear;close;

% vector a desplazar
vector=[0, 4 ,7]; %últimos dígitos cédula
t = 2;
r = 20; %en grados

scatter3(0,0,0,'filled','.k','LineWidth',0.2);
title('Translacion del eje z en 2 unidades','interpreter','latex');
xlabel('x','interpreter','latex','FontSize',16);
ylabel('y','interpreter','latex','FontSize',16);
zlabel('z','interpreter','latex','FontSize',16);
hold on;

%ejes
x=0:0.1:10;
y=0:0.1:10;
z=0:0.1:10;
ceros=0*ones(1,101);
%resaltado ejes sistema de referencia inicial 
scatter3(x,ceros,ceros,'filled','k','LineWidth',0.2);
scatter3(ceros,y,ceros,'filled','k','LineWidth',0.2);
scatter3(ceros,ceros,z,'filled','k','LineWidth',0.2);


graf=scatter3(vector(1),vector(2),vector(3),'filled','k');
graf.LineWidth=5;


tx=[1 0 0 t;0 1 0 0;0 0 1 0;0 0 0 1];
ty=[1 0 0 0;0 1 0 t;0 0 1 0;0 0 0 1];
tz=[1 0 0 0;0 1 0 0;0 0 1 t;0 0 0 1];

rx=[1 0 0 0;0 cosd(r) -sind(r) 0;0 sind(r) cosd(r) 0 ;0 0 0 1];
ry=[cosd(r) 0 sind(r) 0;0 1 0 0;-sind(r) 0 cosd(r) 0 ;0 0 0 1];
rz=[cosd(r)  -sind(r) 0 0;sind(r) cosd(r) 0 0 ;0 0 1 0 ;0 0 0 1];
H=tx;
%transformación
[vector_1]= transformacion(vector,H);
scatter3(vector_1(1),vector_1(2),vector_1(3),'filled','r');
[vector_1]= transformacion(vector_1,H);



%transformación de los ejes para la gráfica
for i=1:length(x)
    [vector_1]= transformacion([x(i),0,0],H);
  scatter3(vector_1(1),vector_1(2),vector_1(3),'filled','r');
     [vector_1]= transformacion([0,y(i),0],H);
  scatter3(vector_1(1),vector_1(2),vector_1(3),'filled','r');
[vector_1]= transformacion([0,0,z(i)],H);
  scatter3(vector_1(1),vector_1(2),vector_1(3),'filled','r');
end
set(gca,'FontSize',18);



function [vector_1] = transformacion(vector,H)

vector_ext=[vector';1];
vector_1=H*vector_ext;
vector_1=[vector_1(1) vector_1(2) vector_1(3)];
end



