vector=[0, 4 ,7]; %últimos dígitos cédula
graf=scatter3(vector(1),vector(2),vector(3),'filled','k');
title('Translacion del eje z en 2 unidades','interpreter','latex');
xlabel('x','interpreter','latex','FontSize',16);
ylabel('y','interpreter','latex','FontSize',16);
zlabel('z','interpreter','latex','FontSize',16);
hold on;
set(gca,'FontSize',18);
vector_ext=[vector';1];
t=2;
tx=[1 0 0 t;0 1 0 0;0 0 1 0;0 0 0 1];
t=-3;
ty=[1 0 0 0;0 1 0 t;0 0 1 0;0 0 0 1];
t=3;
tz=[1 0 0 0;0 1 0 0;0 0 1 t;0 0 0 1];
r=30;
rx=[1 0 0 0;0 cosd(r) -sind(r) 0;0 sind(r) cosd(r) 0 ;0 0 0 1];
r=45;
ry=[cosd(r) 0 sind(r) 0;0 1 0 0;-sind(r) 0 cosd(r) 0 ;0 0 0 1];
r=60;
rz=[cosd(r)  -sind(r) 0 0;sind(r) cosd(r) 0 0 ;0 0 1 0 ;0 0 0 1];

vector=rx*ty*vector_ext;
graf=scatter3(vector(1),vector(2),vector(3),'filled','r');
vector=ty*rx*vector_ext;
graf=scatter3(vector(1),vector(2),vector(3),'filled','g');




