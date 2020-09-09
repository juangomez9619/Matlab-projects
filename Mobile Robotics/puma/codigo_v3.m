clear; 
close all;
clc;

load('prueba3.mat')
div=100;
%longitudes
d1=1120;
d2=355;
d3=185;
L2=650;
L3=600;
d6=250;

%ángulos

theta_1=[20:230/div:250]';
theta_2=[-70:80/div:10]';
theta_3=[-80:110/div:30]';
theta_41=[10:640/div:170];
theta_42=sort(theta_41,'descend');
theta_4=[theta_41 theta_42 theta_41 theta_42]';
 theta_51=[120:360/div:240];
 theta_53=sort(theta_51,'descend');
 theta_5=[theta_51 theta_53 theta_51]';
%theta_5=[120:120/div:240]';
theta_6=[-266:532/div:266]';
base=[0;0;0;1];

%% matrices DH
syms a alpha theta d
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tx=[1 0 0 a;0 1 0 0;0 0 1 0;0 0 0 1];
tz=[1 0 0 0;0 1 0 0;0 0 1 d;0 0 0 1];
rx=[1 0 0 0;0 cosd(alpha) -sind(alpha) 0;0 sind(alpha) cosd(alpha) 0 ;0 0 0 1];
rz=[cosd(theta)  -sind(theta) 0 0;sind(theta) cosd(theta) 0 0 ;0 0 1 0 ;0 0 0 1];
A= simplify(rz*tz*tx*rx);

%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%

% iteraciones
%div+1

for i=1:(div+1)/2

 load('Piezas.mat');
 %load('linksdata_original.mat');
          
 

   
%% 1    
alpha=-90;
a=0;
d=d1;
%theta=5*theta_1(i);
theta=angulos_ci(i,1);
    % alpha theta a d
A1=...
[ cosd(theta), -cosd(alpha)*sind(theta),  sind(alpha)*sind(theta), a*cosd(theta);
 sind(theta),  cosd(alpha)*cosd(theta), -sind(alpha)*cosd(theta), a*sind(theta);
          0,             sind(alpha),             cosd(alpha),            d;
          0,                      0,                      0,            1];
o1=A1*base;


%ajuste de la pieza s2

    for j=1:length(s2.V2(:,1))
    s2.V2(j,:)=A1*s2.V2(j,:)';
    end
    

 
%% 2

alpha=0;
a=L2;
d=d2;
%theta=theta_2(i);   
theta=angulos_ci(i,2);
% alpha theta a d
A2=...
[ cosd(theta), -cosd(alpha)*sind(theta),  sind(alpha)*sind(theta), a*cosd(theta);
 sind(theta),  cosd(alpha)*cosd(theta), -sind(alpha)*cosd(theta), a*sind(theta);
          0,             sind(alpha),             cosd(alpha),            d;
          0,                      0,                      0,            1];
 T=A1*A2;
 o2=T*base;
 
   for j=1:length(s3.V3(:,1))
    s3.V3(j,:)=T*s3.V3(j,:)';
    end

%% 3
% 
% 
alpha=-90;
a=0;
d=-d3;
%theta=theta_3(i);
theta=angulos_ci(i,3);
A3=...
[ cosd(theta), -cosd(alpha)*sind(theta),  sind(alpha)*sind(theta), a*cosd(theta);
 sind(theta),  cosd(alpha)*cosd(theta), -sind(alpha)*cosd(theta), a*sind(theta);
          0,             sind(alpha),             cosd(alpha),            d;
          0,                      0,                      0,            1];
 T=T*A3;
 o3=T*base;
 
 for j=1:length(s4.V4(:,1))
    s4.V4(j,:)=T*s4.V4(j,:)';
 end

    
%% 4

alpha=-90;
a=0;
d=L3;
%theta=theta_4(i);
theta=angulos_ci(i,4);
A4=...
[ cosd(theta), -cosd(alpha)*sind(theta),  sind(alpha)*sind(theta), a*cosd(theta);
 sind(theta),  cosd(alpha)*cosd(theta), -sind(alpha)*cosd(theta), a*sind(theta);
          0,             sind(alpha),             cosd(alpha),            d;
          0,                      0,                      0,            1];
 
 T=T*A4;
 o4=T*base;
 
%  
  for j=1:length(s5.V5(:,1))
     s5.V5(j,:)=T*s5.V5(j,:)';
  end
%% 5       

alpha=-90;
a=0;
d=0;
%theta=theta_5(i)*1;
theta=angulos_ci(i,5);
A5=...
[ cosd(theta), -cosd(alpha)*sind(theta),  sind(alpha)*sind(theta), a*cosd(theta);
 sind(theta),  cosd(alpha)*cosd(theta), -sind(alpha)*cosd(theta), a*sind(theta);
          0,             sind(alpha),             cosd(alpha),            d;
          0,                      0,                      0,            1];

 T=T*A5;
 o5=T*base;

 for j=1:length(s6.V6(:,1))
    s6.V6(j,:)=T*s6.V6(j,:)';
 end



 %% 6

alpha=0;
a=0;
d=d6;
theta=theta_6(i);
A6=...
[ cosd(theta), -cosd(alpha)*sind(theta),  sind(alpha)*sind(theta), a*cosd(theta);
 sind(theta),  cosd(alpha)*cosd(theta), -sind(alpha)*cosd(theta), a*sind(theta);
          0,             sind(alpha),             cosd(alpha),            d;
          0,                      0,                      0,            1];
      
      T=T*A6;
 o6=T*base;
  puntos(i,:)=o6';
  for j=1:length(s7.V7(:,1))
     s7.V7(j,:)=T*s7.V7(j,:)';
  end
 
 
%% 
%%%%%%%%%%%%%%%%%%%%%%% gráfica %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 xp=[0 o1(1) o2(1) o3(1) o4(1) o5(1) o6(1)];
 yp=[0 o1(2) o2(2) o3(2) o4(2) o5(2) o6(2)];
 zp=[0 o1(3) o2(3) o3(3) o4(3) o5(3) o6(3)];

 clf;
 
% 
   plot3(0,0,0,'o','MarkerSize',10,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
  hold on; grid on;  
   xlabel('x','interpreter','latex','FontSize',16);
   ylabel('y','interpreter','latex','FontSize',16);
   zlabel('z','interpreter','latex','FontSize',16);
   title('Puma 762','interpreter','latex','FontSize',16);

%  plot3(xp,yp,zp,'-','LineWidth',3,'Color','r');
%  
%  plot3(o1(1),o1(2),o1(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
%  plot3(o2(1),o2(2),o2(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
%  plot3(o3(1),o3(2),o3(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
%  plot3(o4(1),o4(2),o4(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
%  plot3(o5(1),o5(2),o5(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
%  plot3(o6(1),o6(2),o6(3),'o','MarkerSize',7,'MarkerEdgeColor','g','MarkerFaceColor','g','LineWidth',1);
  grosor=2;
 plot3(s1.V1(:,1),s1.V1(:,2),s1.V1(:,3),'r','LineWidth',grosor);
 plot3(s2.V2(:,1),s2.V2(:,2),s2.V2(:,3),'b','LineWidth',grosor);
 plot3(s3.V3(:,1),s3.V3(:,2),s3.V3(:,3),'k','LineWidth',grosor);
 plot3(s4.V4(:,1),s4.V4(:,2),s4.V4(:,3),'LineWidth',grosor,'Color',[1 1 0.2]);
  plot3(s5.V5(:,1),s5.V5(:,2),s5.V5(:,3),'r','LineWidth',grosor);
 plot3(s6.V6(:,1),s6.V6(:,2),s6.V6(:,3),'b','LineWidth',grosor);
 plot3(s7.V7(:,1),s7.V7(:,2),s7.V7(:,3),'k','LineWidth',grosor);
 %
 plot3(puntos(:,1),puntos(:,2),puntos(:,3),'g','LineWidth',grosor);
 
    set(gca,...
             'Xlim',[-1800 1800],...
             'Ylim',[-1800 1800],...
             'Zlim',[-250 2700])
 set(gca,'fontname','times') 
set(gca,'FontSize',14);

%          set(gca,...
%              'Xlim',[300 800],...
%              'Ylim',[-200 400],...
%              'Zlim',[0 800])

 pause(0.02);

 
%%calculo mediante cinematica inversa
c=o6(1:3)-T(1:3,1:3)*[0;0;d6]

Itheta1=2*atan2(sqrt(c(1)^2+c(2)^2-28900)-c(1),c(2)+170)*(180/pi)
Itheta11=asind((-340*c(1)+sqrt((340*c(1))^2-4*(c(1)^2+c(2)^2)*(170^2-c(2)^2)))/(2*(c(1)^2+c(2)^2)))
Itheta12=asind((-340*c(1)-sqrt((340*c(1))^2-4*(c(1)^2+c(2)^2)*(170^2-c(2)^2)))/(2*(c(1)^2+c(2)^2)))
k1=1120-c(3);
k2=c(1)*cosd(Itheta1)+c(2)*sind(Itheta1);
k3=k1^2+k2^2+62500;
k4=1690000*(k1^2+k2^2);

%Itheta2=asind((2600*k1*k3+sqrt((2600*k1*k3)^2-4*k4*(-1690000*k2^2+k3^2)))/(2*k4))
Itheta2=asind((2600*k1*k3-sqrt((2600*k1*k3)^2-4*k4*(-1690000*k2^2+k3^2)))/(2*k4)) %%% este es para codo abajo

Itheta3=atan2((k1*cosd(Itheta2)-k2*sind(Itheta2)),(k1*sind(Itheta2)-650+k2*cosd(Itheta2)))*(180/pi)-90
num=1120*cosd(Itheta2+Itheta3)+650*sind(Itheta3)-o6(3)*cosd(Itheta2+ Itheta3)-o6(1)*sind(Itheta2+Itheta3)*cosd(Itheta1)-o6(2)*sind(Itheta2+Itheta3)*sind(Itheta1)-600;
if num>250
Itheta5=180
else
Itheta5=acosd(num/(-250))
end
if sind(Itheta5)==0
Itheta4=0
else
Itheta4=acosd((1120*sind(Itheta2+Itheta3)-650*cosd(Itheta3)-o6(3)*sind(Itheta2+ Itheta3)+o6(1)*cosd(Itheta2+Itheta3)*cosd(Itheta1)+o6(2)*cosd(Itheta2+Itheta3)*sind(Itheta1))/(-250*sind(Itheta5)))
end
angulos2_ci(i,:)=[Itheta1 Itheta2 Itheta3 Itheta4 Itheta5];




end
