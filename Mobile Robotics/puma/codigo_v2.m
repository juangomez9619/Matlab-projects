clear; 
close all
clc;

div=20;
%longitudes
d1=1120;
d2=355;
d3=185;
L2=650;
L3=600;
d4=600;
d6=250;

%ángulos

theta_1=[-160:320/div:160]';
theta_2=[-115:230/div:115]';
theta_3=[-135:270/div:135]';
theta_4=[-266:532/div:266]';
theta_5=[-100:200/div:100]';
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

for i=1:div+1

 load('linksdata2.mat');

%      for j=1:length(s6.V6(:,1))
%      alpha=90;  
%       rx=[1 0 0 0;0 cosd(alpha) -sind(alpha) 0;0 sind(alpha) cosd(alpha) 0 ;0 0 0 1];
%     s6.V6(j,:)=rx*s6.V6(j,:)';
%      end
          
%    s7.V7(:,3)=s7.V7(:,3)-250;
%    for j=1:length(s6.V6(:,1))
%      r=180;  
%       ry=[cosd(r) 0 sind(r) 0;0 1 0 0;-sind(r) 0 cosd(r) 0 ;0 0 0 1];
%     s6.V6(j,:)=ry*s6.V6(j,:)';
%    end
%    for j=1:length(s6.V6(:,1))
%        r=180;
%  ry=[cosd(r) 0 sind(r) 0;0 1 0 0;-sind(r) 0 cosd(r) 0 ;0 0 0 1];
%     s6.V6(j,:)=ry*s6.V6(j,:)';
%     end
   
%% 1    
alpha=-90;
a=0;
d=d1;
theta=theta_1(i).*0+90 
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
theta=theta_2(i)*0.1;   
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
theta=theta_3(i)*0+145;
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
 % A4=matrizDH(alfa(5),0,0,d4);
 % alpha theta a d
alpha=-90;
a=0;
d=d4;
theta=theta_4(i)*0;

A4=...
[ cosd(theta), -cosd(alpha)*sind(theta),  sind(alpha)*sind(theta), a*cosd(theta);
 sind(theta),  cosd(alpha)*cosd(theta), -sind(alpha)*cosd(theta), a*sind(theta);
          0,             sind(alpha),             cosd(alpha),            d;
          0,                      0,                      0,            1];
 
 T=T*A4;
 o4=T*base;
 
%  
%   for j=1:length(s5.V5(:,1))
%     s5.V5(j,:)=T*s5.V5(j,:)';
%  end
%       
%% 5
alpha=-90;
a=0;
d=0;
theta=theta_5(i)*0;
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
theta=theta_6(i)*0;
A6=...
[ cosd(theta), -cosd(alpha)*sind(theta),  sind(alpha)*sind(theta), a*cosd(theta);
 sind(theta),  cosd(alpha)*cosd(theta), -sind(alpha)*cosd(theta), a*sind(theta);
          0,             sind(alpha),             cosd(alpha),            d;
          0,                      0,                      0,            1];
      
      T=T*A6;
 o6=T*base;
  puntos(i,:)=o6';
%   for j=1:length(s7.V7(:,1))
%     s7.V7(j,:)=T*s7.V7(j,:)';
%  end
 
 
%%
%%%%%%%%%%%%%%%%%%%%%%% gráfica %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 xp=[0 o1(1) o2(1) o3(1) o4(1) o5(1) o6(1)];
 yp=[0 o1(2) o2(2) o3(2) o4(2) o5(2) o6(2)];
 zp=[0 o1(3) o2(3) o3(3) o4(3) o5(3) o6(3)];

 clf;
 
% 
   plot3(0,0,0,'o','MarkerSize',10,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
  hold on; grid on;  xlabel('X'); ylabel('Y'); zlabel('Z');
% 
 plot3(xp,yp,zp,'-','LineWidth',3,'Color','r');
 
 plot3(o1(1),o1(2),o1(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot3(o2(1),o2(2),o2(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot3(o3(1),o3(2),o3(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot3(o4(1),o4(2),o4(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot3(o5(1),o5(2),o5(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot3(o6(1),o6(2),o6(3),'o','MarkerSize',7,'MarkerEdgeColor','g','MarkerFaceColor','g','LineWidth',1);
%  grosor=2;
%  plot3(s1.V1(:,1),s1.V1(:,2),s1.V1(:,3),'r','LineWidth',grosor);
%  plot3(s2.V2(:,1),s2.V2(:,2),s2.V2(:,3),'b','LineWidth',grosor);
%  plot3(s3.V3(:,1),s3.V3(:,2),s3.V3(:,3),'k','LineWidth',grosor);
%  plot3(s4.V4(:,1),s4.V4(:,2),s4.V4(:,3),'LineWidth',grosor,'Color',[1 1 0.2]);
%  plot3(s5.V5(:,1),s5.V5(:,2),s5.V5(:,3),'r','LineWidth',grosor);
%  plot3(s6.V6(:,1),s6.V6(:,2),s6.V6(:,3),'b','LineWidth',grosor);
%  plot3(s7.V7(:,1),s7.V7(:,2),s7.V7(:,3),'k','LineWidth',grosor);
 %
 plot3(puntos(:,1),puntos(:,2),puntos(:,3));
 
    set(gca,...
             'Xlim',[-1800 1800],...
             'Ylim',[-1800 1800],...
             'Zlim',[-250 2700])
 

%          set(gca,...
%              'Xlim',[300 800],...
%              'Ylim',[-200 400],...
%              'Zlim',[0 800])

 pause(0.02);

end
%%
 c=o6(1:3)-T(1:3,1:3)*[0;0;d6];
 xc=c(1);
yc=c(2);
zc=c(3);
t1=(atan2(yc,xc)-atan2(d3,sqrt(xc^2+yc^2-d3^2)))*(180/pi);


a2 = sqrt(1*(o2(1)-o1(1))^2+1*(o2(2)-o1(2))^2+0*(o2(3)-o1(3))^2);
a3 = sqrt(1*(o3(1)-o2(1))^2+1*(o3(2)-o2(2))^2+0*(o3(3)-o2(3))^2);
D = (xc^2+yc^2-d3^2+zc^2-a2^2-a3^2)/(2*a2*a3)
t3 = atan2(sqrt(1-D^2),D)*180/pi
plot3([xc o2(1)],[o2(2) o2(2)],[zc zc],'-','LineWidth',3,'Color','k');
plot3([xc xc],[yc o2(2)],[zc zc],'-','LineWidth',3,'Color','g');
plot3([o2(1) o2(1)],[o2(2) o2(2)],[zc o2(3)],'-','LineWidth',3,'Color','b');

plot3([o2(1) o1(1)],[o1(2) o1(2)],[o2(3) o2(3)],'-','LineWidth',3,'Color','k');
plot3([o2(1) o2(1)],[o2(2) o1(2)],[o2(3) o2(3)],'-','LineWidth',3,'Color','g');
plot3([o1(1) o1(1)],[o1(2) o1(2)],[o2(3) o1(3)],'-','LineWidth',3,'Color','b')




% Itheta1=2*atan2(sqrt(c(1)^2+c(2)^2-28900)-c(1),c(2)+170)*(180/pi)
% Itheta11=asind((-340*c(1)+sqrt((340*c(1))^2-4*(c(1)^2+c(2)^2)*(170^2-c(2)^2)))/(2*(c(1)^2+c(2)^2)))
% Itheta12=asind((-340*c(1)-sqrt((340*c(1))^2-4*(c(1)^2+c(2)^2)*(170^2-c(2)^2)))/(2*(c(1)^2+c(2)^2)))
% k1=1120-c(3);
% k2=c(1)*cosd(Itheta1)-c(2)*sind(Itheta1);
% k3=k1^2+k2^2+62500;
% k4=1690000*(k1^2+k2^2);
% Itheta2=asind((2600*k1*k3+sqrt((2600*k1*k3)^2-4*k4*(-1690000*k2^2+k3^2)))/(2*k4))
% Itheta22=asind((2600*k1*k3-sqrt((2600*k1*k3)^2-4*k4*(-1690000*k2^2+k3^2)))/(2*k4))
% Itheta3=atan2((k1*cosd(Itheta2)-k2*sind(Itheta2)),(k1*sind(Itheta2)-650+k2*cosd(Itheta2)))*(180/pi)