function [A] = matrizDH(alfa,theta,a,d)
 R_x=[1 0 0 0;0 cosd(alfa) -1*sind(alfa) 0;0 sind(alfa) cosd(alfa) 0;0 0 0 1]; %%matriz de rotación x
 T_x=[1 0 0 a;0 1 0 0;0 0 1 0;0 0 0 1]; %%matriz de traslación x
 R_z=[cosd(theta) -1*sind(theta) 0 0;sind(theta) cosd(theta) 0 0;0 0 1 0;0 0 0 1];     %%matriz de rotación z
 T_z=[1 0 0 0;0 1 0 0;0 0 1 d;0 0 0 1];     %%matriz de rotación z
 A=R_z*T_z*T_x*R_x;
end