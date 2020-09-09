clc;
syms d1 d2 d4 d3 d6 L2 theta_1 theta_2 theta_3 theta_4 theta_5 theta_6
theta1 = sym('theta1','real');
theta2 = sym('theta2','real');
theta3 = sym('theta3','real');
theta4 = sym('theta4','real');
theta5 = sym('theta5','real');
theta6 = sym('theta6','real');
theta7 = sym('theta7','real');
theta8 = sym('theta8','real');
d1=1120;
d2=355;
d3=185;
L2=650;
d4=600;
d6=250;




a = 0;
d=d1;
alfa = -90;
theta = theta_1;
A1=...
[ cosd(theta), -cosd(alfa)*sind(theta),  sind(alfa)*sind(theta), a*cosd(theta);
  sind(theta),  cosd(alfa)*cosd(theta), -sind(alfa)*cosd(theta), a*sind(theta);
          0,             sind(alfa),             cosd(alfa),            d;
          0,                      0,                      0,            1];
      
a = L2;
d=d2;
alfa = 0;
theta = theta_2;

A2=...
[ cosd(theta), -cosd(alfa)*sind(theta),  sind(alfa)*sind(theta), a*cosd(theta);
  sind(theta),  cosd(alfa)*cosd(theta), -sind(alfa)*cosd(theta), a*sind(theta);
          0,             sind(alfa),             cosd(alfa),            d;
          0,                      0,                      0,            1];
a = 0;
d=-d3;
alfa = -90;
theta = theta_3;

A3=...
[ cosd(theta), -cosd(alfa)*sind(theta),  sind(alfa)*sind(theta), a*cosd(theta);
  sind(theta),  cosd(alfa)*cosd(theta), -sind(alfa)*cosd(theta), a*sind(theta);
          0,             sind(alfa),             cosd(alfa),            d;
          0,                      0,                      0,            1];
a = 0;
d=d4;
alfa = -90;
theta = theta_4;

A4=...
[ cosd(theta), -cosd(alfa)*sind(theta),  sind(alfa)*sind(theta), a*cosd(theta);
  sind(theta),  cosd(alfa)*cosd(theta), -sind(alfa)*cosd(theta), a*sind(theta);
          0,             sind(alfa),             cosd(alfa),            d;
          0,                      0,                      0,            1];
      
a = 0;
d= 0;
alfa = -90;
theta = theta_5;

A5=...
[ cosd(theta), -cosd(alfa)*sind(theta),  sind(alfa)*sind(theta), a*cosd(theta);
  sind(theta),  cosd(alfa)*cosd(theta), -sind(alfa)*cosd(theta), a*sind(theta);
          0,             sind(alfa),             cosd(alfa),            d;
          0,                      0,                      0,            1];
a = 0;
d= d6;
alfa = 0;
theta = theta_6;

A6=...
[ cosd(theta), -cosd(alfa)*sind(theta),  sind(alfa)*sind(theta), a*cosd(theta);
  sind(theta),  cosd(alfa)*cosd(theta), -sind(alfa)*cosd(theta), a*sind(theta);
          0,             sind(alfa),             cosd(alfa),            d;
          0,                      0,                      0,            1];
 
% %latex(A8)
% syms T11 T12 T13 T14  T21 T22 T23 T24 T31 T32 T33 T34  
% T1=...
%     [T11,T12,T13,T14;
%     T21,T22,T23,T24;
%     T31,T32,T33,T34;
%     0,0,0,1];

T=simplify(A1*A2*A3*A4*A5*A6);
% 
% %%
IA1=simplify(inv(A1));
IA2=(inv(A2));
IA3=simplify(inv(A3));
IA4=simplify(inv(A4));
IA5=(inv(A5));
IA6=simplify(inv(A6));

% 
% %%
clc;
T=simplify(A4*A5*A6);
x = sym('x','real');
y = sym('y','real');
z = sym('z','real');
T(:,4)=[x;y;z;1];
X = simplify(IA2*IA1*T);
X(:,4)

%%
Y = simplify(A3*A4*A5*A6);
Y(:,4)
% 



%%



% x = sym('x','real');
% y = sym('y','real');
% z = sym('z','real');
% d1 = sym('d1','real');
% L1 = sym('L1','real');
% L2 = sym('L2','real');
% theta1 = sym('theta1','real');
% theta2 = sym('theta2','real');
% theta3 = sym('theta3','real');
% 
% R1=[cos(theta1) 0 -1*sin(theta1);sin(theta1) 0 cos(theta1);0 -1 0];
% R2=[cos(theta2) -1*sin(theta2) 0;sin(theta2) cos(theta2) 0;0 0 1];
% R3=[cos(theta3) -1*sin(theta3) 0;sin(theta3) cos(theta3) 0;0 0 1];
% D1=[0;0;d1];
% D2=[L1*cos(theta2);L1*sin(theta2);0];
% D3=[L2*cos(theta3);L2*sin(theta3);0];
% A1=[R1 D1;0 0 0 1];
% A2=[R2 D2;0 0 0 1];
% A3=[R3 D3;0 0 0 1];
% T=simplify(A1*A2*A3);
% 
% IA1=[R1' -1*R1'*D1;0 0 0 1];
% IA2=[R2' -1*R2'*D2;0 0 0 1];
% IA3=[R3' -1*R3'*D3;0 0 0 1];