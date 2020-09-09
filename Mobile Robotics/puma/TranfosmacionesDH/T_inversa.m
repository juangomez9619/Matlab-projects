x = sym('x','real');
y = sym('y','real');
z = sym('z','real');
d1 = sym('d1','real');
L1 = sym('L1','real');
L2 = sym('L2','real');
theta1 = sym('theta1','real');
theta2 = sym('theta2','real');
theta3 = sym('theta3','real');
R1=[cos(theta1) 0 -1*sin(theta1);sin(theta1) 0 cos(theta1);0 -1 0];
R2=[cos(theta2) -1*sin(theta2) 0;sin(theta2) cos(theta2) 0;0 0 1];
R3=[cos(theta3) -1*sin(theta3) 0;sin(theta3) cos(theta3) 0;0 0 1];
D1=[0;0;d1];
D2=[L1*cos(theta2);L1*sin(theta2);0];
D3=[L2*cos(theta3);L2*sin(theta3);0];
A1=[R1 D1;0 0 0 1];
A2=[R2 D2;0 0 0 1];
A3=[R3 D3;0 0 0 1];
T=simplify(A1*A2*A3);
T(:,4)=[x;y;z;1];
IA1=[R1' -1*R1'*D1;0 0 0 1];
IA2=[R2' -1*R2'*D2;0 0 0 1];
IA3=[R3' -1*R3'*D3;0 0 0 1];

theta1=atan2(y,x);
Ac1=simplify(IA2*IA1*T);

H=x*cos(theta1)+y*sin(theta1);
aux1=(d1-z)*sin(theta2) - L1 + cos(theta2)*H;
aux2=(d1- z)*cos(theta2) -H*sin(theta2);

A1 = sym('A1','real');
A2 = sym('A2','real');

ex1=A1*sin(theta2) - L1 + cos(theta2)*A2;
ex2=A1*cos(theta2) -A2*sin(theta2);
expand(ex1.^2)
expand(ex2.^2)
