clear;close;clc
r=5; 
t=20;
tx=[1 0 0 t;0 1 0 0;0 0 1 0;0 0 0 1];
ty=[1 0 0 0;0 1 0 t;0 0 1 0;0 0 0 1];
tz=[1 0 0 0;0 1 0 0;0 0 1 t;0 0 0 1];
rx=[1 0 0 ;0 cos(r) -sin(r) ;0 sin(r) cos(r)];
ry=[cos(r) 0 sin(r) ;0 1 0 ;-sin(r) 0 cos(r)  ];
rz=[cos(r)  -sin(r) 0 ;sin(r) cos(r) 0  ;0 0 1  ];

(rx*ry*rz)