
%Juan David Gómez Buitrago
%20151005039

clear;close;clc
format short;
r_1=1;
t5=1;
k=2;
contador_8=1;
contador_9=1;
contador_10=1;
t=-0.2;
r=5;

tx=[1 0 0 t;0 1 0 0;0 0 1 0;0 0 0 1];
ty=[1 0 0 0;0 1 0 t;0 0 1 0;0 0 0 1];
tz=[1 0 0 0;0 1 0 0;0 0 1 t;0 0 0 1];

rx=[1 0 0 0;0 cosd(r) -sind(r) 0;0 sind(r) cosd(r) 0 ;0 0 0 1];
ry=[cosd(r) 0 sind(r) 0;0 1 0 0;-sind(r) 0 cosd(r) 0 ;0 0 0 1];
rz=[cosd(r)  -sind(r) 0 0;sind(r) cosd(r) 0 0 ;0 0 1 0 ;0 0 0 1];


global x1 x2 x3 x4 x5 x6 x7 x8
global y1 y2 y3 y4 y5 y6 y7 y8 
global z1 z2 z3 z4 z5 z6 z7 z8 
%posiciones iniciales 0,4,7
x1=0;y1=4;z1=7;
x5=0;y5=6;z5=7;
x2=2;y2=4;z2=7;
x3=0;y3=4;z3=9;

x6=0;y6=6;z6=9;
x4=2;y4=4;z4=9;
x7=2;y7=6;z7=7;
x8=2;y8=6;z8=9;

global x y z
    x=1;
    y=5;
    z=8;
%configuración de la gráfica
iteraciones=765;
global modo
modo=1;
H=tx;

for i=1:iteraciones
 
if(modo~=14)
 clf
end    
    
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tx=[1 0 0 t;0 1 0 0;0 0 1 0;0 0 0 1];
ty=[1 0 0 0;0 1 0 t;0 0 1 0;0 0 0 1];
tz=[1 0 0 0;0 1 0 0;0 0 1 t;0 0 0 1];

rx=[1 0 0 0;0 cosd(r) -sind(r) 0;0 sind(r) cosd(r) 0 ;0 0 0 1];
ry=[cosd(r) 0 sind(r) 0;0 1 0 0;-sind(r) 0 cosd(r) 0 ;0 0 0 1];
rz=[cosd(r)  -sind(r) 0 0;sind(r) cosd(r) 0 0 ;0 0 1 0 ;0 0 0 1];


%%%%%%%%%%%%%%%% animación

switch modo
    case 1
        if(x1<=-0.9)
            H=ty;
            if(y1<-1)
            H=tz;
            if(z1<-0.8)
            modo=2;
            end    
            end
        end
        
        
     mover_cubo(H);   
    case 2
        H=ry;
         r_1=r_1+1;
         
          
         mover_cubo(H);
         if(r_1 == 73)
             modo=3;
             r_1=1;
         end
       
     
    case 3 
        H=rz;
         r_1=r_1+1;
         
          
         mover_cubo(H);
         if(r_1 == 73)
             modo=4;
             r_1=1;
         end
    case 4
        r=-5;
          H=rx;
         r_1=r_1+1;
         mover_cubo(H);
         if(r_1 == 75)
             t=0.2;
             modo=5;
             r_1=1;
              
         end
        
    case 5
  
        t=0.2;
      H=tx;
        mover_cubo(H);
       if(x2>=2)
     modo=6;
        end
    case 6
       
      H=tz;
      mover_cubo(H);
       if(z1>6.8)
     modo=7;
       end
    case 7
         H=ty;
      mover_cubo(H);
       if(y1>=4.0)
     modo=8;
       end
    case 8
x1=-1;y1=-1;z1=-1;
x5=-1;y5=1;z5=-1;
x2=1;y2=-1;z2=-1;
x3=-1;y3=-1;z3=1;

x6=-1;y6=1;z6=1;
x4=1;y4=-1;z4=1;
x7=1;y7=1;z7=-1;
x8=1;y8=1;z8=1;
      modo=9;  
         r_1=1;
    case 9  
        r=5;
         H=ry;
         r_1=r_1+1;
         
          
         mover_cubo(H);
         if(r_1 == 75)
             modo=10;
             r_1=1;
         end
              
    case 10     
          H=rz;
         r_1=r_1+1;
         
          
        mover_cubo(H);
         if(r_1 == 73)
             modo=11;
             r_1=1;
         end
    case 11 
          H=rx;
         r_1=r_1+1;
        mover_cubo(H);
         if(r_1 == 73)
         
             modo=12;
           
              
         end
    case 12 % cambio a centro del cubo          
       t=-0.2;
       H=ry*ty;
       mover_punto(H);
       contador_8=contador_8+1;
       if(contador_8==70)
       modo=13;
       end
    case 13
         H=rz*tz;
       mover_punto(H);
       contador_9=contador_9+1;
       if(contador_9==35)
       modo=14;
       end
    case 14
       H=rx;
       mover_punto(H);
    otherwise
    break;
    
end        
if(modo<12)
 dibujar();
else
    dibujar_punto();
end
 pause(0.0005);

  
end

function [] = mover_cubo(H)
global x1 x2 x3 x4 x5 x6 x7 x8
global y1 y2 y3 y4 y5 y6 y7 y8 
global z1 z2 z3 z4 z5 z6 z7 z8 

 [x1,y1,z1] = transformacion([x1 y1 z1],H);
 [x2,y2,z2] = transformacion([x2 y2 z2],H);
 [x3,y3,z3] = transformacion([x3 y3 z3],H);
 [x4,y4,z4] = transformacion([x4 y4 z4],H);
 [x5,y5,z5] = transformacion([x5 y5 z5],H);
 [x6,y6,z6] = transformacion([x6 y6 z6],H);
 [x7,y7,z7] = transformacion([x7 y7 z7],H);
 [x8,y8,z8] = transformacion([x8 y8 z8],H);


end

function [] = mover_punto(H)
global x y z 

 [x,y,z] = transformacion([x y z],H);


end




function [x,y,z] = transformacion(vector,H)

vector_ext=[vector';1];
vector_1=H*vector_ext;
x=vector_1(1);
y=vector_1(2);
z=vector_1(3);
end
 

function [] = dibujar()

global x1 x2 x3 x4 x5 x6 x7 x8
global y1 y2 y3 y4 y5 y6 y7 y8 
global z1 z2 z3 z4 z5 z6 z7 z8
global modo
if(modo <9)
plot3([x3 x4],[y3 y4],[z3 z4],'b','LineWidth',2);
hold on; grid on;
xlabel('x');
ylabel('y');
zlabel('z');
plot3([x1 x3],[y1 y3],[z1 z3],'b','LineWidth',2);
plot3([x2 x4],[y2 y4],[z2 z4],'b','LineWidth',2);
plot3([x1 x2],[y1 y2],[z1 z2],'b','LineWidth',2);
plot3([x1 x5],[y1 y5],[z1 z5],'g','LineWidth',2);
plot3([x3 x6],[y3 y6],[z3 z6],'g','LineWidth',2);
plot3([x2 x7],[y2 y7],[z2 z7],'k','LineWidth',2);
plot3([x4 x8],[y4 y8],[z4 z8],'k','LineWidth',2);
plot3([x5 x7],[y5 y7],[z5 z7],'r','LineWidth',2);
plot3([x8 x7],[y8 y7],[z8 z7],'r','LineWidth',2);
plot3([x5 x6],[y5 y6],[z5 z6],'r','LineWidth',2);
plot3([x8 x6],[y8 y6],[z8 z6],'r','LineWidth',2);

else
plot3([x3 x4]+1,[y3 y4]+5,[z3 z4]+8,'b','LineWidth',2);
hold on; grid on;
xlabel('x');
ylabel('y');
zlabel('z');
plot3([x1 x3]+1,[y1 y3]+5,[z1 z3]+8,'b','LineWidth',2);
plot3([x2 x4]+1,[y2 y4]+5,[z2 z4]+8,'b','LineWidth',2);
plot3([x1 x2]+1,[y1 y2]+5,[z1 z2]+8,'b','LineWidth',2);
plot3([x1 x5]+1,[y1 y5]+5,[z1 z5]+8,'g','LineWidth',2);
plot3([x3 x6]+1,[y3 y6]+5,[z3 z6]+8,'g','LineWidth',2);
plot3([x2 x7]+1,[y2 y7]+5,[z2 z7]+8,'k','LineWidth',2);
plot3([x4 x8]+1,[y4 y8]+5,[z4 z8]+8,'k','LineWidth',2);
plot3([x5 x7]+1,[y5 y7]+5,[z5 z7]+8,'r','LineWidth',2);
plot3([x8 x7]+1,[y8 y7]+5,[z8 z7]+8,'r','LineWidth',2);
plot3([x5 x6]+1,[y5 y6]+5,[z5 z6]+8,'r','LineWidth',2);
plot3([x8 x6]+1,[y8 y6]+5,[z8 z6]+8,'r','LineWidth',2);

end

         set(gca,'DataAspectRatio',[1 1 1],...
             'PlotBoxAspectRatio',[1 1 1],...
             'Xlim',[-10 10],...
             'Ylim',[-10 10],...
             'Zlim',[-10 10])
        

end

function [] = dibujar_punto()

global x y z
plot3([x-1 x+1],[y-1 y-1],[z-1 z-1],'b','LineWidth',2);
hold on; grid on;
plot3([x-1 x-1],[y-1 y-1],[z-1 z+1],'b','LineWidth',2);
plot3([x+1 x+1],[y-1 y-1],[z-1 z+1],'b','LineWidth',2);
plot3([x-1 x+1],[y-1 y-1],[z+1 z+1],'b','LineWidth',2);

plot3([x-1 x+1],[y+1 y+1],[z-1 z-1],'r','LineWidth',2);
plot3([x-1 x-1],[y+1 y+1],[z-1 z+1],'r','LineWidth',2);
plot3([x+1 x+1],[y+1 y+1],[z-1 z+1],'r','LineWidth',2);
plot3([x-1 x+1],[y+1 y+1],[z+1 z+1],'r','LineWidth',2);
% 
plot3([x-1 x-1],[y-1 y+1],[z-1 z-1],'g','LineWidth',2);
plot3([x-1 x-1],[y-1 y+1],[z+1 z+1],'g','LineWidth',2);


plot3([x+1 x+1],[y-1 y+1],[z-1 z-1],'k','LineWidth',2);
plot3([x+1 x+1],[y-1 y+1],[z+1 z+1],'k','LineWidth',2);
xlabel('x');
ylabel('y');
zlabel('z');

         set(gca,'DataAspectRatio',[1 1 1],...
             'PlotBoxAspectRatio',[1 1 1],...
             'Xlim',[-10 10],...
             'Ylim',[-10 10],...
             'Zlim',[-10 10])
        

end