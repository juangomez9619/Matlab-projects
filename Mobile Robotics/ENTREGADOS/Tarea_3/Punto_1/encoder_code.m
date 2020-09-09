   
clc;clear all; close all;
J=1; B=25; R=1; L=0.01; K=0.18*20; KK=10.5; k1 = 0.8;
a=-4 ; b=-8;  dzi=-1.2; dzs=1; % Modelo 5

% correr la simulación en simulink
%%
clc;
datos = yout2.Data;
rot_a = datos (:,1);
rot_b = datos (:,2);
tiempo = tout;
figure (1),hold on; grid on;
axis([0 20e-3 -.5 10.5])
plot(tiempo,rot_a+5.2,'k','LineWidth',2);
plot(tiempo,rot_b,'r','LineWidth',2);
legend('A','B','interpreter','latex')
%%

%length(tiempo)
format compact


    tiempo_flanco_a = 0;
    t2_a =2;
    t1_a =1;
    velocidad_a = 0;
    %
    tiempo_flanco_b = 0;
    t2_b =2;
    t1_b =1;
    velocidad_b = 0;

length(tiempo)
for i=1:length(tiempo)
    if i~=1
        if (rot_a(i)~=rot_a(i-1)) && (rot_a(i-1)==0) %detección flanco de subida canal A
           tiempo_flanco_a = [tiempo_flanco_a tiempo(i)];
           delta_t_a = tiempo_flanco_a(t2_a) -  tiempo_flanco_a(t1_a);
           if(rot_b(i)==0.0)
           velocidad_inst_a = (60)/(delta_t_a*50);
           else
           velocidad_inst_a = (-60)/(delta_t_a*50);
           end
           velocidad_a = [velocidad_a velocidad_inst_a];
           t1_a=t1_a+1;
           t2_a=t2_a+1;
        end
        
        if (rot_b(i)~=rot_b(i-1)) && (rot_b(i-1)==0) %detección flanco de subida canal B
           tiempo_flanco_b = [tiempo_flanco_b tiempo(i)];
           delta_t_b = tiempo_flanco_b(t2_b) -  tiempo_flanco_b(t1_b);
           if(rot_a(i)==5.0)
           velocidad_inst_b = (60)/(delta_t_b*50);
           else
           velocidad_inst_b = (-60)/(delta_t_b*50);
           end
           velocidad_b = [velocidad_b velocidad_inst_b];
           t1_b=t1_b+1;
           t2_b=t2_b+1;
        end
        
    end
end


if length(tiempo_flanco_a) == length(tiempo_flanco_b)
aux = length(tiempo_flanco_a);
elseif length(tiempo_flanco_a) < length(tiempo_flanco_b)
     aux = length(tiempo_flanco_a);
else
    aux = length(tiempo_flanco_b);
end

 velocidad_rpm(1:aux) = (velocidad_a(1:aux)+velocidad_b(1:aux))/2;
 
 figure(2),plot(tiempo_flanco_a(1:aux),velocidad_rpm,'k','LineWidth',2)
 
 