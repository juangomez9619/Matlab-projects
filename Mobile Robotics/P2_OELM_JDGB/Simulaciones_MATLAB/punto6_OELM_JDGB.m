  
clear all;
close all;
clc;
format compact;

%%%%%%%%%%%%%%%%%%%%%%%%%%%
izq = 1; % seguimiento izq 1, seguimiento derecha 0
mapa = 1; %1 para cargar laberinto 1, 2 para cargar laberinto2
%%%%%%%%%%%%%%%%%%%%
if izq
    Punto_inicial=[0.2;2.2];
Orientacion=0;
else
    Punto_inicial=[0.2;2.5];
Orientacion=-90;
end
% definimos el robot
kinematicModel = differentialDriveKinematics;%definir robot
kinematicModel.WheelRadius = 0.03;
kinematicModel.TrackWidth = 0.15;
%max velocidades de los motores
kinematicModel.WheelSpeedRange = [-10  10]*2*pi;
initialState = [Punto_inicial(1)  Punto_inicial(2) Orientacion*pi/180];   % pose => position in [m], and orientation [rad]

% mapa
if mapa == 1
image = imread('laberinto1n.png');
escala_mapa=108;
elseif mapa == 2
    image = imread('laberinto2n.png');
    escala_mapa=100;
end
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;

% convMap = binaryOccupancyMap(source,resolution);
convMap = binaryOccupancyMap(bwimage,escala_mapa);
refFigure = figure('Name','SimpleMap');
show(convMap);

% Get the axes from the figure
ax1 = refFigure.CurrentAxes;

% definicion de los sensores
sensor = rangeSensor;
sensor.Range = [0.02, 4];
sensor.HorizontalAngle = [-15 15]*pi/180;

% ubicacion de los sensores
%tabla marcos de referencia de cada sensor

% 120.369 mm , 0 mm orientación 0° US1
% 95.631 mm , 40.259 mm orientación +45° US2
% 95.631 mm , -38.354 mm orientación -45° US3
% 56mm , 44.95 mm orientación +90° US4
% 56mm , -44.45 mm orientación -90° US5

sensorx_R = [0.09  0.083  0.083   0     0]';
sensory_R = [  0     0.062 -0.062  0.07869 -0.07869]';
sensorAngle_R = [   0   45  -45  90   -90]';
kte = [1 1 1 1 1];%ponderación
sampleTime = 0.114/10;% Sample time [s]

dt = sampleTime;
t = 0:sampleTime:300;         % Time array
poses(:,1) = initialState';

%inicializacion control
error_angular_km1 = 0;
error_angular_km2 = 0;
wp_km1 = 0;
% constantes del controlador angular
Ki = 0.05;
Kd = 0;%derivativa
Kp = 2.5915;%proporcional

%


b0 = (Kp*sampleTime+Ki*sampleTime^2+Kd)/(sampleTime);
b1 = -(Kp*sampleTime+2*Kd)/(sampleTime);
b2 =  Kd/sampleTime;

%set rate to iterate at
r = rateControl(1/sampleTime);      % rateControl ejecuta el loop a una frecuencia fija
%%
estado = 1;
%numel(t)
 tic;
for idx = 1:numel(t)
   
    position = poses(:,idx)';
    currPose = position(1:2);
    
    theta = poses(3,idx); %% orientación actual del carro
    theta*(180/pi);
    x = currPose(1);     %%posición en x del carro
    y = currPose(2);     %%posición en y del carro
    

    
    % tomar medida de los sensores
    for k = 1:length(sensorx_R)
        %% posición del sensor con respecto al MR global(vector rojo)
        sensor_G(:,k) = [cosd(theta*180/pi)  -sind(theta*180/pi);
                         sind(theta*180/pi)   cosd(theta*180/pi)]*[sensorx_R(k);  sensory_R(k)] + [x; y];
                     
         %% vector orientación del sensor con respecto al MR del carro
        d_R(:,k) = [cosd(sensorAngle_R(k))  -sind(sensorAngle_R(k));
                    sind(sensorAngle_R(k))   cosd(sensorAngle_R(k))]*[0.1;  0] + [sensorx_R(k);  sensory_R(k)];
             
        %% orientación del sensor son respecto al MR global (vector azul)       
        d_G(:,k) = [cosd(theta*180/pi)  -sind(theta*180/pi);
                    sind(theta*180/pi)   cosd(theta*180/pi)]*d_R(:,k) + [x; y];
                     
        %%orienctación del sensor en MR global
        sensorAngle_G(k) = 180/pi*atan2(d_G(2,k)-sensor_G(2,k),d_G(1,k)-sensor_G(1,k));

        truePose(k,:) = [sensor_G(:,k)'  pi/180*sensorAngle_G(k)'];
        [ranges(:,k), angles(:,k)] = sensor(truePose(k,:), convMap);
        ranges(isnan(ranges))=4;
        distancias = sum(ranges)./length(ranges(:,k));
    end
     %seguimiento de pared
    seguir_p = 0.1;
   
   
%     pared_izqn= pared_izq./norm(pared_izq);
%     pared_pi=P1-(P1'*pared_izqn)*pared_izqn;
%     pared_pi_n=pared_pi./norm(pared_pi);
%     v_seguir_pi=seguir_p*pared_izqn+(pared_pi-seguir_p*pared_pi_n);
     
%     U_spi = R*v_seguir_pi+[x;y];
%     A_spi=atan2(U_spi(2),U_spi(1));
  
    
    %validaciones para cambiar de estado
    
    
     
      if izq == 1 
      P1=[cosd(sensorAngle_R(4)) -sind(sensorAngle_R(4));sind(sensorAngle_R(4)) cosd(sensorAngle_R(4))]*[distancias(1,4);0]+[sensorx_R(4);sensory_R(4)];  
      P2=[cosd(sensorAngle_R(2)) -sind(sensorAngle_R(2));sind(sensorAngle_R(2)) cosd(sensorAngle_R(2))]*[distancias(1,2);0]+[sensorx_R(2);sensory_R(2)]; 
      pared_izq=P2-P1;
      R=[cos(theta) -sin(theta);sin(theta) cos(theta)];
      U_spi = R*pared_izq;
      Giro = R*P1;
      A_spi=atan2(U_spi(2),U_spi(1)); 
      A_giro= atan2(Giro(2),Giro(1));
    %%%%%%%%%%%%%%%%%%%%%%%%%%
   switch estado 
       case 1 %seguir derecho
        if distancias(1,1) >=3.9 && distancias(1,3) >=3.9 && distancias(1,5) >=3.9
            break;
        end
        fprintf('\n Derecho')
        error_angular = -0.015; %continue derecho
        vP = 0.4;
  
        if distancias(1,1) >=0.25
            estado = 1;
        else    
            estado = 2;
        end
            if distancias(1,2) >=0.20 && distancias(1,4) >=0.1 && distancias(1,4) <=0.4  
            estado=3;          
            end
%            

       case 2 
         error_angular = wrapToPi(A_spi-theta);
         fprintf('\n Seguimiento pared')       
         vP = 0.01;
         if abs(error_angular) <= 0.04
         estado = 1;
         end
        
       case 3
         error_angular = wrapToPi(A_giro-theta);
         vP = 0.56;
         fprintf('\n giro')   
        if distancias(1,2) <=0.22
         estado = 1;
        end
   end
      else % derecha
      
      P1=[cosd(sensorAngle_R(5)) -sind(sensorAngle_R(5));sind(sensorAngle_R(5)) cosd(sensorAngle_R(5))]*[distancias(1,5);0]+[sensorx_R(5);sensory_R(5)];  
      P2=[cosd(sensorAngle_R(3)) -sind(sensorAngle_R(3));sind(sensorAngle_R(3)) cosd(sensorAngle_R(3))]*[distancias(1,3);0]+[sensorx_R(3);sensory_R(3)]; 
      pared_der=P2-P1;
      R=[cos(theta) -sin(theta);sin(theta) cos(theta)];
      U_spi = R*pared_der;
      Giro = R*P1;
      A_spi=atan2(U_spi(2),U_spi(1)); 
      A_giro= atan2(Giro(2),Giro(1));
  
      
      
      
      
      
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %DERECHA
       switch estado 
       case 1 %seguir derecho
        if distancias(1,1) >=3.9 && distancias(1,2) >=3.9 && distancias(1,4) >=3.9
            break;
        end
        fprintf('\n Derecho')
        error_angular = +0.015; %continue derecho
        vP = 0.4;
  
        if distancias(1,1) >=0.25
            estado = 1;
        else    
            estado = 2;
        end
            if distancias(1,3) >=0.20 && distancias(1,5) >=0.10 && distancias(1,5) <=0.4  
            estado=3;          
            end
%            

       case 2 
         error_angular = wrapToPi(A_spi-theta);
         fprintf('\n Seguimiento pared')       
         vP = 0.01;
         if abs(error_angular) <= 0.04
         estado = 1;
         end
        
       case 3
         error_angular = wrapToPi(A_giro-theta);
         vP = 0.56;
         fprintf('\n giro')   
        if distancias(1,3) <=0.22
         estado = 1;
        end
   end
      
      
      
      
      
      
      
      
      end
  
   
    %control angular
        wP = b0*error_angular+b1*error_angular_km1+b2*error_angular_km2+wp_km1;          
        %actualizaciones
        error_angular_km2 = error_angular_km1;
        error_angular_km1 =  error_angular;
        wp_km1 = wP;
    
    d_x = vP*cosd(theta*180/pi);
    d_y= vP*sind(theta*180/pi);
    d_theta = wP;
    
    x = x + dt*d_x;
    y = y + dt*d_y;
    theta = theta + dt*d_theta;  
    poses(:,idx+1) = [x; y; theta];  %% actualizar el angulo del robot
    
    % Update visualization
    plotTrvec = [poses(1:2, idx+1); 0];
    plotRot = axang2quat([0 0 1 poses(3, idx+1)]);
    
    % Delete image of the last robot to prevent displaying multiple robots
    if(mod(idx,80)==0.0 && idx >= 80)
    if idx > 1
       items = get(ax1, 'Children');
     %  delete(items); 
    end
     
    % Plot robot onto known map
      
    plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'chasis5.stl', 'View', '2D', 'FrameSize', 1/4, 'Parent', ax1);
    % Wait to iterate at the proper rate
    waitfor(r);
    end
    
    
end

fprintf('\n FIN')

fprintf('\n tiempo en segundos:')   
tiempo=idx*sampleTime