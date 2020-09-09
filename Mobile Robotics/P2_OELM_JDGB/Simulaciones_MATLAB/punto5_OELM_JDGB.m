clear all
close all
clc

Punto_inicial=[1.3;0.5];
Orientacion=110;
Punto_final=[2.2;4.5];
% definimos el robot

kinematicModel = differentialDriveKinematics;%definir robot
kinematicModel.WheelRadius = 0.03;
kinematicModel.TrackWidth = 0.15;
%max velocidades de los motores
kinematicModel.WheelSpeedRange = [-10  10]*2*pi;
initialState = [Punto_inicial(1)  Punto_inicial(2) Orientacion*pi/180];   % pose => position in [m], and orientation [rad]
%%punto deseados
xd=Punto_final(1);
yd=Punto_final(2);


% mapa
image = imread('mapa1.png');

grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;

% convMap = binaryOccupancyMap(source,resolution);
convMap = binaryOccupancyMap(bwimage,80);
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
kte = [1 1 1 1 1] ;%ponderación
sampleTime = 0.114/10;% Sample time [s]

dt = sampleTime;
t = 0:sampleTime:200;         % Time array
poses(:,1) = initialState';


%inicializacion control
error_angular_km1 = 0;
error_angular_km2 = 0;
wp_km1 = 0;

vp_km1 = 0;
d_km1 = 0;
d_km2 = 0;
% constantes del controlador velocidad
Ki_v = 0.5;%integral
Kd_v = 0.0;%derivativa
Kp_v = 1;%proporcional
% constantes del controlador angular
Ki = 0.5;
Kd = 0;%derivativa
Kp = 1.5915;%proporcional

b0 = (Kp*sampleTime+Ki*sampleTime^2+Kd)/(sampleTime);
b1 = -(Kp*sampleTime+2*Kd)/(sampleTime);
b2 =  Kd/sampleTime;

f0 = (Kp_v*sampleTime+Ki_v*sampleTime^2+Kd_v)/(sampleTime);
f1 = -(Kp_v*sampleTime+2*Kd_v)/(sampleTime);
f2 =  Kd_v/sampleTime;

%set rate to iterate at
r = rateControl(1/sampleTime);      % rateControl ejecuta el loop a una frecuencia fija
%%

for idx = 1:numel(t)
    position = poses(:,idx)';
    currPose = position(1:2);
    
    theta = poses(3,idx); %% orientación actual del carro
    x = currPose(1);     %%posición en x del carro
    y = currPose(2);     %%posición en y del carro
    distancia_goal(idx)=sqrt((xd-x)^2+(yd-y)^2);
    Ver_d=distancia_goal(idx);
    if distancia_goal(idx)<0.04
        break
    end
    
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
        ranges(isnan(ranges)) = 4;
        distancias = sum(ranges)./length(ranges(:,k));
    end
    %ponderación de las distancias medidas
    %sensorx_R
    for i = 1:length(distancias)
    R=[cosd(sensorAngle_R(i)) -sind(sensorAngle_R(i));sind(sensorAngle_R(i)) cosd(sensorAngle_R(i))];
    on(:,i) = kte(i)*R*[distancias(i);0]+[sensorx_R(i);sensory_R(i)];  
    end
    %sumatoria
    o = sum(on,2);    
    R=[cos(theta) -sin(theta);sin(theta) cos(theta)];
    U_ao = R*o;% vector del obstaculo
    U_gtg= [xd-x;yd-y];  %%  vector hacia la meta
    U_aon= U_ao./norm(U_ao);  %%vector normalizado
    U_gtgn= U_gtg./norm(U_gtg);  %%vector normalizado
    alfa= 0.6;
    U_ao_gtg = U_aon*alfa+(1-alfa)*U_gtgn;
    
    theta_ao_gtg = atan2(U_ao_gtg(2),U_ao_gtg(1));
    theta_gtg= atan2(yd-y,xd-x);
    theta_ao = atan2(U_ao(2),U_ao(1));
    distancias(1,1); %%distancia del sensor delantero
    
    if distancias(1,1) >= 0.4 %libre
        fprintf('\n go to goal')
        %control angular
        vP(idx) = (vp_km1+f0*distancia_goal(idx)+f1*d_km1+f2*d_km2);
        if  distancias(1,2)<= 0.3 || distancias(1,3)<= 0.3 || distancias(1,4)<= 0.3 || distancias(1,5)<= 0.3
            error_angular = wrapToPi(theta_ao-theta);
             vP(idx) = (vp_km1+f0*distancia_goal(idx)+f1*d_km1+f2*d_km2)*0.97;
        else
            error_angular = wrapToPi(theta_gtg-theta);      
        end
 %    <3
    elseif distancias(1,1)>= 0.2 && distancias(1,1)<= 0.4
        
          fprintf('\n go to goal- avoid obstacule')  
        error_angular= wrapToPi(theta_ao_gtg-theta);
        vP(idx) = (vp_km1+f0*distancia_goal(idx)+f1*d_km1+f2*d_km2)*0.83;
    elseif distancias(1,1)< 0.2
        fprintf('\n avoid obstacule')  
        error_angular= wrapToPi(theta_ao-pi-theta);
         vP(idx) = (vp_km1+f0*distancia_goal(idx)+f1*d_km1+f2*d_km2)*0.8;

    end
          if vP(idx)>1
             vP(idx)=1;
          end
     wP = b0*error_angular+b1*error_angular_km1+b2*error_angular_km2+wp_km1;
%         
%      wP=0;
%      vP(idx)=0;
        %actualizaciones
        error_angular_km2 = error_angular_km1;
        error_angular_km1 =  error_angular;
        wp_km1 = wP;
        
        d_km2 = d_km1;
        d_km1 = distancia_goal(idx);
        vp_km1 = vP(idx);
        velocidad=vp_km1;
        
    d_x = vP(idx)*cosd(theta*180/pi);
    d_y= vP(idx)*sind(theta*180/pi);
    d_theta = wP;
    x = x + dt*d_x;
    y = y + dt*d_y;
    theta = theta + dt*d_theta;  
    poses(:,idx+1) = [x; y; theta];  %% actualizar el angulo del robot
    
    % Update visualization
    plotTrvec = [poses(1:2, idx+1); 0];
    plotRot = axang2quat([0 0 1 poses(3, idx+1)]);
    
    fprintf('\n Velocidad %f',velocidad)
    fprintf('\n distancia %f',Ver_d)
    % Delete image of the last robot to prevent displaying multiple robots
     if(mod(idx,10)==0.0 && idx >= 10)
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