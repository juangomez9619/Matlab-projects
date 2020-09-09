clc;clear all; close all;
load('dat_fly3.mat')
entradas = [1:2]';
salidas=  1;
%comentario de prueba
%comentario de prueba 2
%comentario de prueba 5
%entrenamiento con red de tipo feedfordward
%definición arquitectura
net = network(1,3,[1;1;1],[1;0;0],[0 0 0;1 0 0;0 1 0],[0 0 1]); 

% número de capas 
% conexiones de bias (1 existe conexion. 0 sin conexión)
% conexión con la entrada con cada capa
% conexiones entre capas, hacer posibles realimentaciones
%conexión con la salida

%--------------------------------------------------------
% cantidad de neuronas y funciones de activación x capa
net.layers{3}.transferFcn = 'logsig';
net.layers{1}.transferFcn = 'logsig';
net.layers{2}.transferFcn = 'logsig';

% configuración entradas y salidas (cantidad)

% comentario 3

%ver arquitectura de la red
view(net);

%%-------------------------------------------------------------------------
% datos de entrenamiento y prueba
%load o cargar previamente en el workspace
xp=[Vo1' Vo2' Vo3' Vo4' Vo5' Vo6' ;Vi1' Vi2' Vi3' Vi4' Vi5' Vi6' ];
yp=[D1' D2' D3' D4' D5' D6'];


%------ datos de entramiento, prueba y validación y caracteristicas del
%entrenamiento
net.layers{1}.size = 1;
net.layers{2}.size = 1;
net.layers{3}.size = 1;

net.divideFcn='dividerand'; % división aletoria
net.trainFcn = 'trainlm'; % 
net.divideParam.trainRatio = 0.8; 
net.divideParam.valRatio   = 0.1; 
net.divideParam.testRatio  = 0.1; 
net.trainParam.epochs = 10000; 
net.trainParam.min_grad = 1e-13;
 net.trainParam.max_fail = 10000;
% entrenamiento de la red neuronal
net = configure(net,entradas,salidas);
[net tr] = train(net,xp,yp);
%gensim(net)

