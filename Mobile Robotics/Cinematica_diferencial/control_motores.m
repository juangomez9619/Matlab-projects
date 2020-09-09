clear all
close all

% Para el motor derecho
% G_d(s) = Kd/((tau_d1*s+1)*(tau_d2*s+1))

Kd = 0.9;
tau_d1 = 0.05;
tau_d2 = 0.004;
G_d = tf(Kd,conv([tau_d1  1],[tau_d2  1]))
step(G_d)

% Para el motor izquierdo
% G_i(s) = Ki/((tau_i1*s+1)*(tau_i2*s+1))

Ki = 0.85;
tau_i1 = 0.06;
tau_i2 = 0.006;
G_i = tf(Ki,conv([tau_i1  1],[tau_i2  1]))
hold on,step(G_i)

% Diseno del control para el motor derecho
tsd_d = 0.25; 
Mpd_d = 0.01;  % equivalente 5%
% Mp = exp(-pi*sqrt(1 - zeta^2))*100%
zetad_d = sqrt((log(Mpd_d))^2/(pi^2+(log(Mpd_d))^2))
% ts = 4/(zeta*wn)
wnd_d = 4/(zetad_d*tsd_d)
% Gd_d(s) = wn^2/(s^2 + 2*zeta*wn*s + wn^2)
Gd_d = tf([wnd_d^2],[1  2*zetad_d*wnd_d  wnd_d^2])
step(Gd_d,'--b')
T = 0.114/10;

% 1) Discretizamos la funcion de transferencia
Gdz = c2d(G_d,T)
figure(1),step(Gdz)
% 2) polos deseados en discreto
pdd = -zetad_d*wnd_d + j*wnd_d*sqrt(1-zetad_d^2)
zdd = exp(T*pdd)
% 3) Angulo que debe aportar el controlador 
ang_Gd = 180/pi*angle(evalfr(Gdz,zdd))
ang_Cd = -180 - ang_Gd
% 4) control PI => C(z) = K(z-a)/(z-1)
% ang_C = ang_a - ang(z-1)
ang_a = ang_Cd + 180/pi*angle(zdd-1)
% tan(ang_a) = imag(zdd)/(real(zdd)-a)
a = real(zdd) - imag(zdd)/tand(ang_a)
% 5) calculo de la ganancia
PId = zpk([a],[1],1,T)
K = 1/abs(evalfr(Gdz*PId,zdd))
PId = zpk([a],[1],K,T)

figure(2), rlocus(Gdz*PId), hold on
figure(2), plot(real(zdd),imag(zdd),'xr')
figure(1), step(feedback(PId*Gdz,1),'k')

% Diseno del control para el motor izquierdo
tsd_i = 0.26; 
Mpd_i = 0.007;  % equivalente 5%
% Mp = exp(-pi*sqrt(1 - zeta^2))*100%
zetad_i = sqrt((log(Mpd_i))^2/(pi^2+(log(Mpd_i))^2))
% ts = 4/(zeta*wn)
wnd_i = 4/(zetad_i*tsd_i)
% Gd_d(s) = wn^2/(s^2 + 2*zeta*wn*s + wn^2)
Gd_i = tf([wnd_i^2],[1  2*zetad_i*wnd_i  wnd_i^2])
figure(1),step(Gd_i,'--r')
T = 0.114/10;

% 1) Discretizamos la funcion de transferencia
Giz = c2d(G_i,T)
figure(1),step(Giz,'g')
% 2) polos deseados en discreto
pdi = -zetad_i*wnd_i + j*wnd_i*sqrt(1-zetad_i^2)
zdi = exp(T*pdi)
% 3) Angulo que debe aportar el controlador 
ang_Gi = 180/pi*angle(evalfr(Giz,zdi))
ang_Ci = -180 - ang_Gi
% 4) control PI => C(z) = K(z-a)/(z-1)
% ang_C = ang_a - ang(z-1)
ang_a1 = ang_Ci + 180/pi*angle(zdi-1)
% tan(ang_a) = imag(zdd)/(real(zdd)-a)
a1 = real(zdi) - imag(zdi)/tand(ang_a1)
% 5) calculo de la ganancia
PIi = zpk([a1],[1],1,T)
Kci = 1/abs(evalfr(Giz*PIi,zdi))
PIi = zpk([a1],[1],Kci,T)

figure(2), rlocus(Giz*PIi), hold on
figure(2), plot(real(zdi),imag(zdi),'xg')
figure(1), step(feedback(PIi*Giz,1),'--r')

%% Implementacion del controlador digital

%clear all
edkm1 = 0;
udkm1 = 0;
udkm2 = 0;
wRkm1 = 0;
wRkm2 = 0;
%wRk = 0;

T = 0.114/10;
t = 0:T:30*T;
wRd = ones(1,length(t));
% Para el motor derecho
% Gdz = Omegad(z)/Ud(z) = (0.1257 z + 0.04716)/(z^2 - 0.854 z + 0.04605)
% En potencias negativas de z
% Omegad(z)/Ud(z) = ((0.1257 z + 0.04716)(z^(-2)))/((z^2 - 0.854 z + 0.04605)(z^(-2)))
% Omegad(z)/Ud(z) = (0.1257*z^(-1) + 0.04716*z^(-2))/(1 - 0.854*z^(-1) + 0.04605*z^(-2))
% Eliminando denominadores
% Omegad(z)*(1 - 0.854*z^(-1) + 0.04605*z^(-2)) = Ud(z)*(0.1257*z^(-1) + 0.04716*z^(-2))
% Omegad(z) - 0.854*z^(-1)*Omegad(z) + 0.04605*z^(-2)*Omegad(z) = 0.1257*z^(-1)*Ud(z) + 0.04716*z^(-2)*Ud(z)
% Aplicando transformada z inversa
% wR[k] - 0.854*wR[k-1] + 0.04605*wR[k-2] = 0.1257*ud[k-1] + 0.04716*ud[k-2]
% wR[k] = 0.1257*ud[k-1] + 0.04716*ud[k-2] + 0.854*wR[k-1] - 0.04605*wR[k-2]

% controlador motor derecho
    % C_d(z) = Ud(z)/Ed(z) = (0.76595*z-0.5551)/(z-1)
    % En potencias negativas de z
    % Ud(z)/Ed(z) = ((0.76595*z-0.5551)(z^(-1)))/((z-1)(z^(-1)))
    % Ud(z)/Ed(z) = (0.76595-0.5551*z^(-1))/(1-z^(-1))
    % Eliminando denominadores
    % Ud(z)*(1-z^(-1)) = Ed(z)*(0.76595-0.5551*z^(-1))
    % Ud(z)-z^(-1)Ud(z) = 0.76595*Ed(z)-0.5551*z^(-1)*Ed(z)
    % Aplicando transformada z inversa
    % ud[k] - ud[k-1] = 0.76595*ed[k] - 0.5551*ed[k-1]
    % ud = 0.76595*ed[k] - 0.5551*ed[k-1] + ud[k-1];
    
for i = 1:length(t)
    wRk(i) = 0.1257*udkm1 + 0.04716*udkm2 + 0.854*wRkm1 - 0.04605*wRkm2;
    edk(i) = wRd(i);% lazo abierto
    %edk(i) = wRd(i) - wRk(i); % lazo cerrado
    
    udk(i) = 1;% lazo abierto
    %udk(i) = 0.76595*edk(i) - 0.5551*edkm1 + udkm1; % lazo cerrado
    
    % actualizaciones
    udkm2 = udkm1;
    udkm1 = udk(i);
    wRkm2 = wRkm1;
    wRkm1 = wRk(i);
    edkm1 = edk(i);
end

figure(1),plot(t,wRk,'ob')

% motor izquierdo
eikm1 = 0;
uikm1 = 0;
uikm2 = 0;
wLkm1 = 0;
wLkm2 = 0;

t = 0:T:30*T;
wLd = ones(1,length(t));
% Para el motor derecho
    
for i = 1:length(t)
    wLk(i) = 0.08311*uikm1 + 0.04198*uikm2 + 0.9765*wLkm1 - 0.1237*wLkm2;
    eik(i) = wLd(i);% lazo abierto
    %eik(i) = wLd(i) - wLk(i); % lazo cerrado
    
    uik(i) = 1;% lazo abierto
    %uik(i) = 1.0254*eik(i) - 0.8005*eikm1 + uikm1; % lazo cerrado
    
    % actualizaciones
    uikm2 = uikm1;
    uikm1 = uik(i);
    wLkm2 = wLkm1;
    wLkm1 = wLk(i);
    eikm1 = eik(i);
end

figure(1),plot(t,wLk,'*r')

%%

eikm1 = 0;
uikm1 = 0;
% Para el motor izquierdo
% C_i(z) = Ui(z)/Ei(z) = (1.0254*z-0.8005)/(z-1)
% En potencias negativas
% Ui(z)/Ei(z) = ((1.0254*z-0.8005)(z^(-1)))/((z-1)(z^(-1)))
% Ui(z)/Ei(z) = (1.0254-0.8005*z^(-1))/(1-z^(-1))
% Eliminando denominadores
% Ui(z)*(1-z^(-1)) = (1.0254-0.8005*z^(-1))*Ei(z)
% Ui(z)-z^(-1)*Ui(z) = 1.0254*Ei(z)-0.8005*z^(-1)*Ei(z)
% Aplicando transformada z inversa
% ui[k] - ui[k-1] = 1.0254*ei[k] - 0.8005*ei[k-1]
% ui[k] = 1.0254*ei[k] - 0.8005*ei[k-1] + ui[k-1];
uid = 1.0254*eik - 0.8005*eikm1 + uikm1;