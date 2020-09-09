clear;close;clc
figure, hold on; grid on;
axis([-1 1 -1 1])
x = [0  0.2    0.4 0.6 0.65 0.55 0.62 0.65 0.62 0.55 0.42 0.25 0.1  0.1 0.1 -0.1 -0.1 -0.32 -0.5 -0.6 -0.625 -0.61 -0.59 -0.57 -0.57 -0.62 -0.7 -0.71 -0.69 -0.62 -0.51 -0.4 -0.2 0];
y = [-1 -0.98 -0.91  -0.6 -0.3 0 0.14 0.3 0.44 0.49 0.41 0.44 0.5  0.7 1.0 1.0 0.7 0.8 0.84 0.83 0.75 0.62 0.4 0.24 0.105 -0.05 -0.18 -0.3 -0.5 -0.65 -0.79 -0.9 -0.98 -1];

plot(x,y,'or','LineWidth',2);
plot(x,y,'r','LineWidth',2);
