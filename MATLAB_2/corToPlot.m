clear; clc; close all;


% Cambia el nombre según el archivo que quieras ver
points = readmatrix('points.csv');  % por ejemplo: 'points_dragon.csv'

x_pix    = points(:,1);
y_pix    = points(:,2);
y_pix = -y_pix;             % invierte el eje y
% centrar en el origen
x0 = x_pix - mean(x_pix);
y0 = y_pix - mean(y_pix);

% escalar para que quepa en |x|,|y| <= 0.16 (uso un poco menos por seguridad)
max_abs = max([abs(x0); abs(y0)]);
%scale  = 0.14 / max_abs;     % 0.14 en vez de 0.16 para dejar margen
scale = 1;
xd_geom = scale * x0;
yd_geom = scale * y0;

% opcional: cerrar la trayectoria
xd_geom(end+1) = xd_geom(1);
yd_geom(end+1) = yd_geom(1);

figure;
plot(xd_geom, yd_geom, 'k.-'); axis equal; grid on;
title('Trayectoria geométrica del robot en {b}');
xlabel('x [m]'); ylabel('y [m]');

