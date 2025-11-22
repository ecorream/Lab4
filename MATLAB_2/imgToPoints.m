clear; close all; clc;

% 1) Cargar la imagen
img = imread('lion.jpg');   % Asegúrate de que 'mapa.jpg' está en la misma carpeta
figure;
imshow(img); axis on; hold on;
title('Haz click siguiendo la trayectoria y al final presiona Enter');

% 2) Seleccionar puntos con el mouse
[x_pix, y_pix] = ginput;   % Haz todos los clicks y al final presiona Enter
close;

% 3) Guardar los puntos en un CSV
points = [x_pix, y_pix];          % matriz N x 2
writematrix(points, 'points.csv');   % crea/reescribe points.csv

fprintf('Se guardaron %d puntos en "points.csv".\n', size(points,1));
