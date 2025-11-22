%[text] Step 1: Create Trapezoidal Velocity Trajectories
clear all;
close all;
clc;
%[text] (1a) Calculate arc length of Lissajous Curve.
% TODO: replace T, xd, yd with your our lissajous curve
T = 2*pi;
t = linspace(0,T,1000);
A = 0.16;
Bb= 0.08;

a = 4;      
b = 3;    


T_path = 2*pi;                      
t_geom = linspace(0, T_path, 1000); 


xd = A * sin(a * t_geom);
yd = Bb* sin(b * t_geom);

% --- (Workspace) comprobar que |x|,|y|<=0.16 ---
fprintf('max |x| = %.3f, max |y| = %.3f\n', max(abs(xd)), max(abs(yd)));

% --- (Zero terminal points) en el espacio geométrico ---
fprintf('Initial Geometric: (%.3f, %.3f)\n', xd(1), yd(1));
fprintf('Final Geometric:  (%.3f, %.3f)\n', xd(end), yd(end));


d = 0;
for i = 1:length(t_geom)-1
    dx = xd(i+1) - xd(i);
    dy = yd(i+1) - yd(i);
    ds = sqrt(dx^2 + dy^2);
    d  = d + ds;
end

L = d;  
fprintf('Arc length L = %.4f m\n', L);
%[text] Determine the average speed, $c$, of end effector over `tfinal` seconds.
tfinal = 15;                
c = L / tfinal;              
fprintf('Average speed c = %.4f m/s\n', c);

if c > 0.25
    warning('Average speed c > 0.25 m/s. Ajusta A,Bbo tfinal.');
end


%[text] Use forward-euler method to numerically approximate $\\alpha(t)$
% normalized trapezoidal curve
g = @(t, T, ta) (T/(T-ta))*((t < ta) .* (t / ta) + ...
                            (t >= ta & t <= (T - ta)) .* 1 + ...
                            (t > (T - ta)) .* ((T - t) / ta));

dt = 0.002;
t = 0:dt:tfinal;
ta = 0.2 * tfinal;           % tiempo de rampa (20% del total)

% --- 4) Forward–Euler para alpha(t) usando ecuación (7) ---

alpha = zeros(size(t));      % alpha(0) = 0

for i = 1:length(t)-1
    
    % Derivadas de la Lissajous respecto a alpha:
    % x_d'(alpha) = A*a*cos(a*alpha)
    % y_d'(alpha) = B*b*cos(b*alpha)
    xdot = A * a * cos(a * alpha(i));
    ydot = Bb* b * cos(b * alpha(i));
    
    % velocidad geométrica (magnitud de la derivada respecto a alpha)
    s_geom = sqrt(xdot^2 + ydot^2);
    s_geom = max(s_geom, 1e-6);  % evitar división por cero
    
    % velocidad cartesiana deseada en este instante:
    % v_des(t) = c * g(t)  (perfil trapezoidal escalado por c)
    v_des = c * g(t(i), tfinal, ta);
    
    % ecuación (7): alpha_dot = v_des / ||d p_d / d alpha||
    alpha_dot = v_des / s_geom;
    
    % Forward–Euler: alpha(i+1) = alpha(i) + alpha_dot * dt
    alpha(i+1) = alpha(i) + alpha_dot * dt;
end

% Ajuste para que alpha(tfinal) = T_path exactamente (periodo geométrico)
alpha = alpha * (T_path / alpha(end));

%[text] Plot trajectory
x_traj = A * sin(a * alpha);
y_traj = Bb* sin(b * alpha);
fprintf('real start: (%.3f, %.3f)\n', x_traj(1), y_traj(1));
fprintf('real final:  (%.3f, %.3f)\n', x_traj(end), y_traj(end));

figure;
plot(x_traj, y_traj, 'LineWidth', 1.5);
axis equal; grid on;
xlabel('x [m]');
ylabel('y [m]');
title('Lissajous trajectory with time scaling');
xlim([-0.16 0.16]);
ylim([-0.16 0.16]);

%[text]  Plot  alpha(t) 

figure;
plot(t, alpha, 'LineWidth', 2);
grid on;
xlabel('time (s)');
ylabel('\alpha(t)');
title('Plot of \alpha(t)');
yline(T_path, 'k--', 'LineWidth', 2);
legend('\alpha', 'T (period)', 'Location', 'southeast');

%[text] Plot the speed of the trajectory as function of time.
vx = diff(x_traj) / dt;    % (x[k] - x[k-1]) / dt
vy = diff(y_traj) / dt;    % (y[k] - y[k-1]) / dt
v  = sqrt(vx.^2 + vy.^2);  % v[k]

% Vector de tiempo para v[k]
t_v = t(2:end);            % mismo tamaño que v

% Gráfica de v[k]
figure;
plot(t_v, v, 'LineWidth', 3); hold on;

% Línea negra discontinua en y = c (velocidad promedio)
yline(c, 'k--', 'c (average velocity)');

% Línea roja discontinua en y = 0.25 (límite de velocidad)
yline(0.25, 'r--', '0.25 m/s');

ylim([0 0.3]);
grid on;
title('Trajectory Velocity');
xlabel('time (s)');
ylabel('velocity v[k] (m/s)');
legend('velocity', 'average velocity', 'velocity limit', ...
       'Location', 'southoutside', 'Orientation', 'horizontal');
hold off;
%%
%[text] ## Step 2: Forward Kinematics
%[text] (2c) Calculate T0
% these values were obtained from the URDF directly
L1 = 0.2435;
L2 = 0.2132;
W1 = 0.1311;
W2 = 0.0921;
H1 = 0.1519;
H2 = 0.0854;

% home position of end effector
M = [-1 0 0 L1+L2;
    0 0 1 W1+W2;
    0 1 0 H1-H2;
    0 0 0 1];

% screw axes
S1 = [0 0 1 0 0 0]';
S2 = [0 1 0 -H1 0 0]';
S3 = [0 1 0 -H1 0 L1]';
S4 = [0 1 0 -H1 0 L1+L2]';
S5 = [0 0 -1 -W1 L1+L2 0]';
S6 = [0 1 0 H2-H1 0 L1+L2]';
S = [S1 S2 S3 S4 S5 S6];

% body screw axes
B1 = ECE569_Adjoint(M)\S1;
B2 = ECE569_Adjoint(M)\S2;
B3 = ECE569_Adjoint(M)\S3;
B4 = ECE569_Adjoint(M)\S4;
B5 = ECE569_Adjoint(M)\S5;
B6 = ECE569_Adjoint(M)\S6;
B = [B1 B2 B3 B4 B5 B6];

% joint angles
theta0 = [-1.6800   -1.4018   -1.8127   -2.9937   -0.8857   -0.0696]';

% calculate the 4x4 matrix representing the transition
% from end effector frame {b} to the base frame {s} at t=0: Tsb(0)

% TODO: implement ECE569_FKinSpace and ECE569_FKinBody
T0_space = ECE569_FKinSpace(M,S,theta0)
T0_body = ECE569_FKinBody(M,B,theta0)
T0_space-T0_body
T0 = T0_body;
%[text] Calculate Tsd at every time step.
% Calculate Tsd(t) for t=0 to t=tfinal
% Tsd(t) = T0 * Td(t)
N = length(t);
Tsd = zeros(4,4,N);
for i=1:N
    % Tsd(:,:,i) = ...
end
%%
%[text] (2d) Plot (x,y,z) in the s frame
T_path = 2*pi;          % periodo fundamental
N = 12000;
t = linspace(0, T_path, N);

A = 0.16;
Bb= 0.08;
a = 4;
b = 3;

xd = A * sin(a * t);
yd = Bb* sin(b * t);

% Chequeo en {b}
fprintf('geom start (b): (%.4f, %.4f)\n', xd(1), yd(1));
fprintf('geom final (b): (%.4f, %.4f)\n\n', xd(end), yd(end));

%% --- Construir T_sb(t) = T0 * Td(t) ---
Tsd = zeros(4,4,N);
for i = 1:N
    pd = [xd(i); yd(i); 0];   % p_d(t)
    Rd = eye(3);              % R_d(t) = I
    Td = [Rd, pd;
          0 0 0 1];

    Tsd(:,:,i) = T0 * Td;
end

xs = squeeze(Tsd(1,4,:));
ys = squeeze(Tsd(2,4,:));
zs = squeeze(Tsd(3,4,:));

% Chequeo en {s}
fprintf('start in {s}:  (%.4f, %.4f, %.4f)\n', xs(1), ys(1), zs(1));
fprintf('final in {s}:  (%.4f, %.4f, %.4f)\n', xs(end), ys(end), zs(end));
fprintf('difference:    (%.4e, %.4e, %.4e)\n\n', ...
        xs(end)-xs(1), ys(end)-ys(1), zs(end)-zs(1));

%% --- Plot 3D ---
figure;
plot3(xs, ys, zs, 'LineWidth', 1.5); hold on; grid on;

plot3(xs(1),  ys(1),  zs(1),  'go', 'MarkerSize', 10, 'LineWidth', 2);
plot3(xs(end),ys(end),zs(end),'rx', 'MarkerSize', 10, 'LineWidth', 2);

title('End-effector trajectory in \{s\} frame');
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
legend('Trajectory', 'Start', 'End', 'Location', 'bestoutside');
axis equal;
view(3);
hold off;

%%
%[text] ## Step 3: Inverse Kinematics
initialguess = theta0;
Td = T0;

% you need to implement IKinBody
%ECE569_IKinBody(B,M,Td,theta0,1e-6,1e-6);

ECE569_IKinBody(B, M, Td, theta0, 1e-6, 1e-6)  
[thetaSol, success] = ECE569_IKinBody(B, M, Td, theta0, 1e-7, 1e-7);
if (~success)
    close(f);
    error('Error. \nCould not perform IK at index %d.',1)
end
%%
%[text] (3c) Perform IK at each time step
thetaAll = zeros(6, N);
thetaAll(:,1) = theta0;

eomg = 1e-8;
ev   = 1e-8;

f = waitbar(0, sprintf('Inverse Kinematics (1/%d) complete.', N));

for k = 2:N
    %  initial guess
    initialguess = thetaAll(:, k-1);

    % 
    Td = Tsd(:, :, k);

    % IK  body
    [thetaSol, success] = ECE569_IKinBody(B, M, Td, initialguess, eomg, ev);

    if ~success
        close(f);
        error('Error.\nCould not perform IK at index %d.', k);
    end

    thetaAll(:, k) = thetaSol;

    waitbar(k / N, f, sprintf('Inverse Kinematics (%d/%d) complete.', k, N));
end
close(f);

%%
%[text] (3c) Verify that the joint angles don't change very much
dj = diff(thetaAll');
plot(t(1:end-1), dj)
title('First Order Difference in Joint Angles')
legend('J1','J2','J3','J4','J5','J6','Location','northeastoutside')
grid on
xlabel('time (s)')
ylabel('first order difference (rad)')
%%
%[text] (3d) Verify that the joints we found actually trace out our trajectory (forward kinematics)
actualTsd = zeros(4,4,N);

for i = 1:N
    % Forward kinematics en el marco espacial {s}
    % Usamos los ángulos que salieron del IK en cada instante
    actualTsd(:,:,i) = ECE569_FKinSpace(M, S, thetaAll(:, i));
end

% Extraer p(t) = (x,y,z) de las Tsd verificadas
xs = actualTsd(1,4,:);
ys = actualTsd(2,4,:);
zs = actualTsd(3,4,:);

figure;
plot3(xs(:), ys(:), zs(:), 'LineWidth', 1);
title('Verified Trajectory in \{s\} frame');
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
hold on;
plot3(xs(1),  ys(1),  zs(1),  'go', 'MarkerSize', 10, 'LineWidth', 2);   % inicio
plot3(xs(end),ys(end),zs(end),'rx', 'MarkerSize', 10, 'LineWidth', 2);   % final
legend('Trajectory', 'Start', 'End', 'Location', 'best');
grid on;
hold off;

%%
%[text] (3e) Verify that the end effector does not enter a kinematic singularity, by plotting the determinant of your body jacobian
body_dets = zeros(N,1);

for i = 1:N
   
    Jb = ECE569_JacobianBody(B, thetaAll(:, i));

   
    body_dets(i) = det(Jb);
end

figure;
plot(t, body_dets, 'LineWidth', 1.5);
title('Manipulability (determinant of body Jacobian)');
grid on;
xlabel('time (s)');
ylabel('det of J_B');
%%
%[text] (3f) Save to CSV File
% you can play with turning the LEDs on and off
led = ones(N,1);

% save to the CSV file
data = [t' thetaAll' led];

% TODO: change the csv filename to your purdue ID
writematrix(data, 'ecorream.csv')

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":43.2}
%---
