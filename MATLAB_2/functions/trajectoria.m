function reproducir_trayectoria_bonus()
    %%% ----------------- SETUP ROBOT ----------------- %%%
    clearvars -except reproducir_trayectoria_bonus; clc; close all;

    L1 = 0.2435;
    L2 = 0.2132;
    W1 = 0.1311;
    W2 = 0.0921;
    H1 = 0.1519;
    H2 = 0.0854;

    M = [-1 0  0  L1+L2;
          0 0  1  W1+W2;
          0 1  0  H1-H2;
          0 0  0  1];

    S1 = [ 0  0  1      0        0         0 ]';
    S2 = [ 0  1  0    -H1        0         0 ]';
    S3 = [ 0  1  0    -H1        0        L1 ]';
    S4 = [ 0  1  0    -H1        0   (L1+L2)]';
    S5 = [ 0  0 -1    -W1   L1+L2         0 ]';
    S6 = [ 0  1  0  (H2-H1)    0   (L1+L2)]';
    S  = [S1 S2 S3 S4 S5 S6];

    %%% ----------------- CARGAR DATOS ----------------- %%%
    filename = 'ecorream_bonus.csv';   % cambia si tu archivo tiene otro nombre
    data = readmatrix(filename);

    t    = data(:,1);        % tiempo
    q    = data(:,2:7);      % 6 articulaciones
    led  = data(:,8);        % LED 0/1

    N = size(q,1);

    %%% ----------------- FK Y TRAYECTORIA ------------- %%%
    p = zeros(N,3);
    for k = 1:N
        thetalist = q(k,:)';
        T = ECE569_FKinSpace(M, S, thetalist);
        p(k,:) = T(1:3,4)';
    end

    %%% ----------------- FIGURA Y GRAFICOS ------------ %%%
    figure;
    hold on; grid on; axis equal;

    plot3(p(:,1), p(:,2), p(:,3), 'Color', [0.8 0.8 0.8], 'LineWidth', 1);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    margin = 0.02;
    xlim([min(p(:,1))-margin, max(p(:,1))+margin]);
    ylim([min(p(:,2))-margin, max(p(:,2))+margin]);
    zlim([min(p(:,3))-margin, max(p(:,3))+margin]);
    view(3);

    hPoint = plot3(p(1,1), p(1,2), p(1,3), 'r.', 'MarkerSize', 25);
    hTrace = plot3(p(1,1), p(1,2), p(1,3), 'k-', 'LineWidth', 1.5);

    %%% ----------------- CONTROL DE PAUSA ------------- %%%
    isPaused = false;
    stopAnim = false;

    fig = gcf;
    set(fig, 'KeyPressFcn', @keyControl);

    %%% ----------------- PARÁMETROS DE VELOCIDAD ------- %%%
    speed_factor = 0.1;   % >1 = más lento, <1 = más rápido
    step         = 1;   % puedes usar 5 o 10 para saltar muestras

    base_dt = 0.02;     % pausa base (segundos)
    t0 = t(1);

    %%% ----------------- BUCLE DE ANIMACIÓN ------------ %%%
    for k = 1:step:N

        % mientras esté en pausa, espera
        while isPaused && ~stopAnim
            pause(0.05);
            drawnow;
        end

        if stopAnim
            break;
        end

        set(hPoint, 'XData', p(k,1), 'YData', p(k,2), 'ZData', p(k,3));
        set(hTrace, 'XData', p(1:k,1), 'YData', p(1:k,2), 'ZData', p(1:k,3));

        elapsed   = t(k) - t0;
        led_state = led(k);

        title(sprintf('Fila: %d / %d   t = %.3f s   LED = %d   (ESPACIO=Pausa, ESC=Salir)', ...
                      k, N, elapsed, led_state));

        fprintf('Fila %5d de %5d | t = %.3f s | LED = %d\n', ...
                 k, N, elapsed, led_state);

        drawnow limitrate;

        % velocidad de animación controlada por speed_factor
        pause(base_dt * speed_factor);
    end

    disp('Simulación finalizada.');

    %%% ------------- FUNCIÓN ANIDADA: TECLADO ---------- %%%
    function keyControl(~, event)
        switch event.Key
            case 'space'
                isPaused = ~isPaused;   % alterna pausa
            case 'escape'
                stopAnim = true;        % salir
        end
    end
end
