function T = ECE569_MatrixExp6(se3mat)
% ECE569_MatrixExp6
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a se(3) representation of exponential coordinates.
% Returns a T matrix in SE(3) that is achieved by traveling along/about the 
% screw axis S for a distance theta from an initial configuration T = I.
%
% se3mat = [omega_hat * theta, v * theta;
%           0 0 0              0]

    % Extraer la parte de rotación (so3) y traslación
    so3mat = se3mat(1:3, 1:3);
    p_part = se3mat(1:3, 4);

    % Vector omega*theta
    omgtheta = ECE569_so3ToVec(so3mat);

    if ECE569_NearZero(norm(omgtheta))
        % Caso: rotación despreciable -> pura traslación
        % exp([0, p; 0, 0]) = [I, p; 0, 1]
        T = [eye(3), p_part;
             0 0 0   1];
    else
        % Caso: rotación + traslación
        % Obtener theta
        [~, theta] = ECE569_AxisAng3(omgtheta);

        % Matriz de eje unitario: so3mat = omega_hat * theta
        omgmat = so3mat / theta;

        % v (el vector en el twist) se obtiene dividiendo por theta
        v = p_part / theta;

        % Rotación usando la función de rotaciones
        R = ECE569_MatrixExp3(so3mat);  % exp(omega_hat * theta)

        % G(theta) = I*theta + (1-cos(theta))*omega_hat + (theta - sin(theta))*omega_hat^2
        G = eye(3) * theta ...
            + (1 - cos(theta)) * omgmat ...
            + (theta - sin(theta)) * (omgmat * omgmat);

        % Traslación p = G * v
        p = G * v;

        % Matriz homogénea final
        T = [R, p;
             0 0 0 1];
    end
end
