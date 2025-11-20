function T = ECE569_FKinBody(M, Blist, thetalist)
% *** CHAPTER 4: FORWARD KINEMATICS ***
% Takes M: the home configuration (position and orientation) of the
%          end-effector,
%       Blist: The joint screw axes in the end-effector frame when the 
%              manipulator is at the home position,
%       thetalist: A list of joint coordinates.
% Returns T in SE(3) representing the end-effector frame when the joints 
% are at the specified coordinates (i.t.o Body Frame).
% Example Inputs:
% 
% clear; clc;
% M = [[-1, 0, 0, 0]; [0, 1, 0, 6]; [0, 0, -1, 2]; [0, 0, 0, 1]];
% Blist = [[0; 0; -1; 2; 0; 0], [0; 0; 0; 0; 1; 0], [0; 0; 1; 0; 0; 0.1]];
% thetalist = [pi / 2; 3; pi];
% T = FKinBody(M, Blist, thetalist)
% 
% Output:
% T =
%   -0.0000    1.0000         0   -5.0000
%    1.0000    0.0000         0    4.0000
%         0         0   -1.0000    1.6858
%         0         0         0    1.0000

    thetalist = thetalist(:);

    % Si Blist viene en forma 1x(6n) o n×6, intentamos corregir
    % Caso típico de error: Blist es 1xN o 6x1 transpuesto
    if size(Blist, 1) ~= 6 && size(Blist, 2) == 6
        % Si está "acostado" como n×6, lo transponemos a 6×n
        Blist = Blist.';
    end

    % Chequeo de seguridad: Blist debe ser 6×n
    if size(Blist, 1) ~= 6
        error('ECE569_FKinBody: Blist debe ser de tamaño 6xn. Tamaño actual: %dx%d', ...
              size(Blist,1), size(Blist,2));
    end

    n = length(thetalist);
    if size(Blist, 2) ~= n
        error('ECE569_FKinBody: número de columnas de Blist (%d) no coincide con length(thetalist) (%d).', ...
              size(Blist,2), n);
    end

    T = M;
    for i = 1:n
        Bi    = Blist(:, i);                      % 6x1
        se3_i = ECE569_VecTose3(Bi * thetalist(i));  % [Bi * theta_i]^
        T = T * ECE569_MatrixExp6(se3_i);
    end
end