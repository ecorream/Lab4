function se3mat = ECE569_VecTose3(V)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a 6-vector (representing a spatial velocity).
% Returns the corresponding 4x4 se(3) matrix.
% Example Input:
% 
% clear; clc;
% V = [1; 2; 3; 4; 5; 6];
% se3mat = VecTose3(V)
% 
% Output:
% se3mat =
%     0    -3     2     4
%     3     0    -1     5
%    -2     1     0     6
%     0     0     0     0 

% se3mat = ... TODO

    V = V(:);   

    if numel(V) ~= 6
        error('ECE569_VecTose3: V debe tener 6 elementos, pero tiene %d', numel(V));
    end

    omega = V(1:3);
    v     = V(4:6);

    omega_hat = [    0      -omega(3)  omega(2);
                  omega(3)      0      -omega(1);
                 -omega(2)  omega(1)       0    ];

    se3mat = [omega_hat, v;
              0 0 0      0];
end
