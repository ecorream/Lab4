function Jb = ECE569_JacobianBody(Blist, thetalist)
% ECE569_JacobianBody
% *** CHAPTER 5: VELOCITY KINEMATICS AND STATICS ***
% Takes:
%   Blist:    6xn matrix, each column is a body screw axis B_i
%   thetalist: n×1 vector of joint angles
% Returns:
%   Jb: 6xn body Jacobian matrix

    n = length(thetalist);
    Jb = zeros(6, n);
   
    Jb(:, n) = Blist(:, n);
    
    T = eye(4);
    
    for i = n-1 : -1 : 1
        
        se3_next = ECE569_VecTose3(-Blist(:, i+1) * thetalist(i+1));
        T = T * ECE569_MatrixExp6(se3_next);

       
        Jb(:, i) = ECE569_Adjoint(T) * Blist(:, i);
    end
end