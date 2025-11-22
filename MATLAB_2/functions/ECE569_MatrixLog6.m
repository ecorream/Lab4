function expmat = ECE569_MatrixLog6(T)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a transformation matrix T in SE(3).
% Returns the corresponding se(3) representation of exponential 
% coordinates.
% Example Input:
% 
% clear; clc;
% T = [[1, 0, 0, 0]; [0, 0, -1, 0]; [0, 1, 0, 3]; [0, 0, 0, 1]];
% expmat = MatrixLog6(T)
% 
% Output:
% expc6 =
%         0         0         0         0
%         0         0   -1.5708    2.3562
%         0    1.5708         0    2.3562
%         0         0         0         0

[R, p] = ECE569_TransToRp(T);
omgmat = ECE569_MatrixLog3(R);
    if ECE569_NearZero(norm(omgmat))
   
        expmat = [zeros(3,3), p;
                  0 0 0      0];
    else
        
        omgtheta = ECE569_so3ToVec(omgmat);
        [~, theta] = ECE569_AxisAng3(omgtheta);

        
        omgmat_unit = omgmat / theta;

      
        Ginv = (eye(3) / theta) ...
             - 0.5 * omgmat_unit ...
             + (1/theta - 0.5 * cot(theta/2)) * (omgmat_unit * omgmat_unit);

       
        v = Ginv * p;

        vtheta = v * theta;

   
        expmat = [omgmat, vtheta;
                  0 0 0   0];
    end
end