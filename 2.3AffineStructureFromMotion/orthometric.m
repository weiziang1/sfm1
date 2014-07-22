function Q = orthometric(Rh);
% orthometric_ - Metric Transformation in the orthographic case
%
% Synopsis
%  Q = orthometric_(Rh)
%
% Inputs ([]s are optional)
%  (matrix) Rh       2F x 3
%
% Ouputs ([]s are optional)
%  (matrix) Q        3 x 3
%
% References
%  [3] T. Morita and T. Kanade, "A Sequential Factorization Method
%  for Recovering Shape and Motion From Image Streams," IEEE Trans on
%  Pattern Analysis and Machine Intelligence, Vol. 19, No. 8, Aug 1997,
%  pp858 - 867, Sec 2.3
F = size(Rh, 1) / 2;
ihT = Rh(1:F, :);
jhT = Rh(F+1:2*F, :);
G = [gT(ihT, ihT); gT(jhT, jhT); gT(ihT, jhT)]; % 3Fx6, gT() is below
c = [ones(2*F, 1); zeros(F, 1)]; % 3Fx1
I = pinv(G) * c; % 6x1
L = [I(1) I(2) I(3);  % L = Q*Q'
    I(2) I(4) I(5);
    I(3) I(5) I(6)];
% enforcing positive definiteness
% Reference: CSE252B: Computer Vision II Lecture 16, p7
% http://www-cse.ucsd.edu/classes/sp04/cse252b/notes/lec16/lec16.pdf
%L = (L + L') / 2; % symmetricity for eigen decomposition
[V, D] = eigs(L); % eigen decomposition L = V*D*V';
%D(find(D < 0)) = 0; % positive semidefinite approximation
D(find(D < 0)) = 0.00001; % positive definite approximation, Lij > 0
%L = V * D * V';   % restore
Q = V * sqrt(D);
%QT = chol(L); % Cholesky Decomposition. L is a positive def mat
% sqrtm(L) is also possible (assume Q == Q'), and maybe other ways also
%Q  = QT';
end