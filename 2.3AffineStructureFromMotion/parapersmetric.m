function Q = parapersmetric(Mh, x, y);
% parapersmetric_ - Metric Transformation in the paraperspective case
%
% Synopsis
%  Q = parapersmetric(Mh, x, y)
%
% Inputs ([]s are optional)
%  (matrix) Mh       2F x 3 estimated camera rotation (motion)
%  (vector) x        F x 1 representing x coodinates of translation
%  (vector) y        F x 1 representing y coorinates of translation
%
% Outputs ([]s are optional)
%  (matrix) Q        3 x 3
%
% References
%  [2] C. J. Poelman and T. Kanade. "A paraperspective factorization
%  method for shape and motion recovery," IEE Trans on Pattern
%  Analysis and Machine Intelligence. VOL 19, NO. 3, MARCH 1997.
F = size(Mh, 1) / 2;
mhT = Mh(1:F, :); % Fx3
nhT = Mh(F+1:2*F, :);
Eq29L = gT(mhT, mhT) ./ repmat(1+x.^2, 1, 6); % gT is below
Eq29R = gT(nhT, nhT) ./ repmat(1+y.^2, 1, 6);
%  G = [Eq29L - Eq29R; ...
%       gT(mhT, nhT) - 0.5*repmat(x,1,6).*repmat(y,1,6).*(Eq29L+Eq29R); ...
%       gT(mhT(1,:), mhT(1,:))]; % (2F+1)x6, gT() is below
%  c = [zeros(2*F, 1); 1]; % (2F+1)x1
G = [Eq29L; ...
    gT(mhT, nhT); ...
    gT(mhT(1,:), mhT(1,:))];
c = [Eq29R; ...
    0.5*repmat(x,1,6).*repmat(y,1,6).*(Eq29L+Eq29R); ...
    ones(1, 6)];
I = pinv(G) * c; % 6x1
L = [I(1) I(2) I(3);  % L = Q*Q'
    I(2) I(4) I(5);
    I(3) I(5) I(6)];
% enforcing positive definiteness
% Reference: CSE252B: Computer Vision II Lecture 16, p7
% http://www-cse.ucsd.edu/classes/sp04/cse252b/notes/lec16/lec16.pdf
%L = (L + L') / 2; % symmetricity for eigen decomposition
[V, D] = eigs(L); % eigen decomposition L = V*D*V';
if find(D < 0), disp('non-positive definite');, end
%D(find(D < 0)) = 0; % positive semidefinite approximation
D(find(D < 0)) = 0.00001; % positive definite approximation, Lij > 0
%L = V * D * V';   % restore
Q = V * sqrt(D);
%QT = chol(L); % Cholesky Decomposition. L is a positive def mat
% sqrtm(L) is also possible (assume Q == Q'), and maybe other ways also
%Q  = QT';
end