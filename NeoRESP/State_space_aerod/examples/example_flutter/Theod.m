

function Ck = Theod(kk)

% -------------------------------------------------------------------------
% Theodorsen's Function
% -------------------------------------------------------------------------

if abs(kk) < eps
    Ck = 1;
else
    
    % Bessel functions of the first kind.
    J0 = besselj(0,kk);
    J1 = besselj(1,kk);
    
    % Bessel functions of the second kind.
    Y0 = bessely(0,kk);
    Y1 = bessely(1,kk);
    
    H0 = J0 - 1i*Y0;
    H1 = J1 - 1i*Y1;
    % H0 = besselh(0,kk);
    % H1 = besselh(1,kk);
    
    Ck = H1./(H1 + 1i*H0);
end

% Ck(0) = 1;
% Ck(find(kk == 0)) = 1;
Ck(kk == 0) = 1;
