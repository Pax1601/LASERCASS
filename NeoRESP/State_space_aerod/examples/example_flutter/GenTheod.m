

function Ck = GenTheod(p)

% -------------------------------------------------------------------------
% Generalized Theodorsen's Function
% -------------------------------------------------------------------------

%Modified Bessel function of the second kind.
K0 = besselk(0,p);
K1 = besselk(1,p);
Ck = K1./(K0 + K1);

Ck(p == 0) = 1;
