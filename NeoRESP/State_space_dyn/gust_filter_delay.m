function sys = gust_filter_delay(T, order)
%
Ai = {};
Bi = {};
Ci = {};
Di = {};
%
Ti = (order/T);
T2i = Ti*Ti;
%

%for i=1:order
%
  Ai = -Ti;
  Bi = [Ti];
%  Bi = [Ti 0 ];
%  Bi = [Ti 0 0];
%  Ci = [1; -Ti; T2i];
%Ci = [1; -Ti];
Ci = [1];

%  DD = zeros(3);
  DD = zeros(1);
%  DD(2,1) = Ti;
%  DD(3,1) = -T2i;
%  DD(3,2) = Ti;
  Di = DD;
%
%end

filter = ss(Ai,Bi,Ci,Di);
sys = filter;

for i=2:order
  sys = series(sys, filter);
end

%
