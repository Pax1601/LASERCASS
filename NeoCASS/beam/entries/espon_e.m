function s = espon_e(x, width)
% s = espon(x, width)
% Trasforma il numero x nella stringa s nella forma s = 'x.xxe+b'
% viene massimizzato il numero di cifre significative utilizzando 
% width caratteri
%
%-------------------------------------------------------------------------------
% 2012/2013 v1.0
% 22-01-2015 v1.1
% 25-01-2016 v1.2 bug fixed
%

% Zero:
if x == 0
	s = '0.';
	return
end

% Segno del numero
sgn = sign(x);

% numero di cifre decimali
if nargin == 1
	n = 3;
end

% |x| = 10^E10 = 10^M * 10^E

E10 = log10(abs(x));

E = fix(E10);
M = E10 - E ;
R = 10.^M;

if E<0
	R = R*10;
	E = E-1;
end

% There are (width-2) spaces available (1 used for the sign, 1
% used for the dot).
% Case E>=0: exponential notation is used only if the number of
%            digits before the dot is larger than (width-2):
%            E+1 > width-2 ==> E > width-3 (E>=0)
%            (a number with exponent E>=0, when fully written,
%            has E+1 digits before the dot)
% Case E<0 : With exponent E<0 there are |E| trailing zeroes when
%            the number is fully written. The exponential notation
%            occupies 3 digits if |E|<10. Exponential notation is
%            used when |E|>=3.

sgnE = sign(E);
% Avoid exponential notation if -3<E<=(width-3)
if -3 < E && E < width-3
	R = abs(x);
   
	% Decimal positions:
	% 1xdot, 1xsign, if E>=0:(E+1)xnumber
	n = width - 3 - E*(sgnE>0);
	E = 0;

elseif E==width-3
	% in this case the effect of rounding must be taken into account
	if round(R) > R
		strE = num2str(abs(E));
		lE = length(strE);

		n = width - 4 - (sgnE<0) -lE;
	else
		R = abs(x);
		% Decimal positions:
		% 1xdot, 1xsign, if E>=0:(E+1)xnumber
		n = width - 3 - E*(sgnE>0);
		E = 0;
	end

else
	strE = num2str(abs(E));
	lE = length(strE);

	n = width - 4 - (sgnE<0) -lE;
end


% Arrotondamento alla n-esima cifra decimale
c = 10^n;
R = round(R*c)/c;

form = ['%1.', num2str(n), 'f'];
if R == round(R)
	s =  [num2str(sgn*R), '.'];
else
	s =  num2str(sgn*R, form);
end

if sgn>0
	s = [' ', s];
end

if abs(E) > 0
	s = [s, 'e', num2str(E)];
elseif E < 0
	s = [s, 'e-', num2str(abs(E))];
end

return
