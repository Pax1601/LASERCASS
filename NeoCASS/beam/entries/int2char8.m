function out = int2char8(n_in, position)
%
% int2char8(n_in, position)
% 
% Create a 8-character string containing integer number n_in
% position : (optional) 
%        'left' : blank spaces added to the right of number
%        'right': (default) blank zeros added to the left of number
%
%-------------------------------------------------------------------------------
% 08-06-2013
%

field = 8;

nstr = num2str(n_in);

if length(nstr)>field
	nstr = nstr(1:field);
	fprintf('Warning: field name longer than field length\n');
end

if (nargin==1) || (strncmp(position, 'r', 1))
   out = [blanks(field-length(nstr)), nstr];
else
   out = [nstr, blanks(field-length(nstr))];
end

return
