function out = str2char8(str_in, position)
%
% str2char8(str_in, position)
% 
% Create a 8-character string containing string str_in
% position : (optional) 
%        'left' : blank spaces added to the right of the string
%        'right': (default) blank zeros added to the left of the string
%
%-------------------------------------------------------------------------------
% 08-06-2013
%

field = 8;

if length(str_in)>field
	str_in = str_in(1:field);
	fprintf('Warning: field name longer than field length\n');
end


if (nargin==1) || (strncmp(position, 'r', 1))
   out = [blanks(field-length(str_in)), str_in];
else
   out = [str_in, blanks(field-length(str_in))];
end

return
