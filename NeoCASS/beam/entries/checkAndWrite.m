function [] = checkAndWrite(fid, value,      default_value, ...
                                 value_type, default_type, alsoZero)
%
%
% alsoZero : if true also a zero value will be substituted by the default value
%
%-------------------------------------------------------------------------------
% 27-05-2014
% 08-11-2015 v1.1
%

if nargin < 6
	alsoZero = false;
end

if alsoZero
	condition = ~isempty(value) && value~=0;
else
	condition = ~isempty(value);
end

if condition
	writevalue = value;
	type = value_type;
else
	writevalue = default_value;
	type = default_type;
end


switch type
case 'str'
	fprintf(fid, '%s', str2char8(writevalue));
case 'int'
	fprintf(fid, '%s', int2char8(writevalue));
case 'dbl'
	fprintf(fid, '%s', dbl2char8(writevalue));
end

return
