function writeGRID(fid, ID, CP, P, CD, PS, SEID)
%
%  writeGRID(fid, ID, CP, P, CD)
%
%
%
%-------------------------------------------------------------------------------
% 19-04-2016
%

void = '        ';

if ~exist('PS', 'var'); PS = []; end
if ~exist('SEID', 'var'); SEID = []; end
if ~exist('CD', 'var'); CD = []; end

N = length(ID);

if isempty(CP); CP = zeros(N,1); end;
if isempty(CD); CD = zeros(N,1); end;


for i = 1:N
	fprintf(fid, '%s%s%s%s%s%s%s', str2char8('GRID', 'l'), ...
	        int2char8(ID(i)), int2char8(CP(i)), ...
	        dbl2char8(P(i, 1)), dbl2char8(P(i, 2)), dbl2char8(P(i, 3)), ... 
	        int2char8(CD(i)));

	checkAndWrite(fid, PS, void, 'int', 'str');
	checkAndWrite(fid, SEID, '', 'int', 'str');
	fprintf(fid,'\n');
end

return
