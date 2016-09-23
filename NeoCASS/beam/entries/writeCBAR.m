function [] = writeCBAR(fid, EID, PID, G, X, OFFT, P, W)
%
% WriteNAS_CBAR(fid, EID, PID, G, X, OFFT, P, W)
%
% G [2]
% X [3] or [1]
% OFFT [1]
% P [2]
% W [3 x 2]
%
%-------------------------------------------------------------------------------
% 08-11-2015
%

if ~exist('OFFT', 'var')
	OFFT = [];
end

if ~exist('P', 'var') || isempty(P)
	P = [0,0];
end

if ~exist('W', 'var') || isempty(W)
	W = zeros(3,2);
end

void = '        ';

fprintf(fid, '%s%s%s%s%s', str2char8('CBAR','l'), int2char8(EID), ...
                               int2char8(PID), int2char8(G(1)), int2char8(G(2)));

if length(X) == 1;
	fprintf(fid, '%s%s%s', int2char8(X), void, void);
elseif length(X) == 3
	fprintf(fid, '%s%s%s', dbl2char8(X(1)), dbl2char8(X(2)), dbl2char8(X(3)));
else 
	error('X must be a scalar or an array with 3 elements');
end

checkAndWrite(fid, OFFT, void, 'str', 'str');
fprintf(fid, '\n');

writeSecondLine = (max(P)>0) || (~isempty(find(W)));

if writeSecondLine
	fprintf(fid, '%s', void);
	checkAndWrite(fid, P(1), void, 'int', 'str', true);
	checkAndWrite(fid, P(1), void, 'int', 'str', true);
	fprintf(fid,'%s%s%s%s%s%s', dbl2char8(W(1,1)), dbl2char8(W(2,1)), dbl2char8(W(3,1)), ...
	                            dbl2char8(W(1,2)), dbl2char8(W(2,2)), dbl2char8(W(3,2)));
	fprintf(fid, '\n');
end



return
