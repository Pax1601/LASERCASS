function [] = writeSUPORT(fid, IDlist, Clist);
%
%
%
%-------------------------------------------------------------------------------
% 01-08-2016 Federico Fonte
%



nNodes = length(IDlist);

if length(Clist) ~= nNodes
	error('IDlist and Clist must have the same length');
end



void = '        ';

fprintf(fid, '%s', str2char8('SUPORT', 'l'));

lastPositionInLine = 9;
positionInLine = 1;


for iNode = 1:nNodes

	fprintf(fid, '%s%s', int2char8(IDlist(iNode)), int2char8(Clist(iNode)));

	positionInLine = positionInLine + 1;

	if positionInLine == lastPositionInLine && iNode < nNodes
		fprintf(fid, '\n');
		fprintf(fid, '%s', void);
		positionInLine = 1;
	end

end


fprintf(fid, '\n');


return
