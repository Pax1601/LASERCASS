function [] = writeAELINK(fid, ID, LABLD, LABL, C);
%
%
%
%
%
%-------------------------------------------------------------------------------
% 01-08-2016
%

void = '        ';

if isstr(LABL)
	LABL = {LABL};
end

nMaster = length(LABL);
if length(C) ~= nMaster
	error('The number of elements in C and in LABL must be the same');
end



%             1 2 3 
fprintf(fid, '%s%s%s', str2char8('AELINK', 'l'), int2char8(ID), str2char8(LABLD));


lastPositionInLine = 9;
positionInLine = 3;


for iMaster = 1:nMaster

	fprintf(fid, '%s%s', str2char8(LABL{iMaster}), dbl2char8(C(iMaster)));

	positionInLine = positionInLine + 2;

	if positionInLine == lastPositionInLine && iMaster < nMaster
		fprintf(fid, '\n');
		fprintf(fid, '%s', void);
		positionInLine = 1;
	end

end


fprintf(fid,'\n');




return
