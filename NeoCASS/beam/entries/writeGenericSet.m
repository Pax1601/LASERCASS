function [] = writeGenericSet(fid, name, fixedEntries, valueList)
%
%
%
%
%
%
%
%
%-------------------------------------------------------------------------------
% 27-07-2016
%



void = '        ';

n1 = 7;

lastUsedField = 9;

nFixed = length(fixedEntries);
nValues = length(valueList);

%fprintf(fid, '$--01--><--02--><--03--><--04--><--05--><--06--><--07--><--08--><--09--><--10-->\n');
fprintf(fid,'%s', str2char8(name, 'l'));

posInLine = 1;
for iFixed = 1:nFixed
	posInLine = posInLine + 1;
	fprintf(fid,'%s', int2char8(fixedEntries(iFixed)));
end

for iValue = 1:nValues
	posInLine = posInLine + 1;
	fprintf(fid,'%s', int2char8(valueList(iValue)));
	if posInLine==lastUsedField && (iValue < nValues)
		fprintf(fid,'\n');
		fprintf(fid,'%s', void);
		posInLine = 1;
	end
end
fprintf(fid, '\n');






