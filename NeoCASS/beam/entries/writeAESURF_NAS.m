function [] = writeAESURF_NAS(fid, ID, LABEL, CID, ALID, EFF, LDW)
%
%
% TODO : add fields on the continuation entry
%
%-------------------------------------------------------------------------------
% 01-08-2016
%

if nargin == 5;
	EFF = [];
	LDW = [];
end





void = '        ';

%             1 2 3 4 
fprintf(fid, '%s%s%s%s', str2char8('AESURF', 'l'), ...
        int2char8(ID), str2char8(LABEL), int2char8(CID(1)), int2char8(ALID(1)));

if length(CID)==2 && length(ALID)==2
	fprintf(fid, '%s%s', int2char8(CID(2)), int2char8(ALID(2)));
else
	fprintf(fid, '%s%s', void, void);
end

checkAndWrite(fid, EFF,   void, 'dbl', 'str', true);
checkAndWrite(fid, LDW,   void, 'dbl', 'str', false);

fprintf(fid, '\n');







return
