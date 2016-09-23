function writeSPLINE2_NAS(fid, EID, CAERO, BOX1, BOX2, SETG, DZ, DTOR, CID, DTHX, DTHY, USAGE)
%
%
%
%-------------------------------------------------------------------------------
% 01-08-2016 Federico Fonte
%


void = '        ';

%             1 2 3 4 5 
fprintf(fid, '%s%s%s%s%s', str2char8('SPLINE2', 'l'), ...
        int2char8(EID), int2char8(CAERO), int2char8(BOX1), int2char8(BOX2), ...
        int2char8(SETG));

checkAndWrite(fid, DZ,    void, 'dbl', 'str', true);
checkAndWrite(fid, DTOR,  void, 'dbl', 'str', true);
checkAndWrite(fid, CID,   void, 'int', 'str', true);

fprintf(fid, '\n');

if ~isempty(DTHX) || ~isempty(DTHY) || ~isempty(USAGE)
	checkAndWrite(fid, DTHX, void, 'dbl', 'str', true);
	checkAndWrite(fid, DTHY, void, 'dbl', 'str', true);
	fprintf(fid, '%s', void);
	checkAndWrite(fid, USAGE, void, 'str', 'str', false);
	fprintf(fid, '\n');
end




return

function str = nullifzero(value)
void = '        ';
	if ~isempty(value) && value == 0
		str = void;
	else
		str = int2char8(value);
	end

return
