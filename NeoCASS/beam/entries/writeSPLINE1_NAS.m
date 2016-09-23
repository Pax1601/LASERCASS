function writeSPLINE1_NAS(fid, EID, CAERO, BOX1, BOX2, SETG, DZ, METH, USAGE, NELEM, MELEM)
%
%
%
%-------------------------------------------------------------------------------
% 01-08-2016 Federico Fonte
%


void = '        ';

%             1 2 3 4 5 
fprintf(fid, '%s%s%s%s%s', str2char8('SPLINE1', 'l'), ...
        int2char8(EID), int2char8(CAERO), int2char8(BOX1), int2char8(BOX2), ...
        int2char8(SETG));

checkAndWrite(fid, DZ,    void, 'dbl', 'str', true);
checkAndWrite(fid, METH,  void, 'str', 'str', false);
checkAndWrite(fid, USAGE, void, 'str', 'str', false);

fprintf(fid, '\n');

if ~isempty(NELEM) || ~isempty(MELEM)
	checkAndWrite(fid, NELEM, void, 'int', 'str', true);
	checkAndWrite(fid, MELEM, void, 'int', 'str', true);
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
