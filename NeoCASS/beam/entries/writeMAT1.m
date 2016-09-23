function [] = writeMAT1(fid, MID, E, G, NU, RHO)
%
%
%-------------------------------------------------------------------------------
% 31-03-2015
%

void = '        ';

fprintf(fid, '%s%s', str2char8('MAT1','l'), int2char8(MID));

checkAndWrite(fid, E,   void, 'dbl', 'str');
checkAndWrite(fid, G,   void, 'dbl', 'str');
checkAndWrite(fid, NU,  void, 'dbl', 'str');
checkAndWrite(fid, RHO, void, 'dbl', 'str');
fprintf(fid, '\n');

return
