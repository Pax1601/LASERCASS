function writeCAERO_NAS(fid, EID, PID, CP, NSPAN, NCHORD, IGID, ...
                             P1, c1, P2, c2)
%
%
%
%-------------------------------------------------------------------------------
% 12-03-2014 v1.0
% 31-03-2015 v1.1 Panel definition through AEFACT allowed
% 17-09-2015 v1.2 bug fixed
%


void = '        ';

if length(NSPAN)==1 && length(NCHORD)==1
	NSPAN = int2char8(NSPAN);
	NCHORD = int2char8(NCHORD);
	LSPAN = void;
	LCHORD = void;
else
	N = NSPAN;
	L = NCHORD;
	NSPAN = nullifzero(N(1));
	NCHORD = nullifzero(N(2));
	LSPAN = nullifzero(L(1));
	LCHORD = nullifzero(L(2));
end


fprintf(fid, '%s%s%s%s%s%s%s%s%s\n', str2char8('CAERO1', 'l'), ...
        int2char8(EID), int2char8(PID), int2char8(CP), NSPAN, NCHORD, ... 
        LSPAN, LCHORD, int2char8(IGID));
        
fprintf(fid, '%s%s%s%s%s%s%s%s%s\n', void, ...
        dbl2char8(P1(1)), dbl2char8(P1(2)), dbl2char8(P1(3)), dbl2char8(c1), ...
        dbl2char8(P2(1)), dbl2char8(P2(2)), dbl2char8(P2(3)), dbl2char8(c2));




return

function str = nullifzero(value)
void = '        ';
	if value == 0
		str = void;
	else
		str = int2char8(value);
	end

return
