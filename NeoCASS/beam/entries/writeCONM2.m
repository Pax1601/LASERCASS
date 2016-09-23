function [] = writeCONM2(fid, EID, G, CID, M, X, I)
%
% [] = writeCONM2(fid, EID, G, CID, M, X, I)
%
%
%-------------------------------------------------------------------------------
% 19-04-2016
%

void = '        ';

cstr = void;

fprintf(fid, '%s%s%s%s%s%s%s%s\n', str2char8('CONM2', 'l'), ...
        int2char8(EID), int2char8(G), int2char8(CID), ...
        dbl2char8(M), ...
        dbl2char8(X(1)), dbl2char8(X(2)), dbl2char8(X(3)));

fprintf(fid, '%s%s%s%s%s%s%s\n', str2char8(cstr, 'l'), ...
        dbl2char8(I(1)), dbl2char8(I(2)), dbl2char8(I(3)), ...
        dbl2char8(I(4)), dbl2char8(I(5)), dbl2char8(I(6)));




return
