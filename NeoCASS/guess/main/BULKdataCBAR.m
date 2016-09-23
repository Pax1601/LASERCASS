%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2008 - 2011 
% 
% Sergio Ricci (sergio.ricci@polimi.it)
%
% Politecnico di Milano, Dipartimento di Ingegneria Aerospaziale
% Via La Masa 34, 20156 Milano - ITALY
% 
% This file is part of NeoCASS Software (www.neocass.org)
%
% NeoCASS is free software; you can redistribute it and/or
% modify it under the terms of the GNU General Public
% License as published by the Free Software Foundation;
% either version 2, or (at your option) any later version.
%
% NeoCASS is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied
% warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
% PURPOSE.  See the GNU General Public License for more
% details.
%
% You should have received a copy of the GNU General Public
% License along with NeoCASS; see the file GNU GENERAL 
% PUBLIC LICENSE.TXT.  If not, write to the Free Software 
% Foundation, 59 Temple Place -Suite 330, Boston, MA
% 02111-1307, USA.
%

%
%--------------------------------------------------------------------------------------------------
% BULKdataCBAR.m writes fields necessary to define bar characteristics
% 
% |    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |   10    |
%    CBAR       EID       PID       GA        GB        X1        X2         X3       OFT 
%               PA        PB        W1A       W2A       W3A       W1B        W2B      W3B       
% 
% 
% Called by:    writeCBAR2file.m
% 
% Calls:        cnvt2_8chs.m
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
function [] = BULKdataCBAR(fid, EID, PID, GA, GB, X1, X2, X3, OFT, PA, PB, W1A, W2A, W3A, W1B, W2B, W3B)

%--------------------------------------------------------------------------
% Define field 1: CBAR
%
fprintf(fid, 'CBAR    ');

%--------------------------------------------------------------------------
% Define field 2: EID
%
[EIDstr] = cnvt2_8chs(EID);
fprintf(fid, '%c', EIDstr);

%--------------------------------------------------------------------------
% Define field 3: PID
%
[PIDstr] = cnvt2_8chs(PID);
fprintf(fid, '%c', PIDstr);

%--------------------------------------------------------------------------
% Define field 4: GA
%
[GAstr] = cnvt2_8chs(GA);
fprintf(fid, '%c', GAstr);

%--------------------------------------------------------------------------
% Define field 5: GB
%
[GBstr] = cnvt2_8chs(GB);
fprintf(fid, '%c', GBstr);

%--------------------------------------------------------------------------
% Define field 6: X1
%
if isequal(X1, 0)
    fprintf(fid, '%c', '0.0     ');% INTRODUCED 2008-04-28
else
    if isequal( abs(X1), floor(abs(X1)) )
        X1 =[num2str(X1) '.0'];
        X1(length(X1)+1:8)=' ';
        fprintf(fid, '%c', X1);
    else
        [X1str] = cnvt2_8chs(X1);
        fprintf(fid, '%c', X1str);
    end
end

%--------------------------------------------------------------------------
% Define field 7: X2
%
if isequal(X2, 0)
    fprintf(fid, '%c', '0.0     ');% INTRODUCED 2008-04-28
else
    if isequal( abs(X2), floor(abs(X2)) )
        X2 =[num2str(X2) '.0'];
        X2(length(X2)+1:8)=' ';
        fprintf(fid, '%c', X2);
    else
        [X2str] = cnvt2_8chs(X2);
        fprintf(fid, '%c', X2str);
    end
end
    
%--------------------------------------------------------------------------
% Define field 8: X3
%
if isequal(X3, 0)
    fprintf(fid, '%c', '0.0     ');% INTRODUCED 2008-04-28
else
    if isequal( abs(X3), floor(abs(X3)) )
        X3 =[num2str(X3) '.0'];
        X3(length(X3)+1:8)=' ';
        fprintf(fid, '%c', X3);
    else
        [X3str] = cnvt2_8chs(X3);
        fprintf(fid, '%c', X3str);
    end
end
   
%--------------------------------------------------------------------------
% Define field 9: OFT
%
[OFTstr] = cnvt2_8chs(OFT);
fprintf(fid, '%c', OFTstr);

%--------------------------------------------------------------------------
% New line
%
fprintf(fid, '\n');

%--------------------------------------------------------------------------
% Define second line
%--------------------------------------------------------------------------

if (nargin == 17) & ~isequal([W1A; W2A; W3A],zeros(3,1))
    
    %----------------------------------------------------------------------
    % Define field 1, new line: empty
    %----------------------------------------------------------------------
    
    fprintf(fid, '        ');
    
    %----------------------------------------------------------------------
    % Define field 2, new line: PA
    %----------------------------------------------------------------------
    
    [PAstr] = cnvt2_8chs(PA);
    fprintf(fid, '%c', PAstr);

    %----------------------------------------------------------------------
    % Define field 3, new line: PB
    %----------------------------------------------------------------------
    
    [PBstr] = cnvt2_8chs(PB);
    fprintf(fid, '%c', PBstr);
    
    %----------------------------------------------------------------------
    % Define field 4, new line: W1A
    %----------------------------------------------------------------------
    
    if isequal(W1A, 0)
        fprintf(fid, '%c', '0.0     ');% INTRODUCED 2008-04-28
    else
        if isequal( abs(W1A), floor(abs(W1A)) )
            W1A =[num2str(W1A) '.0'];
            W1A(length(W1A)+1:8)=' ';
            fprintf(fid, '%c', W1A);
        else
            [W1Astr] = cnvt2_8chs(W1A);
            fprintf(fid, '%c', W1Astr);
        end
    end
    
%     [W1Astr] = cnvt2_8chs(W1A);
%     fprintf(fid, '%c', W1Astr);
    
    %----------------------------------------------------------------------
    % Define field 5, new line: W2A
    %----------------------------------------------------------------------
    
    if isequal(W2A, 0)
        fprintf(fid, '%c', '0.0     ');% INTRODUCED 2008-04-28
    else
        if isequal( abs(W2A), floor(abs(W2A)) )
            W2A =[num2str(W2A) '.0'];
            W2A(length(W2A)+1:8)=' ';
            fprintf(fid, '%c', W2A);
        else
            [W2Astr] = cnvt2_8chs(W2A);
            fprintf(fid, '%c', W2Astr);
        end
    end
    
    %----------------------------------------------------------------------
    % Define field 6, new line: W3A
    %----------------------------------------------------------------------
    
    if isequal(W3A, 0)
        fprintf(fid, '%c', '0.0     ');% INTRODUCED 2008-04-28
    else
        if isequal( abs(W3A), floor(abs(W3A)) )
            W3A =[num2str(W3A) '.0'];
            W3A(length(W3A)+1:8)=' ';
            fprintf(fid, '%c', W3A);
        else
            [W3Astr] = cnvt2_8chs(W3A);
            fprintf(fid, '%c', W3Astr);
        end
    end
    
    %----------------------------------------------------------------------
    % Define field 7, new line: W1B
    %----------------------------------------------------------------------
    
    if isequal(W1B, 0)
        fprintf(fid, '%c', '0.0     ');% INTRODUCED 2008-04-28
    else
        if isequal( abs(W1B), floor(abs(W1B)) )
            W1B =[num2str(W1B) '.0'];
            W1B(length(W1B)+1:8)=' ';
            fprintf(fid, '%c', W1B);
        else
            [W1Bstr] = cnvt2_8chs(W1B);
            fprintf(fid, '%c', W1Bstr);
        end
    end
    
    %----------------------------------------------------------------------
    % Define field 8, new line: W2B
    %----------------------------------------------------------------------
    
    if isequal(W2B, 0)
        fprintf(fid, '%c', '0.0     ');% INTRODUCED 2008-04-28
    else
        if isequal( abs(W2B), floor(abs(W2B)) )
            W2B =[num2str(W2B) '.0'];
            W2B(length(W2B)+1:8)=' ';
            fprintf(fid, '%c', W2B);
        else
            [W2Bstr] = cnvt2_8chs(W2B);
            fprintf(fid, '%c', W2Bstr);
        end
    end
    
    %----------------------------------------------------------------------
    % Define field 9, new line: W3B
    %----------------------------------------------------------------------
    
    if isequal(W3B, 0)
        fprintf(fid, '%c', '0.0     ');% INTRODUCED 2008-04-28
    else
        if isequal( abs(W3B), floor(abs(W3B)) )
            W3B =[num2str(W3B) '.0'];
            W3B(length(W3B)+1:8)=' ';
            fprintf(fid, '%c', W3B);
        else
            [W3Bstr] = cnvt2_8chs(W3B);
            fprintf(fid, '%c', W3Bstr);
        end
    end
    
    %----------------------------------------------------------------------
    % New line
    %----------------------------------------------------------------------

    fprintf(fid, '\n');    
    
end
