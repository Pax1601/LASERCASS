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
%     RBE0     IDRB0      IDN      IDS1      IDS2      IDS3      IDS4      IDS5      IDS6    
%              IDS7      IDS8
%  
% 
% 
% IDRB0, Identity Number of RB0.
% 
% IDN, Identity Number of master node.
% 
% IDS1-IDS4, Identity Number of 4 slave nodes.
% 
% 
% Called by:    writeCBAR2file.m
% 
% Calls:        cnvt2_8chs.m
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
function [] = BULKdataRB0(fid, IDRB0, IDN, IDS1, IDS2, IDS3, IDS4, IDS5, IDS6, IDS7, IDS8)


%--------------------------------------------------------------------------------------------------
% Define field 1: RB0
%--------------------------------------------------------------------------------------------------

fprintf(fid, 'RBE0    ');


%--------------------------------------------------------------------------------------------------
% Define field 2: IDRB0
%--------------------------------------------------------------------------------------------------

[IDRB0str] = cnvt2_8chs(IDRB0);
fprintf(fid, '%c', IDRB0str);


%--------------------------------------------------------------------------------------------------
% Define field 3: IDN
%--------------------------------------------------------------------------------------------------

[IDNstr] = cnvt2_8chs(IDN);
fprintf(fid, '%c', IDNstr);


%--------------------------------------------------------------------------------------------------
% Define field 4: IDS1
%--------------------------------------------------------------------------------------------------

[IDS1str] = cnvt2_8chs(IDS1);
fprintf(fid, '%c', IDS1str);


%--------------------------------------------------------------------------------------------------
% Define field 5: IDS2
%--------------------------------------------------------------------------------------------------

[IDS2str] = cnvt2_8chs(IDS2);
fprintf(fid, '%c', IDS2str);



%--------------------------------------------------------------------------------------------------
% Define field 6: IDS3
%--------------------------------------------------------------------------------------------------

[IDS3str] = cnvt2_8chs(IDS3);
fprintf(fid, '%c', IDS3str);


%--------------------------------------------------------------------------------------------------
% Define field 7: IDS4
%--------------------------------------------------------------------------------------------------

[IDS4str] = cnvt2_8chs(IDS4);
fprintf(fid, '%c', IDS4str);


%--------------------------------------------------------------------------------------------------
% Define fields if 8 slave nodes have been defined in input
%--------------------------------------------------------------------------------------------------

if (nargin == 11)       % 8 slave nodes

    %----------------------------------------------------------------------
    % Define field 8: IDS5
    %----------------------------------------------------------------------
    
    [IDS5str] = cnvt2_8chs(IDS5);
    fprintf(fid, '%c', IDS5str); 
    
    %----------------------------------------------------------------------    
    % Define field 9: IDS6
    %----------------------------------------------------------------------    
    
    [IDS6str] = cnvt2_8chs(IDS6);
    fprintf(fid, '%c', IDS6str);
    
    %----------------------------------------------------------------------    
    % Create new line with 1st field empty
    %----------------------------------------------------------------------    
    
    fprintf(fid, '\n        ');
    
    %----------------------------------------------------------------------    
    % Define field 2, new line: IDS7
    %----------------------------------------------------------------------    
    
    [IDS7str] = cnvt2_8chs(IDS7);
    fprintf(fid, '%c', IDS7str);    

    %----------------------------------------------------------------------    
    % Define field 3, new line: IDS8
    %----------------------------------------------------------------------    
    
    [IDS8str] = cnvt2_8chs(IDS8);
    fprintf(fid, '%c', IDS8str);
    
end


%--------------------------------------------------------------------------------------------------
% New line
%--------------------------------------------------------------------------------------------------

fprintf(fid, '\n');


