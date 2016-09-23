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

function INPstr = cnvt2_8chs(INP)
%--------------------------------------------------------------------------------------------------
% cnvt2_8chs.m computes the following: for a numerical value, the output is
% a number which can be contained in 8 character field; for a string input
% value: if string contains specification of file (".txt" or ".doc"),
% specification of file is removed.
% 
% 
% 
% 
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
%
% check if input is a char
if ischar(INP)
    
    index_dot = find(INP == '.');       % find dot in the string to denote file extension

    if isempty(index_dot)               % file extension is NOT found
    
        INP(end+1:8) = ' ';             % fill with empty spaces
        INPstr = INP;
        
    else                                % file extension is found
        
        if index_dot > 8
            
            INP(9:end) = [];
            INPstr = INP;
            
        else
        
            INP(index_dot:end) = [];        % cut after dot
            INP(index_dot:8) = ' ';         % add spaces to fill in 8 characters
            INPstr = INP; 

        end
        
    end
    
% input is a number    
else
    
    INPstr = num2str(INP,'%8.6g');              % convert to string
    lenINP = length(INPstr);            % length of string
    
    if lenINP <= 8                      % Number is shorter then 8 blocks
    
        INPstr(lenINP+1:8) = ' ';       % add spaces to fill in 8 characters
        
    else                                % Number is longer then 8 blocks
    
        I  = find(INPstr == '.');       % find comma in the number
        Ie = find(INPstr == 'e');       % find "e" in the number
        
        if isempty(I)                   % no comma in the number
            
            if (lenINP < 11)
            
                INPstr = [INPstr(1) '.' INPstr(2:4) 'e+' num2str(lenINP-1)];
                
            else
                
                INPstr = [INPstr(1) '.' INPstr(2:3) 'e+' num2str(lenINP-1)];
                
            end
            
        else                            % comma in the string
            
            if isempty(Ie)              % no "e" in the string

                if (I <= 7)             % at least keep one decimal after comma

                    INPstr(9:lenINP) = [];

                else

                    if I <= 11
                        INPstr = [INPstr(1) '.' INPstr(2:4) 'e+' num2str(I-2)];

                    else
                        INPstr = [INPstr(1) '.' INPstr(2:3) 'e+' num2str(I-2)];

                    end

                end
                
            else                        % no comma, but "e" in the string
                
%                INPstr(4:4+lenINP-9) = [];
%                INPstr(5:5+lenINP-9) = [];
              rem = INPstr(Ie:end);
              INPstr = [INPstr(1:8-length(rem)), rem];
%              INPstr = [INPstr(1:length(rem)-1), rem];
              nl = length(INPstr);
              if nl<8
                INPstr = [INPstr, blanks(8-nl)];
              end              
%
            end
            
        end
        
    end

end


