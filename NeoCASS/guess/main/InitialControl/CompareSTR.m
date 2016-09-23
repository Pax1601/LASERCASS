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

function varargout = CompareSTR(str1,str2)
% [index, string] = CompareSTR(str1,str2)

% this function compares the two structure fields and returns in
% varargout{1} a index ==1 if the two structure have the same fields 0
% otherwise, while in varargout{2} saves the field  that
% is not present in str2. 

if isstruct(str1)
    field1 = fieldnames(str1);
    if ~isstruct(str2)
       varargout{1} = 0;
       varargout{2} = field1{1};
       return;
    end
    field2 = fieldnames(str2);
    if length(field1) == length(field2)
       if  all(strcmp(sort(field1),sort(field2)))
           varargout{1} = 1;
           varargout{2} = 'every fields are present';
           for i = 1 : length(field1)
              [exit, string] = CompareSTR(getfield(str1,field1{i}),getfield(str2,field2{i}));
              if exit == 0 
                  varargout{1} = 0;
                  if strcmp(varargout{2},'every fields are present')
                      varargout{2} = [field1(i); string]; 
                  else
                      varargout{2} = [varargout{2};[field1(i); string]];
                  end
              end
           end
           return;
       else
           varargout{1} = 0;
           varargout{2} = field1{strcmp(field1,field2)==0};
           return;
       end
    else
        varargout{1} = 1;
        varargout{2} = 'every fields are present';
        for i = 1: length(field1)
           if isempty(find(strcmp(field1{i},field2))==1)
               varargout{1} = 0;
               if strcmp(varargout{2},'every fields are present')
                   varargout{2} = field1{i};
               else
                   varargout{2} = [varargout{2};field1(i)];
               end
           end
        end
        return;
    end
    
else
    varargout{1} = 1;
    varargout{2} = 'every fields are present';
end