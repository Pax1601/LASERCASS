function m = xmlfieldconvert(s)
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
%**************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing
%
%                      Sergio Ricci          <ricci@aero.polimi.it>
%                      Luca Cavagna          <cavagna@aero.polimi.it>
%                      Luca Riccobene        <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     111128      2.0     L.Riccobene      Creation
%
%**************************************************************************
%
% function         m = xmlfieldconvert(s)
%
%
%   DESCRIPTION:  Convert "raw" format from xml2struct: following
%                'Attributes' specifications converts accordingly 'Text'
%                field and then deletes both, leaving only variable name as
%                struct field name and its value, be it double, char etc...
%
%
%         INPUT: NAME           TYPE       DESCRIPTION
%
%                s              struct     "raw" struct coming from
%                                          xml2struct
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%                m              struct     converted output
%
%    REFERENCES:
%
%**************************************************************************

% Avoid to use 'fieldnames' on double when recursing
if isstruct(s)
    fn = fieldnames(s);
else
    m = s;
    return
end

% Keywords
att = 'Attributes';
txt = 'Text';
itm = 'item';

% Cycle through field names
for n = 1:length(fn),
    
    % Descend through nested fields
    if isfield(s, fn{n}) && isstruct(s.(fn{n}))
        
        % If exists 'Attributes' convert field
        subfn = fieldnames(s.(fn{n}));
        if any(ismember(subfn, att))
            
            % Convert value following 'type' specifications and then
            % remove Text and Attributes fields
            
            % This should avoid error when a file is corrupted
            % (uncorrect balance between opening/closure tags)
            if isfield(s.(fn{n}), txt)
                text = s.(fn{n}).(txt);
                s.(fn{n}) = rmfield(s.(fn{n}), txt);
            else
                text = [];
            end
            
            % Local variable for attributes
            attr = s.(fn{n}).(att);
            
            % Remove 'Attribute' field from nested struct
            s.(fn{n}) = rmfield(s.(fn{n}), att);
            
            switch lower(attr.type)
                
                % Struct
                case 'struct'
                    % Do nothing
                    
                % Real number(s): scalar or matrix
                case 'double'
                    sz = sum(str2num(attr.size));
                    switch sz
                        case 0
                            % Doesn't convert since text is empty
                            data = text;
                        case 2
                            % Scalar
                            data = str2double(text);
                        otherwise
                            % Array
                            data = str2num(text);
                            dim  = str2num(attr.size);
                            data = reshape(data, dim);
                    end
                    
                    % Write value
                    s.(fn{n}) = data;
                    
                % Logical
                case 'logical'
                    
                    % Write value
                    s.(fn{n}) = logical(str2num(text));
                    
                % Complex
                case 'complex'
                    
                    % Recover real and imaginary part
                    R = str2double(s.(fn{n}).(itm){1}.(txt));
                    I = str2double(s.(fn{n}).(itm){2}.(txt));
                    
                    % Erase 'item' cell vector
                    s.(fn{n}) = rmfield(s.(fn{n}), itm);
                    
                    % Assign value
                    s.(fn{n}) = complex(R, I);
                    
                % All the other cases (characters or string)
                otherwise
                    
                    % Write value
                    s.(fn{n}) = text;
                    
            end
            
            % Recurse
            s.(fn{n}) = xmlfieldconvert(s.(fn{n}));
            
        end
        
    end
    
end

m = s;
