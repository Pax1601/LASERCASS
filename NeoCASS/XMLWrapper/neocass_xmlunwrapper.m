function u = neocass_xmlunwrapper(xml_filename, s)
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
% function         u = neocass_xmlunwrapper(xml_filename, s)
%
%
%   DESCRIPTION:  Based on struct2xml (written by W. Falkena, ASTI,
%                 TUDelft): write struct on XML file in NeoCASS format; the
%                 function is embedded in the code as it is to avoid
%                 possible conflict with xml2struct in bioinfo toolbox (a
%                 copy is retained in the /XMLWrapper/ directory).
%
%
%         INPUT: NAME           TYPE       DESCRIPTION
%
%                xml_filename   string     XML file name
%
%                s              struct     aircraft model
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%                u              struct     if empty writing to XML file was
%                                          successful otherwise it stores
%                                          "unwrapped" model
%
%    REFERENCES:
%
%**************************************************************************

% "Unwrap" struct
u = struct;
u.root = xmlunwrap(s);

% Add root attributes
att = 'Attributes';
u.root.(att).xml_tb_version = '3.2.1';
u.root.(att).idx  = 1;
u.root.(att).size = '1 1';
u.root.(att).type = 'struct';

% Write XML
try
    
    struct2xml(u, xml_filename);
    u = [];
    
catch excep
    fprintf('\n ## %s ##\n', excep.message);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% STRUCT2XML (BEGIN)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function varargout = struct2xml( s, varargin )
%Convert a MATLAB structure into a xml file
% [ ] = struct2xml( s, file )
% xml = struct2xml( s )
%
% A structure containing:
% s.XMLname.Attributes.attrib1 = "Some value";
% s.XMLname.Element.Text = "Some text";
% s.XMLname.DifferentElement{1}.Attributes.attrib2 = "2";
% s.XMLname.DifferentElement{1}.Text = "Some more text";
% s.XMLname.DifferentElement{2}.Attributes.attrib3 = "2";
% s.XMLname.DifferentElement{2}.Attributes.attrib4 = "1";
% s.XMLname.DifferentElement{2}.Text = "Even more text";
%
% Will produce:
% <XMLname attrib1="Some value">
%   <Element>Some text</Element>
%   <DifferentElement attrib2="2">Some more text</Element>
%   <DifferentElement attrib3="2" attrib4="1">Even more text</DifferentElement>
% </XMLname>
%
% Please note that the following strings are substituted
% '_dash_' by '-', '_colon_' by ':' and '_dot_' by '.'
%
% Written by W. Falkena, ASTI, TUDelft, 27-08-2010
% On-screen output functionality added by P. Orth, 01-12-2010
% Multiple space to single space conversion adapted for speed by T. Lohuis, 11-04-2011
% Val2str subfunction bugfix by H. Gsenger, 19-9-2011

if (nargin ~= 2)
    if(nargout ~= 1 || nargin ~= 1)
        error(['Supported function calls:' sprintf('\n')...
            '[ ] = struct2xml( s, file )' sprintf('\n')...
            'xml = struct2xml( s )']);
    end
end

if(nargin == 2)
    file = varargin{1};
    
    if (isempty(file))
        error('Filename can not be empty');
    end
    
    if (isempty(strfind(file,'.xml')))
        file = [file '.xml'];
    end
end

if (~isstruct(s))
    error([inputname(1) ' is not a structure']);
end

if (length(fieldnames(s)) > 1)
    error(['Error processing the structure:' sprintf('\n') 'There should be a single field in the main structure.']);
end
xmlname = fieldnames(s);
xmlname = xmlname{1};

%substitute special characters
xmlname_sc = xmlname;
xmlname_sc = strrep(xmlname_sc,'_dash_','-');
xmlname_sc = strrep(xmlname_sc,'_colon_',':');
xmlname_sc = strrep(xmlname_sc,'_dot_','.');

%create xml structure
docNode = com.mathworks.xml.XMLUtils.createDocument(xmlname_sc);

%process the rootnode
docRootNode = docNode.getDocumentElement;

%append childs
parseStruct(s.(xmlname),docNode,docRootNode,[inputname(1) '.' xmlname '.']);

if(nargout == 0)
    %save xml file
    xmlwrite(file,docNode);
else
    varargout{1} = xmlwrite(docNode);
end

% ----- Subfunction parseStruct -----
function [] = parseStruct(s,docNode,curNode,pName)

fnames = fieldnames(s);
for i = 1:length(fnames)
    curfield = fnames{i};
    
    %substitute special characters
    curfield_sc = curfield;
    curfield_sc = strrep(curfield_sc,'_dash_','-');
    curfield_sc = strrep(curfield_sc,'_colon_',':');
    curfield_sc = strrep(curfield_sc,'_dot_','.');
    
    if (strcmp(curfield,'Attributes'))
        %Attribute data
        if (isstruct(s.(curfield)))
            attr_names = fieldnames(s.Attributes);
            for a = 1:length(attr_names)
                cur_attr = attr_names{a};
                
                %substitute special characters
                cur_attr_sc = cur_attr;
                cur_attr_sc = strrep(cur_attr_sc,'_dash_','-');
                cur_attr_sc = strrep(cur_attr_sc,'_colon_',':');
                cur_attr_sc = strrep(cur_attr_sc,'_dot_','.');
                
                [cur_str,succes] = val2str(s.Attributes.(cur_attr));
                if (succes)
                    curNode.setAttribute(cur_attr_sc,cur_str);
                else
                    disp(['Warning. The text in ' pName curfield '.' cur_attr ' could not be processed.']);
                end
            end
        else
            disp(['Warning. The attributes in ' pName curfield ' could not be processed.']);
            disp(['The correct syntax is: ' pName curfield '.attribute_name = ''Some text''.']);
        end
    elseif (strcmp(curfield,'Text'))
        %Text data
        [txt,succes] = val2str(s.Text);
        if (succes)
            curNode.appendChild(docNode.createTextNode(txt));
        else
            disp(['Warning. The text in ' pName curfield ' could not be processed.']);
        end
    else
        %Sub-element
        if (isstruct(s.(curfield)))
            %single element
            curElement = docNode.createElement(curfield_sc);
            curNode.appendChild(curElement);
            parseStruct(s.(curfield),docNode,curElement,[pName curfield '.'])
        elseif (iscell(s.(curfield)))
            %multiple elements
            for c = 1:length(s.(curfield))
                curElement = docNode.createElement(curfield_sc);
                curNode.appendChild(curElement);
                if (isstruct(s.(curfield){c}))
                    parseStruct(s.(curfield){c},docNode,curElement,[pName curfield '{' num2str(c) '}.'])
                else
                    disp(['Warning. The cell ' pName curfield '{' num2str(c) '} could not be processed, since it contains no structure.']);
                end
            end
        else
            %eventhough the fieldname is not text, the field could
            %contain text. Create a new element and use this text
            curElement = docNode.createElement(curfield_sc);
            curNode.appendChild(curElement);
            [txt,succes] = val2str(s.(curfield));
            if (succes)
                curElement.appendChild(docNode.createTextNode(txt));
            else
                disp(['Warning. The text in ' pName curfield ' could not be processed.']);
            end
        end
    end
end

%----- Subfunction val2str -----
function [str,succes] = val2str(val)

succes = true;
str = [];

if (isempty(val))
    return; %bugfix from H. Gsenger
elseif (ischar(val))
    %do nothing
elseif (isnumeric(val))
    val = num2str(val);
else
    succes = false;
end

if (ischar(val))
    %add line breaks to all lines except the last (for multiline strings)
    lines = size(val,1);
    val = [val char(sprintf('\n')*[ones(lines-1,1);0])];
    
    %transpose is required since indexing (i.e., val(nonspace) or val(:)) produces a 1-D vector.
    %This should be row based (line based) and not column based.
    valt = val';
    
    remove_multiple_white_spaces = true;
    if (remove_multiple_white_spaces)
        %remove multiple white spaces using isspace, suggestion of T. Lohuis
        whitespace = isspace(val);
        nonspace = (whitespace + [zeros(lines,1) whitespace(:,1:end-1)])~=2;
        nonspace(:,end) = [ones(lines-1,1);0]; %make sure line breaks stay intact
        str = valt(nonspace');
    else
        str = valt(:);
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% STRUCT2XML (END)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%==========================================================================
% Auxiliary function
%
function m = xmlunwrap(s)
%

fn  = fieldnames(s);
att = 'Attributes';
txt = 'Text';

% Cycle through field names
for n = 1:length(fn),
    
    if isfield(s, fn{n})
        
        % Descend through nested fields
        if isstruct(s.(fn{n}))
            
            % Recurse
            s.(fn{n}) = xmlunwrap(s.(fn{n}));
            
            % Add struct Attributes
            s.(fn{n}).(att).idx  = 1;
            s.(fn{n}).(att).size = '1 1';
            s.(fn{n}).(att).type = 'struct';
            
        else
            
            % idx attribute is constant for all
            idx = 1;
            
            % Recover value and erase field to avoid warning
            data = s.(fn{n});
            s.(fn{n}) = [];
            
            % Recover data type and size
            sz   = size(data);
            data = reshape(data, 1, prod(sz));
            type = class(data);
            
            s.(fn{n}).(txt)      = num2str(data);
            s.(fn{n}).(att).idx  = idx;
            s.(fn{n}).(att).size = num2str(sz);
            s.(fn{n}).(att).type = type;
            
        end
        
    end
    
end

m = s;


