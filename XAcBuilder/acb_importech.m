%
% acb_importech.m
%
% Author: Martin Lahuta, (c) 2008,2009 VZLU (www.vzlu.cz)
% Developed within SimSAC project, www.simsacdesign.org
% Any usage without an explicit authorization may be persecuted.
%
% Modifications:
%	DATE		VERS	PROGRAMMER	DESCRIPTION
%	26.11.09	1.0	M. Lahuta	last update
% 
% loads technology data from xml file into 'ac' structure
% called when user selects 'Import XML' from 'Technology' menu
%
function acb_importech(name)

global ac

try

if nargin==0
   disp('acb_importech: missing parameter');
   return
end
disp(['AcBuilder: Loading technology data from file: ' name]);

%tc=xml_load(name);
tc=neocass_xmlwrapper(name);
% check mandatory fields
if isfield(tc,'user_input')
   ac.user_input=tc.user_input;
else
   rmfield(ac,'user_input');
end
if isfield(tc,'experienced_user_input')
   ac.experienced_user_input=tc.experienced_user_input;
else
   rmfield(ac,'experienced_user_input');
end

return

catch

disp('AcBuilder::acb_importech: xml toolbox must be in search path');

end
