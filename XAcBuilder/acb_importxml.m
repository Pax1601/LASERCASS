%
% acb_importxml.m
%
% Author: Martin Lahuta, (c) 2008,2009 VZLU (www.vzlu.cz)
% Developed within SimSAC project, www.simsacdesign.org
% Any usage without an explicit authorization may be persecuted.
%
%
% Modifications:
%	DATE		VERS	PROGRAMMER	DESCRIPTION
%	26.11.09	1.0	M. Lahuta	last update
% 
% 
% load aircraft's data from xml file into 'ac' structure
% called when user selects 'Import XML' from 'Project' menu
%
function acb_importxml(name)

global ac defac

try

if nargin==0
   disp('AcBuilder::acb_importxml: missing parameter');
   return
end
disp(['AcBuilder: Loading data from file: ' name]);
ac=struct([]);
ac=neocass_xmlwrapper(name);
acb_prepac;	% check for missing fields
return

catch

disp('AcBuilder::acb_importxml: xml toolbox must be in search path');

end
