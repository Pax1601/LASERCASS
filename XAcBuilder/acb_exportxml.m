%
% acb_exportxml.m
%
% Author: Martin Lahuta, (c) 2008,2009 VZLU (www.vzlu.cz)
% Developed within SimSAC project, www.simsacdesign.org
% Any usage without an explicit authorization may be persecuted.
%
% Modifications:
%	DATE		VERS	PROGRAMMER	DESCRIPTION
%	26.11.09	1.0	M. Lahuta	last update
% 
% 
% exports data from ac structure into name.xml file
% called when user selects 'Export XML' from 'Project' menu
%
function acb_exportxml(name)

global ac

try

if nargin==0
   disp('AcBuilder::acb_exportxml: missing parameter');
   return
end
disp(['AcBuilder: Saving data to file: ' name]);
acb_postac;
neocass_xmlunwrapper(name,ac);
return

catch

disp('AcBuilder::acb_exportxml: xml toolbox must be in search path');

end
