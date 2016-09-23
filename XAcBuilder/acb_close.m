%
% acb_close.m
%
% Author: Martin Lahuta, (c) 2008,2009 VZLU (www.vzlu.cz)
% Developed within SimSAC project, www.simsacdesign.org
% Any usage without an explicit authorization may be persecuted.
%
% Modifications:
%	DATE		VERS	PROGRAMMER	DESCRIPTION
%	07.12.09	1.0	M. Lahuta	last update
% 
% 
% called when user closes AcBuilder
%
function acb_close

global ac

try

acb_postac;
%disp('acb_close');
return

catch

disp('AcBuilder::acb_close: error');

end
