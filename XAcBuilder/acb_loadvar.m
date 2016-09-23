%
% acb_loadvar.m
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
% function returs value from 'ac' structure specified by 'path' string,
% this function is called only from AcBuilder and must always return
% some value otherwise AcBuilder hangs up.
%
function [res]=acb_loadvar(path)

global ac defac

try
    eval(['res=num2str(ac.' path ');']);
%    disp(['LOAD: ' path '=' res]);
    return
catch
%    disp(['acb_loadvar: ''' path ''' not found in ac, using default']);
end

try
    eval(['res=num2str(defac.' path ');']);
    return
catch
    disp(['acb_loadvar: ''' path ''' not found in defac']);
end

res='err';
return
