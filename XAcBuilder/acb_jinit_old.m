%
% acb_jinit.m
%
% Author: Martin Lahuta, (c) 2009, VZLU (www.vzlu.cz)
% Developed within SimSAC project, www.simsacdesign.org
% Any usage without an explicit authorization may be persecuted.
%
%
% Modifications:
%	DATE		VERS	PROGRAMMER	DESCRIPTION
%	15.12.09	1.0	M. Lahuta	last update
% 
% initialize AcBuilder's files (clears matlab workspace !!!)
%
function acb_jinit

abd=strrep(mfilename('fullpath'),[filesep 'acb_jinit'],'');
% initialize JOGL
comp=computer;
if strcmp(comp,'GLNX86')
   lpath=[abd '/jogl/linux32'];
elseif strcmp(comp,'GLNXA64')
   lpath=[abd '/jogl/linux64'];
elseif strcmp(comp,'PCWIN') || strcmp(comp,'PCWIN64')
   lpath=[abd '\jogl\win32'];
else
   disp('acb_jinit: error - unsupported OS');
   return
end

try
% use LibPathHacker class to change librarypath at runtime
jpaths=char(javaclasspath);
p=[abd filesep 'LibPathHacker.jar'];
if isempty(strmatch(p,jpaths,'exact'))
   javaaddpath(p);
end
lpaths=LibPathHacker.getDirs;
if isempty(strmatch(lpath,lpaths,'exact'))
   LibPathHacker.addDir(lpath);
end
% load JOGL's jar files
p=[lpath filesep 'gluegen-rt-acb.jar'];
if isempty(strmatch(p,jpaths,'exact'))
   javaaddpath(p);
end
p=[lpath filesep 'jogl-acb.jar'];
if isempty(strmatch(p,jpaths,'exact'))
   disp('acb_jinit: Initializing JOGL ...');
   javaaddpath(p);
end
catch
disp('acb_jinit: error - cannot initialize JOGL');
return
end

% add path to AcBuilder's scripts
paths=path;
p=abd;
if isempty(strfind(paths,p))
   disp('acb_jinit: adding path to AcBuilder''s scripts ...');
   addpath(p);
end

% initialize AcBuilder
try
p=[abd filesep 'AcBuilder.jar'];
if isempty(strmatch(p,jpaths,'exact'))
   javaaddpath(p);
end
catch
disp('acb_jinit: error - cannot initialize AcBuilder');
end
