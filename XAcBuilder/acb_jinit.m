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
%	15.12.09	1.0	M. Lahuta	    last update
%	11.11.13		M. Lahuta	    removed external jogl dependency
%   28.08.14        L. Riccobene    fixed Matlab-Java version issues
%
% initialize AcBuilder's files (clears matlab workspace !!!)
%
function acb_jinit

abd = strrep(mfilename('fullpath'), [filesep 'acb_jinit'], '');

% Check Matlab version: if before R2013b, which means embedded Java version
% 1.6, call the old *.jar otherwise call the new one compiled with Java 1.7
if verLessThan('matlab', '8.2')
    % Compiled with Java 1.6
    disp('acb_jinit: Using AcBuilder version compiled with Java 1.6');
    abd_jar = [abd, filesep, 'preR2013b'];
else
    % Compiled with Java 1.7
    disp('acb_jinit: Using AcBuilder version compiled with Java 1.7');
    abd_jar = [abd, filesep, 'aftR2013b'];
end

% add path to AcBuilder's scripts
paths = path;
p     = abd_jar;
if isempty(strfind(paths, p))
    disp('acb_jinit: adding path to AcBuilder''s scripts ...');
    addpath(p);
    addpath(abd);
end
  


% initialize AcBuilder
try
    jpaths = char(javaclasspath('-dynamic'))
    p = [abd_jar filesep 'AcBuilder.jar']
    if isempty(strfind(jpaths, p))
        javaaddpath(p)
    end
catch
    disp('acb_jinit: error - cannot initialize AcBuilder');
end