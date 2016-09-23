%
% acb_geom.m
%
% Author: Martin Lahuta, (c) 2008,2009 VZLU (www.vzlu.cz)
% Developed within SimSAC project, www.simsacdesign.org
% Any usage without an explicit authorization may be persecuted.
%
% Modifications:
%	DATE		VERS	PROGRAMMER	DESCRIPTION
%	08.12.09	1.0	M. Lahuta	last update
% 
% calls geometry module from AcBuilder when user selects 'Geometry (output)'
% from 'Geometry' menu or any following module (except 'Technology').
%
function acb_geom

global ac
global sinp_geo sout_geo

op=pwd;
mp=mfilename('fullpath');
gp=strrep(mp,[ 'acb_geom' ], 'Geo');

try

disp('acbuilder: Calling Geo module ...');

acb_prepac;
acb_postac;

cd(gp);

sinp_geo=ac;

geo_xml;

ac=sout_geo;

cd(op);
return

catch

cd(op);
disp('acb_geom: error occured');
lasterr
return

end
