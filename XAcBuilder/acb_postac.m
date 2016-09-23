%
% acb_postac.m
%
% Author: Martin Lahuta, (c) 2008,2009 VZLU (www.vzlu.cz)
% Developed within SimSAC project, www.simsacdesign.org
% Any usage without an explicit authorization may be persecuted.
%
%
% Modifications:
%	DATE		VERS	PROGRAMMER	DESCRIPTION
%	08.12.09	1.0	M. Lahuta	last update
% 
% function makes data in 'ac' structure coming out of AcBuilder to be
% compatible with other modules (QCARD heritage)
%
function acb_postac

global ac

% wing1
if ac.Wing1.present
    if ac.Wing1.fairing.present
        ac.Fairing1.Forward_chord_fraction=str2num(acb_loadvar('Wing1.fairing.Forward_chord_fraction'));
        ac.Fairing1.Aft_chord_fraction=str2num(acb_loadvar('Wing1.fairing.Aft_chord_fraction'));
        ac.Fairing1.flushness=str2num(acb_loadvar('Wing1.fairing.flushness'));
    else
        ac.Fairing1.Forward_chord_fraction=0.0;
        ac.Fairing1.Aft_chord_fraction=0.0;
        ac.Fairing1.flushness=0.0;
    end
    airfoil=chkaf(ac.Wing1.airfoilRoot);
    ac.Wing1.airfoilRoot=airfoil;
    airfoil=chkaf(ac.Wing1.airfoilKink1);
    ac.Wing1.airfoilKink1=airfoil;
    airfoil=chkaf(ac.Wing1.airfoilKink2);
    ac.Wing1.airfoilKink2=airfoil;
    airfoil=chkaf(ac.Wing1.airfoilTip);
    ac.Wing1.airfoilTip=airfoil;
else
%    ac.Wing1.AR=0.0;
%    ac.Wing1.Span=0.0;
%    ac.Wing1.area=0.0;
    ac.Fairing1.Forward_chord_fraction=0.0;
    ac.Fairing1.Aft_chord_fraction=0.0;
    ac.Fairing1.flushness=0.0;
end

% wing2
if ac.Wing2.present
    if ac.Wing2.fairing.present
        ac.Fairing2.Forward_chord_fraction=str2num(acb_loadvar('Wing2.fairing.Forward_chord_fraction'));
        ac.Fairing2.Aft_chord_fraction=str2num(acb_loadvar('Wing2.fairing.Aft_chord_fraction'));
        ac.Fairing2.flushness=str2num(acb_loadvar('Wing2.fairing.flushness'));
    else
        ac.Fairing2.Forward_chord_fraction=0.0;
        ac.Fairing2.Aft_chord_fraction=0.0;
        ac.Fairing2.flushness=0.0;
    end
    airfoil=chkaf(ac.Wing2.airfoilRoot);
    ac.Wing2.airfoilRoot=airfoil;
    airfoil=chkaf(ac.Wing2.airfoilKink1);
    ac.Wing2.airfoilKink1=airfoil;
    airfoil=chkaf(ac.Wing2.airfoilKink2);
    ac.Wing2.airfoilKink2=airfoil;
    airfoil=chkaf(ac.Wing2.airfoilTip);
    ac.Wing2.airfoilTip=airfoil;
else
%    ac.Wing2.AR=0.0;
%    ac.Wing2.Span=0.0;
%    ac.Wing2.area=0.0;
    ac.Fairing2.Forward_chord_fraction=0.0;
    ac.Fairing2.Aft_chord_fraction=0.0;
    ac.Fairing2.flushness=0.0;
end

if ac.Horizontal_tail.present
    airfoil=chkaf(ac.Horizontal_tail.airfoilRoot);
    ac.Horizontal_tail.airfoilRoot=airfoil;
    airfoil=chkaf(ac.Horizontal_tail.airfoilKink);
    ac.Horizontal_tail.airfoilKink=airfoil;
    airfoil=chkaf(ac.Horizontal_tail.airfoilTip);
    ac.Horizontal_tail.airfoilTip=airfoil;
%else
%    ac.Horizontal_tail.AR=0.0;
%    ac.Horizontal_tail.Span=0.0;
%    ac.Horizontal_tail.area=0.0;
end

if ac.Vertical_tail.present
    airfoil=chkaf(ac.Vertical_tail.airfoilRoot);
    ac.Vertical_tail.airfoilRoot=airfoil;
    airfoil=chkaf(ac.Vertical_tail.airfoilKink);
    ac.Vertical_tail.airfoilKink=airfoil;
    airfoil=chkaf(ac.Vertical_tail.airfoilTip);
    ac.Vertical_tail.airfoilTip=airfoil;
%else
%    ac.Vertical_tail.AR=0.0;
%    ac.Vertical_tail.Span=0.0;
%    ac.Vertical_tail.area=0.0;
end

if ac.Engines1.present
   ac.Engines1.Nacelle1.longitudinal_location=ac.Engines1.x;
%   ac.Engines1.Nacelle1.lateral_location=ac.Engines1.Y_locale;
   ac.Engines1.Nacelle1.vertical_location=ac.Engines1.z;
   ac.Engines1.Nacelle_length_array=ac.Engines1.fineness_ratio*ac.Engines1.d_max;
   if ac.Engines1.Layout_and_config==1 | ac.Engines1.Layout_and_config==2
      if ac.Engines1.Z_locale>=0.0
	 ac.Engines1.Z_locale=1.0;
      else
	 ac.Engines1.Z_locale=-1.0;
      end
   end
end

if ac.Engines2.present
   ac.Engines2.Nacelle3.longitudinal_location=ac.Engines2.x;
%   ac.Engines2.Nacelle3.lateral_location=ac.Engines2.Y_locale;
   ac.Engines2.Nacelle3.vertical_location=ac.Engines2.z;
   ac.Engines2.Nacelle_length_array=ac.Engines2.fineness_ratio*ac.Engines2.d_max;
   if ac.Engines2.Layout_and_config==1 | ac.Engines2.Layout_and_config==2
      if ac.Engines2.Z_locale>=0.0
	 ac.Engines2.Z_locale=1.0;
      else
	 ac.Engines2.Z_locale=-1.0;
      end
   end
end

% ac.miscellaneous.main_landing_gear_x_cg=ac.weight_balance.miscellaneous.main_landing_gear_x_cg;
% ac.miscellaneous.main_landing_gear_z_cg=ac.weight_balance.miscellaneous.main_landing_gear_z_cg;
% ac.miscellaneous.aux_landing_gear_x_cg=ac.weight_balance.miscellaneous.aux_landing_gear_x_cg;
% ac.miscellaneous.aux_landing_gear_z_cg=ac.weight_balance.miscellaneous.aux_landing_gear_z_cg;

ac.Sponson.length=0.0;

% add .dat extension to airfoil's name if it is not NACA airfoil (= only 4 digits)
function [af]=chkaf(af)
if isempty(findstr(af,'.dat'))
    if length(af)==4 & ~isempty(str2num(af))
	return
    end
    af=[af '.dat'];
    return
end
