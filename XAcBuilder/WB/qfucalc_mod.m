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
%

function aircraft = qfucalc_mod(aircraft)
%**************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     090303      2.0     L.Riccobene      Creation
%
%**************************************************************************
%
% function       aircraft = qfucalc_mod(aircraft)
%
%
%   DESCRIPTION:   Modified version of qfucalc.m used only for tanks'
%                  volume estimation (no added fields)
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                  
%                aircraft       struct     coming from Geo module;
%
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                
%                aircraft       struct     equal to input.
%         
%                
%    REFERENCES:
%
%**************************************************************************
%
% Fuel weight estimation using Isikveren's method. Scope is given to predict:
% (1) fuel in the wings
% (2) fuel in the centre-tank
% (3) fuel in the forward and aft fuselage tanks
%

n = 1;
kdrang = 2*pi/360;  % conversion from deg. to rad.

% Fuselage
fuselgt(1,n) = aircraft.Fuselage.Total_fuselage_length;
fusevma(1, n) = aircraft.Fuselage.Aftfuse_X_sect_vertical_diameter;
fusehma(1, n) = aircraft.Fuselage.Aftfuse_X_sect_horizontal_diameter;
aftslgt(1,n) = aircraft.Fuselage.Tail_length;

% Wing 1
WCHDROT(1,n) = aircraft.Reference_wing.Orig_root_chrd_at_ac_CL;
XSECDWF(1,n) = aircraft.Fuselage.X_sect_chord_at_fuse_wing;
WSPNMTX = aircraft.Wing1.Span_matrix_partition_in_mid_outboard;
WTHKMTX = aircraft.Wing1.thickness_coefs_matrix;
WCHRDWF(1,n) = aircraft.Wing1.Original_estimated_fuse_wing_chrd;
WINREFC(1,n) = aircraft.Reference_wing.convention;
wingspn(1,n) = aircraft.Wing1.Span;
winglsw(1,n) = aircraft.Wing1.LE_sweep_inboard;
wingkln(1,n) = aircraft.Wing1.spanwise_kink1;
wingkln(2,n) = aircraft.Wing1.spanwise_kink2;
wingtap(1,n) = aircraft.Wing1.taper_kink1;
wingtap(2,n) = aircraft.Wing1.taper_kink2;
wingtap(3,n) = aircraft.Wing1.taper_tip;
wingplc(1,n) = aircraft.Wing1.placement;
wingapx(1,n) = aircraft.Wing1.apex_locale;
wingdih(1,n) = aircraft.Wing1.dihedral_inboard;
wingdih(2,n) = aircraft.Wing1.dihedral_midboard;
wingdih(3,n) = aircraft.Wing1.dihedral_outboard;
wingthk(1,n) = aircraft.Wing1.thickness_root;
wingthk(2,n) = aircraft.Wing1.thickness_kink1;
wingthk(3,n) = aircraft.Wing1.thickness_kink2;
wingthk(4,n) = aircraft.Wing1.thickness_tip;
wingplc(1,n) = aircraft.Wing1.placement;
wingapx(1,n) = aircraft.Wing1.apex_locale;
wingare(1,n) = aircraft.Wing1.area;
winggar(1,n) = aircraft.Wing1.AR;
winginc(1,n) = aircraft.Wing1.root_incidence;
winginc(2,n) = aircraft.Wing1.kink1_incidence;
winginc(3,n) = aircraft.Wing1.kink2_incidence;
winginc(4,n) = aircraft.Wing1.tip_incidence;
wingqsw(1,n) = aircraft.Wing1.quarter_chord_sweep_inboard;
wingqsw(2,n) = aircraft.Wing1.quarter_chord_sweep_midboard;
wingqsw(3,n) = aircraft.Wing1.quarter_chord_sweep_outboard;
winglsw(1,n) = aircraft.Wing1.LE_sweep_inboard;
winglsw(2,n) = aircraft.Wing1.LE_sweep_midboard;
winglsw(3,n) = aircraft.Wing1.LE_sweep_outboard;

% Wing thickness
[kln,spn,gar,smtrx,crot,xscd,tap,cref,chrf,tref,chwf,troot,tmtrx,qsw,lsw, ...
    netwgae,wsatref,wsarref,wsasref,wsarlsw,wsarqsw,wsarhsw,wsarmac,wsarybr, ...
    wsarthb,inc] = qxemcop(wingkln, wingtap, wingspn, wingare, winggar, ...
    fusehma, wingthk, wingqsw, winglsw, WINREFC, wingplc, winginc, n);
WTHKROT(n) = troot;

% Wing 2
%wi2gare(1,n) = aircraft.Wing2.area;
wi2present(1,n) = aircraft.Wing2.present;
if wi2present(1, n)
%if wi2gare(1,n) > 0.0001

    WS2NMTX = aircraft.Wing2.Span_matrix_partition_in_mid_outboard;
    WT2KMTX = aircraft.Wing2.thickness_coefs_matrix;
    wi2gspn(1,n) = aircraft.Wing2.Span;
    wi2glsw(1,n) = aircraft.Wing2.LE_sweep_inboard;
    wi2glsw(2,n) = aircraft.Wing2.LE_sweep_midboard;
    wi2glsw(3,n) = aircraft.Wing2.LE_sweep_outboard;
    wi2gplc(1,n) = aircraft.Wing2.placement;
    wi2gapx(1,n) = aircraft.Wing2.apex_locale;
    wi2gplc(1,n) = aircraft.Wing2.placement;
    wi2gare(1,n) = aircraft.Wing2.area;
    wi2ggar(1,n) = aircraft.Wing2.AR;
    wi2gspn(1,n) = aircraft.Wing2.Span;
    wi2gkln(1,n) = aircraft.Wing2.spanwise_kink1;
    wi2gkln(2,n) = aircraft.Wing2.spanwise_kink2;
    wi2gtap(1,n) = aircraft.Wing2.taper_kink1;
    wi2gtap(2,n) = aircraft.Wing2.taper_kink2;
    wi2gtap(3,n) = aircraft.Wing2.taper_tip;
    wi2ginc(1,n) = aircraft.Wing2.root_incidence;
    wi2ginc(2,n) = aircraft.Wing2.kink1_incidence;
    wi2ginc(3,n) = aircraft.Wing2.kink2_incidence;
    wi2ginc(4,n) = aircraft.Wing2.tip_incidence;
    wi2gqsw(1,n) = aircraft.Wing2.quarter_chord_sweep_inboard;
    wi2gqsw(2,n) = aircraft.Wing2.quarter_chord_sweep_midboard;
    wi2gqsw(3,n) = aircraft.Wing2.quarter_chord_sweep_outboard;
    wi2glsw(1,n) = aircraft.Wing2.LE_sweep_inboard;
    wi2glsw(2,n) = aircraft.Wing2.LE_sweep_midboard;
    wi2glsw(3,n) = aircraft.Wing2.LE_sweep_outboard;
    wi2gdih(1,n) = aircraft.Wing2.dihedral_inboard;
    wi2gdih(2,n) = aircraft.Wing2.dihedral_midboard;
    wi2gdih(3,n) = aircraft.Wing2.dihedral_outboard;
    wi2gthk(1,n) = aircraft.Wing2.thickness_root;
    wi2gthk(2,n) = aircraft.Wing2.thickness_kink1;
    wi2gthk(3,n) = aircraft.Wing2.thickness_kink2;
    wi2gthk(4,n) = aircraft.Wing2.thickness_tip;

    [kln,spn,gar,smtrx,crot,xscd,tap,cref,chrf,tref,chwf,troot,tmtrx,qsw,lsw, ...
        netwgae,wsatref,wsarref2,wsasref2,wsarlsw,wsarqsw2,wsarhsw,wsarmac,wsarybr, ...
        wsarthb2,inc] = qxemcop(wi2gkln, wi2gtap, wi2gspn, wi2gare, wi2ggar, ...
        fusehma, wi2gthk, wi2gqsw, wi2glsw, WINREFC, wi2gplc, wi2ginc, n);

    WC2DROT(n) = crot;
    XS2CDWF(1,n) = xscd;
    WT2KROT(n) = troot;
    WC2RDWF(n) = chwf;

    % Added some fields for Wing2 - 06/10/2008
    aircraft.Wing2.Weighted_reference_aspect_ratio = wsasref2;
    aircraft.Wing2.Weighted_reference_wing_area = wsarref2;
    aircraft.Wing2.Reference_quarter_chord_sweep = wsarqsw2;
    aircraft.Wing2.Wing_mean_thickness = wsarthb2;
    aircraft.Wing2.Reference_LE_sweep = abs(wsarlsw);
    aircraft.Wing2.Reference_non_dim_y_bar = wsarybr;
    aircraft.Wing2.Reference_MAC = wsarmac;

end

% Fairings
fairfwd(1,n) = aircraft.Fairing1.Forward_chord_fraction;
fairaft(1,n) = aircraft.Fairing1.Aft_chord_fraction;
fairovh(1,n) = aircraft.Fairing1.flushness;

% Fuel
% wingspf(1, n) = aircraft.fuel.Fore_wing_spar_loc_root;
% wingspf(2, n) = aircraft.fuel.Fore_wing_spar_loc_kik1;
% wingspf(3, n) = aircraft.fuel.Fore_wing_spar_loc_kin2;
% wingspf(4, n) = aircraft.fuel.Fore_wing_spar_loc_tip;
% wingspa(1, n) = aircraft.fuel.Aft_wing_spar_loc_root;
% wingspa(2, n) = aircraft.fuel.Aft_wing_spar_loc_kin1;
% wingspa(3, n) = aircraft.fuel.Aft_wing_spar_loc_kin2;
% wingspa(4, n) = aircraft.fuel.Aft_wing_spar_loc_tip;

%==========================================================================
% Modification 18/03/2009 L.Riccobene
% Added checks to preserve compatibility with previous file and granting
% usage of new ones
try
    fuelcut(1, n) = aircraft.fuel.Wing_fuel_tank_cutout_opt;
catch
    fuelcut(1, n) = aircraft.fuel.Wing1_fuel_tank_cutout_opt;
end
fuelobd(1, n) = aircraft.fuel.Outboard_fuel_tank_span;
fuelusu(1, n) = aircraft.fuel.Unusable_fuel_option;
fuelden(1, n) = aircraft.fuel.Assumed_fuel_density ;
try
    fuelwic(1, n) = aircraft.fuel.Incr_weight_for_wing_tanks;
catch
    fuelwic(1, n) = 0.;
end
try
    fuelcen(1, n) = aircraft.fuel.Centre_tank_portion_used;
catch
    fuelcen(1, n) = aircraft.fuel.Centre_tank1_portion_used;
end
try
    fuelcic(1, n) = aircraft.fuel.Increment_for_centre_tank;
catch
    fuelcic(1, n) = 0.;
end
try
    fuelaux(1, n) = aircraft.fuel.Fore_fairing_tank_length;
    fuelaux(2, n) = aircraft.fuel.Aft_fairing_tank_length;
catch
    fuelaux(1, n) = aircraft.fuel.Fore_fairing1_tank_length;
    fuelaux(2, n) = aircraft.fuel.Aft_fairing1_tank_length;
end
fuelafa(1, n) = aircraft.fuel.Aft_fuse_bladder_length;
try
    fuelaic(1, n) = aircraft.fuel.Increment_for_aux_tanks;
catch
    fuelaic(1, n) = 0.;
end
%==========================================================================

% Cabin
cabnhei(1, n) = aircraft.cabin.Cabin_max_internal_height;
cabnwid(1, n) = aircraft.cabin.Cabin_max_internal_width;
cabnfwd(1, n) = aircraft.cabin.Cabin_floor_width;

% Engines
engenum(1, n) = aircraft.Engines1.Number_of_engines;
engenum(2, n) = aircraft.Engines2.Number_of_engines;

% Set engines number
if engenum(2, n) > 0.0
    PN = 2;% prediction for primary and secondary powerplants
else
    PN = 1;% prediction for primary powerplants only
end

ENGEDIA(1, n) = aircraft.Engines1.d_max;
ENGEDIA(2, n) = aircraft.Engines2.d_max;
engeloc(1, n) = aircraft.Engines1.Layout_and_config;
engeloc(2, n) = aircraft.Engines2.Layout_and_config;
engeylc(1, n) = aircraft.Engines1.Y_locale;
engeylc(2, n) = aircraft.Engines2.Y_locale;

if engeloc(1, n) < 3.0

    % calculate the lateral wing distance to inboard cut-out
    BTA1CUT = 0.5*(engeylc(1,n)*wingspn(n) - ENGEDIA(1,n));
    THK1CUT = qxtkcop(BTA1CUT, WSPNMTX, WTHKMTX);% thickness at cut 1
    CTA1CUT = qxcdcop(BTA1CUT, WSPNMTX, WCHDROT(n), wingtap, n);

    % calculate the lateral wing distance to outboard cut-out
    BTA2CUT = 0.5*(engeylc(1,n)*wingspn(n) + ENGEDIA(1,n));
    THK2CUT = qxtkcop(BTA2CUT, WSPNMTX, WTHKMTX);% thickness at cut 2
    CTA2CUT = qxcdcop(BTA2CUT, WSPNMTX, WCHDROT(n), wingtap, n);

    if PN>1

        % calculate the lateral wing distance to inboard cut-out
        BTA3CUT = 0.5*(engeylc(2,n)*wingspn(n) - ENGEDIA(2,n));
        THK3CUT = qxtkcop(BTA3CUT, WSPNMTX, WTHKMTX);% thickness at cut 3
        CTA3CUT = qxcdcop(BTA3CUT, WSPNMTX, WCHDROT(n), wingtap, n);

        % calculate the lateral wing distance to outboard cut-out
        BTA4CUT = 0.5*(engeylc(2,n)*wingspn(n) + ENGEDIA(2,n));
        THK4CUT = qxtkcop(BTA4CUT, WSPNMTX, WTHKMTX);% thickness at cut 4
        CTA4CUT = qxcdcop(BTA4CUT, WSPNMTX, WCHDROT(n), wingtap, n);

    else

        BTA3CUT = 0.;
        BTA4CUT = 0.;

    end

else

    BTA1CUT = 0.;
    BTA2CUT = 0.;
    BTA3CUT = 0.;
    BTA4CUT = 0.;

end

%==========================================================================
% Calculate wingbox extent

% Box elastic axis position (chord percentage)
box = zeros(4, 1);
box(1) = aircraft.fuel.box_ea_loc_root;
box(2) = aircraft.fuel.box_ea_loc_kink1;
box(3) = aircraft.fuel.box_ea_loc_kink2;
box(4) = aircraft.fuel.box_ea_loc_tip;

% Set, conventionally, elastic axis position to middle wing
test = box == 0.;
if any(test)
    box(test) = .5;
end

% Wing front spar locations (chord percentage)
b_box = zeros(4, 1);
b_box(1) = aircraft.fuel.box_semispan_root;
b_box(2) = aircraft.fuel.box_semispan_kink1;
b_box(3) = aircraft.fuel.box_semispan_kink2;
b_box(4) = aircraft.fuel.box_semispan_tip;

% Check if semispan is zero (set default value, a wingbox that spans
% between 30% to 70% of local chord)
test = b_box == 0.;
if any(test)
    b_box(test) = .2;
end

% Compute fore spar positions
wingspf(:, n) = box - b_box;

test = wingspf(:, n) <= 0.;
if any(test)
    error('Fore spar exceeds wing limits!');
end

% Compute rear spar positions
wingspa(:, n) = box + b_box;

test = wingspa(:, n) >= 1.;
if any(test)
    error('Rear spar exceeds wing limits!');
end

% Uncomment to see chord at the four stations (root, kink1, kink2, tip)
% c = zeros(4, 1);
% c(1) = qxcdcop(XSECDWF(n)/2, WSPNMTX, WCHDROT, wingtap, n);
% c(2) = qxcdcop(wingkln(1, n)*wingspn(n)/2, WSPNMTX, WCHDROT, wingtap, n);
% c(3) = qxcdcop(wingkln(2, n)*wingspn(n)/2, WSPNMTX, WCHDROT, wingtap, n);
% c(4) = qxcdcop(wingspn(n)/2, WSPNMTX, WCHDROT, wingtap, n);
% assignin('base', 'c', c);

%==========================================================================

if fuelden(n) < 0.0001
    fuelden(n) = 0.802;% fuel density default is 0.802 kg/cu.m
end

%==========================================================================

% Inspect if the user wants nominal tank span or has specified it
if fuelobd(n) < 0.0001
    fuelobd(n) = 0.70;% user selected 70% wing span for tank
end

if fuelobd(n) > 0.99
    fuelobd(n) = 0.99;% idiot-proof the max tank span
end

%==========================================================================

% intialise the centre of gravity objectives
% xcgwing=0.;
% ycgwing=0.;
% zcgwing=0.;
xcgfair=0.;
ycgfair=0.;
zcgfair=0.;
xcgtaux=0.;
ycgtaux=0.;
zcgtaux=0.;

%==========================================================================
% Compute the inboard tank (No. 1) volume
%

lt1 = WSPNMTX(1) - XSECDWF(n)/2;% tank length
lspard1 = abs(wingspf(1, n) - wingspa(1, n));% usable chord fraction
tk1end = qxtkcop(XSECDWF(n)/2, WSPNMTX, WTHKMTX);% thickness at wing-fuse

% local chord length at wing-fuse
ct1end = qxcdcop(XSECDWF(n)/2, WSPNMTX, WCHDROT, wingtap, n);
lspard2 = abs(wingspf(2, n) - wingspa(2, n));% usable chord fraction
tk2end = qxtkcop(wingkln(1, n)*wingspn(n)/2, WSPNMTX, WTHKMTX);% thickness at kink 1

% local chord length at kink 1 station
ct2end = qxcdcop(wingkln(1, n)*wingspn(n)/2, WSPNMTX, WCHDROT, wingtap, n);

% predict the No. 1 tank volume and centre of gravity
[uscvol1, ycenog1] = fvcomp(lt1, lspard1, tk1end, ct1end(n), lspard2, tk2end, ct2end(n));
uscvol1 = 2*uscvol1;

% locate forward and aft spar centroid at kink 1
xcened2 = lt1*tan(kdrang*winglsw(1,n))+ct2end(n)*(wingspa(2,n)+wingspf(2,n))/2;

% locate the non-dimensional longitudinal cg of tank 1
xcenog1 = (fuselgt(n)*wingapx(n)+XSECDWF(n)/2*tan(kdrang*winglsw(1,n))+ ...
    ct1end(n)*(wingspa(1,n)+wingspf(1,n))/2 +...
    ycenog1/lt1*(xcened2-ct1end(n)/2))/fuselgt(n);

% locate the non-dimensional vertical cg of tank 1
zcenog1 = (fusevma(n)*(wingplc(n)-0.5)+ ...
    (ycenog1+XSECDWF(n)/2)*tan(kdrang*wingdih(1,n)))/fusevma(n);

%==========================================================================
% Predict the centre tank volume
%

% default no fuel in centre tank
centvol=0.0;
xcogcen=0.0;
zcogcen=0.0;
MFCVWEI(n)=0;
MFCWWEI(n)=0;

if fuelcen(n) > 0.0001
    centvol = fuelcen(n)/100*2*fvcomp(XSECDWF(n)/2,lspard1,wingthk(1,n), ...
        WCHRDWF(n),lspard1,WTHKROT(n),WCHDROT(n));
    MFCVWEI(n) = centvol*1000+fuelcic(n)/fuelden(n);
    MFCWWEI(n) = MFCVWEI(n)*fuelden(n);
    xcogcen = (fuselgt(n)*wingapx(n)+XSECDWF(n)/2*tan(kdrang*winglsw(1,n))+ ...
        WCHRDWF(n)*(wingspa(1,n)+wingspf(1,n))/2)/fuselgt(n);% x cg
    zcogcen = WCHDROT(n)*wingthk(1,n)/(2*fusevma(n))-0.5;% z cg
end

%==========================================================================
% Compute the midboard tank (No. 2) volume and centre of gravity
%

lt2 = WSPNMTX(2);% tank length
lspard3 = abs(wingspf(3,n)-wingspa(3,n));% usable chord fraction
tk3end = qxtkcop(wingkln(2,n)*wingspn(n)/2,WSPNMTX,WTHKMTX);% thickness at kink 2

% local chord length at kink 2 station
ct3end = qxcdcop(wingkln(2,n)*wingspn(n)/2,WSPNMTX,WCHDROT,wingtap,n);

% predict the No. 2 volume
[uscvol2, ycenog2] = fvcomp(lt2,lspard2,tk2end,ct2end(n),lspard3,tk3end,ct3end(n));
uscvol2 = 2*uscvol2;

xcened3 = lt1*tan(kdrang*winglsw(1,n))+lt2*tan(kdrang*winglsw(2,n))+ ...
    ct3end(n)*(wingspa(3,n)+wingspf(3,n))/2;

% locate the non-dimensional longitudinal cg of tank 2
xcenog2 = (fuselgt(n)*wingapx(n)+WSPNMTX(1)*tan(kdrang*winglsw(1,n))+ ...
    ct2end(n)*(wingspa(2,n)+wingspf(2,n))/2+ ...
    ycenog2/lt2*(xcened3-xcened2))/fuselgt(n);

% locate the non-dimensional vertical cg of tank 2
zcenog2 = (fusevma(n)*(wingplc(n)-0.5)+WSPNMTX(1)*tan(kdrang*wingdih(1,n))+ ...
    ycenog2*tan(kdrang*wingdih(2,n)))/fusevma(n);

%==========================================================================
% Compute the remaining outboard tank (No. 3) volume
%

lt3 = WSPNMTX(3);% tank length
lspard4 = abs(wingspf(4,n)-wingspa(4,n));% usable chord fraction
tk4end = qxtkcop(wingspn(n)/2,WSPNMTX,WTHKMTX);% thickness at outboard station

% local chord length at outboard tank station
ct4end = qxcdcop(wingspn(n)/2,WSPNMTX,WCHDROT,wingtap,n);

% predict the No. 3 tank volume
[uscvol3,ycenog3] = fvcomp(lt3,lspard3,tk3end,ct3end(n),lspard4,tk4end,ct4end(n));
uscvol3 = 2*uscvol3;
xcened4 = lt1*tan(kdrang*winglsw(1,n))+lt2*tan(kdrang*winglsw(2,n))+ ...
    lt3*tan(kdrang*winglsw(3,n))+ct4end(n)*(wingspa(4,n)+wingspf(4,n))/2;

% locate the non-dimensional longitudinal cg of tank 3
xcenog3 = (fuselgt(n)*wingapx(n)+WSPNMTX(1)*tan(kdrang*winglsw(1,n))+ ...
    WSPNMTX(2)*tan(kdrang*winglsw(2,n))+ ...
    ct3end(n)*(wingspa(3,n)+wingspf(3,n))/2+ ...
    ycenog3/lt3*(xcened4-xcened3))/fuselgt(n);

% locate the non-dimensional vertical cg of tank 3
zcenog3 = (fusevma(n)*(wingplc(n)-0.5)+WSPNMTX(1)*tan(kdrang*wingdih(1,n))+ ...
    WSPNMTX(2)*tan(kdrang*wingdih(2,n))+ ...
    ycenog3*tan(kdrang*wingdih(3,n)))/fusevma(n);

%==========================================================================
% Type of tank cut-out, i.e. max engine diameter or not
%

if fuelcut(n) > 0.0001
    if engeloc(1, n) < 3.0
        % estimate the equivalent lspard at this locale
        lsparc1 = lpcomp(BTA1CUT,lspard1,lspard2,lspard3,lspard4,wingkln,wingspn, ...
            XSECDWF,WSPNMTX,n);
        lsparc2 = lpcomp(BTA2CUT,lspard1,lspard2,lspard3,lspard4,wingkln,wingspn, ...
            XSECDWF,WSPNMTX,n);
        % estimate the cut-out volume - primary
        fvolcu1 = 2*fvcomp(ENGEDIA(1,n),lsparc1,THK1CUT,CTA1CUT,lsparc2,THK2CUT,CTA2CUT);
        if PN > 1
            lsparc3 = lpcomp(BTA3CUT,lspard1,lspard2,lspard3,lspard4,wingkln,wingspn, ...
                XSECDWF,WSPNMTX,n);
            lsparc4 = lpcomp(BTA4CUT,lspard1,lspard2,lspard3,lspard4,wingkln,wingspn, ...
                XSECDWF,WSPNMTX,n);
            % estimate the cut-out volume -secondary
            fvolcu2 = 2*fvcomp(ENGEDIA(2,n),lsparc3,THK3CUT,CTA3CUT,lsparc4,THK4CUT,CTA4CUT);
        else
            fvolcu2 = 0.0;% no secondary powerplant has been selected
        end
    else
        % aft fuse mounted, S or straight duct engines (primary)
        fvolcu1 = 0.0;
        fvolcu2 = 0.0;
    end
else
    fvolcu1 = 0.0;
    fvolcu2 = 0.0;% no cut-out to be employed
end

%==========================================================================
% Maximum fuel in wings predicted, now adjust for limited outboard
% tank span
%

if fuelobd(n) < wingkln(1, n)
    
    lta = (wingkln(1,n)-fuelobd(n))*wingspn(n)/2;% length to cut out from No. 1
    % estimate the equivalent lspard at this locale
    lsparda = 2*(lspard1-lspard2)/(XSECDWF(n)-wingkln(1,n)*wingspn(n))*(fuelobd(n)*wingspn(n)/2- ...
        XSECDWF(n))/2+lspard1;
    tkaend = qxtkcop(fuelobd(n)*wingspn(n)/2,WSPNMTX,WTHKMTX);% thickness at tank end
    % local chord length at tank end station
    ctaend = qxcdcop(fuelobd(n)*wingspn(n)/2,WSPNMTX,WCHDROT,wingtap,n);
    % predict the tank volume cut-out from No. 1
    [uscvola, ycenoge] = fvcomp(lta,lsparda,tkaend,ctaend(n),lspard2,tk2end,ct2end(n));
    uscvola = uscvola*2;
    % locate the non-dimensional longitudinal cg of tank 1 cut-out
    winespf = wingspf(2,n)-2*(wingspf(2,n)-wingspf(1,n))/(wingkln(1,n)*wingspn(n)- ...
        XSECDWF(n))*lta;
    winespa = wingspa(2,n)-2*(wingspa(2,n)-wingspa(1,n))/(wingkln(1,n)*wingspn(n)- ...
        XSECDWF(n))*lta;
    xcenoge = (fuselgt(n)*wingapx(n)+fuelobd(n)*wingspn(n)/2*tan(kdrang*winglsw(1,n))+ ...
        ctaend(n)*(winespa+winespf)/2+ ...
        ycenoge/lta*(xcened2-ctaend(n)/2- ...
        (fuelobd(n)*wingspn(n)-XSECDWF(n))/2*tan(kdrang*winglsw(1, ...
        n))))/fuselgt(n);
    % locate the non-dimensional vertical cg of tank 1 cut-out
    zcenoge = (fusevma(n)*(wingplc(n)-0.5)+ ...
        fuelobd(n)*wingspn(n)/2*tan(kdrang*wingdih(1,n)))/fusevma(n);
    uscvolo = uscvol1 - uscvola;% tally the maximum fuel that can be stored in wings
    xcgmast = (xcenog1*uscvol1-xcenoge*uscvola)/uscvolo;% adjusted x cg
    %======================================================================
%     ycgmast = (ycenog1/fuselgt(n)*uscvol1-ycenoge/fuselgt(n)*uscvola)/uscvolo;% adjusted y cg
    %======================================================================
    zcgmast = (zcenog1*uscvol1-zcenoge*uscvola)/uscvolo;% adjusted z cg
    
elseif fuelobd(n) >= wingkln(1,n) && fuelobd(n) < wingkln(2,n)
    
    lta = (wingkln(2,n)-fuelobd(n))*wingspn(n)/2;% length to cut out from No. 2
    % estimate the equivalent lspard at this locale
    lsparda = (lspard2-lspard3)/(wingkln(1,n)-wingkln(2,n))*(fuelobd(n)-wingkln(1,n))+ ...
        lspard2;
    tkaend = qxtkcop(fuelobd(n)*wingspn(n)/2,WSPNMTX,WTHKMTX);% thickness at tank end
    % local chord length at tank end station
    ctaend = qxcdcop(fuelobd(n)*wingspn(n)/2,WSPNMTX,WCHDROT,wingtap,n);
    % predict the tank volume cut-out from No. 2
    [uscvola, ycenoge] = fvcomp(lta,lsparda,tkaend,ctaend(n),lspard3,tk3end,ct3end(n));
    uscvola = uscvola*2;
    % locate the non-dimensional longitudinal cg of tank 2 cut-out
    winespf = wingspf(3,n)-2*(wingspf(3,n)-wingspf(2,n))/((wingkln(2,n)- ...
        wingkln(1,n))*wingspn(n))*lta;
    winespa = wingspa(3,n)-2*(wingspa(3,n)-wingspa(2,n))/((wingkln(2,n)- ...
        wingkln(1,n))*wingspn(n))*lta;
    xcenoge = (fuselgt(n)*wingapx(n)+ ...
        wingkln(1,n)*wingspn(n)/2*tan(kdrang*winglsw(1,n))+ ...
        (fuelobd(n)-wingkln(1,n))*wingspn(n)/2*tan(kdrang*winglsw(2,n))+ ...
        ctaend(n)*(winespa+winespf)/2+ ...
        ycenoge/lta*(xcened3-ctaend(n)/2- ...
        (WSPNMTX(1)-XSECDWF(n)/2)*tan(kdrang*winglsw(1,n))- ...
        (fuelobd(n)-wingkln(1,n))*wingspn(n)/2*tan(kdrang*winglsw(2, ...
        n))))/fuselgt(n);
    % locate the non-dimensional vertical cg of tank 2 cut-out
    zcenoge = (fusevma(n)*(wingplc(n)-0.5)+ ...
        WSPNMTX(1)*tan(kdrang*wingdih(1,n))+ ...
        (fuelobd(n)-wingkln(1,n))*wingspn(n)/2*tan(kdrang*wingdih(2, ...
        n)))/fusevma(n);
    % tally the maximum fuel that can be stored in the wings
    uscvolo = uscvol1 + uscvol2 - uscvola;
    xcgmast = (xcenog1*uscvol1+xcenog2*uscvol2-xcenoge*uscvola)/uscvolo;% x cg
    %======================================================================
%     ycgmast = (ycenog1/fuselgt(n)*uscvol1+ycenog2/fuselgt(n)*uscvol2-ycenoge/fuselgt(n)*uscvola)/uscvolo;% y cg
    %======================================================================
    zcgmast = (zcenog1*uscvol1+zcenog2*uscvol2-zcenoge*uscvola)/uscvolo;% z cg
    
elseif fuelobd(n) >= wingkln(2,n)
    lta = (1-fuelobd(n))*wingspn(n)/2;% length to cut out from No. 3
    % estimate the equivalent lspard at this locale
    lsparda = (lspard3-lspard4)/(wingkln(2,n)-1)*(fuelobd(n)-wingkln(2,n))+ ...
        lspard3;
    tkaend = qxtkcop(fuelobd(n)*wingspn(n)/2,WSPNMTX,WTHKMTX);% thickness at tank end
    % local chord length at tank end station
    ctaend = qxcdcop(fuelobd(n)*wingspn(n)/2,WSPNMTX,WCHDROT,wingtap,n);
    % predict the tank volume cut-out from No. 3
    [uscvola, ycenoge] = fvcomp(lta,lsparda,tkaend,ctaend(n),lspard4,tk4end,ct4end(n));
    uscvola = uscvola*2;
    % locate the non-dimensional longitudinal cg of tank 3 cut-out
    winespf = wingspf(4,n)-2*(wingspf(4,n)-wingspf(3,n))/((1- ...
        wingkln(2,n))*wingspn(n))*lta;
    winespa = wingspa(4,n)-2*(wingspa(4,n)-wingspa(3,n))/((1- ...
        wingkln(2,n))*wingspn(n))*lta;
    xcenoge = (fuselgt(n)*wingapx(n)+ ...
        wingkln(1,n)*wingspn(n)/2*tan(kdrang*winglsw(1,n))+ ...
        wingkln(2,n)*wingspn(n)/2*tan(kdrang*winglsw(2,n))+ ...
        (fuelobd(n)-wingkln(2,n))*wingspn(n)/2*tan(kdrang*winglsw(3,n))+ ...
        ctaend(n)*(winespa+winespf)/2+ ...
        ycenoge/lta*(xcened4-ctaend(n)/2- ...
        (WSPNMTX(1)-XSECDWF(n)/2)*tan(kdrang*winglsw(1,n))- ...
        WSPNMTX(2)*tan(kdrang*winglsw(2,n))- ...
        (fuelobd(n)-wingkln(2,n))*wingspn(n)/2*tan(kdrang*winglsw(3, ...
        n))))/fuselgt(n);
    % locate the non-dimensional vertical cg of tank 3 cut-out
    zcenoge = (fusevma(n)*(wingplc(n)-0.5)+ ...
        WSPNMTX(1)*tan(kdrang*wingdih(1,n))+ ...
        WSPNMTX(2)*tan(kdrang*wingdih(2,n))+ ...
        (fuelobd(n)-wingkln(2,n))*wingspn(n)/2*tan(kdrang*wingdih(3, ...
        n)))/fusevma(n);
    % tally the maximum fuel that can be stored in the wings
    uscvolo = uscvol1 + uscvol2 + uscvol3 - uscvola;
    xcgmast = (xcenog1*uscvol1+xcenog2*uscvol2+xcenog3*uscvol3- ...
        xcenoge*uscvola)/uscvolo;% adjusted x cg
    %======================================================================        
%     ycgmast = (ycenog1/fuselgt(n)*uscvol1+ycenog2/fuselgt(n)*uscvol2+ycenog3/fuselgt(n)*uscvol3- ...
%         ycenoge/fuselgt(n)*uscvola)/uscvolo;% adjusted y cg
    %======================================================================
    zcgmast = (zcenog1*uscvol1+zcenog2*uscvol2+zcenog3*uscvol3- ...
        zcenoge*uscvola)/uscvolo;% adjusted z cg
end

if fuelobd(n) >= 2*BTA2CUT/wingspn(n)
    % engine cut-out - primary only
    uscvolo = uscvolo - fuelcut(n)*fvolcu1;
end

if fuelobd(n) >= 2*BTA4CUT/wingspn(n)
    % engine cut-out - primary and secondary
    uscvolo = uscvolo - fuelcut(n)*fvolcu2;
end

%==========================================================================
% intially predicted maximum fuel weight in wing No. 1
mfuvwei = (uscvolo+fuelwic(n)/1000/fuelden(n))*1000;
mfuwwei = mfuvwei*fuelden(n);
%==========================================================================
% check if a second wing has been defined
%
uscvole=0.0;
xcgmas2=0.0;
%===========
ycgmas2=0.0;
%===========
zcgmas2=0.0;% default no fuel in wing 2
centvo2=0.0;
xcogce2=0.0;
zcogce2=0.0;% default no fuel in centre tank
%

if wi2present(1,n) == 1
%if wi2gare(n) > 0.0001
    %=====
    % compute the inboard tank (No. 1) volume
    lt1 = WS2NMTX(1)-XS2CDWF(n)/2;% tank length
    tk1end = qxtkcop(XS2CDWF(n)/2,WS2NMTX,WT2KMTX);% thickness at wing-fuse
    % local chord length at wing-fuse
    ct1end = qxcdcop(XS2CDWF(n)/2,WS2NMTX,WC2DROT,wi2gtap,n);
    tk2end = qxtkcop(wi2gkln(1,n)*wi2gspn(n)/2,WS2NMTX,WT2KMTX);% thickness at kink 1
    % local chord length at kink 1 station
    ct2end=qxcdcop(wi2gkln(1,n)*wi2gspn(n)/2,WS2NMTX,WC2DROT,wi2gtap,n);
    % predict the No. 1 tank volume
    [uscvol1,ycenog1]=fvcomp(lt1,lspard1,tk1end,ct1end(n),lspard2,tk2end,ct2end(n));
    uscvol1=2*uscvol1;
    % locate forward and aft spar centroid at kink 1
    xcened2=lt1*tan(kdrang*wi2glsw(1,n))+ct2end(n)*(wingspa(2,n)+wingspf(2,n))/2;
    % locate the non-dimensional longitudinal cg of tank 1
    xcenog1=(fuselgt(n)*wi2gapx(n)+XS2CDWF(n)/2*tan(kdrang*wi2glsw(1,n))+ ...
        ct1end(n)*(wingspa(1,n)+wingspf(1,n))/2+ ...
        ycenog1/lt1*(xcened2-ct1end(n)/2))/fuselgt(n);
    % locate the non-dimensional vertical cg of tank 1
    zcenog1=(fusevma(n)*(wi2gplc(n)-0.5)+ ...
        (ycenog1+XS2CDWF(n)/2)*tan(kdrang*wi2gdih(1,n)))/fusevma(n);
    %=================================
    % predict the centre tank volume
    if fuelcen(n)>0.0001
        centvo2=fuelcen(n)/100*2*fvcomp(XS2CDWF(n)/2,lspard1, ...
            wi2gthk(1,n),WC2RDWF(n),lspard1,WT2KROT(n),WC2DROT(n));
        MFCVWEI(n)=MFCVWEI(n)+centvo2*1000;
        MFCWWEI(n)=MFCVWEI(n)*fuelden(n);
        xcogce2=(fuselgt(n)*wi2gapx(n)+XS2CDWF(n)/2*tan(kdrang*wi2glsw(1,n))+ ...
            WC2RDWF(n)*(wingspa(1,n)+wingspf(1,n))/2)/fuselgt(n);% x cg
        zcogce2=WC2DROT(n)*wi2gthk(1,n)/(2*fusevma(n))-0.5;% z cg
    end
    %=================================
    % compute the midboard tank (No. 2) volume
    lt2=WS2NMTX(2);% tank length
    tk3end=qxtkcop(wi2gkln(2,n)*wi2gspn(n)/2,WS2NMTX,WT2KMTX);% thickness at kink 2
    % local chord length at kink 2 station
    ct3end=qxcdcop(wi2gkln(2,n)*wi2gspn(n)/2,WS2NMTX,WC2DROT,wi2gtap,n);
    % predict the No. 2 volume
    [uscvol2,ycenog2]=fvcomp(lt2,lspard2,tk2end,ct2end(n),lspard3,tk3end,ct3end(n));
    uscvol2=2*uscvol2;
    xcened3=lt1*tan(kdrang*wi2glsw(1,n))+lt2*tan(kdrang*wi2glsw(2,n))+ ...
        ct3end(n)*(wingspa(3,n)+wingspf(3,n))/2;
    % locate the non-dimensional longitudinal cg of tank 2
    xcenog2=(fuselgt(n)*wi2gapx(n)+WS2NMTX(1)*tan(kdrang*wi2glsw(1,n))+ ...
        ct2end(n)*(wingspa(2,n)+wingspf(2,n))/2+ ...
        ycenog2/lt2*(xcened3-xcened2))/fuselgt(n);
    % locate the non-dimensional vertical cg of tank 2
    zcenog2=(fusevma(n)*(wi2gplc(n)-0.5)+WS2NMTX(1)*tan(kdrang*wi2gdih(1,n))+ ...
        ycenog2*tan(kdrang*wi2gdih(2,n)))/fusevma(n);
    %=====
    % compute the remaining outboard tank (No. 3) volume
    lt3=WS2NMTX(3);% tank length
    tk4end=qxtkcop(wi2gspn(n)/2,WS2NMTX,WT2KMTX);% thickness at outboard station
    % local chord length at outboard tank station
    ct4end=qxcdcop(wi2gspn(n)/2,WS2NMTX,WC2DROT,wi2gtap,n);
    % predict the No. 3 tank volume
    [uscvol3,ycenog3]=fvcomp(lt3,lspard3,tk3end,ct3end(n),lspard4,tk4end,ct4end(n));
    uscvol3=2*uscvol3;
    xcened4=lt1*tan(kdrang*wi2glsw(1,n))+lt2*tan(kdrang*wi2glsw(2,n))+ ...
        lt3*tan(kdrang*wi2glsw(3,n))+ct4end(n)*(wingspa(4,n)+wingspf(4,n))/2;
    % locate the non-dimensional longitudinal cg of tank 3
    xcenog3=(fuselgt(n)*wi2gapx(n)+WS2NMTX(1)*tan(kdrang*wi2glsw(1,n))+ ...
        WS2NMTX(2)*tan(kdrang*wi2glsw(2,n))+ ...
        ct3end(n)*(wingspa(3,n)+wingspf(3,n))/2+ ...
        ycenog3/lt3*(xcened4-xcened3))/fuselgt(n);
    % locate the non-dimensional vertical cg of tank 3
    zcenog3=(fusevma(n)*(wi2gplc(n)-0.5)+WS2NMTX(1)*tan(kdrang*wi2gdih(1,n))+ ...
        WS2NMTX(2)*tan(kdrang*wi2gdih(2,n))+ ...
        ycenog3*tan(kdrang*wi2gdih(3,n)))/fusevma(n);
    %=====
    % maximum fuel in wings predicted, now adjust for limited outboard
    % tank span
    if fuelobd(n)<=wi2gkln(1,n)
        
        lta=(wi2gkln(1,n)-fuelobd(n))*wi2gspn(n)/2;% length to cut out from No. 1
        % estimate the equivalent lspard at this locale
        lsparda=2*(lspard1-lspard2)/(XS2CDWF(n)-wi2gkln(1,n)*wi2gspn(n))*(fuelobd(n)*wi2gspn(n)/2- ...
            XS2CDWF(n))/2+lspard1;
        tkaend=qxtkcop(fuelobd(n)*wi2gspn(n)/2,WS2NMTX,WT2KMTX);% thickness at tank end
        % local chord length at tank end station
        ctaend=qxcdcop(fuelobd(n)*wi2gspn(n)/2,WS2NMTX,WC2DROT,wi2gtap,n);
        % predict the tank volume cut-out from No. 1
        [uscvola,ycenoge]=fvcomp(lta,lsparda,tkaend,ctaend(n),lspard2,tk2end,ct2end(n));
        uscvola=uscvola*2;
        % locate the non-dimensional longitudinal cg of tank 1 cut-out
        winespf=wingspf(2,n)-2*(wingspf(2,n)-wingspf(1,n))/(wi2gkln(1,n)*wi2gspn(n)- ...
            XS2CDWF(n))*lta;
        winespa=wingspa(2,n)-2*(wingspa(2,n)-wingspa(1,n))/(wi2gkln(1,n)*wi2gspn(n)- ...
            XS2CDWF(n))*lta;
        xcenoge=(fuselgt(n)*wi2gapx(n)+fuelobd(n)*wi2gspn(n)/2*tan(kdrang*wi2glsw(1,n))+ ...
            ctaend(n)*(winespa+winespf)/2+ ...
            ycenoge/lta*(xcened2-ctaend(n)/2- ...
            (fuelobd(n)*wi2gspn(n)-XS2CDWF(n))/2*tan(kdrang*wi2glsw(1, ...
            n))))/fuselgt(n);
        % locate the non-dimensional vertical cg of tank 1 cut-out
        zcenoge=(fusevma(n)*(wi2gplc(n)-0.5)+ ...
            fuelobd(n)*wi2gspn(n)/2*tan(kdrang*wi2gdih(1,n)))/fusevma(n);
        uscvole=uscvol1-uscvola;% tally the maximum fuel that can be stored in wings
        xcgmas2=(xcenog1*uscvol1-xcenoge*uscvola)/uscvole;% adjusted x cg
        %==================================================================
%         ycgmas2=(ycenog1/fuselgt(n)*uscvol1-ycenoge/fuselgt(n)*uscvola)/uscvole;% adjusted y cg
        %==================================================================
        zcgmas2=(zcenog1*uscvol1-zcenoge*uscvola)/uscvole;% adjusted z cg
        
    elseif fuelobd(n)>wi2gkln(1,n) && fuelobd(n)<=wi2gkln(2,n)
        lta=(wi2gkln(2,n)-fuelobd(n))*wi2gspn(n)/2;% length to cut out from No. 2
        % estimate the equivalent lspard at this locale
        lsparda=(lspard2-lspard3)/(wi2gkln(1,n)-wi2gkln(2,n))*(fuelobd(n)-wi2gkln(1,n))+ ...
            lspard2;
        tkaend=qxtkcop(fuelobd(n)*wi2gspn(n)/2,WS2NMTX,WT2KMTX);% thickness at tank end
        % local chord length at tank end station
        ctaend=qxcdcop(fuelobd(n)*wi2gspn(n)/2,WS2NMTX,WC2DROT,wi2gtap,n);
        % predict the tank volume cut-out from No. 2
        [uscvola,ycenoge]=fvcomp(lta,lsparda,tkaend,ctaend(n),lspard3,tk3end,ct3end(n));
        uscvola=uscvola*2;
        % locate the non-dimensional longitudinal cg of tank 2 cut-out
        winespf=wingspf(3,n)-2*(wingspf(3,n)-wingspf(2,n))/((wi2gkln(2,n)- ...
            wi2gkln(1,n))*wi2gspn(n))*lta;
        winespa=wingspa(3,n)-2*(wingspa(3,n)-wingspa(2,n))/((wi2gkln(2,n)- ...
            wi2gkln(1,n))*wi2gspn(n))*lta;
        xcenoge=(fuselgt(n)*wi2gapx(n)+ ...
            wi2gkln(1,n)*wi2gspn(n)/2*tan(kdrang*wi2glsw(1,n))+ ...
            (fuelobd(n)-wi2gkln(1,n))*wi2gspn(n)/2*tan(kdrang*wi2glsw(2,n))+ ...
            ctaend(n)*(winespa+winespf)/2+ ...
            ycenoge/lta*(xcened3-ctaend(n)/2- ...
            (WS2NMTX(1)-XS2CDWF(n)/2)*tan(kdrang*wi2glsw(1,n))- ...
            (fuelobd(n)-wi2gkln(1,n))*wi2gspn(n)/2*tan(kdrang*wi2glsw(2, ...
            n))))/fuselgt(n);
        % locate the non-dimensional vertical cg of tank 2 cut-out
        zcenoge=(fusevma(n)*(wi2gplc(n)-0.5)+ ...
            WS2NMTX(1)*tan(kdrang*wi2gdih(1,n))+ ...
            (fuelobd(n)-wi2gkln(1,n))*wi2gspn(n)/2*tan(kdrang*wi2gdih(2, ...
            n)))/fusevma(n);
        % tally the maximum fuel that can be stored in the wings
        uscvole=uscvol1+uscvol2-uscvola;
        xcgmas2=(xcenog1*uscvol1+xcenog2*uscvol2-xcenoge*uscvola)/uscvole;% xcg
        %==================================================================
%         ycgmas2=(ycenog1/fuselgt(n)*uscvol1+ycenog2/fuselgt(n)*uscvol2-ycenoge/fuselgt(n)*uscvola)/uscvole;% ycg
        %==================================================================
        zcgmas2=(zcenog1*uscvol1+zcenog2*uscvol2-zcenoge*uscvola)/uscvole;% zcg
        
    elseif fuelobd(n)>wi2gkln(2,n)
        lta=(1-fuelobd(n))*wi2gspn(n)/2;% length to cut out from No. 3
        % estimate the equivalent lspard at this locale
        lsparda=(lspard3-lspard4)/(wi2gkln(2,n)-1)*(fuelobd(n)-wi2gkln(2,n))+ ...
            lspard3;
        tkaend=qxtkcop(fuelobd(n)*wi2gspn(n)/2,WS2NMTX,WT2KMTX);% thickness at tank end
        % local chord length at tank end station
        ctaend=qxcdcop(fuelobd(n)*wi2gspn(n)/2,WS2NMTX,WC2DROT,wi2gtap,n);
        % predict the tank volume cut-out from No. 3
        [uscvola,ycenoge]=fvcomp(lta,lsparda,tkaend,ctaend(n),lspard4,tk4end,ct4end(n));
        uscvola=uscvola*2;
        % locate the non-dimensional longitudinal cg of tank 3 cut-out
        winespf=wingspf(4,n)-2*(wingspf(4,n)-wingspf(3,n))/((1- ...
            wi2gkln(2,n))*wi2gspn(n))*lta;
        winespa=wingspa(4,n)-2*(wingspa(4,n)-wingspa(3,n))/((1- ...
            wi2gkln(2,n))*wi2gspn(n))*lta;
        xcenoge=(fuselgt(n)*wi2gapx(n)+ ...
            wi2gkln(1,n)*wi2gspn(n)/2*tan(kdrang*wi2glsw(1,n))+ ...
            wi2gkln(2,n)*wi2gspn(n)/2*tan(kdrang*wi2glsw(2,n))+ ...
            (fuelobd(n)-wi2gkln(2,n))*wi2gspn(n)/2*tan(kdrang*wi2glsw(3,n))+ ...
            ctaend(n)*(winespa+winespf)/2+ ...
            ycenoge/lta*(xcened4-ctaend(n)/2- ...
            (WS2NMTX(1)-XS2CDWF(n)/2)*tan(kdrang*wi2glsw(1,n))- ...
            WS2NMTX(2)*tan(kdrang*wi2glsw(2,n))- ...
            (fuelobd(n)-wi2gkln(2,n))*wi2gspn(n)/2*tan(kdrang*wi2glsw(3, ...
            n))))/fuselgt(n);
        % locate the non-dimensional vertical cg of tank 3 cut-out
        zcenoge=(fusevma(n)*(wi2gplc(n)-0.5)+ ...
            WS2NMTX(1)*tan(kdrang*wi2gdih(1,n))+ ...
            WS2NMTX(2)*tan(kdrang*wi2gdih(2,n))+ ...
            (fuelobd(n)-wi2gkln(2,n))*wi2gspn(n)/2*tan(kdrang*wi2gdih(3, ...
            n)))/fusevma(n);
        % tally the maximum fuel that can be stored in the wings
        uscvole=uscvol1+uscvol2+uscvol3-uscvola;
        xcgmas2=(xcenog1*uscvol1 + xcenog2*uscvol2 + xcenog3*uscvol3- ...
            xcenoge*uscvola)/uscvole;% adjusted x cg
        %==================================================================
%         ycgmas2=(ycenog1/fuselgt(n)*uscvol1+ycenog2/fuselgt(n)*uscvol2+ycenog3/fuselgt(n)*uscvol3- ...
%             ycenoge/fuselgt(n)*uscvola)/uscvole;% adjusted y cg
        %==================================================================        
        zcgmas2=(zcenog1*uscvol1+zcenog2*uscvol2+zcenog3*uscvol3- ...
            zcenoge*uscvola)/uscvole;% adjusted z cg
    end
    %=====
    % intially predicted maximum fuel weight in wing No. 2
    mfuvwei = mfuvwei + uscvole*1000;
    mfuwwei = mfuvwei*fuelden(n);
end

% compute the combined wing 1 and wing 2 non-dimensional centre of gravity
xcgwing = (xcgmast*uscvolo + xcgmas2*uscvole)/(uscvolo + uscvole); % x cg
% ycgwing = (ycgmast*uscvolo + ycgmas2*uscvole)/(uscvolo + uscvole); % y cg
ycgwing = 0.0;
zcgwing = (zcgmast*uscvolo + zcgmas2*uscvole)/(uscvolo + uscvole); % z cg

%==========================================================================
% predict the auxiliary tank volumes

%=====
% first compute the volume existing in the fuselage for potential
% fuel storage

if cabnwid(n) < 0 || cabnfwd(n) < 0

    if cabnhei(n) < 0.0001 && fusevma(n) > 0.0001
        if fusevmi(n) > 0.5
            cabnhei(n) = fusevmi(n)*fusevma(n)-0.15;% double-bubble cabin height
        else
            if wingplc(n) < 0 || wingplc(n) > 1
                cabnhei(n) = fusevma(n)-0.2;% wingbox does not go through x-section
            else
                if wingplc(n) > 0.5
                    % circular x-section cabin height for high wings
                    cabnhei(n) = fusevma(n)*wingplc(n)-WTHKMTX(2,1)*WCHDROT(n)/2-0.2;
                else
                    % circular x-section cabin height for low wings
                    cabnhei(n) = fusevma(n)*(1-wingplc(n))-WTHKMTX(2,1)*WCHDROT(n)/2-0.2;
                end
            end
        end
    end
    if cabnwid(n) < 0.0001 && fusehma(n) > 0.0001
        cabnwid(n) = fusehma(n)-0.2;    % predict cabin max width
    end
    if cabnfwd(n) < 0.0001 && fusevma(n) > 0.0001
        if fusevmi(n) > 0.5
            sweptan = atan(2*fusevma(n)*(fusevmi(n)-0.5)/fusehma(n))+pi;
            sweptrd = CROSXCF(1)+CROSXCF(2)*sin(sweptan)+CROSXCF(3)*cos(2*sweptan);
            cabnfwd(n) = 2*sweptrd*cos(sweptan-pi)-0.2;% double-bubble floor width
        else
            cabnfwd(n) = ((cabnwid(n)^2)-4*(((fusevma(n)-WTHKMTX(2,1)*WCHDROT(n)- ...
                0.2)/2)^2))^0.5;% predict the floor width
        end
    end


else
    cabxsec = pi/36*(2*fusevma(n)+fusehma(n))^2;% entire fuselage xsec area
    heigbar = cabnhei(n)+0.1-fusevma(n)/2;
    thetwid = real(asin((cabnfwd(n)+0.2)/fusehma(n)));
    uflxsec = thetwid*(fusehma(n)^2)/4-(cabnfwd(n)+0.2)*heigbar/2;% uflr xsec area
end

fwdfair=0.0; fwdfaid=0.0; xcgfwfr=0.0; zcgfwfr=0.0;% default no fuel in fwd fairing

if (fairovh(n)*wingthk(1,n)*WCHDROT(n)/2-wingplc(1,n)*fusevma(n)) > 0.0001

    % estimate the amount of fuel to be stored in forward fairing
    thkmaxf = fairovh(n)*wingthk(1,n)*WCHDROT(n)/2-wingplc(1,n)*fusevma(n);% fairing maximum height
    widmaxf = XSECDWF(n)/2;% maximum horizontal fuselage width
    fwdfaix = uflxsec+pi*thkmaxf*widmaxf/4;
    if fuelaux(1,n) > 0.0001
        fwdfxdi = (1-fuelaux(1,n)/100)*fairfwd(n)/100*WCHDROT(n);
        semyaxf = (fwdfxdi/(fairfwd(n)/100*WCHDROT(n))*widmaxf^2)^0.5;
        semzaxf = (fwdfxdi/(fairfwd(n)/100*WCHDROT(n))*thkmaxf^2)^0.5;
        fwdfaid = 0.92*(uflxsec+pi*semyaxf*semzaxf/4)*fwdfxdi;% volume to be removed
        fwdfair = 0.92*fwdfaix*fairfwd(n)/100*WCHDROT(n);% max possible volume
        % horizontal non-dimensional cg location
        xcgfwfr = (fuselgt(n)*wingapx(n)+WCHDROT(n)*wingspf(1,n)-fwdfxdi/3)/fuselgt(n);
        zcgfwfr = wingplc(n)-0.5;% vertical non-dimensional cg location
    end

end
%=====
% estimate the amount of fuel to be stored in aft fairing
aftfair=0.0;aftfaid=0.0;xcgaffr=0.0;% default no fuel in aft fairing
if (fairovh(n)*wingthk(1,n)*WCHDROT(n)/2-wingplc(1,n)*fusevma(n)) > 0.0001
    if fuelaux(2, n) > 0.0001
        fwdfxdi = (1-fuelaux(2,n)/100)*fairaft(n)/100*WCHDROT(n);
        semyaxf = (fwdfxdi/(fairaft(n)/100*WCHDROT(n))*widmaxf^2)^0.5;
        semzaxf = (fwdfxdi/(fairaft(n)/100*WCHDROT(n))*thkmaxf^2)^0.5;
        aftfaid = 0.92*(uflxsec+pi*semyaxf*semzaxf/4)*fwdfxdi;% volume to be removed
        aftfair = 0.92*fwdfaix*fairaft(n)/100*WCHDROT(n);% max possible volume
        % horizontal non-dimensional cg location
        xcgaffr = (fuselgt(n)*wingapx(n)+WCHDROT(n)*wingspa(1,n)+fwdfxdi/3)/fuselgt(n);
    end
end
%=====
% combined centre of gravity locations for conformal fairing tanks only
if fuelcen(n) > 0.0001 || fuelaux(1, n) > 0.0001 || fuelaux(2, n) > 0.0001
    xcgfair = (xcogcen*centvol+xcogce2*centvo2+xcgfwfr*(fwdfair-fwdfaid)+ ...
        xcgaffr*(aftfair-aftfaid))/(centvol+centvo2+fwdfair-fwdfaid+ ...
        aftfair-aftfaid);% x cg
    ycgfair = 0.0;
    zcgfair = zcgfwfr;% z cg
end
%=====

% Estimate the amount of fuel to be stored in aft-fuselage tank
auxfair = 0.0;% default no fuel in auxiliary tank
if fuelafa(n) > 0.0001
    aftfaix = cabxsec - uflxsec;
    auxfair = 0.92*aftfaix*fuelafa(n);% total volume for fuel
    xcgtaux = 1-(aftslgt(n)+fuelafa(n)/2)/fuselgt(n);% non-dimensional horizontal cg
    ycgtaux = 0.0;
    zcgtaux = ((fusevma(n)-thkmaxf)*0.4-heigbar)/fusevma(n);% non-dim vertical cg
end

%==========================================================================
% tally all sources for auxiliary fuel storage
MFAVWEI(n) = 1000*(fwdfair - fwdfaid + aftfair - aftfaid + auxfair + ...
    fuelaic(n)/1000/fuelden(n));
MFAWWEI(n) = MFAVWEI(n)*fuelden(n);

%==========================================================================
% predict the unusuable fuel if user has not specified it
%

if fuelusu(n) < 0.0001
    fuelusu(n) = 0.02*mfuwwei;
    if fuelusu(n) < 30
        fuelusu(n) = 30;% unusuable fuel cannot be less than 30 kg
    end
end

%==========================================================================
% final adjusted predicted maximum fuel weight in wings
%

MFUVWEI(n) = mfuvwei - fuelusu(n)/fuelden(n);
MFUWWEI(n) = MFUVWEI(n)*fuelden(n);

%==========================================================================
% Ouput parameters from QFUCALC
aircraft.weight_balance.Fuel.Fuel_in_wing_x_cg = xcgwing(1, n);
aircraft.weight_balance.Fuel.Fuel_in_wing_y_cg = ycgwing(1, n);
aircraft.weight_balance.Fuel.Fuel_in_wing_z_cg = zcgwing(1, n);
aircraft.weight_balance.Fuel.Fuel_in_fairings_x_cg = xcgfair(1, n);
aircraft.weight_balance.Fuel.Fuel_in_fairings_y_cg = ycgfair(1, n);
aircraft.weight_balance.Fuel.Fuel_in_fairings_z_cg = zcgfair(1, n);
aircraft.weight_balance.Fuel.Fuel_in_auxiliary_tanks_x_cg = xcgtaux(1, n);
aircraft.weight_balance.Fuel.Fuel_in_auxiliary_tanks_y_cg = ycgtaux(1, n);
aircraft.weight_balance.Fuel.Fuel_in_auxiliary_tanks_z_cg = zcgtaux(1, n);

aircraft.fuel.max_weight_wing = MFUWWEI(1, n);
aircraft.fuel.max_vol_wing = MFUVWEI(1, n);
aircraft.fuel.max_weight_cent_wing_box = MFCWWEI(1, n);
aircraft.fuel.max_vol_cent_wing_box = MFCVWEI(1, n);
aircraft.fuel.max_weight_aux = MFAWWEI(1, n);
aircraft.fuel.max_vol_aux = MFAVWEI(1, n);

% Overwrite values
aircraft.fuel.box_ea_loc_root  = box(1);
aircraft.fuel.box_ea_loc_kink1 = box(2);
aircraft.fuel.box_ea_loc_kink2 = box(3);
aircraft.fuel.box_ea_loc_tip   = box(4);

aircraft.fuel.box_semispan_root  = b_box(1);
aircraft.fuel.box_semispan_kink1 = b_box(2);
aircraft.fuel.box_semispan_kink2 = b_box(3);
aircraft.fuel.box_semispan_tip   = b_box(4);

aircraft.fuel.Fore_wing_spar_loc_root = wingspf(1, n);
aircraft.fuel.Fore_wing_spar_loc_kik1 = wingspf(2, n);
aircraft.fuel.Fore_wing_spar_loc_kin2 = wingspf(3, n);
aircraft.fuel.Fore_wing_spar_loc_tip  = wingspf(4, n);

aircraft.fuel.Aft_wing_spar_loc_root = wingspa(1, n);
aircraft.fuel.Aft_wing_spar_loc_kin1 = wingspa(2, n);
aircraft.fuel.Aft_wing_spar_loc_kin2 = wingspa(3, n);
aircraft.fuel.Aft_wing_spar_loc_tip  = wingspa(4, n);

%==========================================================================


%==========================================================================
% Auxiliary functions
%

function [comp1, comp2] = fvcomp(lgth, spard1, thk1, ct1, spard2, thk2, ct2)
% This computes the available fuel volume
%

kadj = 0.92;  % Accounts for geometric and structural adjustments
s1 = thk1*spard1*(ct1^2);
s2 = thk2*spard2*(ct2^2);
comp1 = lgth/3*kadj*(s1+s2+(s1*s2)^0.5);  % Torenbeek volume
comp2 = lgth/4*(s1+3*s2+2*(s1*s2)^0.5)/(s1+s2+(s1*s2)^0.5);  % Torenbeek cog


function comp2 = lpcomp(lspan, lsp1, lsp2, lsp3, lsp4, kln, spn, xsec, wsp, z)
% compute the equivalent lspard at any station
%

if lspan <= kln(1, z)*spn(z)/2
    % estimate the equivalent lspard at inboard
    comp2 = (lsp1-lsp2)/(xsec(z)/2-wsp(1))*(lspan-xsec(z)/2) + lsp1;
elseif lspan > kln(1, z)*spn(z) && lspan <= kln(2,z)*spn(z)
    % estimate the equivalent lspard at midboard
    comp2 = -(lsp2-lsp3)/wsp(2)*(lspan-wsp(1)) + lsp2;
elseif lspan > kln(2, z)
    % estimate the equivalent lspard at outboard
    comp2 = -(lsp3-lsp4)/wsp(3)*(lspan-wsp(1) + wsp(2)) + lsp3;
end


function [kln,spn,gar,smtrx,crot,xscd,tap,cref,chrf,tref,chwf,troot, ...
    tmtrx,qsw,lsw,netwgae,wsatref,wsarref,wsasref,wsarlsw, ...
    wsarqsw,wsarhsw,wsarmac,wsarybr,wsarthb,inc] = qxemcop(kln,tap, ...
    spn,are,gar,hma,thk,qsw,lsw,refc,loc,inc,z)
% This routine computes the lifting surface geometric and aerodynamic
% attributes. There are two parts to this analysis.
% The first deals with a kinked (with Yehudi) wing. The second option is
% input of a true trapezoidal wing - that is treated as a ghost kinked wing.
% This method is employed in order to supply detailed planform layout for
% subsequent 3D Vortex-Lattice analysis.
%
%=====
% check if either planform kinks have been input and are different from 0
% or 1

% Check that 2 kinks have been defined, and not set at span fraction 0 or 1
% wich will give problems for the CAD generation. In that case we create
% ghost kinks

kdrang=2*pi/360;  % conversion from deg. to rad.

if kln(1,z)<0.0001 || kln(1,z)>0.9999
    kln(1,z)=0.01;% assume a ghost kink 1 at 0.01 semi-span
    tap(1,z)=0; % set the taper to 0 so that it will be automatically recomputed
    if kln(2,z)<0.0001  || kln(2,z)>0.9999
        inc(2,z)=0.01/0.99*inc(3,z)+(kln(2,z)-0.01)/0.99*inc(1,z); % The next if loop will set the spanwise position of kink2 to 0.99
    else
        inc(2,z)=0.01/kln(2,z)*inc(3,z)+(kln(2,z)-0.01)/kln(2,z)*inc(1,z);
    end
end
if kln(2,z)<0.0001  || kln(2,z)>0.9999
    kln(2,z)=0.99;% assume a ghost kink 2 at 0.99 semi-span
    tap(2,z)=0;% set the taper to 0 so that it will be automatically recomputed
    inc(3,z)=(0.99-kln(1,z))/(1-kln(1,z))*inc(4,z)+(1-0.99)/(1-kln(1,z))*inc(2,z);
end


if kln(1,z)>=kln(2,z)
    kln(1,z)=kln(2,z)-0.01;% error trap inboard kink is less than outboard
end
if kln(2,z)<=kln(1,z)
    kln(2,z)=kln(1,z)+0.01;% error trap outboard kink is less than inboard
end
%=====
% check if taper ratios at kinks have been input
if tap(1,z)<0.0001
    tap(1,z)=1-(1-tap(3,z))*kln(1,z);% taper ratio at ghost kink 1
end
if tap(2,z)<0.0001 || hma(z)<0.0001
    % taper ratio at ghost kink 2
    tap(2,z)=tap(1,z)*(1-(1-tap(3,z)/tap(1,z))*(kln(2,z)-kln(1,z))/(1-kln(1,z)));
end
%=====
% check if either AR or span were input (must have at least gross area)
if spn(z)<0.0001
    spn(z)=(are(z)*gar(z))^0.5;% geom. AR known compute span
else
    gar(z)=(spn(z)^2)/are(z);% span known compute geom. AR
end
%=====
% create span matrix to partition inboard-mid-outboard wings
smtrx=[kln(1,z)*spn(z)/2 (kln(2,z)-kln(1,z))*spn(z)/2 (1-kln(2,z))*spn(z)/2];
chrdk1=(1+tap(1,z))*smtrx(1);
chrdk2=(tap(1,z)+tap(2,z))*smtrx(2);
chrdk3=(tap(2,z)+tap(3,z))*smtrx(3);
crot=are(z)/(chrdk1+chrdk2+chrdk3);% actual wing root chord calculation
%=====
wingari=(1+tap(1,z))*crot*smtrx(1);% inbd actual wing area
wingarm=(tap(1,z)+tap(2,z))*crot*smtrx(2);% mibd actual wing area
wingaro=are(z)-wingari-wingarm;% outboard actual wing area
%=====
% the most basic wing reference
wingtpo=tap(3,z)/tap(1,z);% calculate the outboard wing taper
tref=(1-kln(1,z))/(1/wingtpo-kln(1,z));
cref=tap(3,z)*crot/tref;% root chord based on ref. wing
asref=2*spn(z)/(cref+tap(3,z)*crot);% AR based on ref. wing
arref=cref*(1+tref)*spn(z)/2;% ref. wing area
%============================================================================
% Construct a thickness model, assume trapezoidal variation
% the first kink point
if thk(2,z)<0.0001
    if thk(1,z)-thk(4,z)>0.021
        thk(2,z)=thk(1,z)-0.02;% assume 0.02 reduction
    else
        if thk(3,z)>0.0001 && thk(3,z)<thk(4,z)
            thk(3,z)=(thk(1,z)+thk(4,z))/2;% assume mean thickness
        end
        thk(2,z)=(thk(1,z)+thk(3,z))/2;% assume mean thickness
    end
end
% the second kink point
if thk(3,z)<0.0001
    if thk(2,z)-thk(4,z)>0.011
        thk(3,z)=thk(2,z)-0.01;% assume 0.01 reduction
    else
        thk(3,z)=(thk(2,z)+thk(4,z))/2;% assume mean thickness
    end
end
%=====
% Approx. fuselage cross-section chord and fuse-wing chord
if hma(z)>0.0
    if loc(z)>0.5
        loc(z)=1-loc(z);% a high wing has been selected
    end
    xscd=(4*loc(z)*(1-loc(z))*(hma(z)^2)+ ...
        cref*thk(1,z)*(6*(1-2*loc(z))*hma(z)- ...
        9*cref*thk(1,z)))^0.5;% compute the x-sect chord at fuse-wing
    chwf=qxcdcop(xscd/2,smtrx,crot,tap,z);% actual chord at fuse-wing juncture
    chrf=cref*(1-(1-tref)*xscd/spn(z));% ref. wing-fuse chord
    %=====
    % estimate the thickness at wing root (a/c centre-line)
    troot=thk(1,z)*(1-(1-thk(2,z)/thk(1,z))*-xscd/(2*smtrx(1)-xscd));
else
    xscd=0.0;
    chwf=0.0;
    chrf=0.0;
    troot=thk(1,z);
end

%=====
% generate the wing thickness coefficient matrix
tmtrx=[thk(2,z)/troot thk(3,z)/thk(2,z) thk(4,z)/thk(3,z); ...
    troot thk(2,z) thk(3,z)];
%============================================================================
% Find the equivalent sweep angle of the wing
%
if lsw(1,z)<0.0001
    % compute the leading edge sweep angles for inboard-mid-outboard
    lswa=smtrx(1)*tan(kdrang*qsw(1,z))+0.25*crot*(1-tap(1,z));
    lswb=smtrx(2)*tan(kdrang*qsw(2,z))+0.25*crot*(tap(1,z)-tap(2,z));
    lswc=smtrx(3)*tan(kdrang*qsw(3,z))+0.25*crot*(tap(2,z)-tap(3,z));
    lsw(1,z)=atan(lswa/smtrx(1))/kdrang;
    lsw(2,z)=atan(lswb/smtrx(2))/kdrang;
    lsw(3,z)=atan(lswc/smtrx(3))/kdrang;
    % end
    % if qsw(1,z)<0.0001
else
    % compute the quarter chord sweep angles for inboard-mid-outboard
    qswa=smtrx(1)*tan(kdrang*lsw(1,z))+0.25*crot*(tap(1,z)-1);
    qswb=smtrx(2)*tan(kdrang*lsw(2,z))+0.25*crot*(tap(2,z)-tap(1,z));
    qswc=smtrx(3)*tan(kdrang*lsw(3,z))+0.25*crot*(tap(3,z)-tap(2,z));
    qsw(1,z)=atan(qswa/smtrx(1))/kdrang;
    qsw(2,z)=atan(qswb/smtrx(2))/kdrang;
    qsw(3,z)=atan(qswc/smtrx(3))/kdrang;
end
% locate the longitudinal wing tip leading edge
xtip=smtrx(1)*tan(kdrang*lsw(1,z))+smtrx(2)*tan(kdrang*lsw(2,z))+ ...
    smtrx(3)*tan(kdrang*lsw(3,z));
%============================================================================
% Produce data for various reference wing premises
%
%=====
if refc(z)<0.0001
    % the weighted segment area reference wing.
    % the net exposed wing area (less the fuse-wing juncture)
    netwgae=wingari-xscd/2*(crot+chwf)+wingarm+wingaro;
    % calculate the inboard wing MAC
    maci=2/3*crot*(1+tap(1,z)+tap(1,z)^2)/(1+tap(1,z));
    ybari=smtrx(1)/3*(2*tap(1,z)+1)/(1+tap(1,z));% y-bar for inboard
    % calculate the midboard wing MAC
    wingtpm=tap(2,z)/tap(1,z);
    macm=2/3*tap(1,z)*crot*(1+wingtpm+wingtpm^2)/(1+wingtpm);
    ybarm=smtrx(2)/3*(2*wingtpm+1)/(1+wingtpm)+smtrx(1);% y-bar for inboard
    % calculate the outboard wing MAC
    wingtpo=tap(3,z)/tap(2,z);
    maco=2/3*tap(2,z)*crot*(1+wingtpo+wingtpo^2)/(1+wingtpo);
    ybaro=smtrx(3)/3*(2*wingtpo+1)/(1+wingtpo)+smtrx(2)+smtrx(1);% y-bar for outboard
    % calculate the complete wing MAC
    wsarmac=(wingari*maci+wingarm*macm+wingaro*maco)/are(z);% complete wing MAC
    wsarybr=2*(wingari*ybari+wingarm*ybarm+wingaro*ybaro)/(are(z)*spn(z));
    % based on calculated MAC and y-bar, assume mean thickness
    wsarthb=qxtkcop(wsarybr*spn(z)/2,smtrx,tmtrx);
    % weighted root chord length
    wsacref=(wsarmac-crot*tap(3,z)*wsarybr)/(1-wsarybr);
    wsatref=crot*tap(3,z)/wsacref;% weighted taper ratio
    wsarref=spn(z)/2*wsacref*(1+wsatref);% weighted ref. wing area
    wsasref=(spn(z)^2)/wsarref;% weighted ref. aspect ratio
    % calculate the LE mac longitudinal locations for each wing segment
    xmaci=ybari*tan(kdrang*lsw(1,z));
    xmacm=smtrx(1)*tan(kdrang*lsw(1,z))+(ybarm-smtrx(1))*tan(kdrang*lsw(2,z));
    xmaco=smtrx(1)*tan(kdrang*lsw(1,z))+smtrx(2)*tan(kdrang*lsw(2,z))+ ...
        (ybaro-smtrx(2)-smtrx(1))*tan(kdrang*lsw(3,z));
    xmac=(wingari*xmaci+wingarm*xmacm+wingaro*xmaco)/are(z);
    % calculate the equivalent weighted leading edge sweep for ref. wing
    wsarlsw=atan(2*(xtip-xmac)/((1-wsarybr)*spn(z)))/kdrang;
    % ref. quarter chord sweep
    wsarqsw=atan(tan(wsarlsw*kdrang)-(1-wsatref)/(wsasref*(1+wsatref)))/kdrang;
    % ref. half chord sweep
    wsarhsw=atan(tan(wsarlsw*kdrang)-2*(1-wsatref)/(wsasref*(1+wsatref)))/kdrang;
    %=====
elseif refc(z)==1
    % the ESDU equiv. wing reference
    % the net exposed wing area (less the fuse maximum diameter)
    chmd=qxcdcop(hma(z)/2,smtrx,crot,tap,z);% actual chord at max fuse diameter
    netwgae=wingari-hma(z)/2*(crot+chmd)+wingarm+wingaro;% exposed wing area
    wsachwf=2*netwgae/(spn(z)-hma(z))-tap(3,z)*crot;% ref. chord at max fuse diam
    wsacref=(spn(z)*wsachwf-hma(z)*tap(3,z)*crot)/(spn(z)-hma(z));% ref. root chord
    wsatref=tap(3,z)*crot/wsacref;% ref. taper ratio
    wsarref=spn(z)/2*wsacref*(1+wsatref);% ref. wing area
    wsasref=(spn(z)^2)/wsarref;% ref. aspect ratio
    % levered leading edge sweep of mid & out board wing
    wsacinx=hma(z)*tan(lsw(1,z)*kdrang)/2+(tan(lsw(1,z)*kdrang)- ...
        tan(lsw(2,z)*kdrang))*(kln(1,z)*spn(z)-hma(z))*(1- ...
        kln(1,z))*spn(z)/(spn(z)-hma(z))/2+(tan(lsw(2,z)*kdrang)- ...
        tan(lsw(3,z)*kdrang))*(kln(2,z)*spn(z)-hma(z))*(1- ...
        kln(2,z))*spn(z)/(spn(z)-hma(z))/2;
    % ref. leading edge sweep
    wsarlsw=atan(2*(xtip-wsacinx)/(spn(z)-hma(z)))/kdrang;
    % ref. quarter chord sweep
    wsarqsw=atan(tan(wsarlsw*kdrang)-(1-wsatref)/(wsasref*(1+wsatref)))/kdrang;
    % ref. half chord sweep
    wsarhsw=atan(tan(wsarlsw*kdrang)-2*(1-wsatref)/(wsasref*(1+wsatref)))/kdrang;
    % calculate the equiv. wing MAC
    wsarmac=2/3*wsacref*(1+wsatref+wsatref^2)/(1+wsatref);
    wsarybr=(1+2*wsatref)/(1+wsatref)/3;% ref. wing non-dim MAC y bar
    wsarthb=qxtkcop(wsarybr*spn(z)/2,smtrx,tmtrx);% ref. wing mean thickness
    %=====
elseif refc(z)==2
    % the AIRBUS equiv. wing reference
    % the net exposed wing area (less the fuse maximum diameter)
    chmd=qxcdcop(hma(z)/2,smtrx,crot,tap,z);% actual chord at max fuse diameter
    netwgae=wingari-hma(z)/2*(crot+chmd)+wingarm+wingaro;% exposed wing area
    wsarref=netwgae+hma(z)*chmd;% ref. wing area
    wsacref=2*wsarref/spn(z)-tap(3,z)*crot;% ref. root chord
    wsatref=tap(3,z)*crot/wsacref;% ref. taper ratio
    wsasref=(spn(z)^2)/wsarref;% ref. aspect ratio
    wsarlsw=lsw(3,z);% ref. leading edge sweep
    % ref. quarter chord sweep
    wsarqsw=atan(tan(wsarlsw*kdrang)-(1-wsatref)/(wsasref*(1+wsatref)))/kdrang;
    % ref. half chord sweep
    wsarhsw=atan(tan(wsarlsw*kdrang)-2*(1-wsatref)/(wsasref*(1+wsatref)))/kdrang;
    wsarmac=2/3*wsacref*(1+wsatref+wsatref^2)/(1+wsatref);% calculate the equiv. wing MAC
    wsarybr=(1+2*wsatref)/(1+wsatref)/3;% ref. wing non-dim MAC y bar
    wsarthb=qxtkcop(wsarybr*spn(z)/2,smtrx,tmtrx);% ref. wing mean thickness
    %=====
elseif refc(z)==3
    % the BOEING WIMPRESS equiv. wing reference
    % involves account of a weighted area of the inboard trap. panel within main trapezoid no. 1
    chmd=qxcdcop(hma(z)/2,smtrx,crot,tap,z);% actual chord at max fuse diameter
    netwgae=wingari-hma(z)/2*(crot+chmd)+wingarm+wingaro;% exposed wing area
    % chord at centre-line based on colinear projection of trapezoidal panel 2 trailing edge
    bwcrotp=tap(1,z)*crot*(1-(1-tap(2,z)/tap(1,z))*-2*smtrx(1)/spn(z))+ ...
        smtrx(1)*(tan(lsw(1,z)*kdrang)-tan(lsw(2,z)*kdrang));
    % chord at wing-fuse based on colinear projection of trapezoidal panel 2 trailing edge
    bwchwfp=tap(1,z)*crot*(1-(1-tap(2,z)/tap(1,z))*(-2*smtrx(1)+hma(z))/spn(z))+ ...
        (smtrx(1)-hma(z)/2)*(tan(lsw(1,z)*kdrang)-tan(lsw(2,z)*kdrang));
    bwaiwfp=hma(z)/2*(crot-bwcrotp+chmd-bwchwfp);% area within trap. panel 1 i/b of wing-fuse juncture
    wsarref=wingari+bwaiwfp*((2*smtrx(2)-hma(z))/(spn(z)-hma(z))-1)+wingarm+wingaro;% ref. wing area
    wsacref=2*wsarref/spn(z)-tap(3,z)*crot;% ref. root chord
    wsatref=tap(3,z)*crot/wsacref;% ref. taper ratio
    wsasref=(spn(z)^2)/wsarref;% ref. aspect ratio
    wsarlsw=lsw(3,z);% ref. leading edge sweep
    % ref. quarter chord sweep
    wsarqsw=atan(tan(wsarlsw*kdrang)-(1-wsatref)/(wsasref*(1+wsatref)))/kdrang;
    % ref. half chord sweep
    wsarhsw=atan(tan(wsarlsw*kdrang)-2*(1-wsatref)/(wsasref*(1+wsatref)))/kdrang;
    wsarmac=2/3*wsacref*(1+wsatref+wsatref^2)/(1+wsatref);% calculate the equiv. wing MAC
    wsarybr=(1+2*wsatref)/(1+wsatref)/3;% ref. wing non-dim MAC y bar
    wsarthb=qxtkcop(wsarybr*spn(z)/2,smtrx,tmtrx);% ref. wing mean thickness
    %=====
elseif refc(z)==4
    % the MOST BASIC reference wing - simple trapezoid from projection
    %                                 of outboard wing to fuse CL
    wsatref=tref;% most basic taper
    wsacref=cref;% most basic root chord
    wsasref=asref;% most basic AR
    wsarref=spn(z)/2*wsacref*(1+wsatref);% most basic ref. wing area
    chmd=wsacref*(1-(1-wsatref)*hma(z)/spn(z));% actual chord at max fuse diameter
    netwgae=wsarref-hma(z)/2*(wsacref+chmd);% most basic exposed wing area
    % ref. leading edge sweep
    wsarlsw=lsw(3,z);
    % ref. quarter chord sweep
    wsarqsw=atan(tan(wsarlsw*kdrang)-(1-wsatref)/(wsasref*(1+wsatref)))/kdrang;
    % ref. half chord sweep
    wsarhsw=atan(tan(wsarlsw*kdrang)-2*(1-wsatref)/(wsasref*(1+wsatref)))/kdrang;
    % calculate the equiv. wing MAC
    wsarmac=2/3*wsacref*(1+wsatref+wsatref^2)/(1+wsatref);
    wsarybr=(1+2*wsatref)/(1+wsatref)/3;% ref. wing non-dim MAC y bar
    wsarthb=qxtkcop(wsarybr*spn(z)/2,smtrx,tmtrx);% ref. wing mean thickness
end

function comp3 = qxtkcop(spanin, spanmtrx, thkmtr)
% THKCOMP: this computes the wing thickness distribution for any span
%

% Identify the matrix coefficients for the computation
if spanin == spanmtrx(1) %| spanin==spanmtrx(1)+spanmtrx(2)
    correct = -1;% correction to heavyside result
else
    correct = 0;% no correction to heavyside required
end

tthkc = thkmtr(1, correct + 1 + qxheavy(spanin,spanmtrx(1),1)+ ...
    qxheavy(spanin,spanmtrx(1)+spanmtrx(2),1) );% thickness taper ratio
rthkc = thkmtr(2, correct + 1 + qxheavy(spanin,spanmtrx(1),1)+ ...
    qxheavy(spanin,spanmtrx(1)+spanmtrx(2),1) );% datum thickness
spanc = spanmtrx(correct + 1 + qxheavy(spanin,spanmtrx(1),1)+ ...
    qxheavy(spanin,spanmtrx(1)+spanmtrx(2),1));

% Additional corrections before interpolation is executed
if spanin > spanmtrx(1) && spanin < (spanmtrx(1) + spanmtrx(2))
    spanin = spanin - spanmtrx(1);% correct input span to local datum
elseif spanin >= (spanmtrx(1) + spanmtrx(2))
    spanin = spanin - spanmtrx(1) - spanmtrx(2);% correct input span to local datum
end

comp3 = rthkc*(1-(1-tthkc)*spanin/spanc);% computed thickness result


function comp1 = qxheavy(input, limit, type)
%HEAVY A heavyside step function to switch "on" or "off"

if type < 1
    comp1 = 0.5 + 0.5*tanh(110*(input-limit));
else
    comp1 = round(0.5+0.5*tanh(110*(input-limit)));
end


function chord = qxcdcop(spanin, spanmtrx, rcrd, tapmtrx, z)
% CHDCOMP This computes the local chord length at given span station
%         compute chord on actual planform geometry
%

if spanin == spanmtrx(1) || spanin == (spanmtrx(1)+spanmtrx(2))
    correct = -1;% correction to heavyside result
else
    correct = 0;% no correction to heavyside required
end

% identify the taper ratio for given wing segment
taprs = tapmtrx(correct+1+qxheavy(spanin,spanmtrx(1),1)+ ...
    qxheavy(spanin,spanmtrx(1)+spanmtrx(2),1),z);

% identify the local span length for given wing segment
spanc = spanmtrx(correct+1+qxheavy(spanin,spanmtrx(1),1)+ ...
    qxheavy(spanin,spanmtrx(1)+spanmtrx(2),1));

% additional corrections before interpolation is executed
if spanin > spanmtrx(1) && spanin < (spanmtrx(1)+spanmtrx(2))
    spanin = spanin-spanmtrx(1);% correct input span to local datum
    taprs = taprs/tapmtrx(1,z);% correct identified taper to local datum
    rcrd = rcrd*tapmtrx(1,z);% correct for local chord datum
elseif spanin >= (spanmtrx(1)+spanmtrx(2))
    spanin = spanin-spanmtrx(1)-spanmtrx(2);% correct input span to local datum
    taprs = taprs/tapmtrx(2,z);% correct identified taper to local datum
    rcrd = rcrd*tapmtrx(2,z);% correct for local chord datum
end

chord = rcrd*(1-(1-taprs)*spanin/spanc);% computed chord result
