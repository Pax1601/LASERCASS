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

function [] = exportCAERO1(fid, aircraft)
%--------------------------------------------------------------------------------------------------
% CAERO1 card
% 
% Inputs:       fid, indicating file to write in
%               
% 
% Called by:    
% 
% Calls:        BULKdataCAERO1.m
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------



%--------------------------------------------------------------------------------------------------
% Inputs to BULKdataCAERO1.m
% 
%--------------------------------------------------------------------------------------------------

ID       = 0;       % Identity number for each panel
DIH      = [];      % Dihedral angle [deg]
CP       = [];      % Identity for reference frame
NY       = [];      % Spanwise distribution in each panel [-]               SET TO 1 NOW!!!
NX       = [];      % Chordwise distribution in each panel [-]              SET TO 1 NOW!!!
FOIL1    = [];      % Airfoil name 
FOIL2    = [];      % Airfoil name 
MESHTYPE = [];      % Meshtype [-]
Xw       = [];      % X coordinate of one point (wing) [m]
Yw       = [];      % Y coordinate of one point [m]
Zw       = [];      % Z coordinate of one point [m]
Xv       = [];      % X coordinate of one point (vertical tail) [m]
Yv       = [];      % Y coordinate of one point [m]
Zv       = [];      % Z coordinate of one point [m]
Xh       = [];      % X coordinate of one point (horizontal tail) [m]
Yh       = [];      % Y coordinate of one point [m]
Zh       = [];      % Z coordinate of one point [m]
CR       = [];      % Mean Aerodynamic Chord [m]
SPAN     = [];      % Span of each panel [m]
TAP      = [];      % Taper ratio [-]
SW       = [];      % Sweep angle [deg]
TW1      = [];      % Twist angle [deg]
TW2      = [];      % Twist angle [deg]
FRACORD  = [];      % Chord fraction of control surface (if defined)
FNX      = [];      % Control surface chordwise distribution 
NAME     = [];      % Control surface name



%--------------------------------------------------------------------------------------------------
% Right Wing
% 
%--------------------------------------------------------------------------------------------------

Xw = aircraft.wing1.longitudinal_location;
Yw = 0;
Zw = aircraft.wing1.vertical_location;
CR = 2*aircraft.wing1.area/(aircraft.wing1.span*(1+aircraft.wing1.taper_tip));  % Root chord in symmetry plane


%--------------------------------
% Sector 1 between root and kink1

SPAN1 = aircraft.wing1.spanwise_kink1*(aircraft.wing1.span/2);

if SPAN1 ~= 0
    ID       = ID + 1;
    DIH      = aircraft.wing1.dihedral_inboard;
    CP       = 1;
    NY       = 1;
    NX       = 1;
    FOIL1    = aircraft.wing1.airfoil;
    FOIL2    = aircraft.wing1.airfoil;
    MESHTYPE = 1;
    X        = Xw(1,end);
    Y        = Yw(1,end);
    Z        = Zw(1,end);
    c2       = CR*aircraft.wing1.taper_kink1;
    C        = MAC(CR, c2);
    SPAN     = SPAN1;
    TAP      = aircraft.wing1.taper_kink1;
    SW       = aircraft.wing1.quarter_chord_sweep_inboard;
    TW1      = aircraft.wing1.root_incidence;
    TW2      = aircraft.wing1.kink1_incidence;
%     FRACORD  = 0;
%     FNX      = 0;
%     NAME     = 0;
    

    BULKdataCAERO1(fid, ID, DIH, CP, NY, NX, FOIL1, FOIL2, MESHTYPE, X, Y, Z, C, SPAN, TAP, SW, TW1, TW2);
    
    
    % Update the new point to define next panel
    [XYZ] = rot3D(DIH*(pi/180), TW1*(pi/180), SW*(pi/180), SPAN);
    Xw = [Xw, Xw(1,end)+XYZ(1)];
    Yw = [Yw, Yw(1,end)+XYZ(2)];
    Zw = [Zw, Zw(1,end)+XYZ(3)];
    
end


%---------------------------------
% Sector 2 between kink1 and kink2

SPAN2 = aircraft.wing1.spanwise_kink2*(aircraft.wing1.span/2)- SPAN1;

if SPAN2 ~= 0
    ID       = ID + 1;
    DIH      = aircraft.wing1.dihedral_midboard;
    CP       = 1;
    NY       = 1;
    NX       = 1;
    FOIL1    = aircraft.wing1.airfoil;
    FOIL2    = aircraft.wing1.airfoil;
    MESHTYPE = 1;
    X        = Xw(1,end);
    Y        = Yw(1,end);
    Z        = Zw(1,end);
    c2       = CR*aircraft.wing1.taper_kink2;
    C        = MAC(CR, c2);
    SPAN     = SPAN2;
    TAP      = aircraft.wing1.taper_kink2;
    SW       = aircraft.wing1.quarter_chord_sweep_midboard;
    TW1      = aircraft.wing1.kink1_incidence;
    TW2      = aircraft.wing1.kink2_incidence;
%     FRACORD  = 0;
%     FNX      = 0;
%     NAME     = 0;
    

    BULKdataCAERO1(fid, ID, DIH, CP, NY, NX, FOIL1, FOIL2, MESHTYPE, X, Y, Z, C, SPAN, TAP, SW, TW1, TW2);
    
    
    % Update the new point to define next panel
    [XYZ] = rot3D(DIH*(pi/180), TW1*(pi/180), SW*(pi/180), SPAN);
    Xw = [Xw, Xw(1,end)+XYZ(1)];
    Yw = [Yw, Yw(1,end)+XYZ(2)];
    Zw = [Zw, Zw(1,end)+XYZ(3)];
    
end


%-------------------------------
% Sector 2 between kink2 and tip

SPAN3 = (aircraft.wing1.span/2) - (SPAN1 + SPAN2);

if SPAN3 ~= 0
    ID       = ID + 1;
    DIH      = aircraft.wing1.dihedral_outboard;
    CP       = 1;
    NY       = 1;
    NX       = 1;
    FOIL1    = aircraft.wing1.airfoil;
    FOIL2    = aircraft.wing1.airfoil;
    MESHTYPE = 1;
    X        = Xw(1,end);
    Y        = Yw(1,end);
    Z        = Zw(1,end);
    c2       = CR*aircraft.wing1.taper_tip;
    C        = MAC(CR, c2);
    SPAN     = SPAN3;
    TAP      = aircraft.wing1.taper_tip;
    SW       = aircraft.wing1.quarter_chord_sweep_outboard;
    TW1      = aircraft.wing1.kink2_incidence;
    TW2      = aircraft.wing1.tip_incidence;
%     FRACORD  = 0;
%     FNX      = 0;
%     NAME     = 0;
    

    BULKdataCAERO1(fid, ID, DIH, CP, NY, NX, FOIL1, FOIL2, MESHTYPE, X, Y, Z, C, SPAN, TAP, SW, TW1, TW2);
    
    
    % Update the new point to define next panel
    [XYZ] = rot3D(DIH*(pi/180), TW1*(pi/180), SW*(pi/180), SPAN);
    Xw = [Xw, Xw(1,end)+XYZ(1)];
    Yw = [Yw, Yw(1,end)+XYZ(2)];
    Zw = [Zw, Zw(1,end)+XYZ(3)];
    
end



%--------------------------------------------------------------------------------------------------
% Vertical Tail
% 
%--------------------------------------------------------------------------------------------------

Xv = aircraft.Vertical_tail.longitudinal_location;
Yv = 0;
Zv = aircraft.Vertical_tail.vertical_location;
CR = 2*aircraft.Vertical_tail.area/(aircraft.Vertical_tail.span*(1+aircraft.Vertical_tail.taper_tip));


%--------------------------------
% Sector 1 between root and kink1

SPAN1 = aircraft.Vertical_tail.spanwise_kink*aircraft.Vertical_tail.span;

if SPAN1 ~= 0
    ID       = ID + 1;
    DIH      = 90;% SHOULD BE: aircraft.Vertical_tail.dihedral_inboard, BUT IS SET TO ZERO ACTUALLY
    CP       = 1;
    NY       = 1;
    NX       = 1;
    FOIL1    = aircraft.Vertical_tail.airfoil;
    FOIL2    = aircraft.Vertical_tail.airfoil;
    MESHTYPE = 1;
    X        = Xv(1,end);
    Y        = Yv(1,end);
    Z        = Zv(1,end);
    c2       = CR*aircraft.Vertical_tail.taper_kink;
    C        = MAC(CR, c2);
    SPAN     = SPAN1;
    TAP      = aircraft.Vertical_tail.taper_kink;
    SW       = aircraft.Vertical_tail.quarter_chord_sweep_inboard;
    TW1      = 0;
    TW2      = 0;
%     FRACORD  = 0;
%     FNX      = 0;
%     NAME     = 0;
    

    BULKdataCAERO1(fid, ID, DIH, CP, NY, NX, FOIL1, FOIL2, MESHTYPE, X, Y, Z, C, SPAN, TAP, SW, TW1, TW2);
    
    
    % Update the new point to define next panel
    [XYZ] = rot3D(0*(pi/180), TW1*(pi/180), SW*(pi/180), SPAN);
    Xv = [Xv, Xv(1,end)+XYZ(1)];
    Yv = [Yv, Yv(1,end)+XYZ(3)];
    Zv = [Zv, Zv(1,end)+XYZ(2)];
    
end


%-------------------------------
% Sector 2 between kink1 and tip

SPAN2 = aircraft.Vertical_tail.span - SPAN1;

if SPAN2 ~= 0
    ID       = ID + 1;
    DIH      = 90;% SHOULD BE: aircraft.Vertical_tail.dihedral_outboard, BUT IT IS SET TO ZERO
    CP       = 1;
    NY       = 1;
    NX       = 1;
    FOIL1    = aircraft.Vertical_tail.airfoil;
    FOIL2    = aircraft.Vertical_tail.airfoil;
    MESHTYPE = 1;
    X        = Xv(1,end);
    Y        = Yv(1,end);
    Z        = Zv(1,end);
    c2       = CR*aircraft.Vertical_tail.taper_tip;
    C        = MAC(CR, c2);
    SPAN     = SPAN2;
    TAP      = aircraft.Vertical_tail.taper_tip;
    SW       = aircraft.Vertical_tail.quarter_chord_sweep_outboard;
    TW1      = 0;
    TW2      = 0;
%     FRACORD  = 0;
%     FNX      = 0;
%     NAME     = 0;
    

    BULKdataCAERO1(fid, ID, DIH, CP, NY, NX, FOIL1, FOIL2, MESHTYPE, X, Y, Z, C, SPAN, TAP, SW, TW1, TW2);
    
    
    % Update the new point to define next panel
    [XYZ] = rot3D(0*(pi/180), TW1*(pi/180), SW*(pi/180), SPAN);
    Xv = [Xv, Xv(1,end)+XYZ(1)];
    Yv = [Yv, Yv(1,end)+XYZ(3)];
    Zv = [Zv, Zv(1,end)+XYZ(2)];
    
end



%--------------------------------------------------------------------------------------------------
% Horizontal Tail
% 
%--------------------------------------------------------------------------------------------------

Xh = aircraft.Horizontal_tail.longitudinal_location;
Yh = 0;
Zh = aircraft.Horizontal_tail.vertical_location;
CR = 2*aircraft.Horizontal_tail.area/(aircraft.Horizontal_tail.span*(1+aircraft.Horizontal_tail.taper_tip));


%--------------------------------
% Sector 1 between root and kink1

SPAN1 = aircraft.Horizontal_tail.spanwise_kink*(aircraft.Horizontal_tail.span/2);

if SPAN1 ~= 0
    ID       = ID + 1;
    DIH      = aircraft.Horizontal_tail.dihedral_inboard;
    CP       = 1;
    NY       = 1;
    NX       = 1;
    FOIL1    = aircraft.Horizontal_tail.airfoil;
    FOIL2    = aircraft.Horizontal_tail.airfoil;
    MESHTYPE = 1;
    X        = Xh(1,end);
    Y        = Yh(1,end);
    Z        = Zh(1,end);
    c2       = CR*aircraft.Horizontal_tail.taper_kink;
    C        = MAC(CR, c2);
    SPAN     = SPAN1;
    TAP      = aircraft.Horizontal_tail.taper_kink;
    SW       = aircraft.Horizontal_tail.quarter_chord_sweep_inboard;
    TW1      = aircraft.Horizontal_tail.root_incidence;
    TW2      = aircraft.Horizontal_tail.kink_incidence;
%     FRACORD  = 0;
%     FNX      = 0;
%     NAME     = 0;
    

    BULKdataCAERO1(fid, ID, DIH, CP, NY, NX, FOIL1, FOIL2, MESHTYPE, X, Y, Z, C, SPAN, TAP, SW, TW1, TW2);
    
    
    % Update the new point to define next panel
    [XYZ] = rot3D(DIH*(pi/180), TW1*(pi/180), SW*(pi/180), SPAN);
    Xh = [Xh, Xh(1,end)+XYZ(1)];
    Yh = [Yh, Yh(1,end)+XYZ(2)];
    Zh = [Zh, Zh(1,end)+XYZ(3)];
    
end


%-------------------------------
% Sector 2 between kink1 and tip

SPAN2 = (aircraft.Horizontal_tail.span/2) - SPAN1;

if SPAN2 ~= 0
    ID       = ID + 1;
    DIH      = aircraft.Horizontal_tail.dihedral_outboard;
    CP       = 1;
    NY       = 1;
    NX       = 1;
    FOIL1    = aircraft.Horizontal_tail.airfoil;
    FOIL2    = aircraft.Horizontal_tail.airfoil;
    MESHTYPE = 1;
    X        = Xh(1,end);
    Y        = Yh(1,end);
    Z        = Zh(1,end);
    c2       = CR*aircraft.Horizontal_tail.taper_tip;
    C        = MAC(CR, c2);
    SPAN     = SPAN2;
    TAP      = aircraft.Horizontal_tail.taper_tip;
    SW       = aircraft.Horizontal_tail.quarter_chord_sweep_outboard;
    TW1      = aircraft.Horizontal_tail.kink_incidence;
    TW2      = aircraft.Horizontal_tail.tip_incidence;
%     FRACORD  = 0;
%     FNX      = 0;
%     NAME     = 0;
    

    BULKdataCAERO1(fid, ID, DIH, CP, NY, NX, FOIL1, FOIL2, MESHTYPE, X, Y, Z, C, SPAN, TAP, SW, TW1, TW2);
    
    
    % Update the new point to define next panel
    [XYZ] = rot3D(DIH*(pi/180), TW1*(pi/180), SW*(pi/180), SPAN);
    Xh = [Xh, Xh(1,end)+XYZ(1)];
    Yh = [Yh, Yh(1,end)+XYZ(2)];
    Zh = [Zh, Zh(1,end)+XYZ(3)];
    
end


