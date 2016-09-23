%function xout=geo_xml(xin)
function geo_xml

global sinp_geo sout_geo

global qverson
% QCARD began in Apr00, version gamma Dec01
qverson(1,:)='QCARD-MMVI - Feb 06';
% Constants for various properties
global rhosls g kdrang asls kmfeet kkglbs
global knwlbf knaumm slspres kpressd
asls=340.3;		  % sonic velocity at sea level standard conditions (m/s)
rhosls=1.225;	  % density at sea level standard (kg/cu.m)
g=9.81;			  % acceleration due to gravity (m/s)
slspres=101.325;  % pressure at sea level standard conditions (kPa)
%kcmspd=0.5144;	  % conversion from KTAS to m/s.
kdrang=2*pi/360;  % conversion from deg. to rad.
kmfeet=0.3048;    % conversion from m to feet
kkglbs=2.2046;    % conversion from kg to lb
knwlbf=4.45;      % conversion from N to lbf
knaumm=1852.0;    % conversion from nm to m
kpressd=6.894757; % conversion from psi to kPa

% Fuselage definition parameters
global fusfvma fusfvmi fusfhma foreslp foredsw forefin forelgt ffsalgn 
global midfrat fuselgt fusevma fusevmi fusehma btallow aftfusw aftsfin 
global aftslgt
% Sponson definition parameters
global spsnxlc spsnzlc spsnlgt spsnxzs spsnwid 
% Wing definition parameters
global wingcfg wingplc wingapx wingare winggar wingspn 
global wingkln wingtap wingthk winginc wingqsw winglsw wingdih wletspn 
global wlettap wletlsw wletvos wletinc flapCrd ailrPos ailrChr ailrSpa 
global slatCrd slatSpa
% Fairing definition parameters
global fairfwd fairaft fairovh
% Wing 2 definition parameters
% PM 13.10: added Winglets for Wing2
global wi2gcfg wi2gplc wi2gapx wi2gare wi2ggar wi2gspn 
global wi2gkln wi2gtap wi2gthk wi2ginc wi2gqsw wi2glsw wi2gdih wle2spn 
global wle2tap wle2lsw wle2vos wle2inc fla2Crd ail2Pos ail2Chr ail2Spa
global  sla2Crd sla2Spa
% Fairing 2 definition parameters
global fa2rfwd fa2raft fa2rovh 
% Horizontal tail definition parameters
global htalare htalgar htalspn htalkln htaltap htalthk htalinc 
global htalqsw htallsw htaldih taillay htalver htalapx flHTCrd 
global HTHKMTX
% Vertical tail definition parameters
global vtalare vtalgar vtalspn vtalkln vtaltap vtalthk vtalinc 
global vtalqsw vtallsw vtaldih vtalver vtalapx flVTCrd  
global vtalblt vtalbsl vtaldlc vtaldsw vfinvlc vfinhlc vfinvcd vfinvsp 
global vfinvkl vfinvtp vfinvls vfinvdh 
global VTHKMTX
% Canard definition parameters
global canrare canrgar canrspn canrkln canrtap canrinc canrqsw 
global canrlsw canrdih canrver canrapx flCNCrd CTHKMTX canrthk
% Results from geometry computations
global WINGNET ZSERDWF WCHDROT WCHRDWF WCHDTIP 
global WINREFC REFWARE REFWTAP REFWGAR REFWLSW REFWQSW REFWHSQ REFWMAC 
global REFWYBR HCHDROT HTAWARE HTAWTAP 
global HTAWLSW HTAWQSW HTAWHSQ HTALMAC HTALYBR HTALTHB 
global VCHDROT VTAWARE VTAWTAP VTAWLSW VTAWQSW VTAWHSQ VTALMAC 
global VTALYBR VTALTHB WC2DROT
global CCHDROT CANWARE CANWTAP 
global CANWLSW CANWQSW CANWHSQ CANRMAC CANRYBR CANRTHB
% Engine definition parameters
global engenum engeloc insttyp engeylc engexlc engezlc engetoe 
global engepit maxistc bypasem nacetyp nacefin nacemdi nacefcl
global propdia PN
% Wetted area data
global wingiwt WINGWET wletiwt WLETWET htaliwt HTALWET vtaliwt VTALWET 
global fuseiwt FUSEWET pylniwt PYLNWET pwrpiwt PWRPWET ANCIWET TOTLWET 
global dfinrot dchrdbt dchrdb1 dchrdb2 dspnmtx dthkmtx FUSEGMX CANRWET canriwt
%Fuel tank details
global wingspf wingsau wingspa fuelcut fuelobd fuelusu fuelden fuelwic 
global MFUWWEI MFUVWEI fuelcen fuelcic MFCWWEI MFCVWEI fuelaux fuelafa 
global fuelaic MFAWWEI MFAVWEI

global cabnhei cabnwid cabnlgt cabnfwd baggapx baggtyp baggvol bagglgt

global xcgwing ycgwing zcgwing xcgfair ycgfair zcgfair xcgtaux ycgtaux 
global zcgtaux xcgwi2g ycgwi2g zcgwi2g xcgfa2r ycgfa2r zcgfa2r

% For visualisation
% global winxdat wi2xdat htaxdat vtaxdat canxdat FlTEDEF

%%
global CROSXCF CROSXFF
global WSPNMTX WS2NMTX WINWTAP WI2WGAR WI2WARE WI2WTAP WI2WQSW WI2WTHB REFWAPX WINWGAR WINWTHB wletwfc
global HTALMOA HSPNMTX VTALMOA VSPNMTX dorswet cabnvol XSECDWF XS2CDWF WTHKMTX WT2KMTX CANRMOA CTALMOA CSPNMTX
global NACENOX NACENOY NACENOZ PYLNCLW ENGECHD NACELGT WINGAPX WINGAPZ WI2GAPX WI2GAPZ HTAPEXX HTAPEXZ VTAPEXX VTAPEXZ CNAPEXX CNAPEXZ
global l_central XFAIR ZFAIR l_fore l_aft Xsponson
global aircraft fusedvt destype cabnpas

%% Inittialize some values
PYLNWET=0;
dorswet=0;
ycgwing=0;
ycgfair=0;
ycgtaux=0;

winxdat=0;
wi2xdat=0;
htaxdat=0;
vtaxdat=0;
canxdat=0;
PYLNCLW=0;

% The control surface deflection should be an user input
FlTEDEF=[0;0;0];

NACENOX=[0;0]; NACENOY=[0;0]; NACENOZ=[0;0];

aircraft=sinp_geo;
n=1;

destype(n)=aircraft.miscellaneous.Design_classification;

% make variables determining presence of components conformable
% with 'present' parameter
if ~aircraft.Wing1.present
   aircraft.Wing1.Span=0.0;
   aircraft.Wing1.area=0.0;
else
   if ~aircraft.Wing1.winglet.present
      aircraft.Wing1.winglet.Span=0.0;
   end
   if ~aircraft.Wing1.aileron.present
      aircraft.Wing1.aileron.Span=0.0;
   end
end
if ~aircraft.Wing2.present
   aircraft.Wing2.Span=0.0;
   aircraft.Wing2.area=0.0;
else
   if ~aircraft.Wing2.winglet.present
      aircraft.Wing2.winglet.Span=0.0;
   end
   if ~aircraft.Wing2.aileron.present
      aircraft.Wing2.aileron.Span=0.0;
   end
end
if ~aircraft.Horizontal_tail.present
   aircraft.Horizontal_tail.Span=0.0;
   aircraft.Horizontal_tail.area=0.0;
end
if ~aircraft.Vertical_tail.present
   aircraft.Vertical_tail.Span=0.0;
   aircraft.Vertical_tail.area=0.0;
end
if ~aircraft.Canard.present
   aircraft.Canard.Span=0.0;
   aircraft.Canard.area=0.0;
end

%% Fuselage inputs
fusfvma(1,n)=aircraft.Fuselage.Forefuse_X_sect_vertical_diameter;
fusfvmi(1,n)=aircraft.Fuselage.Forefuse_Xs_distortion_coefficient;
fusfhma(1,n)=aircraft.Fuselage.Forefuse_X_sect_horizontal_diameter;
foreslp(1,n)=aircraft.Fuselage.omega_nose;
foredsw(1,n)=aircraft.Fuselage.phi_nose;
forefin(1,n)=aircraft.Fuselage.epsilon_nose;
ffsalgn(1,n)=aircraft.Fuselage.shift_fore;
midfrat(1,n)=aircraft.Fuselage.fraction_fore;
fuselgt(1,n)=aircraft.Fuselage.Total_fuselage_length;
fusevma(1,n)=aircraft.Fuselage.Aftfuse_X_sect_vertical_diameter;
fusevmi(1,n)=aircraft.Fuselage.Aftfuse_Xs_distortion_coefficient;
fusehma(1,n)=aircraft.Fuselage.Aftfuse_X_sect_horizontal_diameter;
btallow(1,n)=aircraft.Fuselage.omega_tail;
aftfusw(1,n)=aircraft.Fuselage.phi_tail;
aftsfin(1,n)=aircraft.Fuselage.epsilon_tail;

%% Sponson inputs
spsnxlc(1,n)=0;
spsnzlc(1,n)=0;
spsnlgt(1,n)=0;
spsnxzs(1,n)=0;
spsnwid(1,n)=0;

%% Wing 1 inputs
wingcfg(1,n)=aircraft.Wing1.configuration;
wingplc(1,n)=aircraft.Wing1.placement;
wingapx(1,n)=aircraft.Wing1.apex_locale;
wingare(1,n)=aircraft.Wing1.area;
winggar(1,n)=aircraft.Wing1.AR;
wingspn(1,n)=aircraft.Wing1.Span;
wingkln(1,n)=aircraft.Wing1.spanwise_kink1;
wingkln(2,n)=aircraft.Wing1.spanwise_kink2;
wingtap(1,n)=aircraft.Wing1.taper_kink1;
wingtap(2,n)=aircraft.Wing1.taper_kink2;
wingtap(3,n)=aircraft.Wing1.taper_tip;
winginc(1,n)=aircraft.Wing1.root_incidence;
winginc(2,n)=aircraft.Wing1.kink1_incidence;
winginc(3,n)=aircraft.Wing1.kink2_incidence;
winginc(4,n)=aircraft.Wing1.tip_incidence;
wingqsw(1,n)=aircraft.Wing1.quarter_chord_sweep_inboard;
wingqsw(2,n)=aircraft.Wing1.quarter_chord_sweep_midboard;
wingqsw(3,n)=aircraft.Wing1.quarter_chord_sweep_outboard;
winglsw(1,n)=aircraft.Wing1.LE_sweep_inboard;
winglsw(2,n)=aircraft.Wing1.LE_sweep_midboard;
winglsw(3,n)=aircraft.Wing1.LE_sweep_outboard;
wingdih(1,n)=aircraft.Wing1.dihedral_inboard;
wingdih(2,n)=aircraft.Wing1.dihedral_midboard;
wingdih(3,n)=aircraft.Wing1.dihedral_outboard;
if aircraft.Wing1.flap.present
    flapCrd(1,n)=aircraft.Wing1.flap.root_chord;
    flapCrd(2,n)=aircraft.Wing1.flap.kink1_chord;
    flapCrd(3,n)=aircraft.Wing1.flap.kink2_chord;
else
    flapCrd(1,n)=0;
    flapCrd(2,n)=0;
    flapCrd(3,n)=0;
end
if aircraft.Wing1.aileron.present
    ailrPos(1,n)=aircraft.Wing1.aileron.position;
    ailrChr(1,n)=aircraft.Wing1.aileron.chord;
    ailrSpa(1,n)=aircraft.Wing1.aileron.Span;
else
    ailrPos(1,n)=0;
    ailrChr(1,n)=0;
    ailrSpa(1,n)=0;
end
if aircraft.Wing1.slat.present
    slatCrd(1,n)=aircraft.Wing1.slat.chord;
    slatSpa(1,n)=aircraft.Wing1.slat.root_position;
    slatSpa(2,n)=aircraft.Wing1.slat.tip_position;
else
    slatCrd(1,n)=0;
    slatSpa(1,n)=0;
    slatSpa(2,n)=0;
end
wingthk(1,n)=aircraft.Wing1.thickness_root;
wingthk(2,n)=aircraft.Wing1.thickness_kink1;
wingthk(3,n)=aircraft.Wing1.thickness_kink2;
wingthk(4,n)=aircraft.Wing1.thickness_tip;

if aircraft.Wing1.winglet.present
    % PM 13.10: winglet
    wletspn(1,n)=aircraft.Wing1.winglet.Span;
    wlettap(1,n)=aircraft.Wing1.winglet.taper_ratio;
    wletlsw(1,n)=aircraft.Wing1.winglet.LE_sweep;
    wletvos(1,n)=aircraft.Wing1.winglet.Cant_angle;
    wletinc(1,n)=aircraft.Wing1.winglet.root_incidence;
    wletinc(4,n)=aircraft.Wing1.winglet.tip_incidence;
else
    wletspn(1,n)=0;
    wlettap(1,n)=0;
    wletlsw(1,n)=0;
    wletvos(1,n)=0;
    wletinc(1,n)=0;
    wletinc(4,n)=0;
end

%% Fairing 1 input
if aircraft.Fairing1.present
    fairfwd(1,n)=aircraft.Fairing1.Forward_chord_fraction;
    fairaft(1,n)=aircraft.Fairing1.Aft_chord_fraction;
    fairovh(1,n)=aircraft.Fairing1.flushness;
else
    fairfwd(1,n)=0;
    fairaft(1,n)=0;
    fairovh(1,n)=0;
end
WINREFC(1,n)=1;

%% Wetted areas icrements
fuseiwt(1,n)=0;
wingiwt(1,n)=0;
wletiwt(1,n)=0; 
htaliwt(1,n)=0;
vtaliwt(1,n)=0;
canriwt(1,n)=0;
pylniwt(1,n)=0;
pwrpiwt(1,n)=0;
ANCIWET(1,n)=0;

%% Wing 2 inputs
if aircraft.Wing2.present
    % PM 13.10: if area = 0 => no 2nd wing
    wi2gare(1,n)=aircraft.Wing2.area;
    wi2gcfg(1,n)=aircraft.Wing2.configuration;
else
    wi2gare(1,n)=0;
    wi2gcfg(1,n)=0;
end
if (wi2gare(1,n) > 0.001)
    wi2gcfg(1,n)=aircraft.Wing2.configuration;
    wi2gplc(1,n)=aircraft.Wing2.placement;
    % PM 13.10: added changed to Apex_locale to apex_locale
    wi2gapx(1,n)=aircraft.Wing2.apex_locale;
    wi2gare(1,n)=aircraft.Wing2.area;
    wi2ggar(1,n)=aircraft.Wing2.AR;
    wi2gspn(1,n)=aircraft.Wing2.Span;
    wi2gkln(1,n)=aircraft.Wing2.spanwise_kink1;
    wi2gkln(2,n)=aircraft.Wing2.spanwise_kink2;
    wi2gtap(1,n)=aircraft.Wing2.taper_kink1;
    wi2gtap(2,n)=aircraft.Wing2.taper_kink2;
    wi2gtap(3,n)=aircraft.Wing2.taper_tip;
    wi2ginc(1,n)=aircraft.Wing2.root_incidence;
    wi2ginc(2,n)=aircraft.Wing2.kink1_incidence;
    wi2ginc(3,n)=aircraft.Wing2.kink2_incidence;
    wi2ginc(4,n)=aircraft.Wing2.tip_incidence;
    wi2gqsw(1,n)=aircraft.Wing2.quarter_chord_sweep_inboard;
    wi2gqsw(2,n)=aircraft.Wing2.quarter_chord_sweep_midboard;
    wi2gqsw(3,n)=aircraft.Wing2.quarter_chord_sweep_outboard;
    wi2glsw(1,n)=aircraft.Wing2.LE_sweep_inboard;
    wi2glsw(2,n)=aircraft.Wing2.LE_sweep_midboard;
    wi2glsw(3,n)=aircraft.Wing2.LE_sweep_outboard;
    wi2gdih(1,n)=aircraft.Wing2.dihedral_inboard;
    wi2gdih(2,n)=aircraft.Wing2.dihedral_midboard;
    wi2gdih(3,n)=aircraft.Wing2.dihedral_outboard;
    if aircraft.Wing2.flap.present
        fla2Crd(1,n)=aircraft.Wing2.flap.root_chord;
        fla2Crd(2,n)=aircraft.Wing2.flap.kink1_chord;
        fla2Crd(3,n)=aircraft.Wing2.flap.kink2_chord;
    else
        fla2Crd(1,n)=0;
        fla2Crd(2,n)=0;
        fla2Crd(3,n)=0;
    end
    if aircraft.Wing2.aileron.present
        ail2Pos(1,n)=aircraft.Wing2.aileron.position;
        ail2Chr(1,n)=aircraft.Wing2.aileron.chord;
        ail2Spa(1,n)=aircraft.Wing2.aileron.Span;
    else
        ail2Pos(1,n)=0;
        ail2Chr(1,n)=0;
        ail2Spa(1,n)=0;
    end
    if aircraft.Wing2.slat.present
        sla2Crd(1,n)=aircraft.Wing2.slat.chord;
        sla2Spa(1,n)=aircraft.Wing2.slat.root_position;
        sla2Spa(2,n)=aircraft.Wing2.slat.tip_position;
    else
        sla2Crd(1,n)=0;
        sla2Spa(1,n)=0;
        sla2Spa(2,n)=0;
    end
        % PM 13.10: added thicknesses
    wi2gthk(1,n)=aircraft.Wing2.thickness_root;
    wi2gthk(2,n)=aircraft.Wing2.thickness_kink1;
    wi2gthk(3,n)=aircraft.Wing2.thickness_kink2;
    wi2gthk(4,n)=aircraft.Wing2.thickness_tip;
    if aircraft.Wing2.winglet.present
        % PM 13.10: added Winglet
        wle2spn(1,n)=aircraft.Wing2.winglet.Span;
        wle2tap(1,n)=aircraft.Wing2.winglet.taper_ratio;
        wle2lsw(1,n)=aircraft.Wing2.winglet.LE_sweep;
        wle2vos(1,n)=aircraft.Wing2.winglet.Cant_angle;
        wle2inc(1,n)=aircraft.Wing2.winglet.root_incidence;
        wle2inc(4,n)=aircraft.Wing2.winglet.tip_incidence;
    else
        wle2spn(1,n)=0;
        wle2tap(1,n)=0;
        wle2lsw(1,n)=0;
        wle2vos(1,n)=0;
        wle2inc(1,n)=0;
        wle2inc(4,n)=0;
    end
end

%% Fairing 2 inputs
if aircraft.Fairing2.present
    fa2rfwd(1,n)=aircraft.Fairing2.Forward_chord_fraction;
    fa2raft(1,n)=aircraft.Fairing2.Aft_chord_fraction;
    fa2rovh(1,n)=aircraft.Fairing2.flushness;
else
    fa2rfwd(1,n)=0;
    fa2raft(1,n)=0;
    fa2rovh(1,n)=0;
end

%% Horizontal tail inputs
if aircraft.Horizontal_tail.present
    taillay(1,n)=aircraft.Horizontal_tail.empennage_layout;
    htalare(1,n)=aircraft.Horizontal_tail.area;
    htalgar(1,n)=aircraft.Horizontal_tail.AR;
    htalspn(1,n)=aircraft.Horizontal_tail.Span;
    htalkln(1,n)=aircraft.Horizontal_tail.spanwise_kink;
    htaltap(1,n)=aircraft.Horizontal_tail.taper_kink;
    htaltap(3,n)=aircraft.Horizontal_tail.taper_tip;
    htalinc(1,n)=aircraft.Horizontal_tail.root_incidence;
    htalinc(2,n)=aircraft.Horizontal_tail.kink_incidence;
    htalinc(4,n)=aircraft.Horizontal_tail.tip_incidence;
    htalqsw(1,n)=aircraft.Horizontal_tail.quarter_chord_sweep_inboard;
    htalqsw(3,n)=aircraft.Horizontal_tail.quarter_chord_sweep_outboard;
    htallsw(1,n)=aircraft.Horizontal_tail.LE_sweep_inboard;
    htallsw(3,n)=aircraft.Horizontal_tail.LE_sweep_outboard;
    htaldih(1,n)=aircraft.Horizontal_tail.dihedral_inboard;
    htaldih(3,n)=aircraft.Horizontal_tail.dihedral_outboard;
    htalver(1,n)=aircraft.Horizontal_tail.vertical_locale;
    htalapx(1,n)=aircraft.Horizontal_tail.apex_locale;
    htalthk(1,n)=aircraft.Horizontal_tail.thickness_root;
    htalthk(2,n)=aircraft.Horizontal_tail.thickness_kink;
    htalthk(4,n)=aircraft.Horizontal_tail.thickness_tip;

    if aircraft.Horizontal_tail.Elevator.present
        flHTCrd(1,n)=aircraft.Horizontal_tail.Elevator.chord;
        htalkln(2,n)=aircraft.Horizontal_tail.Elevator.Span;
    else
        flHTCrd(1,n)=0;
        htalkln(2,n)=0;
    end

    if (htalkln(1,n)==1)
        htalinc(3,n)=htalkln(2,n)*htalinc(4,n)+(1-htalkln(2,n))*htalinc(1,n);
    else
        htalinc(3,n)=(htalkln(2,n)-htalkln(1,n))/(1-htalkln(1,n))*htalinc(4,n)+(1-htalkln(2,n))/(1-htalkln(1,n))*htalinc(2,n); % Ghost kink incidence
    end
else
    taillay(1,n)=0;
    htalare(1,n)=0;
    htalgar(1,n)=0;
    htalspn(1,n)=0;
    htalkln(1,n)=0;
    htaltap(1,n)=0;
    htaltap(3,n)=0;
    htalinc(1,n)=0;
    htalinc(2,n)=0;
    htalinc(4,n)=0;
    htalqsw(1,n)=0;
    htalqsw(3,n)=0;
    htallsw(1,n)=0;
    htallsw(3,n)=0;
    htaldih(1,n)=0;
    htaldih(3,n)=0;
    htalver(1,n)=0;
    htalapx(1,n)=0;
    htalthk(1,n)=0;
    htalthk(2,n)=0;
    htalthk(4,n)=0;

    flHTCrd(1,n)=0;
    htalkln(2,n)=0;
end

%% Canard inputs
if aircraft.Canard.present
    canrare(1,n)=aircraft.Canard.area;
    canrgar(1,n)=aircraft.Canard.AR;
    canrspn(1,n)=aircraft.Canard.Span;
    canrkln(1,n)=aircraft.Canard.spanwise_kink;
    canrtap(1,n)=aircraft.Canard.taper_kink;
    canrtap(3,n)=aircraft.Canard.taper_tip;
    canrinc(1,n)=aircraft.Canard.root_incidence;
    canrinc(2,n)=aircraft.Canard.kink_incidence;
    canrinc(4,n)=aircraft.Canard.tip_incidence;
    canrqsw(1,n)=aircraft.Canard.quarter_chord_sweep_inboard;
    canrqsw(3,n)=aircraft.Canard.quarter_chord_sweep_outboard;
    canrlsw(1,n)=aircraft.Canard.LE_sweep_inboard;
    canrlsw(3,n)=aircraft.Canard.LE_sweep_outboard;
    canrdih(1,n)=aircraft.Canard.dihedral_inboard;
    canrdih(3,n)=aircraft.Canard.dihedral_outboard;
    canrver(1,n)=aircraft.Canard.vertical_locale;
    canrapx(1,n)=aircraft.Canard.apex_locale;
    canrthk(1,n)=aircraft.Canard.thickness_root;
    canrthk(2,n)=aircraft.Canard.thickness_kink;
    canrthk(4,n)=aircraft.Canard.thickness_tip;

    if aircraft.Canard.Elevator.present
        flCNCrd(1,n)=aircraft.Canard.Elevator.chord;
        canrkln(2,n)=aircraft.Canard.Elevator.Span;
    else
        flCNCrd(1,n)=0;
        canrkln(2,n)=0;
    end

    if (canrkln(1,n)==1)
        canrinc(3,n)=canrkln(2,n)*canrinc(4,n)+(1-canrkln(2,n))*canrinc(1,n);
    else
        canrinc(3,n)=(canrkln(2,n)-canrkln(1,n))/(1-canrkln(1,n))*canrinc(4,n)+(1-canrkln(2,n))/(1-canrkln(1,n))*canrinc(2,n); % Ghost kink incidence
    end
else
    canrare(1,n)=0;
    canrgar(1,n)=0;
    canrspn(1,n)=0;
    canrkln(1,n)=0;
    canrtap(1,n)=0;
    canrtap(3,n)=0;
    canrinc(1,n)=0;
    canrinc(2,n)=0;
    canrinc(4,n)=0;
    canrqsw(1,n)=0;
    canrqsw(3,n)=0;
    canrlsw(1,n)=0;
    canrlsw(3,n)=0;
    canrdih(1,n)=0;
    canrdih(3,n)=0;
    canrver(1,n)=0;
    canrapx(1,n)=0;
    canrthk(1,n)=0;
    canrthk(2,n)=0;
    canrthk(4,n)=0;

    flCNCrd(1,n)=0;
    canrkln(2,n)=0;
end

%% Vertical tail inputs
if aircraft.Vertical_tail.present
    vtalare(1,n)=aircraft.Vertical_tail.area;
    vtalgar(1,n)=aircraft.Vertical_tail.AR;
    vtalspn(1,n)=aircraft.Vertical_tail.Span;
    vtalkln(1,n)=aircraft.Vertical_tail.spanwise_kink;
    vtaltap(1,n)=aircraft.Vertical_tail.taper_kink;
    vtaltap(3,n)=aircraft.Vertical_tail.taper_tip;
    vtalqsw(1,n)=aircraft.Vertical_tail.quarter_chord_sweep_inboard;
    vtalqsw(3,n)=aircraft.Vertical_tail.quarter_chord_sweep_outboard;
    vtallsw(1,n)=aircraft.Vertical_tail.LE_sweep_inboard;
    vtallsw(3,n)=aircraft.Vertical_tail.LE_sweep_outboard;
    vtalver(1,n)=aircraft.Vertical_tail.vertical_locale;
    vtalapx(1,n)=aircraft.Vertical_tail.apex_locale;
    vtalthk(1,n)=aircraft.Vertical_tail.thickness_root;
    vtalthk(2,n)=aircraft.Vertical_tail.thickness_kink;
    vtalthk(4,n)=aircraft.Vertical_tail.thickness_tip;
    vtaldlc(1,n)=aircraft.Vertical_tail.Dorsal_location;
    vtaldsw(1,n)=aircraft.Vertical_tail.Dorsal_sweep;
    vtaldih(1,n)=aircraft.Vertical_tail.dihedral_inboard;
    vtaldih(3,n)=aircraft.Vertical_tail.dihedral_outboard;
    vtalinc(1,n)=aircraft.Vertical_tail.root_incidence;
    vtalinc(2,n)=aircraft.Vertical_tail.kink_incidence;
    vtalinc(4,n)=aircraft.Vertical_tail.tip_incidence;
    if aircraft.Vertical_tail.Rudder.present
        flVTCrd(1,n)=aircraft.Vertical_tail.Rudder.chord;
        vtalkln(2,n)=aircraft.Vertical_tail.Rudder.Span;
    else
        flVTCrd(1,n)=0;
        vtalkln(2,n)=0;
    end
    
    %Bullet fairing inputs
%    vtalblt(1,n)=aircraft.Vertical_tail.Bullet_more_vertical_tip_chord; 	
%    vtalbsl(1,n)=aircraft.Vertical_tail.Bullet_fairing_slenderness; 
    vtalblt(1,n)=0.0;
    vtalbsl(1,n)=0.0;

    if (vtalkln(1,n)==1)
        vtalinc(3,n)=vtalkln(2,n)*vtalinc(4,n)+(1-vtalkln(2,n))*vtalinc(1,n);
    else
        vtalinc(3,n)=(vtalkln(2,n)-vtalkln(1,n))/(1-vtalkln(1,n))*vtalinc(4,n)+(1-vtalkln(2,n))/(1-vtalkln(1,n))*vtalinc(2,n); % Ghost kink incidence
    end
else
    vtalare(1,n)=0;
    vtalgar(1,n)=0;
    vtalspn(1,n)=0;
    vtalkln(1,n)=0;
    vtaltap(1,n)=0;
    vtaltap(3,n)=0;
    vtalqsw(1,n)=0;
    vtalqsw(3,n)=0;
    vtallsw(1,n)=0;
    vtallsw(3,n)=0;
    vtalver(1,n)=0;
    vtalapx(1,n)=0;
    vtalthk(1,n)=0;
    vtalthk(2,n)=0;
    vtalthk(4,n)=0;
    vtaldlc(1,n)=0;
    vtaldsw(1,n)=0;
    vtaldih(1,n)=0;
    vtaldih(3,n)=0;
    vtalinc(1,n)=0;
    vtalinc(2,n)=0;
    vtalinc(4,n)=0;
    flVTCrd(1,n)=0;
    vtalkln(2,n)=0;
    vtalblt(1,n)=0; 	
    vtalbsl(1,n)=0;
end

%% Ventral fin inputs
if aircraft.Ventral_fin.present
    vfinvcd(1,n)=aircraft.Ventral_fin.chord_fraction_at_midfuse;
    vfinvsp(1,n)=aircraft.Ventral_fin.Span;
    vfinvkl(2,n)=aircraft.Ventral_fin.spanwise_kink;
    vfinvtp(2,n)=aircraft.Ventral_fin.taper_kink ; 
    vfinvtp(3,n)=aircraft.Ventral_fin.taper_tip; 
    vfinvls(2,n)=aircraft.Ventral_fin.LE_sweep_inboard; 
    vfinvls(3,n)=aircraft.Ventral_fin.LE_sweep_outboard; 
    vfinvdh(2,n)=aircraft.Ventral_fin.cant_inbord; 
    vfinvdh(3,n)=aircraft.Ventral_fin.cant_outboard; 
    vfinvlc(1,n)=aircraft.Ventral_fin.X_locale;
    vfinhlc(1,n)=aircraft.Ventral_fin.Z_locale;
else
    vfinvcd(1,n)=0;
    vfinvsp(1,n)=0;
    vfinvkl(2,n)=0;
    vfinvtp(2,n)=0; 
    vfinvtp(3,n)=0; 
    vfinvls(2,n)=0; 
    vfinvls(3,n)=0; 
    vfinvdh(2,n)=0; 
    vfinvdh(3,n)=0; 
    vfinvlc(1,n)=0;
    vfinhlc(1,n)=0;
end

PN=0;
%% Main Engines inputs
if aircraft.Engines1.present
    engenum(1,n)=aircraft.Engines1.Number_of_engines;
    engeloc(1,n)=aircraft.Engines1.Layout_and_config;
    insttyp(1,n)=aircraft.Engines1.Propulsion_type;
    engeylc(1,n)=aircraft.Engines1.Y_locale;
    engexlc(1,n)=aircraft.Engines1.X_locale;
    engezlc(1,n)=aircraft.Engines1.Z_locale;
    engetoe(1,n)=aircraft.Engines1.toe_in;
    engepit(1,n)=aircraft.Engines1.pitch;
    nacetyp(1,n)=aircraft.Engines1.Nacelle_body_type;
    nacefcl(1,n)=aircraft.Engines1.Fan_cowl_length_ratio;
    nacefin(1,n)=aircraft.Engines1.fineness_ratio;
    nacemdi(1,n)=aircraft.Engines1.d_max;
    propdia(1,n)=aircraft.Engines1.Propeller_diameter;
    maxistc(1,n)=aircraft.Engines1.Max_thrust;
    bypasem(1,n)=aircraft.Engines1.Bypass_ratio_to_emulate;
    PN=1;
else
    engenum(1,n)=0;
    engeloc(1,n)=0;
    insttyp(1,n)=0;
    engeylc(1,n)=0;
    engexlc(1,n)=0;
    engezlc(1,n)=0;
    engetoe(1,n)=0;
    engepit(1,n)=0;
    nacetyp(1,n)=0;
    nacefcl(1,n)=0;
    nacefin(1,n)=0;
    nacemdi(1,n)=0;
    propdia(1,n)=0;
    maxistc(1,n)=0;
    bypasem(1,n)=0;
end

%% Secondary engines inputs
if aircraft.Engines2.present
    engenum(2,n)=aircraft.Engines2.Number_of_engines;
    engeloc(2,n)=aircraft.Engines2.Layout_and_config;
    insttyp(2,n)=aircraft.Engines2.Propulsion_type;
    engeylc(2,n)=aircraft.Engines2.Y_locale;
    engexlc(2,n)=aircraft.Engines2.X_locale;
    engezlc(2,n)=aircraft.Engines2.Z_locale;
    engetoe(2,n)=aircraft.Engines2.toe_in;
    engepit(2,n)=aircraft.Engines2.pitch;
    nacetyp(2,n)=aircraft.Engines2.Nacelle_body_type;
    nacefin(2,n)=aircraft.Engines2.fineness_ratio;
    nacefcl(2,n)=aircraft.Engines2.Fan_cowl_length_ratio;
    nacemdi(2,n)=aircraft.Engines2.d_max;
    propdia(2,n)=aircraft.Engines2.Propeller_diameter;
    maxistc(2,n)=aircraft.Engines2.Max_thrust;
    bypasem(2,n)=aircraft.Engines2.Bypass_ratio_to_emulate;
    PN=2;
else
    engenum(2,n)=0;
    engeloc(2,n)=0;
    insttyp(2,n)=0;
    engeylc(2,n)=0;
    engexlc(2,n)=0;
    engezlc(2,n)=0;
    engetoe(2,n)=0;
    engepit(2,n)=0;
    nacetyp(2,n)=0;
    nacefin(2,n)=0;
    nacefcl(2,n)=0;
    nacemdi(2,n)=0;
    propdia(2,n)=0;
    maxistc(2,n)=0;
    bypasem(2,n)=0;
end

%% Fuel inputs
wingspf(1,n)=aircraft.fuel.Fore_wing_spar_loc_root;
wingspf(2,n)=aircraft.fuel.Fore_wing_spar_loc_kik1;
wingspf(3,n)=aircraft.fuel.Fore_wing_spar_loc_kin2;
wingspf(4,n)=aircraft.fuel.Fore_wing_spar_loc_tip;
%wingsau(1,n)=aircraft.fuel.Aux_wing_spar_loc_root;
wingspa(1,n)=aircraft.fuel.Aft_wing_spar_loc_root;
wingspa(2,n)=aircraft.fuel.Aft_wing_spar_loc_kin1;
wingspa(3,n)=aircraft.fuel.Aft_wing_spar_loc_kin2;
wingspa(4,n)=aircraft.fuel.Aft_wing_spar_loc_tip;
fuelcut(1,n)=aircraft.fuel.Wing_fuel_tank_cutout_opt;
fuelobd(1,n)=aircraft.fuel.Outboard_fuel_tank_span;
fuelusu(1,n)=aircraft.fuel.Unusable_fuel_option;
fuelden(1,n)=aircraft.fuel.Assumed_fuel_density ;
fuelwic(1,n)=aircraft.fuel.Incr_weight_for_wing_tanks;
fuelcen(1,n)=aircraft.fuel.Centre_tank_portion_used;
fuelcic(1,n)=aircraft.fuel.Increment_for_centre_tank;
fuelaux(1,n)=aircraft.fuel.Fore_fairing_tank_length;
fuelaux(2,n)=aircraft.fuel.Aft_fairing_tank_length;
fuelafa(1,n)=aircraft.fuel.Aft_fuse_bladder_length;
fuelaic(1,n)=aircraft.fuel.Increment_for_aux_tanks;

%% Baggage and cabin input

baggtyp(1,n)=aircraft.Baggage.installation_type;
baggvol(1,n)=aircraft.Baggage.gross_volume;

bagglgt(1,n)=aircraft.Baggage.Baggage_combined_length; 
baggapx(1,n)=aircraft.Baggage.Baggage_apex_per_fuselgt;
cabnlgt(1,n)=aircraft.cabin.Cabin_length_to_aft_cab; 
cabnhei(1,n)=aircraft.cabin.Cabin_max_internal_height;
cabnwid(1,n)=aircraft.cabin.Cabin_max_internal_width;
cabnfwd(1,n)=aircraft.cabin.Cabin_floor_width;
cabnvol(1,n)=aircraft.cabin.Cabin_volume;

%% Additional
cabnpas(n)=aircraft.cabin.Passenger_accomodation;
cabnvol(n)=aircraft.cabin.Cabin_volume;

%% Execute the geometry routines
%% Locations

qgeotry('geom1',n);
qgeotry('ccabn',n);
Abs_locations;
qwetted(n);
% removed by Martin Lahuta
%qfucalc(n);

%% Add the computed or recomputed data to the structure containing aircraft
%% data

%% Fuselage
aircraft.Fuselage.present = 1;

% aircraft.Fuselage.Nose_length=forelgt(1,n);
% aircraft.Fuselage.Tail_length=aftslgt(1,n);
%
% aircraft.Fuselage.a0_fore=CROSXFF(1,1);
% aircraft.Fuselage.a1_fore=CROSXFF(1,3);
% aircraft.Fuselage.b1_fore=CROSXFF(1,2);
%
% aircraft.Fuselage.a0_nose=CROSXFF(1,1);
% aircraft.Fuselage.a1_nose=CROSXFF(1,3);
% aircraft.Fuselage.b1_nose=CROSXFF(1,2);
%
% aircraft.Fuselage.a0_aft=CROSXCF(1,1);
% aircraft.Fuselage.a1_aft=CROSXCF(1,3);
% aircraft.Fuselage.b1_aft=CROSXCF(1,2);
%
% aircraft.Fuselage.a0_tail=CROSXCF(1,1);
% aircraft.Fuselage.a1_tail=CROSXCF(1,3);
% aircraft.Fuselage.b1_tail=CROSXCF(1,2);

aircraft.Fuselage.X_sect_chord_at_fuse_wing=XSECDWF(1,n);
% aircraft.Fuselage.fraction_fore=midfrat(1,n);
aircraft.Fuselage.geometry_matrix(1:6,1:4)=FUSEGMX;
aircraft.Fuselage.depth_fuse_vtail=fusedvt(1,n);

%% Wing 1
% aircraft.Wing1.configuration    =   wingcfg(1,n);
% aircraft.Wing1.present          =   1;
% aircarft.Wing1.winglet.present  =   0;
% aircraft.Wing1.aileron.present  =   0;
% aircraft.Wing1.slat.present     =   0;
% aircraft.Wing1.flap.present     =   0;
% aircarft.Fairing1.present       =   0;
% aircraft.Wing2.present          =   0;
% aircarft.Wing2.winglet.present  =   0;
% aircraft.Wing2.aileron.present  =   0;
% aircraft.Wing2.slat.present     =   0;
% aircraft.Wing2.flap.present     =   0;
% aircraft.Fairing2.present       =   0;
% aircarft.Horizontal_tail.present=   0;
% aircarft.Vertical_tail.present  =   0;
% aircraft.Engines1.present       =   0;
% aircraft.Engines2.present       =   0;
% aircraft.Ventral_fin.present    =   0;
% aircraft.Horizontal_tail.Elevator.present = 1;
% aircraft.Vertical_tail.Rudder.present = 1;

% if wletspn(1,n) > 0.001
%     aircraft.Wing1.winglet.present = 1;
% end
% if ailrChr(1,n) > 0.001
%     aircraft.Wing1.aileron.present = 1;
% end
% if slatCrd(1,n) > 0.001
%     aircraft.Wing1.slat.present = 1;
% end
% if flapCrd(1,n) > 0.001 || flapCrd(2,n) > 0.001 || flapCrd(3,n) > 0.001
%     aircraft.Wing1.flap.present = 1;
% end

% aircraft.Wing1.AR=winggar(1,n);
% aircraft.Wing1.Span=wingspn(1,n);
% aircraft.Wing1.area=wingare(1,n);
aircraft.Wing1.quarter_chord_sweep_inboard=wingqsw(1,n);
aircraft.Wing1.quarter_chord_sweep_midboard=wingqsw(2,n);
aircraft.Wing1.quarter_chord_sweep_outboard=wingqsw(3,n);
% aircraft.Wing1.LE_sweep_inboard=winglsw(1,n);
% aircraft.Wing1.LE_sweep_midboard=winglsw(2,n);
% aircraft.Wing1.LE_sweep_outboard=winglsw(3,n);
aircraft.Wing1.Span_matrix_partition_in_mid_outboard=WSPNMTX(1,1:3)';
aircraft.Wing1.Weighted_taper_ratio=WINWTAP(1,n);
aircraft.Wing1.longitudinal_location=WINGAPX;
aircraft.Wing1.vertical_location=WINGAPZ;
%aircraft.Wing1.Root_Airfoil=wingAirfoilSpline(1,:);
%aircraft.Wing1.Kink1_Airfoil=wingAirfoilSpline(2,:);
%aircraft.Wing1.Kink2_Airfoil=wingAirfoilSpline(3,:);
%aircraft.Wing1.Tip_Airfoil=wingAirfoilSpline(4,:);
aircraft.Wing1.thickness_coefs_matrix(1:2,1:3)=WTHKMTX;
% aircraft.Wing1.spanwise_kink1=wingkln(1,n);
% aircraft.Wing1.spanwise_kink2=wingkln(2,n);
% aircraft.Wing1.taper_kink1=wingtap(1,n);
% aircraft.Wing1.taper_kink2=wingtap(2,n);
% aircraft.Wing1.taper_tip=wingtap(3,n);
aircraft.Wing1.Original_estimated_fuse_wing_chrd=WCHRDWF(1,n);	
aircraft.Wing1.Original_planform_tip_chord=WCHDTIP(1,n);
aircraft.Wing1.Fuse_wing_junct_BL_locale=ZSERDWF(1,n);
aircraft.Wing1.Total_exposed_area=WINGNET(1,n);
% aircraft.Wing1.root_incidence=winginc(1,n);
% aircraft.Wing1.kink1_incidence=winginc(2,n);
% aircraft.Wing1.kink2_incidence=winginc(3,n);
% aircraft.Wing1.tip_incidence=winginc(4,n);
% aircraft.Wing1.winglet.root_incidence=wletinc(1,n);
% aircraft.Wing1.winglet.tip_incidence=wletinc(3,n);

%% Fairing 1
% if fairovh(n)>0.001
%     aircarft.Fairing1.present       =   1.0;
%     aircraft.Fairing1.l_central=l_central(1,n);
%     aircraft.Fairing1.width=width(1,n);
%     aircraft.Fairing1.thickness=thickness(1,n);
%     aircraft.Fairing1.longitudinal_location=XFAIR(1,n);
%     aircraft.Fairing1.vertical_location=ZFAIR(1,n);
%     aircraft.Fairing1.l_fore=l_fore(1,n);
%     aircraft.Fairing1.l_aft=l_aft(1,n);
% end

if spsnxlc(n)>0.001
    aircraft.sponson.longitudinal_location=Xsponson(1,n);
    aircraft.sponson.vertical_location=Xsponson(1,n);
end

%% Wing 2
%Give a default value to the area of wing 2 which will be needed in W&B
aircraft.Wing2.Weighted_reference_wing_area=0;
aircraft.Wing2.Weighted_reference_aspect_ratio=0;
aircraft.Wing2.Weighted_reference_wing_area=0;
aircraft.Wing2.Weighted_taper_ratio=0;
aircraft.Wing2.Reference_quarter_chord_sweep=0;
aircraft.Wing2.Wing_mean_thickness=0;
if aircraft.Wing2.present
%    aircraft.Wing2.present          =   1.0;
%    aircarft.Wing2.winglet.present  =   0.0;
%    aircraft.Wing2.aileron.present  =   0.0;
%    aircraft.Wing2.slat.present     =   0.0;
%    aircraft.Wing2.flap.present     =   0.0;
%    aircraft.Wing2.AR=wi2ggar(1,n);
%    aircraft.Wing2.Span=wi2gspn(1,n);
%    aircraft.Wing2.area=wi2gare(1,n);
    aircraft.Wing2.quarter_chord_sweep_inboard=wi2gqsw(1,n);
    aircraft.Wing2.quarter_chord_sweep_midboard=wi2gqsw(2,n);
    aircraft.Wing2.quarter_chord_sweep_outboard=wi2gqsw(3,n);
%    aircraft.Wing2.LE_sweep_inboard=wi2glsw(1,n);
%    aircraft.Wing2.LE_sweep_midboard=wi2glsw(2,n);
%    aircraft.Wing2.LE_sweep_outboard=wi2glsw(3,n);
    aircraft.Wing2.Weighted_reference_aspect_ratio=WI2WGAR(1,n);
    aircraft.Wing2.Weighted_reference_wing_area=WI2WARE(1,n);
    aircraft.Wing2.Weighted_taper_ratio=WI2WTAP(1,n); 
    aircraft.Wing2.Reference_quarter_chord_sweep=WI2WQSW(1,n);
    aircraft.Wing2.Wing_mean_thickness=WI2WTHB(1,n);
    aircraft.Wing2.longitudinal_location=WI2GAPX;
    aircraft.Wing2.vertical_location=WI2GAPZ;
    aircraft.Wing2.Span_matrix_partition_in_mid_outboard=WS2NMTX(1,1:3)';
    aircraft.Wing2.thickness_coefs_matrix(1:2,1:3)=WT2KMTX;
%    aircraft.Wing2.Root_Airfoil=wi2gAirfoilSpline(1,:);
%    aircraft.Wing2.Kink1_Airfoil=wi2gAirfoilSpline(2,:);
%    aircraft.Wing2.Kink2_Airfoil=wi2gAirfoilSpline(3,:);
%    aircraft.Wing2.Tip_Airfoil=wi2gAirfoilSpline(4,:);
%    aircraft.Wing2.spanwise_kink1=wi2gkln(1,n);
%    aircraft.Wing2.spanwise_kink2=wi2gkln(2,n);
%    aircraft.Wing2.taper_kink1=wi2gtap(1,n);
%    aircraft.Wing2.taper_kink2=wi2gtap(2,n);
%    aircraft.Wing2.taper_tip=wi2gtap(3,n);
%    aircraft.Wing2.root_incidence=wi2ginc(1,n);
%    aircraft.Wing2.kink1_incidence=wi2ginc(2,n);
%    aircraft.Wing2.kink2_incidence=wi2ginc(3,n);
%    aircraft.Wing2.tip_incidence=wi2ginc(4,n);
%    aircraft.Wing2.winglet.root_incidence=wle2inc(1,n);
%    aircraft.Wing2.winglet.tip_incidence=wle2inc(3,n);
    
%    if wle2spn(1,n) > 0.001
%        aircraft.Wing1.winglet.present = 1;
%    end
%    if ail2Chr(1,n) > 0.001
%        aircraft.Wing2.aileron.present = 1;
%    end
%    if sla2Crd(1,n) > 0.001
%        aircraft.Wing2.slat.present = 1;
%    end
%    if fla2Crd(1,n) > 0.001 || fla2Crd(2,n) > 0.001 || fla2Crd(3,n) > 0.001
%        aircraft.Wing2.flap.present = 1;
%    end
    
    aircraft.Reference_wing2.Orig_root_chrd_at_ac_CL=WC2DROT(1,n);
    
    aircraft.Fuselage.X_sect_chord_at_fuse_wing2(1,n)=XS2CDWF(1,n);
    
    % PM 14.10: added if fa2rovh(n)>0.001
%    if fa2rovh(n)>0.001
%        aircarft.Fairing1.present       =      1.0;
%        aircraft.Fairing2.l_central=l_central(2,n);
%        aircraft.Fairing2.width=width(2,n);
%        aircraft.Fairing2.thickness=thickness(2,n);
%        aircraft.Fairing2.longitudinal_location=XFAIR(2,n);
%        aircraft.Fairing2.vertical_location=ZFAIR(2,n);
%        aircraft.Fairing2.l_fore=l_fore(2,n);
%        aircraft.Fairing2.l_aft=l_aft(2,n);
%    end
end

%% Reference wing
aircraft.Reference_wing.taper_ratio=REFWTAP(1,n);
aircraft.Reference_wing.planform_AR=REFWGAR(1,n);
% aircraft.Refererence_wing.mean_thickness=REFWTHB(1,n);
aircraft.Reference_wing.Weighted_area=REFWARE(1,n);
aircraft.Reference_wing.LE_sweep=REFWLSW(1,n);
aircraft.Reference_wing.MAC=REFWMAC(1,n);
aircraft.Reference_wing.relative_apex=REFWAPX(1,n);
aircraft.Reference_wing.Orig_root_chrd_at_ac_CL=WCHDROT(1,n);	
aircraft.Reference_wing.Half_chord_sweep=REFWHSQ(1,n);
aircraft.Reference_wing.Quarter_chord_sweep=REFWQSW(1,n);
aircraft.Reference_wing.non_dim_MAC_y_bar=REFWYBR(1,n);
aircraft.Reference_wing.Weighted_aspect_ratio=WINWGAR(1,n);
aircraft.Reference_wing.mean_thickness=WINWTHB(1,n);
aircraft.Reference_wing.Wing_area_for_weight_balance_analysis=wletwfc(1,n);

%% Horizontal tail

% PM 13.10: htalare(n) => htalare(1,n)
if htalare(1,n)>0.0001
%    aircarft.Horizontal_tail.present=   1.0;
%    aircraft.Horizontal_tail.area=htalare(1,n);
%    aircraft.Horizontal_tail.AR=htalgar(1,n);
%    aircraft.Horizontal_tail.Span=htalspn(1,n);
%    aircraft.Horizontal_tail.spanwise_kink=htalkln(1,n);
%    aircraft.Horizontal_tail.taper_kink=htaltap(1,n);
%    aircraft.Horizontal_tail.taper_tip=htaltap(3,n);
    aircraft.Horizontal_tail.quarter_chord_sweep_inboard=htalqsw(1,n);
    aircraft.Horizontal_tail.quarter_chord_sweep_outboard=htalqsw(3,n);
%    aircraft.Horizontal_tail.LE_sweep_inboard=htallsw(1,n);
%    aircraft.Horizontal_tail.LE_sweep_outboard=htallsw(3,n);
    aircraft.Horizontal_tail.original_root_chord=HCHDROT(1,n);
    aircraft.Horizontal_tail.reference_wing_area =HTAWARE(1,n);
    aircraft.Horizontal_tail.reference_wing_taper_ratio=HTAWTAP(1,n);
    aircraft.Horizontal_tail.reference_wing_LE_chord_sweep=HTAWLSW(1,n);
    aircraft.Horizontal_tail.reference_wing_quarter_chord_sweep=HTAWQSW(1,n);
    aircraft.Horizontal_tail.reference_wing_half_chord_sweep=HTAWHSQ(1,n);
    aircraft.Horizontal_tail.reference_wing_MAC=HTALMAC(1,n);
    aircraft.Horizontal_tail.reference_wing_Y_bar_non_dim=HTALYBR(1,n);
    aircraft.Horizontal_tail.reference_wing_mean.thickness=HTALTHB(1,n);
    aircraft.Horizontal_tail.Moment_arm_to_HT=HTALMOA(1,n);
    aircraft.Horizontal_tail.Span_matrix_partition_in_mid_outboard=HSPNMTX(1,1:3)';
    aircraft.Horizontal_tail.longitudinal_location=HTAPEXX;
    aircraft.Horizontal_tail.vertical_location=HTAPEXZ;
    aircraft.Horizontal_tail.thickness_coefs_matrix(1:2,1:3)=HTHKMTX;
%    aircraft.Horizontal_tail.apex_locale=HTAPEXX/fuselgt(1,n);
%    aircraft.Horizontal_tail.vertical_locale=HTAPEXZ/fusevma(n);
    aircraft.Horizontal_tail.root_incidence=htalinc(1,n);
    aircraft.Horizontal_tail.kink_incidence=htalinc(2,n);
    aircraft.Horizontal_tail.tip_incidence=htalinc(4,n);
%    aircraft.Horizontal_tail.Root_Airfoil=HTAirfoilSpline(1,:);
%    aircraft.Horizontal_tail.Kink_Airfoil=HTAirfoilSpline(2,:);
%    aircraft.Horizontal_tail.Tip_Airfoil=HTAirfoilSpline(4,:);
    
    if flHTCrd(1,n) > 0.001
        aircraft.Horizontal_tail.Elevator.present = 1;
    end
end

%% Vertical tail

% PM 13.10: htalare(n) => htalare(1,n)
if vtalare(1,n)>0.0001
%    aircarft.aircarft.Vertical_tail.present   =   1.0;
%    aircraft.Vertical_tail.area=vtalare(1,n);
%    aircraft.Vertical_tail.AR=vtalgar(1,n);
%    aircraft.Vertical_tail.Span=vtalspn(1,n);
%    aircraft.Vertical_tail.spanwise_kink=vtalkln(1,n);  
%    aircraft.Vertical_tail.taper_kink=vtaltap(1,n);
%    aircraft.Vertical_tail.taper_tip=vtaltap(3,n);
    aircraft.Vertical_tail.quarter_chord_sweep_inboard=vtalqsw(1,n);
    aircraft.Vertical_tail.quarter_chord_sweep_outboard=vtalqsw(3,n);
%    aircraft.Vertical_tail.LE_sweep_inboard=vtallsw(1,n);
%    aircraft.Vertical_tail.LE_sweep_outboard=vtallsw(3,n);
    aircraft.Vertical_tail.original_root_chord=VCHDROT(1,n);
    aircraft.Vertical_tail.reference_wing_area=VTAWARE(1,n);
    aircraft.Vertical_tail.reference_wing_taper_ratio=VTAWTAP(1,n);
    aircraft.Vertical_tail.reference_wing_LE_sweep=VTAWLSW(1,n);
    aircraft.Vertical_tail.reference_wing_quarter_chord_sweep=VTAWQSW(1,n);
    aircraft.Vertical_tail.reference_wing_Half_chord_sweep=VTAWHSQ(1,n);
    aircraft.Vertical_tail.reference_wing_MAC=VTALMAC(1,n);
    aircraft.Vertical_tail.reference_Y_bar_non_dim=VTALYBR(1,n);
    aircraft.Vertical_tail.reference_wing_mean_thickness=VTALTHB(1,n);
    aircraft.Vertical_tail.Moment_arm_to_VT=VTALMOA(1,n);
    aircraft.Vertical_tail.Span_matrix_partition_in_mid_outboard=VSPNMTX(1,1:3)';
    aircraft.Vertical_tail.longitudinal_location=VTAPEXX;
    aircraft.Vertical_tail.vertical_location=VTAPEXZ;
    aircraft.Vertical_tail.thickness_coefs_matrix(1:2,1:3)=VTHKMTX;
%    aircraft.Vertical_tail.root_incidence=vtalinc(1,n);
%    aircraft.Vertical_tail.kink_incidence=vtalinc(2,n);
%    aircraft.Vertical_tail.tip_incidence=vtalinc(4,n);
%    aircraft.Vertical_tail.Root_Airfoil=VTAirfoilSpline(1,:);
%    aircraft.Vertical_tail.Kink_Airfoil=VTAirfoilSpline(2,:);
%    aircraft.Vertical_tail.Tip_Airfoil=VTAirfoilSpline(4,:);
    
%    if flVTCrd(1,n) > 0.001
%        aircraft.Vertical_tail.Rudder.present = 1;
%    end
end

% Canard
if aircraft.Canard.present
    aircraft.Canard.quarter_chord_sweep_inboard=canrqsw(1,n);
    aircraft.Canard.quarter_chord_sweep_outboard=canrqsw(3,n);
%    aircraft.Canard.LE_sweep_inboard=canrlsw(1,n);
%    aircraft.Canard.LE_sweep_outboard=canrlsw(3,n);
    aircraft.Canard.original_root_chord=CCHDROT(1,n);
    aircraft.Canard.reference_wing_area =CANWARE(1,n);
    aircraft.Canard.reference_wing_taper_ratio=CANWTAP(1,n);
    aircraft.Canard.reference_wing_LE_chord_sweep=CANWLSW(1,n);
    aircraft.Canard.reference_wing_quarter_chord_sweep=CANWQSW(1,n);
    aircraft.Canard.reference_wing_half_chord_sweep=CANWHSQ(1,n);
    aircraft.Canard.reference_wing_MAC=CANRMAC(1,n);
    aircraft.Canard.reference_wing_Y_bar_non_dim=CANRYBR(1,n);
    aircraft.Canard.reference_wing_mean.thickness=CANRTHB(1,n);
    aircraft.Canard.Moment_arm_to_CA=CANRMOA(1,n);
    aircraft.Canard.Span_matrix_partition_in_mid_outboard=CSPNMTX(1,1:3)';
    aircraft.Canard.thickness_coefs_matrix(1:2,1:3)=CTHKMTX;
%    aircraft.Canard.Root_Airfoil=CNAirfoilSpline(1,:);
%    aircraft.Canard.Kink_Airfoil=CNAirfoilSpline(2,:);
%    aircraft.Canard.Tip_Airfoil=CNAirfoilSpline(4,:);
    
%    if flCNCrd(1,n) > 0.001
%        aircraft.Canard.Elevator.present = 1;
%    end
end

%% Other components that might be present
%if vfinvsp(1,n) > 0.001
%    aircraft.Vertical_fin.present = 1;
%end
%if engenum(1,n) > 0.001
%    aircraft.Engines1.present = 1;
%end
%if engenum(2,n) > 0.001
%    aircraft.Engines2.present = 1;
%end

%% added items

if vtaldlc>0.001
  aircraft.fins.final_dorsal_root_chord=dfinrot(1,n);
  aircraft.fins.local_dorsal_chord_tip=dchrdbt(1,n);
  aircraft.fins.local_dorsal_chord1=dchrdb1(1,n);
  aircraft.fins.local_dorsal_chord2=dchrdb2(1,n);
  aircraft.fins.dorsal_span_matrix=dspnmtx(1:3);
  aircraft.fins.dorsal_thickness_matrix=dthkmtx;
end

%%Fuel

% commented out by Martin Lahuta
%aircraft.fuel.max_weight_wing=MFUWWEI(1,n);
%aircraft.fuel.max_vol_wing=MFUVWEI(1,n);
%aircraft.fuel.max_weight_cent_wing_box=MFCWWEI(1,n);
%aircraft.fuel.max_vol_cent_wing_box=MFCVWEI(1,n);
%aircraft.fuel.max_weight_aux=MFAWWEI(1,n);
%aircraft.fuel.max_vol_aux=MFAVWEI(1,n);
%aircraft.fuel.Fore_wing_spar_loc_root=wingspf(1,n);
%aircraft.fuel.Fore_wing_spar_loc_kik1=wingspf(2,n);
%aircraft.fuel.Fore_wing_spar_loc_kin2=wingspf(3,n);
%aircraft.fuel.Fore_wing_spar_loc_tip=wingspf(4,n);
%aircraft.fuel.Aux_wing_spar_loc_root=wingsau(1,n);
%aircraft.fuel.Aft_wing_spar_loc_root=wingspa(1,n);
%aircraft.fuel.Aft_wing_spar_loc_kin1=wingspa(2,n);
%aircraft.fuel.Aft_wing_spar_loc_kin2=wingspa(3,n);
%aircraft.fuel.Aft_wing_spar_loc_tip=wingspa(4,n);
%aircraft.fuel.Outboard_fuel_tank_span=fuelobd(1,n);
%aircraft.fuel.Unusable_fuel_option=fuelusu(1,n);
%aircraft.fuel.Assumed_fuel_density =fuelden(1,n);

%aircraft.weight_balance.Fuel.Fuel_in_wing_x_cg=xcgwing(1,n); 
%aircraft.weight_balance.Fuel.Fuel_in_wing_y_cg=ycgwing(1,n); 
%aircraft.weight_balance.Fuel.Fuel_in_wing_z_cg=zcgwing(1,n);
%aircraft.weight_balance.Fuel.Fuel_in_wing2_x_cg=xcgwi2g(1,n);
%aircraft.weight_balance.Fuel.Fuel_in_wing2_y_cg=ycgwi2g(1,n);
%aircraft.weight_balance.Fuel.Fuel_in_wing2_z_cg=zcgwi2g(1,n);
%aircraft.weight_balance.Fuel.Fuel_in_fairings_x_cg=xcgfair(1,n); 
%aircraft.weight_balance.Fuel.Fuel_in_fairings_y_cg=ycgfair(1,n); 
%aircraft.weight_balance.Fuel.Fuel_in_fairings_z_cg=zcgfair(1,n);
%aircraft.weight_balance.Fuel.Fuel_in_fairing2_x_cg=xcgfa2r(1,n); 
%aircraft.weight_balance.Fuel.Fuel_in_fairing2_y_cg=ycgfa2r(1,n); 
%aircraft.weight_balance.Fuel.Fuel_in_fairing2_z_cg=zcgfa2r(1,n); 
%aircraft.weight_balance.Fuel.Fuel_in_auxiliary_tanks_x_cg=xcgtaux(1,n); 
%aircraft.weight_balance.Fuel.Fuel_in_auxiliary_tanks_y_cg=ycgtaux(1,n); 
%aircraft.weight_balance.Fuel.Fuel_in_auxiliary_tanks_z_cg=zcgtaux(1,n);


%% Engines

for i=1:engenum(1,n)
    
    if (engeloc(1,n)==0 || engeloc(1,n)==3) % There are pylons
        
        eval(strcat('aircraft.Engines1.Pylon',num2str(i),'.longitudinal_location=PYLLOCX(1,n);'));  
        eval(strcat('aircraft.Engines1.Pylon',num2str(i),'.vertical_location=PYLLOCZ(1,n);'));
        eval(strcat('aircraft.Engines1.Pylon',num2str(i),'.lateral_location=-sign(cos(i*pi))*PYLLOCY(1,n);'));
        if engeloc(1,n)==0
            eval(strcat('aircraft.Engines1.Pylon',num2str(i),'.rotation=PYLROTX(1,n);'));
        else
            eval(strcat('aircraft.Engines1.Pylon',num2str(i),'.rotation=sign(cos(i*pi))*PYLROTX(1,n);'));
        end
        eval(strcat('aircraft.Engines1.Pylon',num2str(i),'.root_chord=PYLRCH(1,n);')); 
        eval(strcat('aircraft.Engines1.Pylon',num2str(i),'.taper_kink1=PYLTAP(1,1);')); 
        eval(strcat('aircraft.Engines1.Pylon',num2str(i),'.taper_kink2=PYLTAP(1,2);')); 
        eval(strcat('aircraft.Engines1.Pylon',num2str(i),'.taper_tip=PYLTAP(1,3);')); 
        eval(strcat('aircraft.Engines1.Pylon',num2str(i),'.LE_sweep_inboard=PYLLESW(1,1);')); 
        eval(strcat('aircraft.Engines1.Pylon',num2str(i),'.LE_sweep_midboard=PYLLESW(1,2);')); 
        eval(strcat('aircraft.Engines1.Pylon',num2str(i),'.LE_sweep_outboard=PYLLESW(1,3);')); 
        eval(strcat('aircraft.Engines1.Pylon',num2str(i),'.inboard_span=PYSPMTRX(1,1);')); 
        eval(strcat('aircraft.Engines1.Pylon',num2str(i),'.midboard_span=PYSPMTRX(1,2);')); 
        eval(strcat('aircraft.Engines1.Pylon',num2str(i),'.outboard_span=PYSPMTRX(1,3);')); 
        
    end
    
    eval(strcat('aircraft.Engines1.Nacelle',num2str(i),'.longitudinal_location=NACENOX(1,n);'));  
    eval(strcat('aircraft.Engines1.Nacelle',num2str(i),'.vertical_location=NACENOZ(1,n);'));
    eval(strcat('aircraft.Engines1.Nacelle',num2str(i),'.lateral_location=-sign(cos(i*pi))*NACENOY(1,n);')); % Engines with even  index are on the starboard, odd on the port side
    eval(strcat('aircraft.Engines1.Nacelle',num2str(i),'.d_max=nacemdi(1,n);'));
    eval(strcat('aircraft.Engines1.Nacelle',num2str(i),'.fineness_ratio=nacefin(1,n);'));
    eval(strcat('aircraft.Engines1.Nacelle',num2str(i),'.toe_in=engetoe(1,n);'));
    eval(strcat('aircraft.Engines1.Nacelle',num2str(i),'.pitch=engepit(1,n);'));
    
    aircraft.Engines1.Nacelle_length_array=NACELGT(1,n);
    
end


% If there are no engines, set to 0 the data that will be required by WB
if engenum(1,n)==0
    aircraft.Engines1.Nacelle_length_array=0;
    aircraft.Engines1.Nacelle1.longitudinal_location=0;
    aircraft.Engines1.Nacelle1.lateral_location=0;
    aircraft.Engines1.Nacelle1.vertical_location=0;
end

for i=engenum(1,n)+1:engenum(1,n)+engenum(2,n)   
    if (engeloc(2,n)==0 || engeloc(2,n)==3) % There are pylons
        eval(strcat('aircraft.Engines2.Pylon',num2str(i),'.longitudinal_location=PYLLOCX(2,n);'));  
        eval(strcat('aircraft.Engines2.Pylon',num2str(i),'.vertical_location=PYLLOCZ(2,n);'));
        eval(strcat('aircraft.Engines2.Pylon',num2str(i),'.lateral_location=-sign(cos(i*pi))*PYLLOCY(2,n);'));
        if engeloc(2,n)==0
            eval(strcat('aircraft.Engines2.Pylon',num2str(i),'.rotation=PYLROTX(2,n);'));
        else
            eval(strcat('aircraft.Engines2.Pylon',num2str(i),'.rotation=sign(cos(i*pi))*PYLROTX(2,n);'));
        end
        eval(strcat('aircraft.Engines2.Pylon',num2str(i),'.root_chord=PYLRCH(2,n);')); 
        eval(strcat('aircraft.Engines2.Pylon',num2str(i),'.taper_kink1=PYLTAP(2,1);')); 
        eval(strcat('aircraft.Engines2.Pylon',num2str(i),'.taper_kink2=PYLTAP(2,2);')); 
        eval(strcat('aircraft.Engines2.Pylon',num2str(i),'.taper_tip=PYLTAP(2,3);')); 
        eval(strcat('aircraft.Engines2.Pylon',num2str(i),'.LE_sweep_inboard=PYLLESW(2,1);')); 
        eval(strcat('aircraft.Engines2.Pylon',num2str(i),'.LE_sweep_midboard=PYLLESW(2,2);')); 
        eval(strcat('aircraft.Engines2.Pylon',num2str(i),'.LE_sweep_outboard=PYLLESW(2,3);')); 
        eval(strcat('aircraft.Engines2.Pylon',num2str(i),'.inboard_span=PYSPMTRX(2,1);')); 
        eval(strcat('aircraft.Engines2.Pylon',num2str(i),'.midboard_span=PYSPMTRX(2,2);')); 
        eval(strcat('aircraft.Engines2.Pylon',num2str(i),'.outboard_span=PYSPMTRX(2,3);'));       
    end
    
    eval(strcat('aircraft.Engines2.Nacelle',num2str(i),'.longitudinal_location=NACENOX(2,n);'));  
    eval(strcat('aircraft.Engines2.Nacelle',num2str(i),'.vertical_location=NACENOZ(2,n);'));
    eval(strcat('aircraft.Engines2.Nacelle',num2str(i),'.lateral_location=-sign(cos(i*pi))*NACENOY(2,n);'));
    eval(strcat('aircraft.Engines2.Nacelle',num2str(i),'.d_max=nacemdi(2,n);'));
    eval(strcat('aircraft.Engines2.Nacelle',num2str(i),'.fineness_ratio=nacefin(2,n);'));
    eval(strcat('aircraft.Engines2.Nacelle',num2str(i),'.toe_in=engetoe(2,n);'));
    eval(strcat('aircraft.Engines2.Nacelle',num2str(i),'.pitch=engepit(2,n);'));
    
    aircraft.Engines2.Nacelle_length_array=NACELGT(2,n);
    
end

% If there are no engines, set to 0 the data that will be required by WB
if engenum(2,n)==0
    aircraft.Engines2.Nacelle_length_array=0;
    aircraft.Engines2.Nacelle3.longitudinal_location=0;
    aircraft.Engines2.Nacelle3.lateral_location=0;
    aircraft.Engines2.Nacelle3.vertical_location=0;
end
    

%% computed wetted areas
aircraft.Wetted_areas.Total_wetted_area=TOTLWET(1,n);
aircraft.Wetted_areas.Fuselage_fairing=FUSEWET(1,n);
aircraft.Wetted_areas.Wings=WINGWET(1,n);
aircraft.Wetted_areas.Winglet=WLETWET(1,n);
aircraft.Wetted_areas.Vertical_tail=VTALWET(1,n);
aircraft.Wetted_areas.Dorsal_fin=dorswet(1,n);
aircraft.Wetted_areas.Horizontal_tail=HTALWET(1,n);
aircraft.Wetted_areas.Canard=CANRWET(1,n);
aircraft.Wetted_areas.Pylons=PYLNWET(1,n);
aircraft.Wetted_areas.Powerplant=PWRPWET(1,n);

%%

% aircraft.Engines_results.Wetted_area_pylons=PYLNWET(1,n);
% aircraft.Engines_results.Wetted_area_powerplant=PWRPWET(1,n);
aircraft.Engines_results.wing_chord_at_engine_location=ENGECHD(:,n);
% aircraft.Wing_results.Wetted_area=WINGWET(1,n);
% aircraft.Wing_results.Wetted_area_winglet=WLETWET(1,n);
% aircraft.total_wetted_area=TOTLWET(1,n);

%xout=xml_format(aircraft);
sout_geo=aircraft;
% xml_save('aircraft.xml',aircraft)

