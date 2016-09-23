function qwetted(n)
% Constants for various properties
global rhosls kcmspd g kdrang kcmspd asls
%=====
% Input parameters from QVIEWAC
global PYLNCLW pylnwfl
%=====
% Input parameters from QGEOTRY
global WCHDROT XSECDWF WTHKROT WCHRDWF WINGTHB ENGEDIA WSPNMTX WTHKMTX
global XS2CDWF WT2KROT WC2RDWF WS2NMTX WT2KMTX WC2DROT
global WI2WTHB WI2WMAC
global CROSXCF CIRCUMX
global HCHDROT HSPNMTX HTALTHB HTHKMTX
global VTHKMTX VCHDROT VSPNMTX VTALTHB
global CCHDROT CSPNMTX CANRTHB CTHKMTX
global NACELGT nacbase nacdase PN
%=====
% Output parameters from QWETTED
global THK1CUT CTA1CUT THK2CUT CTA2CUT THK3CUT CTA3CUT THK4CUT CTA4CUT
global BTA1CUT BTA2CUT BTA3CUT BTA4CUT wletwfc
global FUSEGMX
global WI2GWET
global drshint vchrdb0 dfinrot vchrdb1 dchrdb1 vchrdb2 dchrdb2 vchrdbt
global dchrdbt dthkmtx dspnmtx dorswet
global NAWBASE NACEWET PWRPWET TOTLWET FUSEFIN
%=====
% Scripts for executing the global parameter declarations
% Fuselage definition parameters
global fusfvma fusfvmi fusfhma foreslp foredsw forefin forelgt ffsalgn midfrat fuselgt fusevma fusevmi fusehma btallow aftfusw aftsfin aftslgt
% Sponson definition parameters
global spsnxlc spsnzlc spsnlgt spsnxzs spsnwid
% Wing definition parameters
global wingcfg wingprl winglam wingplc wingapx wingare winggar wingspn wingkln wingtap wingthk winginc wingqsw winglsw wingdih wletspn wlettap wletlsw wletvos wletinc flapCrd ailrPos ailrChr ailrSpa slatCrd slatSpa
% Fairing definition parameters
global fairfwd fairaft fairovh
% Wing 2 definition parameters
global wi2gcfg wi2gprl wi2glam wi2gplc wi2gapx wi2gare wi2ggar wi2gspn wi2gkln wi2gtap wi2gthk wi2ginc wi2gqsw wi2glsw wi2gdih fla2Crd fla2Crd ail2Pos ail2Chr ail2Spa sla2Crd sla2Spa
% Fairing 2 definition parameters
global fa2rfwd fa2raft fa2rovh
% Horizontal tail definition parameters
global htalare htalgar htalspn htalkln htaltap htallam htalthk htalinc htalqsw htallsw htaldih taillay htalver htalapx htalvol flHTCrd htalkln
% Canard definition parameters
global canrare canrgar canrspn canrkln canrtap canrthk canrinc canrqsw canrlsw canrdih canrver canrapx canrvol flCNCrd canrkln
% Vertical tail definition parameters
global vtalare vtalgar vtalspn vtalkln vtaltap vtallam vtalthk vtalinc vtalqsw vtallsw vtaldih vtalver vtalapx vtalvol flVTCrd vtalkln vtalblt vtalbsl vtaldlc vtaldsw vfinvlc vfinhlc vfinvcd vfinvsp vfinvkl vfinvtp vfinvls vfinvdh vfinvhg
% Results from geometry computations
global wingare winggar wingspn WINGNET ZSERDWF WCHDROT WCHRDWF WCHDTIP WINREFC REFWARE REFWTAP REFWGAR REFWLSW REFWQSW REFWHSQ REFWMAC REFWYBR REFWTHB htalare htalgar htalspn HCHDROT HTAWARE HTAWTAP HTAWLSW HTAWQSW HTAWHSQ HTALMAC HTALYBR HTALTHB vtalare vtalgar vtalspn VCHDROT VTAWARE VTAWTAP VTAWLSW VTAWQSW VTAWHSQ VTALMAC VTALYBR VTALTHB
global canrare canrgar canrspn CCHDROT CANWARE CANWTAP CANWLSW CANWQSW CANWHSQ CANRMAC CANRYBR CANRTHB
% Engine definition parameters
global fixdthw engenum engeloc insttyp engeylc engexlc engezlc engetoe engepit thrtrev maxistc bypasem thrtred nacetyp nacefin nacemdi propdia
% Wetted area data
global wingiwt WINGWET wletiwt WLETWET htaliwt HTALWET canriwt CANRWET vtaliwt VTALWET fuseiwt FUSEWET pylniwt PYLNWET pwrpiwt PWRPWET ANCIWET TOTLWET
%Fuel tank details
global wingspf wingsau wingspa fuelcut fuelobd fuelusu fuelden fuelwic MFUWWEI MFUVWEI fuelcen fuelcic MFCWWEI MFCVWEI fuelaux fuelafa fuelaic MFAWWEI MFAVWEI

global cabnhei cabnwid cabnlgt cabnfwd baggapx baggtyp baggvol bagglgt PYLNCLW
%============================================================================
% eval(['global' blanks(1) GPARAMT]);% declare the global parameters
% Estimate the wing number one wetted area.
% Based on methods using Isikveren but with Raymer underlying premise.
%=====
onwgcta=0.0;% aft fuse mounted, S or straight duct engines (primary)
%=================================
if engeloc(1,n)<3.0
    % calculate the lateral wing distance to inboard cut-out
    BTA1CUT=0.5*(engeylc(1,n)*wingspn(n)-ENGEDIA(1,n));
    THK1CUT=qxtkcop(BTA1CUT,WSPNMTX,WTHKMTX);% thickness at cut 1
    CTA1CUT=qxcdcop(BTA1CUT,WSPNMTX,WCHDROT(n),wingtap,n);
    % calculate the lateral wing distance to outboard cut-out
    BTA2CUT=0.5*(engeylc(1,n)*wingspn(n)+ENGEDIA(1,n));
    THK2CUT=qxtkcop(BTA2CUT,WSPNMTX,WTHKMTX);% thickness at cut 2
    CTA2CUT=qxcdcop(BTA2CUT,WSPNMTX,WCHDROT(n),wingtap,n);
    % estimate the amount of cut-out due to on-wing installation
    onwgct1=wetcomp(CTA1CUT,CTA2CUT/CTA1CUT,ENGEDIA(1,n), ...
        THK1CUT,THK2CUT/THK1CUT);
    if PN>1
        % calculate the lateral wing distance to inboard cut-out
        BTA3CUT=0.5*(engeylc(2,n)*wingspn(n)-ENGEDIA(2,n));
        THK3CUT=qxtkcop(BTA3CUT,WSPNMTX,WTHKMTX);% thickness at cut 3
        CTA3CUT=qxcdcop(BTA3CUT,WSPNMTX,WCHDROT(n),wingtap,n);
        % calculate the lateral wing distance to outboard cut-out
        BTA4CUT=0.5*(engeylc(2,n)*wingspn(n)+ENGEDIA(2,n));
        THK4CUT=qxtkcop(BTA4CUT,WSPNMTX,WTHKMTX);% thickness at cut 4
        CTA4CUT=qxcdcop(BTA4CUT,WSPNMTX,WCHDROT(n),wingtap,n);
        % estimate the amount of cut-out due to on-wing installation
        onwgct2=wetcomp(CTA3CUT,CTA4CUT/CTA3CUT,ENGEDIA(2,n), ...
            THK3CUT,THK4CUT/THK3CUT);
    else
        onwgct2=0.0;% no decrement due to secondary powerplant
    end
    %=================================
    onwgcta=onwgct1+onwgct2;% tally the total wetted decrement
end
%=====
% predict the inboard wing wetted area (less the fuse barrel)

wingwti=wetcomp(WCHRDWF(n),WCHDROT(n)*wingtap(1,n)/WCHRDWF(n),...
    WSPNMTX(1)-XSECDWF(n)/2,wingthk(1,n),WTHKMTX(2,2)/WTHKMTX(2,1));
% predict the midboard wing wetted area
wingwtm=wetcomp(WCHDROT(n)*wingtap(1,n),wingtap(2,n)/wingtap(1,n),...
    WSPNMTX(2),WTHKMTX(2,2),WTHKMTX(1,2));
% predict the outboard wing wetted area
wingwto=wetcomp(WCHDROT(n)*wingtap(2,n),wingtap(3,n)/wingtap(2,n),...
    WSPNMTX(3),WTHKMTX(2,3),WTHKMTX(1,3));

WINGWET(n)=wingwti+wingwtm+wingwto-onwgcta+wingiwt(n);% final wing wetted
wletwfc=WINGWET(n);% store the wing area for weights analysis
%============================================================================
% Estimate the winglets wetted area.
%=====
if wletspn(n)>0.0001
    WLETWET(n)=wetcomp(WCHDROT(n)*wingtap(3,n),wlettap(n), ...
        WCHDROT(n)*wingtap(3,n)*wletspn(n),wingthk(4,n),1);
else
    WLETWET(n)=0;% no winglets to add
end
WLETWET(n)=WLETWET(n)+wletiwt(n);% tally the winglet wetted with increment
%============================================================================
% Estimate the wing number two wetted area.
% Based on methods using Isikveren but with Raymer underlying premise.
%=====
WI2GWET(n)=0.0;% default of no wetted area for wing number two
on2gcta=0.0;% tally the total wetted decrement
if wi2gare(n)>0.0001
    % predict the inboard wing wetted area (less the fuse barrel)
    wi2gwti=wetcomp(WC2RDWF(n),WC2DROT(n)*wi2gtap(1,n)/WC2RDWF(n),...
        WS2NMTX(1)-XS2CDWF(n)/2,wi2gthk(1,n),WT2KMTX(2,2)/WT2KMTX(2,1));
    % predict the midboard wing wetted area
    wi2gwtm=wetcomp(WC2DROT(n)*wi2gtap(1,n),wi2gtap(2,n)/wi2gtap(1,n),...
        WS2NMTX(2),WT2KMTX(2,2),WT2KMTX(1,2));
    % predict the outboard wing wetted area
    wi2gwto=wetcomp(WC2DROT(n)*wi2gtap(2,n),wi2gtap(3,n)/wi2gtap(2,n),...
        WS2NMTX(3),WT2KMTX(2,3),WT2KMTX(1,3));
    WI2GWET(n)=wi2gwti+wi2gwtm+wi2gwto-on2gcta;% final wing No.2 wetted
    WINGWET(n)=WINGWET(n)+WI2GWET(n);% tally the total wetted area
end
%============================================================================
% Estimate the fuselage wetted area.
%=====
% define the constituent fuselage section geometries
% investigate the upper fuselage forebody
[acoef,bcoef,abarc,bdlgt]=gocomp(foreslp(n),foredsw(n),forefin(n),...
    fusfvma(n),1);
FUSEGMX=[acoef bcoef abarc bdlgt];
% investigate the lower fuselage forebody
FUSEGMX(2,:)=[acoef bcoef abarc bdlgt];
% investigate the lower fuselage aftbody
[acoef,bcoef,abarc,bdlgt]=gocomp(btallow(n),aftfusw(n),aftsfin(n),...
    fusevma(n),1);
FUSEGMX(5,:)=[acoef bcoef abarc bdlgt];
% investigate the upper fuselage aftbody
FUSEGMX(4,:)=[acoef bcoef abarc bdlgt];
% cylindrical fuselage
FUSEGMX(3,:)=[fusevma(n) 0 (fuselgt(n)-FUSEGMX(1,4)-FUSEGMX(4,4))*(1- ...
    midfrat(n)) (fuselgt(n)-FUSEGMX(1,4)-FUSEGMX(4,4))*(1- ...
    midfrat(n))];
% tapered fuselage frustum
FUSEGMX(6,:)=[fusevma(n) 0 ((((fuselgt(n)-FUSEGMX(1,4)- ...
    FUSEGMX(4,4))*midfrat(n))^2)+ ...
    (fusfvma(n)-fusevma(n))^2)^0.5 ((((fuselgt(n)-FUSEGMX(1,4)- ...
    FUSEGMX(4,4))*midfrat(n))^2)+(fusfvma(n)-fusevma(n))^2)^0.5];
%=====
% sum the constituent wetted areas
cfuswet=0.0;
for i=1:5
    % correction to wetted area due to cross-section distortion is included
    cfuswet=cfuswet+CIRCUMX(i)*pi*FUSEGMX(i,1)*FUSEGMX(i,3);
end
% additional increment due to presence of fwd fuse tapering
cfusewet=cfuswet+(CIRCUMX(3)+CIRCUMX(6))*pi*FUSEGMX(6,1)*FUSEGMX(6,3)/2;
%=====
% compute the fuselage fineness ratio, i.e. based on vertical cross-section
% diameter
FUSEFIN(n)=fuselgt(n)/fusevma(n);% fuselage fineness
%=====
% Estimate the fuselage-wing fairing wetted area. Method based on
% Isikveren. Use empirical correlation between fairing compared to total
% fuselage length.
%=====
wfrlgth=(fairfwd(n)+fairaft(n))/100*WCHDROT(n);% estimate the fairing length
% final fairing wetted area for primary wing
WFAIWET(n)=0.75*wfrlgth/fuselgt(n)*cfuswet;
if wi2gare(n)>0.0001
    wf2lgth=(fa2rfwd(n)+fa2raft(n))/100*WC2DROT(n);% estimate the fairing length
    % new wetted area estimate with second wing
    WFAIWET(n)=WFAIWET(n)+ ...
        0.75*wf2lgth/fuselgt(n)*cfuswet;
end
FUSEWET(n)=cfuswet+fuseiwt(n)+0.2*WFAIWET(n);% final fuselage wetted area
%============================================================================
% Estimate the horizontal tail wetted area.
%=====
% locate where the v-tail is placed on the aft fuse geometry
% Add abs() since with tailbooms configuration VT apex is greater than
% fuselage length (i.e. greater than 1.)
vtallca = fuselgt(n)*abs(1-vtalapx(n))-VCHDROT(n)/2;

% estimate fuse diameter in side view
fusudsv = FUSEGMX(4,1)*vtallca^FUSEGMX(4,2);% upper geometry
fusldsv = FUSEGMX(5,1)*vtallca^FUSEGMX(5,2);% upper geometry
fuseldi = fusudsv+fusldsv;% total local fuselage diameter
% v-tail cut-out span
vtalslc = fusudsv+(FUSEGMX(5,4)-vtallca)*tan(aftfusw(n)*kdrang)- ...
    fuseldi*vtalver(n);
vtalcut = qxcdcop(vtalslc/2,VSPNMTX,VCHDROT(n),vtaltap,n);% fuse-vtail chord
%=====
if htalare(n)>0.0001
    if taillay(n)>0.5
        if taillay(n)<1.0
            % cruciform
        else
            % the h-tail is located on the v-tail (T-tail)
            vttkcut=vtalcut*vtalthk(1,n);% thickness at fuse-vtail cut
            htalslc=vttkcut;% h-tail cut-out
        end
        htalslc=0.0;% no fuselage cut-out is to be examined
    else
        % the h-tail is located on the fuselage (conventional)
        % locate where the h-tail is placed on the aft fuse geometry
        % Add abs() since with tailbooms configuration HT apex is greater than
        % fuselage length (i.e. greater than 1.)
        htallca=fuselgt(n)*abs(1-htalapx(n))-HCHDROT(n)/2;
        % local fuse diameter in plan view
        fusudsv=FUSEGMX(4,1)*htallca^FUSEGMX(4,2);% upper geometry
        % h-tail cut-out span
        htalslc=fusudsv+(FUSEGMX(5,4)-htallca)*tan(aftfusw(n)*kdrang)- ...
            fusevma(n)*htalver(n);
    end
    %=====
    htalcut=qxcdcop(htalslc/2,HSPNMTX,HCHDROT(n),htaltap,n);% fuse-htail chord
    % predict the inboard h-tail wetted area (less span cut-out)
    htalwti=wetcomp(htalcut,HCHDROT(n)*htaltap(1,n)/htalcut,...
        HSPNMTX(1)-htalslc/2,htalthk(1,n),HTHKMTX(2,2)/HTHKMTX(2,1));
    % predict the midboard h-tail wetted area
    htalwtm=wetcomp(HCHDROT(n)*htaltap(1,n),htaltap(2,n)/htaltap(1,n),...
        HSPNMTX(2),HTHKMTX(2,2),HTHKMTX(1,2));
    % predict the outboard h-tail wetted area
    htalwto=wetcomp(HCHDROT(n)*htaltap(2,n),htaltap(3,n)/htaltap(2,n),...
        HSPNMTX(3),HTHKMTX(2,3),HTHKMTX(1,3));
    % final estimate for wetted area
    HTALWET(n)=htalwti+htalwtm+htalwto+htaliwt(n);
else
    HTALWET(n)=0.0;% no horizontal tail exists
end
%============================================================================
% Estimate the Canard wetted area.
%=====
if canrare(n)>0.0001
    % the canard is located on the fuselage (conventional)
    % locate where the h-tail is placed on the fore fuse geometry
    canrlca=fuselgt(n)*(1-canrapx(n))-CCHDROT(n)/2;
    % local fuse diameter in plan view
    fusudsv=FUSEGMX(4,1)*canrlca^FUSEGMX(4,2);% upper geometry
    % canard cut-out span
    canrslc=fusudsv+(FUSEGMX(5,4)-canrlca)*tan(foredsw(n)*kdrang)- ...
        fusfvma(n)*canrver(n);
    %=====
    canrcut=qxcdcop(canrslc/2,CSPNMTX,CCHDROT(n),canrtap,n);% fuse-canard chord
    % predict the inboard canard wetted area (less span cut-out)
    canrwti=wetcomp(canrcut,CCHDROT(n)*canrtap(1,n)/canrcut,...
        CSPNMTX(1)-canrslc/2,canrthk(1,n),CTHKMTX(2,2)/CTHKMTX(2,1));
    % predict the midboard canard wetted area
    canrwtm=wetcomp(CCHDROT(n)*canrtap(1,n),canrtap(2,n)/canrtap(1,n),...
        CSPNMTX(2),CTHKMTX(2,2),CTHKMTX(1,2));
    % predict the outboard canard wetted area
    canrwto=wetcomp(CCHDROT(n)*canrtap(2,n),canrtap(3,n)/canrtap(2,n),...
        CSPNMTX(3),CTHKMTX(2,3),CTHKMTX(1,3));
    % final estimate for wetted area
    CANRWET(n)=canrwti+canrwtm+canrwto+canriwt(n);
else
    CANRWET(n)=0.0;% no canard exists
end
%============================================================================
% Estimate the vertical tail wetted area.
%=====
if vtalare(n)>0.0001
    % predict the inboard v-tail wetted area (less span cut-out)
    vtalwti=wetcomp(vtalcut,VCHDROT(n)*vtaltap(1,n)/vtalcut,...
        VSPNMTX(1)-vtalslc/2,vtalthk(1,n),VTHKMTX(2,2)/VTHKMTX(2,1));
    % predict the midboard v-tail wetted area
    vtalwtm=wetcomp(VCHDROT(n)*vtaltap(1,n),vtaltap(2,n)/vtaltap(1,n),...
        VSPNMTX(2),VTHKMTX(2,2),VTHKMTX(1,2));
    % predict the midboard v-tail wetted area
    vtalwto=wetcomp(VCHDROT(n)*vtaltap(2,n),vtaltap(3,n)/vtaltap(2,n),...
        VSPNMTX(3),VTHKMTX(2,3),VTHKMTX(1,3));
    % final estimate for wetted area
    VTALWET(n)=(vtalwti+vtalwtm+vtalwto)/2+vtaliwt(n);
    % check to see if a T-tail arrangement is employed
    if taillay(n)==1
        % increment the vertical tail wetted area with a bullet
        bullwet=wetcomp(VCHDROT(n)*vtaltap(3,n),1,2*HCHDROT(n)*htalthk(1,n), ...
            vtalthk(4,n),1)/2;
        VTALWET(n)=VTALWET(n)+bullwet;
    end
else
    VTALWET(n)=0.0;
end
%============================================================================
% Estimate the dorsal fin wetted area.
%=====
if vtaldlc(n)>0.0001
    % the dorsal fin root chord (w/o v-tail)
    dfinrop=(vtalapx(n)-vtaldlc(n))*fuselgt(n);%+ ...
    %(1-2*vtalver(n))*fusevma(n)/2*tan(vtallsw(1,n)*kdrang);
    %=====
    % find out the height for intersection between dorsal and v-tail
    hbar=dfinrop*sin((90-vtaldsw(n))*kdrang)/sin((vtaldsw(n)- ...
        vtallsw(1,n))*kdrang);
    drshint=hbar*sin((90-vtallsw(1,n))*kdrang);
    %=====
    % establish the local dorsal fin thickness matrix
    vchrdb0=qxcdcop((1-2*vtalver(n))*fusevma(n)/2,VSPNMTX,VCHDROT(n), ...
        vtaltap,n);% v-tail chrd @ root
    dfinrot(n)=dfinrop+vchrdb0;% the final dorsal root chord
    vchrdb1=qxcdcop((1-2*vtalver(n))*fusevma(n)/2+drshint/3,VSPNMTX, ...
        VCHDROT(n),vtaltap,n);% v-tail chrd @ 1
    dchrdb1=vchrdb1+2*dfinrop/3;% local dorsal chord @ 1
    vchrdb2=qxcdcop((1-2*vtalver(n))*fusevma(n)/2+2*drshint/3,VSPNMTX, ...
        VCHDROT(n),vtaltap,n);% v-tail chrd @ 2
    dchrdb2=vchrdb2+dfinrop/3;% local dorsal chord @ 2
    vchrdbt=qxcdcop((1-2*vtalver(n))*fusevma(n)/2+drshint,VSPNMTX, ...
        VCHDROT(n),vtaltap,n);% v-tail chrd @ tip
    dchrdbt=vchrdbt;% local dorsal chord @ tip
    dorthkr=qxtkcop((1-2*vtalver(n))*fusevma(n)/2,VSPNMTX, ...
        VTHKMTX)*vchrdb0/dfinrot(n);% thickness @ root
    dorthk1=qxtkcop((1-2*vtalver(n))*fusevma(n)/2+drshint/3,VSPNMTX, ...
        VTHKMTX)*vchrdb1/dchrdb1;% thickness @ 1
    dorthk2=qxtkcop((1-2*vtalver(n))*fusevma(n)/2+2*drshint/3,VSPNMTX, ...
        VTHKMTX)*vchrdb2/dchrdb2;% thickness @ 2
    dorthkt=qxtkcop((1-2*vtalver(n))*fusevma(n)/2+drshint,VSPNMTX, ...
        VTHKMTX)*vchrdbt/dchrdbt;% thickness @ tip
    dthkmtx=[dorthk1/dorthkr dorthk2/dorthk1 dorthkt/dorthk2; ...
        dorthkr dorthk1 dorthk2];
    %=====
    dspnmtx=[drshint/3 drshint/3 drshint/3];% dorsal span matrix
    %=====
    dorswet=wetcomp(dfinrop,0,drshint,dorthkr,dorthkt/dorthkr)/2;
    % final estimate for wetted area
    VTALWET(n)=VTALWET(n)+dorswet;
end
%============================================================================
% Estimate the nacelle wetted area.
%=====
% Legend for powerplant configuration selection:
% (0) slung in vicinity of the wing
% (1) on-wing nacelle (2) on-wing integrated with undercarraige
% (3) aft-fuse
% (4) Straight duct   (5) S-duct
% nacetype (1) long-duct (2)short-duct
NACEWET(1:2,n)=0;% initialise for calculation
for i=1:PN
    % the nacelle diameter and length parameters for further treatment
    psq=(NACELGT(i,n)/(4.81))^2;
    msq=(nacemdi(i,n)/(4.93))^2;
    %   dsq=(nacemdi(i,n)^2)/97.22;lsq=(NACELGT(i,n)^2)/92.54;
    %=====
    % predict the nacelle wetted area for all installations
    nacewec=2*0.2028*(pi^2)*nacemdi(i,n)*(((4.7595*psq+1.1333*msq)^0.5)+ ...
        ((4.2874*psq+1.8375*msq)^0.5)- ...
        ((0.1175*psq+0.3918*msq)^0.5)- ...
        (0.3820*psq+0.8913*msq)^0.5);
    % now compute the basic pitot nacelle wetted area
    psq=(nacbase(i,n)/(4.81))^2;
    msq=(nacdase(i,n)/(4.93))^2;
    NAWBASE(i,n)=2*0.2028*(pi^2)*nacdase(i,n)*(((4.7595*psq+1.1333*msq)^0.5)+ ...
        ((4.2874*psq+1.8375*msq)^0.5)- ...
        ((0.1175*psq+0.3918*msq)^0.5)- ...
        (0.3820*psq+0.8913*msq)^0.5);
    if engeloc(i,n)<1
        NAWBASE(i,n)=nacewec;% make sure pitot
    end
    %=====
    if engeloc(i,n)==1
        nacewec=nacewec/2;% adjust the computed wetted area
        % account for the two side panels
        nacewec=nacewec+2*NACELGT(i,n)*nacemdi(i,n)/2;
    elseif engeloc(i,n)==2
        % account for the two side panels
        nacewec=nacewec+2*NACELGT(i,n)*nacemdi(i,n)/2;
    end
    NACEWET(i,n)=nacewec*engenum(i,n);% compute wetted area for all
end
%=====
if pylnwfl>0.0001
    PYLNWET(n)=PYLNCLW(n)+pylniwt(n);% the final total pylon wetted area
else
    pylnwfl=1;
end
% the final total nacelle wetted area
PWRPWET(n)=NACEWET(1,n)+NACEWET(2,n)+pwrpiwt(n);
%============================================================================
% Estimate the total wetted area for aircraft.
TOTLWET(n) = WINGWET(n) + WLETWET(n) + FUSEWET(n) + HTALWET(n) + VTALWET(n)+ ...
    PYLNWET(n) + PWRPWET(n) + ANCIWET(n);

if ~isreal(TOTLWET(n))
    fprintf('!Warning: Total wetted area is complex, making it real.\n');
    TOTLWET(n) = real(TOTLWET(n));
end

% save('qwetuav.mat');

return
%============================================================================
function [acoef,bcoef,abarc,bdlgt]=gocomp(angleb,angler,fin,diam,orien)
global kdrang
% This routine computes the various coefficients required for geometrical
% definition and subsequent computation of fuselage wetted area.
% Based on method developed by Isikveren.
bdlgt=fin*diam;% body length calculation
dslc=angleb-angler;% adjusted body slope to reference angle
% compute the values of the geometric model
bcoef=0.54+0.1*tan(kdrang*dslc);% geometric model exponent
acoef=diam/(2*bdlgt^bcoef);% geometrical model coefficient
% this section computes the integral coefficent used as an approximation
% for wetted area prediction later on
evalfl=(((bdlgt)^(2*bcoef))+((acoef*bcoef)^2)*(bdlgt)^(4*bcoef-2))^0.5;
evalfq=(((bdlgt/4)^(2*bcoef))+((acoef*bcoef)^2)*(bdlgt/4)^(4*bcoef-2))^0.5;
acoefp=exp((log(evalfq)-(1-log(4)/log(bdlgt))*log(evalfl))/(log(4)/log(bdlgt)));
bcoefp=(log(evalfl)-log(acoefp))/log(bdlgt);
abarc=acoefp/(1+bcoefp)*bdlgt^(bcoefp+1);% integral coefficient
return
%============================================================================
function wetted=wetcomp(chord,stapr,span,thick,ttapr)
% This routine predicts the wetted area of lifting surfaces
a=2;b=8.5;
weta=chord*span;wetb=(a+b*thick^2)*(1+stapr);
wetc=2/3*b*(thick^2)*(ttapr-1)*(1+2*stapr);
wetted=weta*(wetb+wetc);
%wetc=1+stapr+2*b*(thick^2)*(ttapr-1)*(3+2*span*(stapr-1))/3;
%wetted=weta*wetb*wetc;
return
%============================================================================