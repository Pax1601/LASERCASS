function qgeotry(action,n)
% Constants for various properties
global kdrang knwlbf
%=====
% Output parameters from QGEOTRY
global WINWARE WINWTAP WINWGAR WINWTHB WINGEQS LWINGEM REFWAPX WINGNET
global WI2GNET nacbase nacdase WINWQSW
% Input parameters from QGEOTRY
global wletare dcldawl wingase wgevorx clazero wltvorx wletinf wletinr
global wletmac wletthk wletqsw
global XSECDWF WTHKROT WCHRDWF WSPNMTX WTHKMTX WINWHSQ
global XS2CDWF WT2KROT WC2RDWF WS2NMTX WT2KMTX WC2DROT WI2WTHB WI2WMAC
global WI2WARE WI2WGAR WI2WTAP WI2WQSW WI2WHSQ
global CROSXCF CROSXFF CIRCUMX 
global HSPNMTX HTHKMTX HTALMOA REFHAPX 
global CSPNMTX CTHKMTX CNALMOA REFCAPX 
global VTHKMTX VSPNMTX VTALMOA VTAWGAR REFVAPX REFVCRC REFVCAP
global ENGEDIA NACELGT ENGECHD MAXISTE
global PN
global volflag
%=====
% Scripts for executing the global parameter declarations
% global GPARAMT
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
global canrare canrgar canrspn canrkln canrtap canrlam canrthk canrinc canrqsw canrlsw canrdih canrver canrapx canrvol flCNCrd canrkln
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

global cabnhei cabnwid cabnlgt cabnfwd baggapx baggtyp baggvol bagglgt

% Fuselage section coefficients
global CROSXCF CROSXFF

global WSPNMTX WINWTAP WI2WGAR WI2WARE WI2WTAP WI2WQSW WI2WTHB REFWAPX WINWGAR WINWTHB 
global HTALMOA HSPNMTX CANRMOA CSPNMTX VTALMOA VSPNMTX destype cabnpas cabnvol
%============================================================================
% Define the actions executed by the input interface
%=====
% dummy routine to run the different cases
if nargin<1
  action='initialize';
end
%=====

switch action
%=================================
case 'ccabn'
% this routine computes/checks the cabin & baggage dimensions and properties
if cabnhei(n)<0.0001 & fusevma(n)>0.0001
   if fusevmi(n)>0.5
      cabnhei(n)=fusevmi(n)*fusevma(n)-0.15;% double-bubble cabin height
   else
      if wingplc(n)<0 | wingplc(n)>1
         cabnhei(n)=fusevma(n)-0.2;% wingbox does not go through x-section
      else
         if wingplc(n)>0.5
% circular x-section cabin height for high wings
            cabnhei(n)=fusevma(n)*wingplc(n)-WTHKMTX(2,1)*WCHDROT(n)/2-0.2;
         else   
% circular x-section cabin height for low wings
            cabnhei(n)=fusevma(n)*(1-wingplc(n))-WTHKMTX(2,1)*WCHDROT(n)/2-0.2;
         end      
      end
   end   
end
if cabnwid(n)<0.0001 & fusehma(n)>0.0001
   cabnwid(n)=fusehma(n)-0.2;% predict cabin max width   
end
if cabnfwd(n)<0.0001 & fusevma(n)>0.0001
   if fusevmi(n)>0.5
      sweptan=atan(2*fusevma(n)*(fusevmi(n)-0.5)/fusehma(n))+pi;
      sweptrd=CROSXCF(1)+CROSXCF(2)*sin(sweptan)+CROSXCF(3)*cos(2*sweptan);
      cabnfwd(n)=2*sweptrd*cos(sweptan-pi)-0.2;% double-bubble floor width
   else   
      cabnfwd(n)=((cabnwid(n)^2)-4*(((fusevma(n)-WTHKMTX(2,1)*WCHDROT(n)- ...
                                     0.2)/2)^2))^0.5;% predict the floor width
   end
end
if volflag>0.0001
   hbar=0.5*((cabnwid(n)^2)-cabnfwd(n)^2)^0.5;cabth=atan(2*hbar/cabnfwd(n));
   cabnvol(n)=cabnlgt(n)/4*(cabnwid(n)*(pi*cabnhei(n)+cabth*cabnwid(n))+ ...
              hbar*(2*cabnfwd(n)-pi*cabnwid(n)));% predict cabin volume
   volflag=0;% reset volume computation flag
end
% baggage is to be located underfloor
if baggapx(n)<0.0001
   if baggtyp(n)~=0
      baggapx(n)=1.5*forelgt(n)/fuselgt(n);% estimate the u/flr bagg 1 apex
   else      
      baggapx(n)=(forelgt(n)+cabnlgt(n))/fuselgt(n);% est. aft baggage apex
   end
end
if baggvol(n)==-1
   maxibag=0.382;scalbag=1;% upper limit of baggage vol per PAX = 13.5 cu.ft
   if destype(n)<2
      scalbag=0.48;% scale for commercial transportation
   elseif destype(n)==2
      scalbag=0.63;% scale for business jets larger than super midsize 
   end
   baggvol(n)=cabnpas(n)*scalbag*maxibag;% baggage volume prediction 
   if baggtyp(n)~=0
      baggxsc=2*(fusevma(n)-cabnhei(n)-0.45)*cabnfwd(n)/3;% x-sec area u/floor
      bagglgt(n)=baggvol(n)/baggxsc;% estimate u/floor total baggage length
   else
      bagglgt(n)=baggvol(n)/cabnvol(n)*cabnlgt(n);% est. aft baggage length
   end
elseif baggvol(n)<0.0001 & bagglgt(n)>0.0001
   if baggtyp(n)~=0
      baggxsc=2*(fusevma(n)-cabnhei(n)-0.45)*cabnfwd(n)/3;% x-sec area u/floor
      baggvol(n)=baggxsc*bagglgt(n);% estimate u/floor baggage volume
   else   
      baggvol(n)=cabnvol(n)/cabnlgt(n)*bagglgt(n);% estimate baggage volume
   end
elseif baggvol(n)>0.0001 & bagglgt(n)<0.0001
   if baggtyp(n)~=0
      baggxsc=2*(fusevma(n)-cabnhei(n)-0.45)*cabnfwd(n)/3;% x-sec area u/floor
      bagglgt(n)=baggvol(n)/baggxsc;% estimate u/floor baggage length
   else   
      bagglgt(n)=baggvol(n)/cabnvol(n)*cabnlgt(n);% estimate baggage length
   end
end
%=================================
case 'geom1'
% The wing number 1 geometric and aerodynamic definitions.
%=====
[kln,spn,gar,smtrx,crot,xscd,tap,cref,chrf,tref,chwf,troot,tmtrx,qsw,lsw, ...
 netwgae,wsatref,wsarref,wsasref,wsarlsw,wsarqsw,wsarhsw,wsarmac,wsarybr, ...
 wsarthb,inc]=qxemcop(wingkln,wingtap,wingspn,wingare,winggar, ...
 fusehma,wingthk,wingqsw,winglsw,WINREFC,wingplc,winginc,n);

%=====
% output details about the original planform
winginc(:,n)=inc;
wingkln(:,n)=kln(:,n);wingspn(n)=spn(n);winggar(n)=gar(n);
wingtap(:,n)=tap(:,n);wingthk(2,n)=tmtrx(2,2);wingthk(3,n)=tmtrx(2,3);
WTHKROT(n)=troot;WCHDROT(n)=crot;WCHRDWF(n)=chwf;
WCHDTIP(n)=WCHDROT(n)*wingtap(3,n);WCHDRTR(n)=cref;WCHDWFR(n)=chrf;
WTAPERR(n)=tref;XSECDWF(n)=xscd;
wingqsw=qsw;winglsw=lsw;WSPNMTX=smtrx;WTHKMTX=tmtrx;

%=====
% output details about the chosen reference wing
WINGNET(n)=netwgae;ZSERDWF(n)=xscd/2;
WINWARE(n)=wsarref;WINWTAP(n)=wsatref;WINWGAR(n)=wsasref;
WINWLSW(n)=abs(wsarlsw);WINWQSW(n)=abs(wsarqsw);WINWHSQ(n)=abs(wsarhsw);
WINWMAC(n)=wsarmac;WINWYBR(n)=wsarybr;WINWTHB(n)=wsarthb;
WINGEQS(n)=WINWQSW(n);

%=====
% output details about the computed equivalent reference wing
REFWARE(n)=WINWARE(n);REFWTAP(n)=WINWTAP(n);REFWGAR(n)=WINWGAR(n);
REFWLSW(n)=WINWLSW(n);REFWQSW(n)=WINWQSW(n);REFWHSQ(n)=WINWHSQ(n);
REFWMAC(n)=WINWMAC(n);REFWYBR(n)=WINWYBR(n);REFWTHB(n)=WINWTHB(n);
comspan=wingspn(n);
%=====

% compute location of the reference wing 0.25MAC
xibad=WSPNMTX(1)*tan(kdrang*winglsw(1,n));% inboard sweep adjustment
xmbad=WSPNMTX(2)*tan(kdrang*winglsw(2,n));% midboard sweep adjustment
xobad=WSPNMTX(3)*tan(kdrang*winglsw(3,n));% outboard sweep adjustment
xwing=wingapx(n)*fuselgt(n)+xibad+xmbad+xobad- ...
      0.5*wingspn(n)*(1-WINWYBR(n))*tan(kdrang*WINWLSW(n))+0.25*WINWMAC(n);
% locate the reference wing relative apex 
REFWAPX(n)=(wingapx(n)*fuselgt(n)+xibad+xmbad+xobad- ...
            0.5*wingspn(n)*tan(kdrang*WINWLSW(n)))/fuselgt(n);
% calculate the length of the wing for supersonic drag prediction
if (xibad+xmbad+xobad+WCHDTIP(n))>WCHDROT(n)
   LWINGEM(1,n)=xibad+xmbad+xobad+WCHDTIP(n);
else
   LWINGEM(1,n)=WCHDROT(n);
end
%============================================================================
% geometric definitions to be used for winglet computations
%=====
if wletspn(n)>0.0001
   null(n)=0.0;% this signifies no juncture chord calc is required
   wletkln(1,n)=0.0;wletkln(2,n)=0.0;% declare the ghost kink location array
   wlettpr(1,n)=0.0;wlettpr(2,n)=0.0;wlettpr(3,n)=wlettap(n);% taper ratio
   wltspan(n)=wletspn(n)*WCHDTIP(n);% quantify the winglet span
% individual winglet planform area
   wletare(n)=wltspan(n)/2*WCHDTIP(n)*(1+wlettap(n));
   wletasr(n)=(wltspan(n)^2)/wletare(n);% winglet geometric aspect ratio
% set up the winglet thickness distribution
   wletthk(1,n)=wingthk(4,n);wletthk(2,n)=wingthk(4,n);
   wletthk(3,n)=wingthk(4,n);wletthk(4,n)=wingthk(4,n);
% Q.chd sweep array
   wletqsw(1,n)=0.0;wletqsw(2,n)=wletqsw(1,n);wletqsw(3,n)=wletqsw(2,n);
   wletlsw(2,n)=wletlsw(1,n);wletlsw(3,n)=wletlsw(2,n);% LE sweep array
%=====
   [kln,spn,gar,smtrx,crot,xscd,tap,cref,chrf,tref,chwf,troot,tmtrx,qsw,lsw, ...
    netwgae,wsatref,wsarref,wsasref,wsarlsw,wsarqsw,wsarhsw,wsarmac,wsarybr, ...
    wsarthb,inc]=qxemcop(wletkln,wlettpr,wltspan,wletare,wletasr, ...
    null,wletthk,wletqsw,wletlsw,WINREFC,null,wletinc,n);
%=====
% output details using the chosen reference wing convention
   wletinc(:,n)=inc;
   
   wletqsw(1,n)=wsarqsw;wletmac(n)=wsarmac;
   wletinr(n)=(wletinc(1,n)-wletinc(3,n))*wsarybr+wletinc(1,n);
% lift-curve slope of section (per deg.)
   dcldawl(n)=0.088*wletasr(n)/(2+(4+wletasr(n)^2)^0.5)*cos(kdrang*wletqsw(1,n));
% quantify an equivalent increase in span due to presence of winglets
   wingase(n)=((wingspn(n)+ ...
                2*wltspan(n)*tan(kdrang*wletvos(n)))^2)/(WINWARE(n)+ ...
                2*wletare(n)*tan(kdrang*wletvos(n)));
% fractional change in vortex-induced drag factor due to improved 
% lift-distribution
   wgevorx(n)=(WINWGAR(n)-wingase(n))/(wingase(n)*(1+pi/150*WINWGAR(n)));
   clazero(n)=dcldawl(n)*3.0;% assume zero-lift at alpha = -3.0 deg.
   wltvorx(n)=1.05/(pi*wletasr(n))+0.007;% winglet induced drag factor
% estimate the interference generated by the winglet on the wing
   wletinf(n)=(1-wletvos(n)/90)*((2+4*wletthk(1,n)+ ...
                                  240*(wletthk(1,n)^4))/2-1)+1;
end
%============================================================================
% The wing number 2 geometric and aerodynamic definitions.
%
% see if a second planform geometry has been entered
WI2WMAC(n)=0.0;% initialise this variable
if wi2gare(n)>0.001
   [kln,spn,gar,smtrx,crot,xscd,tap,cref,chrf,tref,chwf,troot,tmtrx,qsw,lsw, ...
    netwgae,wsatref,wsarref,wsasref,wsarlsw,wsarqsw,wsarhsw,wsarmac,wsarybr, ...
    wsarthb,inc]=qxemcop(wi2gkln,wi2gtap,wi2gspn,wi2gare,wi2ggar, ...
    fusehma,wi2gthk,wi2gqsw,wi2glsw,WINREFC,wi2gplc,wi2ginc,n);
%=====
% output details about the original planform
   wi2ginc(:,n)=inc;
   
   wi2gkln(:,n)=kln(:,n);wi2gspn(n)=spn(n);wi2ggar(n)=gar(n);
   wi2gtap(:,n)=tap(:,n);wi2gthk(2,n)=tmtrx(2,2);wi2gthk(3,n)=tmtrx(2,3);
   WT2KROT(n)=troot;WC2DROT(n)=crot;WC2RDWF(n)=chwf;
   WC2DTIP(n)=WC2DROT(n)*wi2gtap(3,n);WC2DRTR(n)=cref;WC2DWFR(n)=chrf;
   WT2PERR(n)=tref;XS2CDWF(n)=xscd;
   wi2gqsw=qsw;wi2glsw=lsw;WS2NMTX=smtrx;WT2KMTX=tmtrx;
%=====
% output details about the chosen reference wing
   WI2GNET(n)=netwgae;XS2RDWF(n)=xscd/2;
   WI2WARE(n)=wsarref;WI2WTAP(n)=wsatref;WI2WGAR(n)=wsasref;
   WI2WLSW(n)=abs(wsarlsw);WI2WQSW(n)=abs(wsarqsw);WI2WHSQ(n)=abs(wsarhsw);
   WI2WMAC(n)=wsarmac;WI2WYBR(n)=wsarybr;
   WI2WTHB(n)=wsarthb;WI2GEQS(n)=WI2WQSW(n);
%=================================
% An equivalent reference wing must be constructed if two wings are defined.
% This method was developed by Isikveren and employs work conducted by
% Munk (stagger theorem), Prandtl, Kerber and Obert. 
%=====
% compute the superpositioned Oswald Efficiency factor
   spanras=(wi2gspn(n)/wingspn(n))^2;liftras=(WI2WARE(n)/WINWARE(n))^2;
% estimate the mutual drag factor (from Kerber) - this is an analytical 
% representation of original chart
   intsgap=abs(wingplc(n)-wi2gplc(n))*fusevma(n)/wingspn(n);% inter-surf gap
   mdrgfac=((spanras^0.5)+66.167*(intsgap^4)-38.1*(intsgap^3)+ ...
           2.5983*(intsgap^2)+1.697*intsgap)/(276.67*(intsgap^4)- ...
           165.02*(intsgap^3)+35.158*(intsgap^2)+4.6327*intsgap+1);
% find the equivalent reference wing aspect ratio   
   comarea=(WINWARE(n)+WI2WARE(n));
   areara1=(WINWARE(n)^2)/comarea;areara2=(WI2WARE(n)^2)/comarea;
   relspn1=wingspn(n)^2;relspn2=wi2gspn(n)^2;
   asratio=1.05/(areara1/relspn1+areara2/relspn2+ ...
           2*mdrgfac*WINWARE(n)*WI2WARE(n)/(comarea*wingspn(n)*wi2gspn(n))- ...
           0.007*pi);
% compute the equivalent reference wing span
   comspan=(asratio*comarea)^0.5;        
% calculate the equivalent reference wing MAC and leading edge position
% based on weighted planform areas   
   combmac=(WINWARE(n)*WINWMAC(n)+WI2WARE(n)*WI2WMAC(n))/comarea;% 
   combybr=(WINWARE(n)*WINWYBR(n)*wingspn(n)+ ...
            WI2WARE(n)*WI2WYBR(n)*wi2gspn(n))/(comarea*comspan);
   mactip1=wingapx(n)*fuselgt(n)+WINWYBR(1)*tan(kdrang*WINWLSW(n));
   mactip2=wi2gapx(n)*fuselgt(n)+WI2WYBR(1)*tan(kdrang*WI2WLSW(n));
   macxtip=(mactip1*WINWARE(n)+mactip2*WINWARE(n))/comarea;
% define the equivalent reference wing thickness
   combthk=(WINWTHB(n)*WINWARE(n)+WI2WTHB(n)*WINWARE(n))/comarea;
% define a new wingtip geometry based on weighted planform areas    
   wnxtip1=wingapx(n)*fuselgt(n)+WSPNMTX(1)*tan(kdrang*winglsw(1,n))+ ...
           WSPNMTX(2)*tan(kdrang*winglsw(2,n))+ ...
           WSPNMTX(3)*tan(kdrang*winglsw(3,n));
   wnxtip2=wi2gapx(n)*fuselgt(n)+WS2NMTX(1)*tan(kdrang*wi2glsw(1,n))+ ...
           WS2NMTX(2)*tan(kdrang*wi2glsw(2,n))+ ...
           WS2NMTX(3)*tan(kdrang*wi2glsw(3,n));
   cmwxtip=(wnxtip1*WINWARE(n)+wnxtip2*WINWARE(n))/comarea;
   combtip=(WCHDTIP(n)*WINWARE(n)+WC2DTIP(n)*WI2WARE(n))/comarea;    
% calculate the equivalent weighted leading edge sweep for ref. wing
   comblsw=atan(2*(cmwxtip-macxtip)/((1-combybr)*comspan))/kdrang;
% weighted root chord length
   comcref=(combmac-combtip*combybr)/(1-combybr);
   comtref=combtip/comcref;% weighted taper ratio
% ref. quarter chord sweep
   combqsw=atan(tan(comblsw*kdrang)-(1-comtref)/(asratio*(1+comtref)))/kdrang;
% ref. half chord sweep
   combhsw=atan(tan(comblsw*kdrang)-2*(1-comtref)/(asratio*(1+comtref)))/kdrang;
%=====
% output details about the computed equivalent reference wing
   REFWARE(n)=comarea;REFWTAP(n)=comtref;REFWGAR(n)=asratio;
   REFWLSW(n)=abs(comblsw);REFWQSW(n)=abs(combqsw);REFWHSQ(n)=abs(combhsw);
   REFWMAC(n)=combmac;REFWYBR(n)=combybr;REFWTHB(n)=combthk;   
%=====
% compute location of the reference wing 0.25MAC
   xwing=cmwxtip+0.25*REFWMAC(n)- ...
         0.5*comspan*(1-REFWYBR(n))*tan(kdrang*REFWLSW(n));
% calculate the length of the wing for supersonic drag prediction
   if (wnxtip2-wi2gapx(n)*fuselgt(n)+WC2DTIP(n))>WC2DROT(n)
      LWINGEM(2,n)=wnxtip2-wi2gapx(n)*fuselgt(n)+WC2DTIP(n);
   else
      LWINGEM(2,n)=WC2DROT(n);
   end
else
   WI2WTHB(n)=0.0;WI2WARE(n)=0.0;WI2WGAR(n)=0.0;WI2WTAP(n)=0.0;
   WI2WQSW(n)=0.0;LWINGEM(2,n)=0.0;LWINGEM(2,n)=0.0;WI2GNET(n)=0.0;
end   
%============================================================================
% The fuselage geometric characteristics.
% A Fourier Series Expansion (FSE) is conducted to derive the final 
% cross-section geometry. This is crucial because the method must be 
% universally applicable for not only cylindrical but double-bubble and ovoid
% cross-sections as well. The cross-section circumference calculation is based
% on an average radius concept.
% The FSE objective function is called "qxfours.m"
%=====
%% Compute the nose and tail length
forelgt(1,n)=forefin(1,n)*fusfvma(1,n);
aftslgt(1,n)=aftsfin(1,n)*fusevma(1,n);

if fusfvma(n)==0
   fusfvma(n)=fusevma(n);% error trap for fwd fuse x-sect
end
if fusfhma(n)==0
   fusfhma(n)=fusehma(n);% error trap for fwd fuse x-sect
end
[CROSXFF]=fndfscf(fusfhma,fusfvma,fusfvmi,n);% derive FS coeff. for fwd fuse
[CROSXCF]=fndfscf(fusehma,fusevma,fusevmi,n);% derive FS coeff. for cntr + aft fuse
% parameters used to quantify the circumference for cross-section
circfmu=CROSXFF(1)*pi+2*CROSXFF(2);% approx. upper curve circum. fwd fuse
circfml=CROSXFF(1)*pi-2*CROSXFF(2);% approx. lower curve circum. fwd fuse
circumu=CROSXCF(1)*pi+2*CROSXCF(2);% approx. upper curve circum. centre + aft
circuml=CROSXCF(1)*pi-2*CROSXCF(2);% approx. lower curve circum. centre + aft
circfru=2*circfmu/(pi*fusevma(n));% circum. factor for upper fwd fuse geom.
circfrl=2*circfml/(pi*fusevma(n));% circum. factor for lower fwd fuse geom.
circmru=2*circumu/(pi*fusevma(n));% circum. factor for upper centre + aft fuselage geom.
circmrl=2*circuml/(pi*fusevma(n));% circum. factor for lower centre + aft fuselage geom.
circfrm=(circfmu+circfml)/(pi*fusevma(n));% total cross-section circumference
circmrm=(circumu+circuml)/(pi*fusevma(n));% total cross-section circumference
CIRCUMX=[circfru circfrl circmrm circmru circmrl circfrm];% wetted area scaling factor
%============================================================================
% Horizontal tail geometric and aerodynamic definitions.
%=====
if htalare(n)>0.0001
   null(n)=0.0;% this signifies no juncture chord calc is required
% the empennage only has a 2 segment definition, need to assume that 
% ghost segment 2 is equal to segment 1
   htalqsw(2,n)=htalqsw(1,n);htallsw(2,n)=htallsw(1,n);
   htalthk(3,n)=htalthk(4,n);
%=====
   [kln,spn,gar,smtrx,crot,xscd,tap,cref,chrf,tref,chwf,troot,tmtrx,qsw,lsw, ...
    netwgae,wsatref,wsarref,wsasref,wsarlsw,wsarqsw,wsarhsw,wsarmac,wsarybr, ...
    wsarthb,inc]=qxemcop(htalkln,htaltap,htalspn,htalare,htalgar, ...
    null,htalthk,htalqsw,htallsw,WINREFC,null,htalinc,n);
%=====
% output details about the original planform
   htalinc(:,n)=inc;

   htalkln(:,n)=kln(:,n);htalspn(n)=spn(n);htalgar(n)=gar(n);
   htaltap(:,n)=tap(:,n);htalthk(2,n)=tmtrx(2,2);htalthk(3,n)=tmtrx(2,3);
   htaldih(2,n)=htaldih(3,n);HTHKROT(n)=troot;HCHDROT(n)=crot;
   htalqsw=qsw;htalqsw(2,n)=htalqsw(3,n);htallsw=lsw;htallsw(2,n)=htallsw(3,n);
   HSPNMTX=smtrx;HTHKMTX=tmtrx;
%=====
% output details about the chosen reference wing
   HTAWARE(n)=wsarref;HTAWTAP(n)=wsatref;HTAWGAR(n)=wsasref;
   HTAWLSW(n)=wsarlsw;HTAWQSW(n)=wsarqsw;HTAWHSQ(n)=wsarhsw;
   HTALMAC(n)=wsarmac;HTALYBR(n)=wsarybr;
   HTALTHB(n)=wsarthb;HTALEQS(n)=HTAWQSW(n);
%=====
% compute the moment arm to h-tail 0.25MAC and the volume coefficient
   xibad=HSPNMTX(1)*tan(kdrang*htallsw(1,n));% inboard sweep adjustment
   xmbad=HSPNMTX(2)*tan(kdrang*htallsw(2,n));% midboard sweep adjustment
   xobad=HSPNMTX(3)*tan(kdrang*htallsw(3,n));% outboard sweep adjustment
   xhtal=htalapx(n)*fuselgt(n)+xibad+xmbad+xobad- ...
         0.5*htalspn(n)*(1-HTALYBR(n))*tan(kdrang*HTAWLSW(n))+0.25*HTALMAC(n);
% locate the reference horizontal tail relative apex 
   REFHAPX(n)=(htalapx(n)*fuselgt(n)+xibad+xmbad+xobad- ...
               0.5*htalspn(n)*tan(kdrang*HTAWLSW(n)))/fuselgt(n);
   HTALMOA(n)=abs(xhtal-xwing);% moment arm to h-tail   
% compute the volume coefficient   
   htalvol(n)=HTAWARE(n)*HTALMOA(n)/(REFWARE(n)*REFWMAC(n));
else
   HTALMOA(n)=1.0;
end
%============================================================================
% Canard geometric and aerodynamic definitions.
%=====
if canrare(n)>0.0001
   null(n)=0.0;% this signifies no juncture chord calc is required
% the empennage only has a 2 segment definition, need to assume that 
% ghost segment 2 is equal to segment 1
   canrqsw(2,n)=canrqsw(1,n);canrlsw(2,n)=canrlsw(1,n);
   canrthk(3,n)=canrthk(4,n);
%=====
   [kln,spn,gar,smtrx,crot,xscd,tap,cref,chrf,tref,chwf,troot,tmtrx,qsw,lsw, ...
    netwgae,wsatref,wsarref,wsasref,wsarlsw,wsarqsw,wsarhsw,wsarmac,wsarybr, ...
    wsarthb,inc]=qxemcop(canrkln,canrtap,canrspn,canrare,canrgar, ...
    null,canrthk,canrqsw,canrlsw,WINREFC,null,canrinc,n);
%=====
% output details about the original planform
   canrinc(:,n)=inc;

   canrkln(:,n)=kln(:,n);canrspn(n)=spn(n);canrgar(n)=gar(n);
   canrtap(:,n)=tap(:,n);canrthk(2,n)=tmtrx(2,2);canrthk(3,n)=tmtrx(2,3);
   canrdih(2,n)=canrdih(3,n);CNHKROT(n)=troot;CCHDROT(n)=crot;
   canrqsw=qsw;canrqsw(2,n)=canrqsw(3,n);canrlsw=lsw;canrlsw(2,n)=canrlsw(3,n);
   CSPNMTX=smtrx;CTHKMTX=tmtrx;
%=====
% output details about the chosen reference wing
   CANWARE(n)=wsarref;CANWTAP(n)=wsatref;CANWGAR(n)=wsasref;
   CANWLSW(n)=wsarlsw;CANWQSW(n)=wsarqsw;CANWHSQ(n)=wsarhsw;
   CANRMAC(n)=wsarmac;CANRYBR(n)=wsarybr;
   CANRTHB(n)=wsarthb;CANREQS(n)=CANWQSW(n);
%=====
% compute the moment arm to h-tail 0.25MAC and the volume coefficient
   xibad=CSPNMTX(1)*tan(kdrang*canrlsw(1,n));% inboard sweep adjustment
   xmbad=CSPNMTX(2)*tan(kdrang*canrlsw(2,n));% midboard sweep adjustment
   xobad=CSPNMTX(3)*tan(kdrang*canrlsw(3,n));% outboard sweep adjustment
   xcanr=canrapx(n)*fuselgt(n)+xibad+xmbad+xobad- ...
         0.5*canrspn(n)*(1-CANRYBR(n))*tan(kdrang*CANWLSW(n))+0.25*CANRMAC(n);
% locate the reference canard relative apex 
   REFCAPX(n)=(canrapx(n)*fuselgt(n)+xibad+xmbad+xobad- ...
               0.5*canrspn(n)*tan(kdrang*CANWLSW(n)))/fuselgt(n);
   CANRMOA(n)=abs(xcanr-xwing);% moment arm to canard
% compute the volume coefficient   
   canrvol(n)=CANWARE(n)*CANRMOA(n)/(REFWARE(n)*REFWMAC(n));
else
   CANRMOA(n)=1.0;
end   
%============================================================================
% Vertical tail geometric and aerodynamic definitions.
% the empennage only has a 2 segment definition, need to assume that 
% ghost segment 2 is equal to segment 1
%=====
if vtalare(n)>0.0001
   null(n)=0.0;% this signifies no juncture chord calc is required
   vtalqsw(2,n)=vtalqsw(1,n);vtallsw(2,n)=vtallsw(1,n);
   vtalthk(3,n)=vtalthk(4,n);%vtaltap(1,n)=0.0;
%=====
   [kln,spn,gar,smtrx,crot,xscd,tap,cref,chrf,tref,chwf,troot,tmtrx,qsw,lsw, ...
    netwgae,wsatref,wsarref,wsasref,wsarlsw,wsarqsw,wsarhsw,wsarmac,wsarybr, ...
    wsarthbinc,inc]=qxemcop(vtalkln,vtaltap,vtalspn,vtalare,vtalgar, ...
    null,vtalthk,vtalqsw,vtallsw,WINREFC,null,vtalinc,n);
%=====
% output details about the original planform
   vtalinc(:,n)=inc;

   vtalkln(:,n)=kln(:,n);vtalspn(n)=spn(n);vtalgar(n)=gar(n);
   vtaltap(:,n)=tap(:,n);vtalthk(2,n)=tmtrx(2,2);vtalthk(3,n)=tmtrx(2,3);
   VTHKROT(n)=troot;VCHDROT(n)=crot;
   vtalqsw=qsw;vtalqsw(2,n)=vtalqsw(3,n);vtallsw=lsw;vtallsw(2,n)=vtallsw(3,n);
   VSPNMTX=2*smtrx;VTHKMTX=tmtrx;
%=====
% output details about the chosen reference wing
   VTAWARE(n)=wsarref;VTAWTAP(n)=wsatref;VTAWGAR(n)=wsasref;
   VTAWLSW(n)=wsarlsw;VTAWQSW(n)=wsarqsw;VTAWHSQ(n)=wsarhsw;
   VTALMAC(n)=wsarmac;VTALYBR(n)=wsarybr;
   VTALTHB(n)=wsarthb;VTALEQS(n)=VTAWQSW(n);
%=====
% compute the moment arm to v-tail 0.25MAC and the volume coefficient
   xibad=VSPNMTX(1)*tan(kdrang*vtallsw(1,n));% inboard sweep adjustment
   xmbad=VSPNMTX(2)*tan(kdrang*vtallsw(2,n));% midboard sweep adjustment
   xobad=VSPNMTX(3)*tan(kdrang*vtallsw(3,n));% outboard sweep adjustment
   xvtal=vtalapx(n)*fuselgt(n)+xibad+xmbad+xobad- ...
         vtalspn(n)*(1-VTALYBR(n))*tan(kdrang*VTAWLSW(n))+0.25*VTALMAC(n);
% locate the reference vertical tail relative apex 
   REFVAPX(n)=(vtalapx(n)*fuselgt(n)+xibad+xmbad+xobad- ...
               vtalspn(n)*tan(kdrang*VTAWLSW(n)))/fuselgt(n);
% locate reference vertical tail down to the fuselage reference plane
   REFVCAP(n)=(vtalapx(n)*fuselgt(n)- ...
               fusevma(n)*abs(vtalver(n))*tan(kdrang*VTAWLSW(n)))/fuselgt(n);
% reference vertical tail root chord at fuselage reference plane
   REFVCRC(n)=qxcdcop(-fusevma(n)*abs(vtalver(n)),VSPNMTX, ...
              2*VTAWARE(n)/(vtalspn(n)*(1+VTAWTAP(n))),VTAWTAP,n);
   VTALMOA(n)=abs(xvtal-xwing);% moment arm to v-tail   
% compute the volume coefficient   
   vtalvol(n)=VTAWARE(n)*VTALMOA(n)/(REFWARE(n)*comspan);
end   
%=================================
qgeotry('geom2',n);% compute the powerplant dimensions
%=================================
case 'geom2'
% eval(['global' blanks(1) GPARAMT]);% declare the global parameters
%============================================================================
% Predict the engine and subsequent nacelle diameters.
% Based on methods developed by Isikveren.
%=====
NACELGT(1:2,n)=0.0;% initialise for calculation
MAXISTE=zeros(2,15);% initialise the by-pass ratio emulation variable
%=====
if engenum(2,n)>0.0
   PN=2;% prediction for primary and secondary powerplants
else
   PN=1;% prediction for primary powerplants only
end
for i=1:PN
% length of actual wing chord at engine location   
    ENGECHD(i,n)=qxcdcop(abs(engeylc(i,n))*wingspn(n)/2,WSPNMTX, ...
                         WCHDROT(n),wingtap,n);
%=====
   onwings(i,n)=0;% initialise variables
% predict the engine diameter   
   ENGEDIA(i,n)=((2)^-0.5)*(1.73*log(maxistc(i,n))-pi)^0.5;
   nacdase(i,n)=ENGEDIA(i,n)+0.25;% generic pitot nacelle diameter
   if nacemdi(i,n)<0.0001   
% For on-wing installations   
	   if engeloc(i,n)==1.0 | engeloc(i,n)==2.0
         onwings(i,n)=1;% invoke on-wing installation 
      end
% predicted nacelle maximum diameter
      nacemdi(i,n)=(4-onwings(i,n))*(0.0625+ENGEDIA(i,n)/4);
   else
	   ENGEDIA(i,n)=nacemdi(i,n)-0.25;% predicted engine max. diameter
   end    
%=====
% the propeller diameter has not been given
   if propdia(i,n)<0.0001
      propdia(i,n)=(qxheavy(insttyp(i,n),1,1)*3+ ...
                                      0.48*qxheavy(insttyp(i,n),3,1))*ENGEDIA(i,n);
   end
   if insttyp(i,n)>0.0001
      nacetyp(i,n)=1;% automatically choose long duct for props
   end
%=====
% predicted engine length
   engelgt=(maxistc(i,n)^0.9839)/(2*pi*(1.73*log(maxistc(i,n))-pi));
   nacbase(i,n)=5/3*engelgt;% generic pitot nacelle length
   if nacefin(i,n)<0.0001 & engeloc(i,n)>3
% predict the nacelle length for S and straight ducts only   
      NACELGT(i,n)=(5+nacetyp(i,n)+0.44*qxheavy(insttyp(i,n),3,1)+ ...
                    7/4*qxheavy(engeloc(i,n),4,1))/3*engelgt+ ...
                    onwings(i,n)*ENGECHD(i,n);
   elseif nacefin(i,n)<0.0001 & engeloc(i,n)<4 & insttyp(i,n)<0.0001
      NACELGT(i,n)=nacbase(i,n);% assume original estimate
   elseif nacefin(i,n)<0.0001 & engeloc(i,n)<4 & insttyp(i,n)>2
      NACELGT(i,n)=1.81*nacbase(i,n);% assume original estimate
   else
      NACELGT(i,n)=nacefin(i,n)*nacemdi(i,n);% user defined nacelle
   end
%=====
% check to see if a by-pass ratio emulation is required
   if bypasem(i,n)<0.0001
% predict the by-pass ratio for turbofans -
% sourced from C. Svoboda, Aircraft Design Journal, 3(2000) 17-31
      bypasem(i,n)=3.2+0.01*(maxistc(i,n)*1000/knwlbf)^0.5;
      MAXISTE(i,n)=maxistc(i,n);
      if insttyp(i,n)>0.0
         bypasem(i,n)=3*bypasem(i,n);% equivalent by-pass for props
      end
   else
      if insttyp(i,n)>0.0
         bypasem(i,n)=bypasem(i,n)/3;% assume fan for modelling purposes
      end
      if bypasem(i,n)<3.43
         bypasem(i,n)=3.43;% minimum fan by-pass ratio for emulation
      end
      MAXISTE(i,n)=0.001*knwlbf*(100*(bypasem(i,n)-3.2))^2;
      if insttyp(i,n)>0.0
         bypasem(i,n)=3*bypasem(i,n);% assume fan for modelling purposes
      end
   end
   nacefin(i,n)=NACELGT(i,n)/nacemdi(i,n);% compute the nac. fineness    
end   

return
%=================================
otherwise
  disp([mfilename ': unknown action ''' action ''''])
end
%============================================================================
function [cROSXCF]=fndfscf(hma,vma,vmi,z);
% a shape function correction is introduced for cross-sections having only
% vertical axis-symmetric properties
% set up the easily discernable cross-section points
% note that the distortion coefficient is "fusevmi"
deltahm=1  ;fusetar=hma(z);% starting point for convergence
the1=pi/2  ;rad1=vmi(z)*vma(z);% distorted vertical radius
the3=3*pi/2;rad3=(1-vmi(z))*vma(z);% distorted vertical radius
the5=pi/2  ;rad5=vmi(z)*vma(z);% distorted vertical radius
crosxc0(1:3)=0.0001;% initial estimate for coefficients
for i=1:2
   if vmi(z)<0.5
      the2=3*pi/2-atan(hma(z)/(2*(0.5-vmi(z))*vma(z)));
      the4=3*pi/2+atan(hma(z)/(2*(0.5-vmi(z))*vma(z)));
   elseif vmi(z)>0.5
      the2=pi/2+atan(hma(z)/(2*(vmi(z)-0.5)*vma(z)));
      the4=pi/2-atan(hma(z)/(2*(vmi(z)-0.5)*vma(z)));
   else
      the2=pi;the4=2*pi;
   end   
   rad2=hma(z)/2;% the start point for ray-tracing
   rad4=hma(z)/2;% maximum horizontal radius
   thedat=[the1 the2 the3 the4 the5];% the angular trace matrix
   raddat=[rad1 rad2 rad3 rad4 rad5];% the radii trace matrix
% the computed correlation coefficients for the cross-section
   [cROSXCF,resnorm]=lsqcurvefitjo('qxfours',crosxc0,thedat,raddat,0,2*pi);
% ensure the maximum horizontal width is not violated
   if vmi(z)<0.5
      fusehsa=fix(100*2*sin(atan(hma(z)/(2*(0.5- ...
                  vmi(z))*vma(z))))*(cROSXCF(1)+cROSXCF(2)*sin(the2)+ ...
                  cROSXCF(3)*cos(2*the2)))/100;
   elseif vmi(z)>0.5
      fusehsa=fix(100*2*sin(atan(hma(z)/(2*(vmi(z)- ...
                  0.5)*vma(z))))*(cROSXCF(1)+cROSXCF(2)*sin(the2)+ ...
                  cROSXCF(3)*cos(2*the2)))/100;
   else
      break;% jump out of the loop
   end
   deltahm=fusehsa/fusetar;hma(z)=hma(z)/deltahm;
end
hma(z)=fusetar;% go back to original input hma(z)

return
%============================================================================
