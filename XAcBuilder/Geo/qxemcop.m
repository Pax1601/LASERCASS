function [kln,spn,gar,smtrx,crot,xscd,tap,cref,chrf,tref,chwf,troot, ...
          tmtrx,qsw,lsw,netwgae,wsatref,wsarref,wsasref,wsarlsw, ...
          wsarqsw,wsarhsw,wsarmac,wsarybr,wsarthb,inc]=qxemcop(kln,tap, ...
          spn,are,gar,hma,thk,qsw,lsw,refc,loc,inc,z)
global kdrang                                         
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

if kln(1,z)<0.0001 || kln(1,z)>0.9999
   kln(1,z)=0.35;% assume a ghost kink 1 at 0.01 semi-span
   tap(1,z)=0; % set the taper to 0 so that it will be automatically recomputed
   if kln(2,z)<0.0001  || kln(2,z)>0.9999
       inc(2,z)=0.01/0.99*inc(3,z)+(kln(2,z)-0.01)/0.99*inc(1,z); % The next if loop will set the spanwise position of kink2 to 0.99
   else
       inc(2,z)=0.01/kln(2,z)*inc(3,z)+(kln(2,z)-0.01)/kln(2,z)*inc(1,z);
   end
end
if kln(2,z)<0.0001  || kln(2,z)>0.9999
   kln(2,z)=0.75;% assume a ghost kink 2 at 0.99 semi-span
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
   xscd=(abs(4*loc(z)*(1-loc(z))*(hma(z)^2)+ ...
         cref*thk(1,z)*(6*(1-2*loc(z))*hma(z)- ...
         9*cref*thk(1,z))))^0.5;% compute the x-sect chord at fuse-wing
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
% S.R. 261212: added abs(lsw(1,z) to include the case of negative swept angle
%
if abs(lsw(1,z))<0.0001
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

return
