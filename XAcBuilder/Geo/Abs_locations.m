
%=====
global NACELGT ENGECHD PN NACENOX NACENOY NACENOZ WINGAPX WINGAPZ WI2GAPX WI2GAPZ HTAPEXX HTAPEXZ VTAPEXX VTAPEXZ
global l_central width thickness XFAIR ZFAIR l_fore l_aft midfrat forelgt aftslgt Xsponson Zsponson
global fusedvt

global aircraft
n=1;


%% Pylons and nacelles

% % Location of the powerplants.
% Legend for powerplant configuration selection:
% (0) slung in vicinity of the wing 
% (1) on-wing nacelle (2) on-wing integrated with undercarraige
% (3) aft-fuse
% (4) Straight duct   (5) S-duct
% nacetype (0) short-duct (1) long-duct 

% Initialize the Y location of the nacelles: it is 0 by default
yloc=0;

for i=1:PN
   if i==1 & ~aircraft.Engines1.present
      continue
   end
   if i==2 & ~aircraft.Engines2.present
      continue
   end
% not applicable for aft-fuse, S-duct and straight duct installations
   if engeloc(i,n)<3.0
% longitudinal location of engine wing chord leading edge   
      if engeylc(i,n)*wingspn(n)/2<WSPNMTX(1)
% engines located at inboard segment
	 xlenge=wingapx(n)*fuselgt(n)+ ...
		  abs(engeylc(i,n))*wingspn(n)/2*tan(kdrang*winglsw(1,n));
	 zlenge=(2*wingplc(n)-1)*fusevma(n)/2+ ...
		  abs(engeylc(i,n))*wingspn(n)*tan(kdrang*wingdih(1,n))/2;
         sparlc=1.0;% used for pylon drafting
      elseif engeylc(i,n)*wingspn(n)/2>=WSPNMTX(1) & engeylc(i,n)*wingspn(n)/2<WSPNMTX(2)+WSPNMTX(1)
% engines located at midboard segment
	 xlenge=wingapx(n)*fuselgt(n)+ ...
		  WSPNMTX(1)*tan(kdrang*winglsw(1,n))+ ...
                  (abs(engeylc(i,n))*wingspn(n)/2- ...
                  WSPNMTX(1))*tan(kdrang*winglsw(2,n));
         zlenge=(2*wingplc(n)-1)*fusevma(n)/2+ ...
                  WSPNMTX(1)*tan(kdrang*wingdih(1,n))+ ...
                  (abs(engeylc(i,n))*wingspn(n)/2- ...
                  WSPNMTX(1))*tan(kdrang*wingdih(2,n));
         sparlc=2.0;% used for pylon drafting
      elseif engeylc(i,n)*wingspn(n)/2>=WSPNMTX(2)+WSPNMTX(1)
% engines located at outboard segment
         xlenge=wingapx(n)*fuselgt(n)+ ...
                  WSPNMTX(1)*tan(kdrang*winglsw(1,n))+ ...
                  WSPNMTX(2)*tan(kdrang*winglsw(2,n))+ ...
                  (abs(engeylc(i,n))*wingspn(n)/2- ...
                  WSPNMTX(2)-WSPNMTX(1))*tan(kdrang*winglsw(3,n));
         zlenge=(2*wingplc(n)-1)*fusevma(n)/2+ ...
                  WSPNMTX(1)*tan(kdrang*wingdih(1,n))+ ...
                  WSPNMTX(2)*tan(kdrang*wingdih(2,n))+ ...
                  (abs(engeylc(i,n))*wingspn(n)/2- ...
                  WSPNMTX(2)-WSPNMTX(1))*tan(kdrang*wingdih(3,n));
         sparlc=3.0;% used for pylon drafting
      end
   end      
%=====
% generate the placement coordinates for each powerplant option
% all powerplant installations have common y
   yloc=engeylc(i,n)*wingspn(n)/2;
%=================================
% only applicable for slung engines
   if engeloc(i,n)<0.0001   
% distinghuish between long and short ducts for placement on draft
      if nacetyp(i,n)>0.0001
% long duct placement      
	 xloc=xlenge-NACELGT(i,n)+engexlc(i,n)*ENGECHD(i,n);
      else
% short duct placement
	 if nacefcl(i,n)<0.0001
	    nacefcl(i,n)=70.0;% default to fan cowl length 70% of nacelle
         end
         xloc=xlenge-NACELGT(i,n)*nacefcl(i,n)/100+engexlc(i,n)*ENGECHD(i,n);
      end
      if engezlc(i,n)>=0.0   
% nacelle in wing vicinity - local wing chord is used as datum for x & z
	 zloc=zlenge-nacemdi(i,n)/2-engezlc(i,n)*ENGECHD(i,n)- ...
	       qxtkcop(abs(engeylc(i,n))*wingspn(n)/2,WSPNMTX, ...
               WTHKMTX)*ENGECHD(i,n)/2;
         zlok=zloc;
      elseif engezlc(i,n)<0.0
% nacelle above wing - local wing chord is used as datum for x & z
	 zloc=zlenge+nacemdi(i,n)/2+abs(engezlc(i,n))*ENGECHD(i,n)+ ...
	       qxtkcop(abs(engeylc(i,n))*wingspn(n)/2,WSPNMTX, ...
               WTHKMTX)*ENGECHD(i,n)/2;
         zlok=zloc;
      end      
%=================================
% nacelle-wing integrated configurations
   elseif engeloc(i,n)==1 | engeloc(i,n)==2   
% on-wing nacelle - intersection of powerplant with wing geometry
      xloc=xlenge-NACELGT(i,n)+engexlc(i,n)*ENGECHD(i,n);
      if engezlc(i,n)~=1.0 | engezlc(i,n)~=-1.0 
	 engezlc(i,n)=1.0;% default to main nacelle on wing premise
      end
      if engezlc(i,n)==-1.0  
% main nacelle on wing underside
	 zlok=zlenge-qxtkcop(abs(engeylc(i,n))*wingspn(n)/2, ...
	       WSPNMTX,WTHKMTX)*ENGECHD(i,n)/2-nacemdi(i,n)/2;
	 zloc=zlok+qxtkcop(abs(engeylc(i,n))*wingspn(n)/2,WSPNMTX, ...
	       WTHKMTX)*ENGECHD(i,n)+nacemdi(i,n)/2;
      elseif engezlc(i,n)==1.0  
% main nacelle on wing upperside
	 zloc=zlenge+qxtkcop(abs(engeylc(i,n))*wingspn(n)/2, ...
	       WSPNMTX,WTHKMTX)*ENGECHD(i,n)/2+nacemdi(i,n)/2;
	 zlok=zloc-qxtkcop(abs(engeylc(i,n))*wingspn(n)/2,WSPNMTX, ...
	       WTHKMTX)*ENGECHD(i,n)-nacemdi(i,n)/2;
      end          
% aft-fuse mounted on pylons powerplant arrangement
   elseif engeloc(i,n)>2.0
% aft-fuse mounted pylons - x & z depends on relative fuselage measures   
      xloc=engexlc(i,n)*fuselgt(n);
      zloc=engezlc(i,n)*fusevma(n);zlok=zloc;
   end  
   NACENOX(i,n)=xloc;NACENOZ(i,n)=zloc; NACENOY(i,n)=yloc;
end

%% Computes the position of the wing 1 apex
% Longitudinal location
WINGAPX=wingapx(1,n)*fuselgt(1,n);

%Vertical location
WINGAPZ=(2*wingplc(n)-1)*fusevma(n)/2;

   
%% Computes the position of the wing 2 apex
% If there is a second wingm compute its position
if aircraft.Wing2.present
    % Longitudinal location
    WI2GAPX=wi2gapx(1,n)*fuselgt(1,n);

    %Vertical location
    WI2GAPZ=(2*wi2gplc(n)-1)*fusevma(n)/2;
% If there is no second wing just set to the default value 0
else 
    WI2GAPX=0;
    WI2GAPZ=0;
end
%% Position of the horizontal tail apex

  if taillay(n)>0.001
     if taillay(n)<=vtalkln(1,n)
        xincr=taillay(n)/vtalkln(1,n)*VSPNMTX(1)*tan(kdrang*vtallsw(1,n))+ ...
              VCHDROT(n)*vtaltap(1,n)*0.1;
        zincr=taillay(n)/vtalkln(1,n)*VSPNMTX(1);
     elseif taillay(n)>vtalkln(1,n) & taillay(n)<=vtalkln(2,n)  
        xincr=VSPNMTX(1)*tan(kdrang*vtallsw(1,n))+ ...
              (taillay(n)-vtalkln(1,n))/(vtalkln(2,n)- ...
              vtalkln(1,n))*VSPNMTX(2)*tan(kdrang*vtallsw(2,n))+ ...
              VCHDROT(n)*vtaltap(2,n)*0.1;
        zincr=VSPNMTX(1)+(taillay(n)-vtalkln(1,n))/(vtalkln(2,n)- ...
              vtalkln(1,n))*VSPNMTX(2);
     elseif taillay(n)>vtalkln(2,n)   
        xincr=VSPNMTX(1)*tan(kdrang*vtallsw(1,n))+ ...
              VSPNMTX(2)*tan(kdrang*vtallsw(2,n))+ ...
              (taillay(n)-vtalkln(2,n))/(1- ...
              vtalkln(2,n))*VSPNMTX(3)*tan(kdrang*vtallsw(3,n))+ ...
              VCHDROT(n)*vtaltap(3,n)*0.1;
        zincr=VSPNMTX(1)+VSPNMTX(2)+ ...
              (taillay(n)-vtalkln(2,n))/(1- ...
              vtalkln(2,n))*VSPNMTX(3);
     end      
  else
     xincr=0.0;zincr=0.0;% user has selected the horizontal and vertical
  end
  if xincr>0.0
     htalapx(n)=vtalapx(n)+xincr/fuselgt(n);
     htalver(n)=vtalver(n)+zincr/fusevma(n);
  end

  % Longitudinal location
  HTAPEXX=fuselgt(1,n)*htalapx(n);
  % Vertical location
  HTAPEXZ=fusevma(n)*htalver(n);

%% Position of vertical tail apex
% Longitudinal location
VTAPEXX=vtalapx(n)*fuselgt(1,n);

% Vertical location
VTAPEXZ=fusevma(n)*vtalver(n);

%% Position of the fairing
   if fairfwd(n)<0.0001
      fairfwd(n)=50;
   end
   if fairaft(n)<0.0001
      fairaft(n)=50;
   end
%=====
% create wing no. 1 fairing
   if fairovh(n)>0.001
      if fairovh(n)<1.0
         fairovh(n)=1.0;% minimum acceptable value
      end
      % Length of the central elliptical cylinder
      l_central(1,n)=0.4*WCHDROT(n);
      % Width of the ellipse
      width(1,n)=XSECDWF(n);
      % Heigth of the ellipse
      thickness(1,n)=fairovh(n)*WTHKMTX(2,1)*WCHDROT(n)/2;
      % Positioning of the fairing
      XFAIR(1,n)=wingapx(n)*fuselgt(n)+30/100*WCHDROT(n);
      ZFAIR(1,n)=(2*wingplc(n)-1)*fusevma(n)/2+WTHKMTX(2,1)*WCHDROT/2;
      l_fore(1,n)=(30+fairfwd(n))/100*WCHDROT(n);
      l_aft(1,n)=(30+fairaft(n))/100*WCHDROT(n);
   end
   
%=====
   if aircraft.Wing2.present
% create wing no. 2 fairing
      if fa2rfwd(n)<0.0001
         fa2rfwd(n)=50;
      end
      if fa2raft(n)<0.0001
         fa2raft(n)=50;
      end
      if fa2rovh(n)>0.001
         if fa2rovh(n)<1.0
            fa2rovh(n)=1.0;% minimum acceptable value
         end
         % Length of the central elliptical cylinder
       l_central(2,n)=0.4*WC2DROT(n);
      % Width of the ellipse
      width(2,n)=XS2CDWF(n);
      % Heigth of the ellipse
      thickness(2,n)=fa2rovh(n)*WT2KMTX(2,1)*WC2DROT(n)/2;
      % Positioning of the fairing
      XFAIR(2,n)=wi2gapx(n)*fuselgt(n)+30/100*WC2DROT(n);
      ZFAIR(2,n)=(2*wi2gplc(n)-1)*fusevma(n)/2;
                 (2*wingplc(n)-1)*fusevma(n)/2;
      l_fore(2,n)=(30+fa2rfwd(n))/100*WC2DROT(n);
      l_aft(2,n)=(30+fa2raft(n))/100*WC2DROT(n);
      end
   end

   if (midfrat(1,n)==0 | midfrat(1,n)==1)
       midfrat(1,n)=0.5;
   end

%% Sponson
% create the ancillary fairing or sponson
   if spsnxlc(n)>0.001
      if spsnlgt(n)<0.001
         spsnlgt(n)=0.40;% default length
      end
      if spsnxzs(n)<0.001
         spsnxzs(n)=3.50;% default slenderness
      end
      if spsnwid(n)<0.001
         spsnwid(n)=XSECDWF(n)/fusehma(n);% default width
      end
%Positionning of the sponson
      Xsponson(1,n)=spsnxlc(n)*fuselgt(n);
      Zsponson(1,n)=(2*spsnzlc(n)-1)*fusevma(n)/2; 
   end   
 
     
%% Compute the airfoil data for wing 1, wing 2, Horizontal Tail and
%% Vertical Tail

wingAirfoilSpline=wgcomp(aircraft.Wing1.airfoilRoot,WCHDROT,wingtap,WSPNMTX,winglsw,winginc,wingdih,n);

if aircraft.Wing2.present
    wi2gAirfoilSpline=wgcomp(aircraft.Wing2.airfoilRoot,WC2DROT,wi2gtap,WS2NMTX,wi2glsw,wi2ginc,wi2gdih,n);
end

if aircraft.Horizontal_tail.present
    HTAirfoilSpline=wgcomp(aircraft.Horizontal_tail.airfoilRoot,HCHDROT,htaltap,HSPNMTX,htallsw,htalinc,htaldih,n);
end

if aircraft.Canard.present
    CNAirfoilSpline=wgcomp(aircraft.Canard.airfoilRoot,CCHDROT,canrtap,CSPNMTX,canrlsw,canrinc,canrdih,n);
end

if aircraft.Vertical_tail.present
    VTAirfoilSpline=wgcomp(aircraft.Vertical_tail.airfoilRoot,VCHDROT,vtaltap,VSPNMTX,vtallsw,vtalinc,vtaldih,n);
end


%% Powerplants and pylons
% Draw the powerplants.
% The nacelle drawings are based on an analytical geometry method developed
% by Isikveren. A template quadrant is first produced and symmetry is
% subsequently employed for the full figure.
%%=====
clear xval yval zval;
% Legend for powerplant configuration selection:
% (0) slung in vicinity of the wing
% (1) on-wing nacelle
% (2) on-wing integrated with undercarraige
% (3) aft-fuse
% (4) Straight duct
% (5) S-duct
% nacetype (0) short-duct (1) long-duct
%=====
for i=1:PN
    %=====
    % construct the basic nacelle quadrant template used for all powerplant types
    seg=12;
    for k=0:seg
        theta(k+1,1:seg+1)=k/seg*pi/2;
    end
    for j=0:seg
        tparam=pi*j/(2*seg);
        if insttyp(i)>3
            % if a pusher turboprop or propfan selected, reverse the nacelle orientation
            xval(seg+1:1,j+1)=exp(tparam)*sin(tparam)*NACELGT(i)/4.81;
        else
            % generate traditional intake fore and nozzle aft orientation
            xval(1:seg+1,j+1)=exp(tparam)*sin(tparam)*NACELGT(i)/4.81;
        end
        % generate the arcing radial values for y & z coordinate computation
        lrad(1:seg+1,j+1)=(exp(tparam)*cos(tparam)+1)*nacemdi(i)/4.93;
    end
    yval=lrad.*cos(theta);% y coordinates
    zval=lrad.*sin(theta);% z coordinates
    %=================================
    % not applicable for aft-fuse, S-duct and straight duct installations
    if engeloc(i)<3.0
        % longitudinal location of engine wing chord leading edge
        if engeylc(i)*wingspn/2<WSPNMTX(1)
            % engines located at inboard segment
            xlenge=wingapx*fuselgt+ ...
                abs(engeylc(i))*wingspn/2*tan(kdrang*winglsw(1));
            zlenge=(2*wingplc-1)*fusevma/2+ ...
                abs(engeylc(i))*wingspn*tan(kdrang*wingdih(1))/2;
            sparlc=1.0;% used for pylon drafting
        elseif engeylc(i)*wingspn/2>=WSPNMTX(1) && engeylc(i)*wingspn/2<WSPNMTX(2)+WSPNMTX(1)
            % engines located at midboard segment
            xlenge=wingapx*fuselgt+ ...
                WSPNMTX(1)*tan(kdrang*winglsw(1))+ ...
                (abs(engeylc(i))*wingspn/2- ...
                WSPNMTX(1))*tan(kdrang*winglsw(2));
            zlenge=(2*wingplc-1)*fusevma/2+ ...
                WSPNMTX(1)*tan(kdrang*wingdih(1))+ ...
                (abs(engeylc(i))*wingspn/2- ...
                WSPNMTX(1))*tan(kdrang*wingdih(2));
            sparlc=2.0;% used for pylon drafting
        elseif engeylc(i)*wingspn/2>=WSPNMTX(2)+WSPNMTX(1)
            % engines located at outboard segment
            xlenge=wingapx*fuselgt+ ...
                WSPNMTX(1)*tan(kdrang*winglsw(1))+ ...
                WSPNMTX(2)*tan(kdrang*winglsw(2))+ ...
                (abs(engeylc(i))*wingspn/2- ...
                WSPNMTX(2)-WSPNMTX(1))*tan(kdrang*winglsw(3));
            zlenge=(2*wingplc-1)*fusevma/2+ ...
                WSPNMTX(1)*tan(kdrang*wingdih(1))+ ...
                WSPNMTX(2)*tan(kdrang*wingdih(2))+ ...
                (abs(engeylc(i))*wingspn/2- ...
                WSPNMTX(2)-WSPNMTX(1))*tan(kdrang*wingdih(3));
            sparlc=3.0;% used for pylon drafting
        end
    end
    %=====
    % generate the placement coordinates for each powerplant option
    % all powerplant installations have common y
    yloc=engeylc(i)*wingspn/2;
    %=================================
    % only applicable for slung engines
    if engeloc(i)<0.0001
        % distinghuish between long and short ducts for placement on draft
        if nacetyp(i)>0.0001
            % long duct placement
            xloc=xlenge-NACELGT(i)+engexlc(i)*ENGECHD(i);
        else
            % short duct placement
            if nacefcl(i)<0.0001
                nacefcl(i)=70.0;% default to fan cowl length 70% of nacelle
            end
            xloc=xlenge-NACELGT(i)*nacefcl(i)/100+engexlc(i)*ENGECHD(i);
        end
        if engezlc(i)>=0.0
            % nacelle in wing vicinity - local wing chord is used as datum for x & z
            zloc=zlenge-nacemdi(i)/2-engezlc(i)*ENGECHD(i)- ...
                qxtkcop(abs(engeylc(i))*wingspn/2,WSPNMTX, ...
                WTHKMTX)*ENGECHD(i)/2;
            zlok=zloc;
        elseif engezlc(i)<0.0
            % nacelle above wing - local wing chord is used as datum for x & z
            zloc=zlenge+nacemdi(i)/2+abs(engezlc(i))*ENGECHD(i)+ ...
                qxtkcop(abs(engeylc(i))*wingspn/2,WSPNMTX, ...
                WTHKMTX)*ENGECHD(i)/2;
            zlok=zloc;
        end
        %=================================
        % nacelle-wing integrated configurations
    elseif engeloc(i)==1 || engeloc(i)==2
        % on-wing nacelle - intersection of powerplant with wing geometry
        xloc=xlenge-NACELGT(i)+engexlc(i)*ENGECHD(i);
        if engezlc(i)~=1.0 || engezlc(i)~=-1.0
            engezlc(i)=1.0;% default to main nacelle on wing premise
        end
        if engezlc(i)==-1.0
            % main nacelle on wing underside
            zlok=zlenge-qxtkcop(abs(engeylc(i))*wingspn/2, ...
                WSPNMTX,WTHKMTX)*ENGECHD(i)/2-nacemdi(i)/2;
            zloc=zlok+qxtkcop(abs(engeylc(i))*wingspn/2,WSPNMTX, ...
                WTHKMTX)*ENGECHD(i)+nacemdi(i)/2;
        elseif engezlc(i)==1.0
            % main nacelle on wing upperside
            zloc=zlenge+qxtkcop(abs(engeylc(i))*wingspn/2, ...
                WSPNMTX,WTHKMTX)*ENGECHD(i)/2+nacemdi(i)/2;
            zlok=zloc-qxtkcop(abs(engeylc(i))*wingspn/2,WSPNMTX, ...
                WTHKMTX)*ENGECHD(i)-nacemdi(i)/2;
        end
        if wingcfg~=-2 || wi2gcfg~=-2
            % special side flat panels for on-wing powerplants
            xvaf(1,:)=xval(1,:);xvaf(2,:)=xval(1,:);
            yvaf(1,:)=yval(1,:);yvaf(2,:)=yval(1,:);
            zvaf(1,:)=zval(1,:);
            zvaf(2,:)=zvaf(1,:)+qxtkcop(abs(engeylc(i))*wingspn/2, ...
                WSPNMTX,WTHKMTX)*ENGECHD(i)/2+nacemdi(i)/2;
            % special horizontal flat panels for on-wing powerplants
            yvag(1,:)=yval(1,:);yvag(2,:)=-yvag(1,:);zvag=zeros(2,13);

        end

    elseif engeloc(i)>2.0
        % aft-fuse mounted pylons - x & z depends on relative fuselage measures
        xloc=engexlc(i)*fuselgt;
        zloc=engezlc(i)*fusevma;zlok=zloc;zcorr=zloc;
    end

    NACENOX(i)=xloc;NACENOZ(i)=zloc; NACENOY(i,n)=yloc;
    %=================================
    % Draw the pylons for (0) slung under(over)wing and
    %                     (3) aft-fuse mounted on pylons
    %=====
    % pylon draft assuming no TOLS with WPEBS (see Isikveren)
    if engeloc(i)<0.0001 && wingcfg<0.0001 && wi2gcfg<0.0001
        engcthk=qxtkcop(abs(engeylc(i))*wingspn/2,WSPNMTX,WTHKMTX);
        % call a special routine that generates all input pylon data for drafting
        [pspnmtx,pthkmtx,pylonxr,pylnlsw,pylninc,pylndih,pdumapx, ...
            pdumlgt,pfintap,pylnwet]=pycomp(engeloc,wingcfg,wi2gcfg,engezlc, ...
            xlenge,zlenge,xloc,zloc,zval,ENGECHD(i),engcthk,sparlc, ...
            XSECDWF,wingspn,wingkln,wingspa,NACELGT,engeylc,0,0,i);

        % draft the powerplant pylons in starboard
        ycorr=-engeylc(i)*wingspn/2;% place the object laterally
        if engezlc(i)<0.0
            signc=-1.0;% overwing podded
        else
            signc=1.0;% underwing podded
        end
        % place the object vertically
        zcorr=zloc+signc*max(zval(:,1))-pspnmtx(1);

        PYLNCLW=pylnwet*engenum(i);% pylon wetted area
        % increment the total powerplant wetted area and re-evaluate the total wetted
        % area for aircraft.
        %          PWRPWET=PWRPWET+PYLNWET;TOTLWET=TOTLWET+PWRPWET;
        %=================================
        % pylon draft assuming TOLS with WPEBS (see Isikveren)
    elseif engeloc(i)<0.0001 && wingcfg>0.0001 && wi2gcfg>0.0001
        % starboard lower pylon
        engcthk=qxtkcop(abs(engeylc(i))*wingspn/2,WSPNMTX,WTHKMTX);
        % call a special routine that generates all input pylon data for drafting
        [pspnmtx,pthkmtx,pylonxr,pylnlsw,pylninc,pylndih,pdumapx, ...
            pdumlgt,pfintap,pylnwe1]=pycomp(engeloc,wingcfg,wi2gcfg,engezlc, ...
            xlenge,zlenge,xloc,zloc,zval,ENGECHD(i),engcthk,sparlc, ...
            XSECDWF,wingspn,wingkln,wingspa,NACELGT,engeylc,1,0,i);

        ycorr=engeylc(i)*wingspn/2;% place the object laterally
        zcorr=zloc-max(zval(:,1))-pspnmtx(1);% place the object vertically
        zwraw=zwraw+ycorr;ywraw=ywraw+zcorr;% geo placement corrections

        % port lower pylon
        xlengs=2*wingapx*fuselgt-xlenge;
        [pspnmtx,pthkmtx,pylonxr,pylnlsw,pylninc,pylndih,pdumapx, ...
            pdumlgt,pfintap,pylnwe2]=pycomp(engeloc,wingcfg,wi2gcfg,engezlc, ...
            xlengs,zlenge,xloc,zloc,zval,ENGECHD(i),engcthk,sparlc, ...
            XSECDWF,wingspn,wingkln,wingspa,NACELGT,engeylc,1,1,i);

        ycorr=-engeylc(i)*wingspn/2;                % place the object laterally
        zcorr=zloc-max(zval(:,1))-pspnmtx(1);       % place the object vertically
        %             zwraw=zwraw+ycorr;ywraw=ywraw+zcorr;        % geo placement corrections
        %             surfmat(61+(i-1)*8)=surf(xwraw,zwraw,ywraw);% construct the pylon
        %=====
        if engeylc(i)*wingspn/2<WS2NMTX(1)
            % engines located at inboard segment
            xl2nge=wi2gapx*fuselgt+ ...
                abs(engeylc(i))*wingspn/2*tan(kdrang*wi2glsw(1));
            zl2nge=((2*wi2gplc-1)*fusevma/2+ ...
                abs(engeylc(i))*wingspn*tan(kdrang*wi2gdih(1))/2);
        elseif engeylc(i)*wingspn/2>=WS2NMTX(1) && engeylc(i)*wingspn/2<WS2NMTX(2)+WS2NMTX(1)
            % engines located at midboard segment
            xl2nge=wi2gapx*fuselgt+ ...
                WS2NMTX(1)*tan(kdrang*wi2glsw(1))+ ...
                (abs(engeylc(i))*wingspn/2- ...
                WS2NMTX(1))*tan(kdrang*wi2glsw(2));
            zl2nge=((2*wi2gplc-1)*fusevma/2+ ...
                abs(engeylc(i))*wingspn*tan(kdrang*wi2gdih(1))/2)+ ...
                (abs(engeylc(i))*wingspn/2- ...
                WS2NMTX(1))*tan(kdrang*wi2gdih(2));
        elseif engeylc(i)*wingspn/2>=WS2NMTX(2)+WS2NMTX(1)
            % engines located at outboard segment
            xl2nge=wi2gapx*fuselgt+ ...
                WS2NMTX(1)*tan(kdrang*wi2glsw(1))+ ...
                WS2NMTX(2)*tan(kdrang*wi2glsw(2))+ ...
                (abs(engeylc(i))*wingspn/2- ...
                WS2NMTX(2)-WS2NMTX(1))*tan(kdrang*wi2glsw(3));
            zl2nge=((2*wi2gplc-1)*fusevma/2+ ...
                abs(engeylc(i))*wingspn*tan(kdrang*wi2gdih(1))/2)+ ...
                WS2NMTX(2)*tan(kdrang*wi2gdih(2))+ ...
                (abs(engeylc(i))*wingspn/2- ...
                WS2NMTX(2)-WS2NMTX(1))*tan(kdrang*wi2gdih(3));
        end
        % %=====
        % starboard upper pylon
        EN2ECHD(i)=qxcdcop(abs(engeylc(i))*wingspn/2,WS2NMTX, ...
            WC2DROT,wi2gtap);
        engcthk=qxtkcop(abs(engeylc(i))*wingspn/2,WS2NMTX,WT2KMTX);
        xl2ngs=2*wi2gapx*fuselgt-xl2nge;
        % call a special routine that generates all input pylon data for drafting
        [pspnmtx,pthkmtx,pylonxr,pylnlsw,pylninc,pylndih,pdumapx, ...
            pdumlgt,pfintap,pylnwe3]=pycomp(engeloc,wingcfg,wi2gcfg,engezlc, ...
            xl2ngs,zl2nge,xloc,zloc,zval,EN2ECHD(i),engcthk,sparlc, ...
            XSECDWF,wingspn,wingkln,wingspa,NACELGT,engeylc,2,0,i);

        ycorr=-engeylc(i)*wingspn/2;% place the object laterally
        zcorr=zloc+max(zval(:,1))-pspnmtx(1);% place the object vertically
        zwraw=zwraw+ycorr;ywraw=ywraw+zcorr;% geo placement corrections
        %             surfmat(63+(i-1)*8)=surf(xwraw,zwraw,ywraw);% construct the pylon
        %=====
        % port upper pylon
        % call a special routine that generates all input pylon data for drafting
        [pspnmtx,pthkmtx,pylonxr,pylnlsw,pylninc,pylndih,pdumapx, ...
            pdumlgt,pfintap,pylnwe4]=pycomp(engeloc,wingcfg,wi2gcfg,engezlc, ...
            xl2nge,zl2nge,xloc,zloc,zval,EN2ECHD(i),engcthk,sparlc, ...
            XSECDWF,wingspn,wingkln,wingspa,NACELGT,engeylc,2,1,i);

        ycorr=engeylc(i)*wingspn/2;% place the object laterally
        zcorr=zloc+max(zval(:,1))-pspnmtx(1);% place the object vertically
        zwraw=zwraw+ycorr;ywraw=ywraw+zcorr;% geo placement corrections

        %=====
        PYLNCLW=pylnwe1+pylnwe2+pylnwe3+pylnwe4;% pylon wetted area
        % % increment the total powerplant wetted area and re-evaluate the total wetted
        % % area for aircraft.
        PWRPWET=PWRPWET+PYLNWET;TOTLWET=TOTLWET+PWRPWET;
        %=================================
    elseif engeloc(i)==3.0 || engeloc(i)==4.0
        % dummy variables for the function call
        pylninc(1:4)=0.0;% dummy incidence of zero
        if engeloc(i)==4.0
            pylndih(1:3)=0.0;% no dihedral for straight duct
            pspn1=max(yval(:,1))-max(yval(:,13));
            pspn2=0.5*(abs(engezlc(i))*fusevma-pspn1-max(yval(:,13)));
        else
            pylndih(1:3)=-5.0;% dummy dihedral of 5 deg.
            pspn1=max(zval(:,1))-max(zval(:,13));
            pspn2=0.5*(abs(engeylc(i))*wingspn/2-pspn1-max(zval(:,13)));
        end
        pdumapx=1.0;% dummy location
        %=====
        % construct the thickness matrix
        pthkmtx=[1.00 1.00 1.00; ...
            0.08 0.08 0.08];% assumed pylon thickness matrix
        %=====
        % construct the span matrix
        pspn3=pspn2;
        pspnmtx=[pspn1 pspn2 pspn3];% pylon span matrix
        %=====
        % pylon taper ratio matrix
        if insttyp(i)<3
            pylonxr=NACELGT(i);
            pfintap(1)=1.0;pfintap(2)=1.14;pfintap(3)=1.28;
            pylnlsw(1)=0.0;pdumlgt=xloc;
        else
            % set values specific to propfan power plants
            pylonxr=0.39*NACELGT(i);
            pfintap(1)=1.0;pfintap(2)=1.0;pfintap(3)=1.0;
            pylnlsw(1)=-15.0;pdumlgt=xloc+0.2*NACELGT(i);
        end
        pylnlsw(2)=pylnlsw(1);pylnlsw(3)=pylnlsw(2);

        % draft the powerplant pylons in starboard
        % place the object vertically and laterally
        if engeloc(i)==4.0
            ycorr=-engezlc(i)*fusevma+max(yval(:,13));
            zloc=0;
        else
            ycorr=-engeylc(i)*wingspn/2+max(zval(:,13));
            zloc=zloc-engeylc(i)*wingspn/2*tan(pylndih(1)*kdrang)- ...
                pthkmtx(1,1)*pthkmtx(2,1)*pylonxr/2;
        end
        % This routine predicts the wetted area of the pylons
        weta=pylonxr*pspnmtx(2);wetb=2+8.5*pthkmtx(2,2)^2;
        stapr=pfintap(2)/pfintap(1);
        wetc=0.0;
        pylnwet=weta*(wetb+wetc);
        PYLNCLW=pylnwet*engenum(i);% pylon wetted area
        % increment the total powerplant wetted area and re-evaluate the total wetted
        % area for aircraft.
        PWRPWET=PWRPWET+PYLNWET;TOTLWET=TOTLWET+PWRPWET;
    end

    if engeloc(i)==3.0
        PYLROTX(i,n)=180;
    elseif engeloc(i)==0.0
        PYLROTX(i,n)=-signc*90;
    end


    if (engeloc(i,n)==0 || engeloc(i,n)==3)

        if engeloc(i,n)==0
            tiltcorr=engepit(i,n);
            %% tiltcorr is a correction factor to the root chord and X
            %% locale of the nacelle. tiltcorr ensures that the pylon does
            %% not pierce through the inlet or outlet of the nacelle.
        else
            tiltcorr=engetoe(i,n);
        end

        pylonxr=pylonxr*abs(cos(tiltcorr*pi/180));
        PYLRCH(i,n)=((1-pfintap(1))*abs(NACENOZ(i)-zcorr)/(pspnmtx(1)*0.95) + 1)*pylonxr*0.95;
        PYLTAP(i,:)=0.95*pylonxr/PYLRCH(i,n).*[pfintap(1);pfintap(2);pfintap(3)];
        PYLLESW(i,:)=[pylnlsw(1);pylnlsw(2);pylnlsw(3)];
        PYSPMTRX(i,:)=[pspnmtx(1)*0.95+abs(NACENOZ(i)-zcorr);pspnmtx(2);pspnmtx(3)];
        PYLLOCX(i,n)=xloc+(pspnmtx(1)+abs(NACENOZ(i)-zcorr))*tan(tiltcorr*pi/180);
        PYLLOCY(i,n)=-ycorr;
        PYLLOCZ(i,n)=NACENOZ(i);
    end

end;% the powerplants are finished

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Further precisions for strutural analysis

if (vtalapx(1,n)*fuselgt(1,n)+VCHDROT(1,n)/4>0 && vtalapx(1,n)*fuselgt(1,n)+VCHDROT(1,n)/4<fuselgt(1,n))
    if (vtalapx(1,n)*fuselgt(1,n)+VCHDROT(1,n)/4<forefin(1,n)*fusfvma(1,n))
        beta=0.54+0.1*tan(foreslp(1,n)-foredsw(1,n));
        fusedvt(1,n)=2*fusfvma(1,n)^(1-beta)/2+((vtalapx(1,n)*fuselgt(1,n)+VCHDROT(1,n)/4)/forefin(1,n))^beta;
    elseif (vtalapx(1,n)*fuselgt(1,n)+VCHDROT(1,n)/4>forefin(1,n)*fusfvma(1,n) && vtalapx(1,n)*fuselgt(1,n)+VCHDROT(1,n)/4<midfrat(1,n)*(fuselgt(1,n)-forefin(1,n)*fusfvma(1,n)-aftsfin(1,n)*fusevma(1,n))+forefin(1,n)*fusfvma(1,n))
        xquarterVT=(vtalapx(1,n)*fuselgt(1,n)+VCHDROT(1,n)/4 - forefin(1,n)*fusfvma(1,n))/(midfrat(1,n)*(fuselgt(1,n)-forefin(1,n)*fusfvma(1,n)-aftsfin(1,n)*fusevma(1,n)));
        fusedvt(1,n)=fusfvma(1,n)*(1-xquarterVT)+fusevma(1,n)*xquarterVT;
    elseif (vtalapx(1,n)*fuselgt(1,n)+VCHDROT(1,n)/4>fuselgt(1,n)-aftsfin(1,n)*fusevma(1,n))
        beta=0.54+0.1*tan(btallow(1,n)-aftfusw(1,n));
        fusedvt(1,n)=2*fusevma(1,n)^(1-beta)/2+((fuselgt(1,n)-vtalapx(1,n)*fuselgt(1,n)-VCHDROT(1,n)/4)/aftsfin(1,n))^beta;
    else
        fusedvt(1,n)=fusevma(1,n);
    end
else 
    fusedvt(1,n)=0;
end
