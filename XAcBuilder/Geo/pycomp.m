function [pspnmtx,pthkmtx,pylonxr,pylnlsw,pylninc,pylndih,pdumapx,pdumlgt, ...
    pfintap,pylnwet]=pycomp(engeloc,wingcfg,wi2gcfg,engezlc,xleng, ...
    zleng,xloc,zloc,zval,engchrd,engcthk,sparlc,XSECDWF,wingspn, ...
    wingkln,wingspa,NACELGT,engeylc,calc,rcalc,i)
global kdrang
% This routine generates geometric data used to create surface meshes for
% slung or hanging engine pylons
%=====
if engezlc(i)<0.0
    signc=-1.0;% overwing podded
    pylz=zleng-zloc+max(zval(:,1));
else
    signc=1.0;% underwing podded
    pylz=zleng-zloc-max(zval(:,1));
end
%=====
% Specific to TOLS with WPEBS aircraft design
if wingcfg>0.0001 && wi2gcfg>0.0001
    if engezlc(i)<0.0
        signc=-1.0;% overwing podded
        pylz=zleng-zloc+max(zval(:,1));
        if calc>1
            signc=1.0;% underwing support structure
            pylz=zleng+zloc-max(zval(:,1));
        end
    else
        if calc>1
            signc=-1.0;% overwing support structure
            pylz=zleng+zloc+max(zval(:,1));
        end
    end
end
%=====
% generic computations
pylx=xleng-xloc;
pylnlsw(2)=atan(pylx/pylz)/kdrang;% pylon l.e. sweep angles
pylnlsw(1)=0.0;pylnlsw(3)=pylnlsw(2);
% dummy variables for the function call
pylninc(1:4)=0.0;pylndih(1:3)=0.0;% dummy incidence and dihedral
pdumapx=1.0;pdumlgt=xloc;% dummy location
%=====
pspn1=signc*(max(zval(:,1))-max(zval(:,13)));
pspn3=signc*engcthk*engchrd/2;
pspn2=pylz-pspn3;% height of pylon
pspnmtx=[pspn1 pspn2 pspn3];% pylon span matrix
%=====
if calc<0.0001
    % predict where the aft pylon structure will terminate in the wing
    if sparlc<2.0
        % engines located inboard - use inboard wing spars
        special1=XSECDWF/wingspn;
        special2=wingkln(sparlc);
    elseif sparlc==2.0
        % engines located midboard - use midboard wing spars
        special1=wingkln(sparlc-1);
        special2=wingkln(sparlc);
    elseif sparlc>2.0
        % engines located outboard - use outboard wing spars
        special1=wingkln(sparlc-1);
        special2=1.0;
    end
    % aft spar location for pylon wing segment
    aftspar=(wingspa(sparlc)-wingspa(sparlc+1))/(special1- ...
        special2)*(engeylc(i)-special1)+wingspa(sparlc);
else
    aftspar=0.95;% assume full length of local wing chord
end
pylonxt=(aftspar+0.05)*engchrd;% pylon length at wing
pylonxr=NACELGT(i);% pylon length at nacelle
%=====
if rcalc<0.0001
    paftxdf=pylonxt+xleng-xloc-pylonxr;
    pylonx1=pylonxr+paftxdf*(1-pspn2/(pspn1+pspn2));
    pylonx2=pylonxt+pylx*(1-pspn2/(pspn2+pspn3));
elseif rcalc>0.0001
    paftxdf=pylonxr-xleng+xloc-pylonxt;
    pylonx1=pylonxr-paftxdf*pspn1/(pspn1+pspn2+pspn3);
    pylonx2=pylonxt+paftxdf*(1-(pspn1+pspn2)/(pspn1+pspn2+pspn3));
end
%=====
% assumed pylon thickness matrix
pthkmtx=[pylonxr/pylonx1 pylonxr/pylonx2 pylonxr/pylonxt; ...
    0.08               0.08               0.08];
%=====
% pylon taper ratio matrix
pfintap(1)=pylonx1/pylonxr;pfintap(2)=pylonx2/pylonxr;
pfintap(3)=pylonxt/pylonxr;
%=====
% This routine predicts the wetted area of the pylons
a=2;b=8.5;weta=pylonxr*pspnmtx(2);stapr=pthkmtx(1,1)/pthkmtx(1,2);
wetb=(a+b*pthkmtx(2,2)^2)*(1+stapr);wetc=2/3*b*(pthkmtx(2,2)^2)*(1-1)*(1+2*stapr);
pylnwet=weta*(wetb+wetc);
return