% input:
% PANE: caero1 vertex
% NSPAN: matrix with number of span elements 
% NCHORD: matrix with number of  chord elements
% CFRAC: chord fraction for control
% SFRAC: span fraction for control
% POS: position flag
% LABEL: control name
% output
% SPLINE_SET: matrix to be used in SPLINE card ID1-ID2
% output:
% MESH_DATA: matrix with user inputs
function MESH_DATA = aero_model(PANE, CFRAC, SFRAC, POS, LABEL, ribs_data, nrib_span, meshinput)
fid = 1;
MESH_DATA = [];
OFFSET = 1999999;
PAERO = 2000;
SURF = {};
SPLINE_SET = {};
vers_ae = ribs_data.vers_ae;
Xcross = crossm([1 0 0]);
naer = size(PANE,2) / 4;
ok = 0;
ninp = 0;
while ok==0

%-------------------------------------------------------------------------------
  fp = fopen('caero.dat','w');
  fprintf(fp,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
  fprintf(fp,'PAERO1  %8d\n', PAERO);
%-------------------------------------------------------------------------------
  count = 0;
  ns = size(vers_ae,1);
  offsetr = nrib_span(1) +2;
  cont = 0;
  for i=1:naer
    fprintf(fp,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
    if i>1
      r1 = offsetr;
      r2 = offsetr + nrib_span(i) +1;
      offsetr = r2;
    %
      P1 = ribs_data.C(r1).coord';
      P2 = ribs_data.C(r2).coord';
      Y = P2 - P1;    
      Y = Y./norm(Y);
      Z = Xcross * Y; 
      Z = Z ./ norm(Z); 
      P2 = P1 + Z;
      X = crossm(Y) * Z; 
      X = X ./ norm(X); 
      P3 = P1 + X;
    %
      cord2r(fp,i+2000,P1,P2,P3); 
      SET = zeros(r2-r1,1); 
      offset = 0;
      for k=r1:r2
        offset = offset+1;
        SET(offset) = ribs_data.C(k).grid + OFFSET;
      end
  %
      nas_list(fp, 'SET1    ', i, SET)
  %-------------------------------------------------------------------------------
    end

    offset = 4*(i-1) +1;
    X1 = PANE(:,offset);
    X2 = PANE(:,offset+1);
    X3 = PANE(:,offset+2);
    X4 = PANE(:,offset+3);
    CHORD(1) = norm(X4-X1);
    CHORD(2) = norm(X3-X2);
    CONTR = 1;
  %
    fprintf(fid,'\nPatch %d:\n', i);
    if SFRAC(i) == 1
      % add one CAERO
      if  CFRAC(i,1)==0 && CFRAC(i,2)==0
         ninp = ninp +1;
        if isempty(meshinput)
          NSPAN  = input('        Number of spanwise elements: ');
          NCHORD = input('        Number of chordwise elements: ');
        else
          NSPAN  = meshinput(ninp,1);
          NCHORD = meshinput(ninp,2); 
        end
        count = count+1;
        MESH_DATA(ninp, :) = [NSPAN, NCHORD, 0, 0, 0];
        IDP = count*1000+1;
        nas_caero1(fp, IDP, PAERO, 0, NSPAN, NCHORD, X1, X2, CHORD);
        plot_caero(X1,X2,CHORD,NSPAN,NCHORD);
        if i>1
          nas_spline2(fp, count, IDP, IDP, IDP+NSPAN*NCHORD-1,i,i+2000);
        end
      else   
      % add two CAEROs
        ninp = ninp +1;
        if isempty(meshinput)
          NSPAN   = input('        Number of spanwise elements: ');
          NCHORD  = input('        Number of chordwise elements: ');
          NCCHORD = input('        Number of chordwise elements along control: ');
        else
          NSPAN  = meshinput(ninp,1);
          NCHORD = meshinput(ninp,2); 
          NCCHORD = meshinput(ninp,3); 
        end
%
        P1 = (X4-X1).*(1-CFRAC(i,1)) + X1;
        P2 = (X3-X2).*(1-CFRAC(i,2)) + X2;
        C1 = norm(P1-X1);
        C2 = norm(P2-X2);
        C3 = norm(P1-X4);
        C4 = norm(P2-X3);
        count = count+1;
        MESH_DATA(ninp, :) = [NSPAN, NCHORD, NCCHORD, 0, 0];
        IDP = count*1000+1;
        nas_caero1(fp, IDP, PAERO, 0, NSPAN, NCHORD, X1, X2, [C1, C2]);
        plot_caero(X1,X2,[C1, C2],NSPAN,NCHORD);
        if i>1
          nas_spline2(fp, count, IDP, IDP, IDP+NSPAN*NCHORD-1,i,i+2000);
        end
        count = count+1;
        IDP = count*1000+1;
        nas_caero1(fp, IDP, PAERO, 0, NSPAN, NCCHORD, P1, P2, [C3, C4]);
        plot_caero(P1,P2,[C3, C4],NSPAN,NCCHORD);
        nas_list(fp, 'AELIST  ', count, [IDP:IDP+NSPAN*NCCHORD-1]);
        Y = P2 - P1; Y = Y ./ norm(Y); Z = Xcross * Y; Z = Z ./norm(Z);
        X = crossm(Y) * Z; X = X ./ norm(X);
        cord2r(fp,IDP,P1,P1+Z,P1+X); 
        nas_aesurf(fp, IDP, LABEL(i,:), IDP, count);
        if i>1
          nas_spline2(fp, count, IDP, IDP, IDP+NSPAN*NCCHORD-1,i,i+2000);
        end

      end

    else % split required

      switch(POS(i))


      case 0 % left
        ninp = ninp +1;
        if isempty(meshinput)
          NSPAN    = input('        Number of spanwise elements: ');
          NCHORD   = input('        Number of chordwise elements: ');
          NCCHORD  = input('        Number of chordwise elements along control: ');
          NSPAN2   = input('        Number of spanwise elements along neighbouring patch: ');
          NCHORD2  = input('        Number of chordwise elements along neighbouring patch: ');
        else
          NSPAN  = meshinput(ninp,1);
          NCHORD = meshinput(ninp,2); 
          NCCHORD = meshinput(ninp,3); 
          NSPAN2  = meshinput(ninp,4);
          NCHORD2 = meshinput(ninp,5); 
        end
        [X2c, X3c] = cut_patch(X1,X2,X3,X4,SFRAC(i));
        count = count+1;
        MESH_DATA(ninp, :) = [NSPAN, NCHORD, NCCHORD, NSPAN2, NCHORD2];
        IDP = count*1000+1;
        P1 = (X4-X1).*(1-CFRAC(i,1)) + X1;
        P2 = (X3c-X2c).*(1-CFRAC(i,2)) + X2c;
        C1 = norm(P1-X1);
        C2 = norm(P2-X2c);
        C3 = norm(P1-X4);
        C4 = norm(P2-X3c);

        nas_caero1(fp, IDP, PAERO, 0, NSPAN, NCHORD, X1, X2c, [C1, C2]);
        plot_caero(X1,X2c,[C1, C2],NSPAN,NCHORD);
        if i>1
          nas_spline2(fp, count, IDP, IDP, IDP+NSPAN*NCHORD-1,i,i+2000);
        end

        count = count+1;
        IDP = count*1000+1;
        nas_caero1(fp, IDP, PAERO, 0, NSPAN2, NCHORD2, X2c, X2, [norm(X3c-X2c), CHORD(2)]);
        plot_caero(X2c,X2,[norm(X3c-X2c), CHORD(2)],NSPAN2,NCCHORD2);
        if i>1
          nas_spline2(fp, count, IDP, IDP, IDP+NSPAN2*NCHORD2-1,i,i+2000);
        end

        count = count+1;
        IDP = count*1000+1;
        nas_caero1(fp, IDP, PAERO, 0, NSPAN, NCCHORD, P1, P2, [C3, C4]);
        plot_caero(P1,P2,[C3, C4],NSPAN,NCCHORD);
        nas_list(fp, 'AELIST  ', count, [IDP:IDP+NSPAN*NCCHORD-1]);
        Y = P2 - P1; Y = Y ./ norm(Y); Z = Xcross * Y; Z = Z ./norm(Z);
        X = crossm(Y) * Z; X = X ./ norm(X);
        cord2r(fp,IDP,P1,P1+Z,P1+X); 
        nas_aesurf(fp, IDP, LABEL(i,:), IDP, count);
        if i>1
          nas_spline2(fp, count, IDP, IDP, IDP+NSPAN*NCCHORD-1,i,i+2000);
        end

      case 1 % right
        ninp = ninp +1;
        if isempty(meshinput)
          NSPAN    = input('        Number of spanwise elements: ');
          NCHORD   = input('        Number of chordwise elements: ');
          NCCHORD  = input('        Number of chordwise elements along control: ');
          NSPAN2   = input('        Number of spanwise elements along neighbouring patch: ');
          NCHORD2  = input('        Number of chordwise elements along neighbouring patch: ');
        else
          NSPAN  = meshinput(ninp,1);
          NCHORD = meshinput(ninp,2); 
          NCCHORD = meshinput(ninp,3); 
          NSPAN2  = meshinput(ninp,4);
          NCHORD2 = meshinput(ninp,5); 
        end
       [X2c, X3c] = cut_patch(X1,X2,X3,X4,1-SFRAC(i));
        count = count+1;
        MESH_DATA(ninp, :)  = [NSPAN, NCHORD, NCCHORD, NSPAN2, NCHORD2];
        IDP = count*1000+1;
        P1 = (X3c-X2c).*(1-CFRAC(i,1)) + X2c;
        P2 = (X3-X2).*(1-CFRAC(i,2)) + X2;
        C1 = norm(P1-X2c);
        C2 = norm(P2-X2);
        C3 = norm(P1-X3c);
        C4 = norm(P2-X3);
        nas_caero1(fp, IDP, PAERO, 0, NSPAN2, NCHORD2, X1, X2c, [CHORD(1), norm(X3c-X2c)]);
        plot_caero(X1,X2c,[CHORD(1), norm(X3c-X2c)],NSPAN2,NCHORD2);
        if i>1
          nas_spline2(fp, count, IDP, IDP, IDP+NSPAN2*NCHORD2-1,i,i+2000);
        end
        count = count+1;
        IDP = count*1000+1;
        nas_caero1(fp, IDP, PAERO, 0, NSPAN, NCHORD, X2c, X2, [C1, C2]);
        plot_caero(X2c,X2,[C1, C2],NSPAN,NCCHORD);
        if i>1
          nas_spline2(fp, count, IDP, IDP, IDP+NSPAN*NCHORD-1,i,i+2000);
        end
        count = count+1;
        IDP = count*1000+1;
        nas_caero1(fp, IDP, PAERO, 0, NSPAN, NCCHORD, P1, P2, [C3, C4]);
        plot_caero(P1,P2,[C3, C4],NSPAN,NCCHORD);
        nas_list(fp, 'AELIST  ', count, [IDP:IDP+NSPAN*NCCHORD-1]);
        Y = P2 - P1; Y = Y ./ norm(Y); Z = Xcross * Y; Z = Z ./norm(Z);
        X = crossm(Y) * Z; X = X ./ norm(X);
        cord2r(fp,IDP,P1,P1+Z,P1+X); 
        nas_aesurf(fp, IDP, LABEL(i,:), IDP, count);
        if i>1
          nas_spline2(fp, count, IDP, IDP, IDP+NSPAN*NCCHORD-1,i,i+2000);
        end

      case 2 % centered
        ninp = ninp +1;
        if isempty(meshinput)
          NSPAN    = input('        Number of spanwise elements: ');
          NCHORD   = input('        Number of chordwise elements: ');
          NCCHORD  = input('        Number of chordwise elements along control: ');
          NSPAN2   = input('        Number of spanwise elements along neighbouring patch: ');
          NCHORD2  = input('        Number of chordwise elements along neighbouring patch: ');
        else
          NSPAN  = meshinput(ninp,1);
          NCHORD = meshinput(ninp,2); 
          NCCHORD = meshinput(ninp,3); 
          NSPAN2  = meshinput(ninp,4);
          NCHORD2 = meshinput(ninp,5); 
        end
        [X2c, X3c] = cut_patch(X1,X2,X3,X4,(1-SFRAC(i))/2);
        count = count+1;
        MESH_DATA(ninp, :) = [NSPAN, NCHORD, NCCHORD, NSPAN2, NCHORD2];
        IDP = count*1000+1;
        nas_caero1(fp, IDP, PAERO, 0, NSPAN2, NCHORD2, X1, X2c, [CHORD(1), norm(X3c-X2c)]);
        plot_caero(X1,X2c,[CHORD(1), norm(X3c-X2c)],NSPAN2,NCHORD2);
        if i>1
          nas_spline2(fp, count, IDP, IDP, IDP+NSPAN2*NCHORD2-1,i,i+2000);
        end

        [X2cc, X3cc] = cut_patch(X1,X2,X3,X4,SFRAC(i)+(1-SFRAC(i))/2);
        count = count+1;
        IDP = count*1000+1;
        P1 = (X3c-X2c).*(1-CFRAC(i,1)) + X2c;
        P2 = (X3cc-X2cc).*(1-CFRAC(i,2)) + X2cc;
        C1 = norm(P1-X2c);
        C2 = norm(P2-X2cc);
        C3 = norm(P1-X3c);
        C4 = norm(P2-X3cc);

        nas_caero1(fp, IDP, PAERO, 0, NSPAN, NCHORD, X2c, X2cc, [C1, C2]);
        plot_caero(X2c,X2cc,[C1, C2],NSPAN,NCHORD);
        if i>1
          nas_spline2(fp, count, IDP, IDP, IDP+NSPAN*NCHORD-1,i,i+2000);
        end

        count = count+1;
        IDP = count*1000+1;
        nas_caero1(fp, IDP, PAERO, 0, NSPAN, NCCHORD, P1, P2, [C3, C4]);
        plot_caero(X2c,X2cc,[C1, C2],NSPAN,NCCHORD);
        nas_list(fp, 'AELIST  ', count, [IDP:IDP+NSPAN*NCCHORD-1]);
        Y = P2 - P1; Y = Y ./ norm(Y); Z = Xcross * Y; Z = Z ./norm(Z);
        X = crossm(Y) * Z; X = X ./ norm(X);
        cord2r(fp,IDP,P1,P1+Z,P1+X); 
        nas_aesurf(fp, IDP, LABEL(i,:), IDP, count);
        if i>1
          nas_spline2(fp, count, IDP, IDP, IDP+NSPAN*NCCHORD-1,i,i+2000);
        end

        count = count+1;
        IDP = count*1000+1;
        nas_caero1(fp, IDP, PAERO, 0, NSPAN2, NCHORD2, X2cc, X2, [norm(C3cc-C2cc), CHORD(2)]);
        plot_caero(X2cc,X2,[norm(C3cc-C2cc), CHORD(2)],NSPAN2,NCHORD2);
        if i>1
          nas_spline2(fp, count, IDP, IDP, IDP+NSPAN2*NCHORD2-1,i,i+2000);
        end

      end

    end

  end

  fclose(fp);
  ok = 1;
  if isempty(meshinput)
   ok = input('OK to proceed? Yes = 1, No = 0: ');
   if ok~=1
    ok = 0;
   end
  end 
%
 if (ok==0)
    figure(1); close; 
    open('wing_fem.fig'); view(2); hold on; axis equal;
  end
end
end
%-------------------------------------------------------------------------------
function nas_caero1(fp, EID, PID, CP, NSPAN, NCHORD, X1, X2, CHORD)

  fprintf(fp,'%8s%8d%8d%8d%8d%8d                1\n','CAERO1  ',...
    EID,PID,CP,NSPAN,NCHORD);
  fprintf(fp,'        %8.4f%8.4f%8.4f%8.4f%8.4f%8.4f%8.4f%8.4f\n', ...
            X1(1), X1(2), X1(3), CHORD(1), ...
            X2(1), X2(2), X2(3), CHORD(2));

end
%-------------------------------------------------------------------------------
function nas_aesurf(fp, EID, LABEL, CID, LIST)
  ns =  length(LABEL);
  if ns>8 
    LABEL = LABEL(1:8);
  else
    LABEL = [LABEL, blanks(ns-8)];
  end

  fprintf(fp,'%8s%8d%8s%8d%8d\n','AESURF  ',EID,LABEL,CID,LIST);

end

%-------------------------------------------------------------------------------
function [P1, P2] = cut_patch(X1,X2,X3,X4,frac)
% LE
v1 = X2-X1; v1 = v1./norm(v1);
P2 = X2;
P2(1) = X1(1);
SPAN = norm(P2-X1);
v2 = P2-X1; v2 = v2./norm(v2);
proj = dot(v2,v1);
len = SPAN * frac / proj;
P1 = X1 + v1.*proj;
% TE
v1 = X3-X4;v1 = v1./norm(v1);
P2 = X3;
P2(1) = X4(1);
SPAN = norm(P2-X4);
v2 = P2-X4;v2 = v2./norm(v2);
proj = dot(v2,v1);
len = SPAN * frac / proj;
P2 = X4 + v1.*proj;

end
%-------------------------------------------------------------------------------
function nas_spline2(fp, ID, AERID,ID1,ID2,SET,CID)

  fprintf(fp,'%8s%8d%8d%8d%8d%8d                %8d\n             1.0     1.0\n',...
      'SPLINE2 ',ID,AERID,ID1,ID2,SET,CID);

end
%-------------------------------------------------------------------------------
function nas_list(fp, card_name, ID, DATA)
%
  fprintf(fp,'%8s%8d',card_name,ID);
%
  ndata = length(DATA);
  n = [];
  if (ndata<=7)
    n1 = ndata;
    nr = 0;
  else
    n1 = 7;
    nr = ceil((ndata-7)/8);
    n(1:nr) = 8;
    n(end) = ndata - 7 - (nr-1)*8;
  end
  for j=1:n1
    fprintf(fp,'%8d',DATA(j));
  end
  cont = n1;
  if nr>0
    for j=1:nr
      fprintf(fp,'\n');
      fprintf(fp,'        ');
      for k=1:n(j)
        fprintf(fp,'%8d',DATA(k+cont));
      end
      cont = cont + n(j);
    end
    fprintf(fp,'\n');
  else
    fprintf(fp,'\n');
  end
%
end
%-------------------------------------------------------------------------------
function plot_caero(V1,V2,CHORD,NSPAN,NCHORD)

V4 = V1;
V3 = V2;
V4(1) = V4(1) + CHORD(1);
V3(1) = V3(1) + CHORD(2);
%
figure(1); hold on;

LE = V2 -V1;
TE = V3 -V4;
delta1 = norm(LE) / NSPAN;
v1 = LE ./norm(LE);
delta2 = norm(TE) / NSPAN;
v2 = TE ./norm(TE);

for k=1:NSPAN
  VLE(k,:) = V1 + (k*delta1).*v1;
  VTE(k,:) = V4 + (k*delta2).*v2;
  plot3([VLE(k,1);VTE(k,1)],[VLE(k,2);VTE(k,2)],[VLE(k,3);VTE(k,3)],'-k');
end
C1 = V4 -V1;
C2 = V3 -V2;
delta1 = norm(C1) / NCHORD;
v1 = C1 ./norm(C1);
delta2 = norm(C2) / NCHORD;
v2 = C2 ./norm(C2);

for k=1:NCHORD
  VLE(k,:) = V1 + (k*delta1).*v1;
  VTE(k,:) = V2 + (k*delta2).*v2;
  plot3([VLE(k,1);VTE(k,1)],[VLE(k,2);VTE(k,2)],[VLE(k,3);VTE(k,3)],'-k');
end

plot3([V1(1);V4(1)],[V1(2);V4(2)],[V1(3);V4(3)],'-k');
plot3([V3(1);V4(1)],[V3(2);V4(2)],[V3(3);V4(3)],'-k');
plot3([V1(1);V2(1)],[V1(2);V2(2)],[V1(3);V2(3)],'-k');

end
%-------------------------------------------------------------------------------
function cord2r(fout, ID, P1, P2, P3)
  fprintf(fout,'%8s%8d%8d%8.4f%8.4f%8.4f%8.4f%8.4f%8.4f\n','CORD2R  ',...
    ID,0,P1(1),P1(2),P1(3),P2(1),P2(2),P2(3));
  fprintf(fout,'        %8.4f%8.4f%8.4f\n',P3(1),P3(2),P3(3)); 
end