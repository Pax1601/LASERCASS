%
% Add SPC 1 entries at three pin points.
% 2 pins are placed at the fore spar, 1 pin placed at the read spar
% Modify the following function to change constraints type.
%
%
% Inputs:
% fore_spar: ID of nodes belonging to fore spar (from top to bottom)
% aft_spar:  ID of nodes belonging to aft spar (from top to bottom)
%
function spc_fuse(fore_spar, aft_spar, spar_v)
%
OFFSETID = 2000000;
pin = zeros(3,3);
V1 = spar_v(1,:);
V2 = spar_v(2,:);
V3 = spar_v(3,:);
V4 = spar_v(4,:);
fout=fopen('spc_fuse.dat','w');
d = norm(V2-V1); v = (V2-V1) ./ d;
% place a pin at 1/4 and 3/4 of spar height
pin(1,:) = V1 + v.*(d/4);
pin(2,:) = V1 + v.*(3*d/4);
% place a pin at mid spar height
d = norm(V4-V3); v = (V4-V3) ./ d;
pin(3,:) = V3 + v.*(d/2);
fprintf(fout,'$\n$ Pin centers\n$\n');
fprintf(fout,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
for k=1:3
  fprintf(fout,'%8s%8d%8d%8.4f%8.4f%8.4f%8d%8d\n','GRID    ',k,0,pin(k,1),pin(k,2),pin(k,3),0,123);
end
%
% connect half nodes of the front spar to first pin, the remaining half to second pin
%
n1 = ceil(length(fore_spar)/2);
n2 = length(fore_spar) -n1;
fprintf(fout,'$ Pin rigid links to spars\n');
fprintf(fout,'$ Top pin fore spar\n');
fprintf(fout,'%8s%8d%8d%8d','RBE2    ',OFFSETID+1,1,123);
cont = 4;
offset = length(n1);
% 1st pin
for i=1:offset
  cont = cont+1;
  fprintf(fout,'%8d',fore_spar(i));
  if (cont==9)
    fprintf(fout,'\n        ');
    cont = 1;
  end
end
% 2nd pin
fprintf(fout,'\n$ Bottom pin fore spar');
fprintf(fout,'\n%8s%8d%8d%8d','RBE2    ',OFFSETID+2,2,123);
cont = 4;
for i=n1+1:length(fore_spar)
  cont = cont+1;
  fprintf(fout,'%8d',fore_spar(i));
  if (cont==9)
    fprintf(fout,'\n        ');
    cont = 1;
  end
end
% 3rd pin
fprintf(fout,'\n$ Pin aft spar');
fprintf(fout,'\n%8s%8d%8d%8d','RBE2    ',OFFSETID+3,3,123);
cont = 4;
for i=1:length(aft_spar)
  cont = cont+1;
  fprintf(fout,'%8d',aft_spar(i));
  if (cont==9)
    fprintf(fout,'\n        ');
    cont = 1;
  end
end
%
fclose(fout);