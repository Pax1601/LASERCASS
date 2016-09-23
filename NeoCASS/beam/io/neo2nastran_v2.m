function neo2nastran(beam_model)

dotpos = find(beam_model.Param.FILE == '.')-1;
headname = beam_model.Param.FILE(1:dotpos);
Info = beam_model.Info;
str_file = [headname, '_str.nas'];
fp = fopen(str_file, 'w');


nrbe2 = 0;


fprintf(fp, '$\n$ Warning: remember to include NASTRAN BARMASS=1 \n$          at the begin of NASTRAN file.\n$');

fprintf(fp,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fp,'\n$ Node definition');
fprintf(fp,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');

for i=1:Info.ngrid

  id = sprintf('%-8d', beam_model.Node.ID(i));
  c1 = sprintf('%-8g', beam_model.Node.Coord(i,1));
  c2 = sprintf('%-8g', beam_model.Node.Coord(i,2));
  c3 = sprintf('%-8g', beam_model.Node.Coord(i,3));
  fprintf(fp,'\nGRID    %s%s%s%s', id, c1,c2,c3);

end

fprintf(fp,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fp,'\n$ Material properties');
fprintf(fp,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');

for i=1:Info.nmat

  id = sprintf('%-8d', beam_model.Mat.ID(i));
  E = sprintf('%-.3e', beam_model.Mat.E(i));
  nu = sprintf('%-8g', beam_model.Mat.nu(i));
  RHO = sprintf('%-8g', beam_model.Mat.Rho(i));
  ST = sprintf('%-8g', beam_model.Mat.ST(i));
  SC = sprintf('%-8g', beam_model.Mat.SC(i));
  SS = sprintf('%-8g', beam_model.Mat.SS(i));
  fprintf(fp,'\nMAT1    %s%s        %s%s%s%s%s', id, E, nu, RHO);
  fprintf(fp,'\n        %s%s%s', ST,SC,SS);

end

fprintf(fp,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fp,'\n$ Bar properties');
fprintf(fp,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');

for i=1:Info.npbar

  id = sprintf('%-8d', beam_model.PBar.ID(i));
  pid = sprintf('%-8d', beam_model.Mat.ID(beam_model.PBar.Mat(i)));
  A = sprintf('%-8g', beam_model.PBar.A(i));
  I1 = sprintf('%-8g', beam_model.PBar.I(i,1));
  I2 = sprintf('%-8g', beam_model.PBar.I(i,2));
  J = sprintf('%-8g', beam_model.PBar.J(i));
  RHO = sprintf('%-8g', beam_model.PBar.RhoNS(i));
  fprintf(fp,'\nPBAR    %s%s%s%s%s%s%s\n        ', id, pid, A, I1, I2, J, RHO);
  for n=1:4
    c1 = sprintf('%-8g', beam_model.PBar.Str_point(n,1,i));
    c2 = sprintf('%-8g', beam_model.PBar.Str_point(n,2,i));
    fprintf(fp,'%s%s', c1, c2);
  end
%  K1 = sprintf('%-8f', beam_model.PBar.Kshear(i,1));
%  K2 = sprintf('%-8f', beam_model.PBar.Kshear(i,2));
%  I3 = sprintf('%-8f', beam_model.PBar.I(i,3));
%  fprintf(fp,'\n%s%s%s', K1, K2, I3);

end

fprintf(fp,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fp,'\n$ Bar elements');
fprintf(fp,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');

for i=1:Info.nbar

  id = sprintf('%-8d', beam_model.Bar.ID(i));
  pid = sprintf('%-8d', beam_model.PBar.ID(beam_model.Bar.PID(i)));
  n1 = sprintf('%-8d', beam_model.Node.ID(beam_model.Bar.Conn(i,1)));
  n2 = sprintf('%-8d', beam_model.Node.ID(beam_model.Bar.Conn(i,3)));
  c1 = sprintf('%-8g', beam_model.Bar.Orient(i,1));
  c2 = sprintf('%-8g', beam_model.Bar.Orient(i,2));
  c3 = sprintf('%-8g', beam_model.Bar.Orient(i,3));
  fprintf(fp,'\nCBAR    %s%s%s%s%s%s%sGGG', id, pid, n1, n2, c1,c2,c3);

end

fprintf(fp,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fp,'\n$ Lumped masses');
fprintf(fp,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
ncom = 0;
for i=1:Info.nconm
  ncom = ncom+1;
  id = sprintf('%-8d', ncom);
  n = sprintf('%-8d', beam_model.Node.ID(beam_model.ConM.Node(ncom)));
  M11 = sprintf('%-8g', beam_model.ConM.M(1,1,ncom));
  M21 = sprintf('%-8g', beam_model.ConM.M(2,1,ncom));
  M22 = sprintf('%-8g', beam_model.ConM.M(2,2,ncom));
%
  M31 = sprintf('%-8g', beam_model.ConM.M(3,1,ncom));
  M32 = sprintf('%-8g', beam_model.ConM.M(3,2,ncom));
  M33 = sprintf('%-8g', beam_model.ConM.M(3,3,ncom));
%
  M41 = sprintf('%-8g', beam_model.ConM.M(4,1,ncom));
  M42 = sprintf('%-8g', beam_model.ConM.M(4,2,ncom));
  M43 = sprintf('%-8g', beam_model.ConM.M(4,3,ncom));
  M44 = sprintf('%-8g', beam_model.ConM.M(4,4,ncom));
%
  M51 = sprintf('%-8g', beam_model.ConM.M(5,1,ncom));
  M52 = sprintf('%-8g', beam_model.ConM.M(5,2,ncom));
  M53 = sprintf('%-8g', beam_model.ConM.M(5,3,ncom));
  M54 = sprintf('%-8g', beam_model.ConM.M(5,4,ncom));
  M55 = sprintf('%-8g', beam_model.ConM.M(5,5,ncom));
%
  M61 = sprintf('%-8g', beam_model.ConM.M(6,1,ncom));
  M62 = sprintf('%-8g', beam_model.ConM.M(6,2,ncom));
  M63 = sprintf('%-8g', beam_model.ConM.M(6,3,ncom));
  M64 = sprintf('%-8g', beam_model.ConM.M(6,4,ncom));
  M65 = sprintf('%-8g', beam_model.ConM.M(6,5,ncom));
  M66 = sprintf('%-8g', beam_model.ConM.M(6,6,ncom));
%
  fprintf(fp,'\nCONM1   %s%s        %s%s%s%s%s', id, n, M11,M21,M22,M31,M32);
  fprintf(fp,'\n        %s%s%s%s%s%s%s%s', M33,M41,M42,M43,M44,M51,M52,M53);
  fprintf(fp,'\n        %s%s%s%s%s%s%s%s', M54,M55,M61,M62,M63,M64,M65,M66);
%
end

if (Info.nrbe0)
  fprintf(fp,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
  fprintf(fp,'\n$ Box cross nodes');
  fprintf(fp,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
  for i=1:length(beam_model.Node.Aero.Index)
    if ~isempty(beam_model.Node.Aero.Index(i).data)
      nrbe2 = nrbe2+1;
      cont = sprintf('%-8d', nrbe2);
      id = sprintf('%-8d', i);
      fprintf(fp,'\nRBE2    %s%s 123456 ', cont, id);
      for k=1:length(beam_model.Node.Aero.Index(i).data)
        id = sprintf('%-8d', beam_model.Node.ID(beam_model.Node.Aero.Index(i).data(k)));
        fprintf(fp,'%s', id);
      end
    end
  end
end

if (Info.nrbe2)
  fprintf(fp,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
  fprintf(fp,'\n$ Rigid connections');
  fprintf(fp,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
  for i=1:Info.nrbe2
    nrbe2 = nrbe2+1;
    cont = sprintf('%-8d', nrbe2);
    id = sprintf('%-8d', beam_model.RBE2.IDM(i));
    fprintf(fp,'\nRBE2    %s%s%s', cont, id);
    gdl = [];
    k=0;
    for k=1:length(beam_model.RBE2.GDL(i).data)
      id = sprintf('%-d', beam_model.RBE2.GDL(i).data(k));
      fprintf(fp,'%c', id);
    end
    fprintf(fp, '%c', blanks(8-k));
    for k=1:length(beam_model.RBE2.IDS(i).data)
      id = sprintf('%-8d', beam_model.RBE2.IDS(i).data(k));
      fprintf(fp,'%s', id);
    end
  end
end


fclose(fp);