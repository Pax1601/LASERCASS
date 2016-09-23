function edge_nas_cou(edge_headfile, IPRM, nas_headfile, nas_include_file, edge_mod_file, DAMP)
%
if (IPRM==0)
  edge_flag_in  = [edge_headfile,'_FAE202.ainp'];
  edge_data_in  = [edge_headfile,'_DAE202.ainp'];
  edge_flag_out = [edge_headfile,'_FAE202.ainp'];
  edge_data_out = [edge_headfile,'_DAE202.ainp'];
else
  edge_flag_in  = [edge_headfile,'_FAE202.ainp_in'];
  edge_data_in  = [edge_headfile,'_DAE202.ainp_in'];
  edge_flag_out = [edge_headfile,'_FAE202.ainp_out'];
  edge_data_out = [edge_headfile,'_DAE202.ainp_out'];
end
edge_flag_in_bk = 'bk.ainp';
%-------------------------------------------------------------------------------
DELAY = 0.5;
KQ = 0;
NQTRIM = 10;
%-------------------------------------------------------------------------------
%for NITER = 1:MAXITER
%
%-------------------------------------------------------------------------------
  while ( KQ < NQTRIM )
  %
%
    [STATUS] = load_flag(edge_flag_out);
    if (STATUS == 2) % Edge has finished one time step
      if (KQ == 0)
%
        [ds,ps] = ffabload(edge_mod_file,'mute');
        [pc,in,ierr] = ffa_find(ds,'name','grid_idents');
        if (in==0)
          error('grid_idents dabatase not available in %s file.', edge_mod_file);
        end
        id = ffa_get(ds, ps{in});
%
        [pc,in,ierr] = ffa_find(ds,'name','str_coordinates');
        if (in==0)
          error('str_coordinates dabatase not available in %s file.', edge_mod_file);
        end
        coord = ffa_get(ds, ps{in});
        BK = zeros(length(id),3);
      end
      fprintf('\n RUN NASTRAN...\n\n')
      KQ = KQ+1;
      edge2nas_force(edge_data_out, nas_include_file);
      command = ['/msc/MD_Nastran/20111/md20111/linux64/nastran ', nas_headfile,'.dat batch=no old=no'];
      system(command);
      pause(DELAY);
%
      disp = nas_disp2edge([nas_headfile,'.pch'], edge_headfile, IPRM, id, coord, BK, DAMP);
      BK = disp.*(1-DAMP) + DAMP * BK; 
%
      fp = fopen(edge_flag_in_bk, 'w');
      fprintf(fp,'UPDATE,N,0,0,1\n');
      fprintf(fp,'FLAG,I,1,1,0\n');
      fprintf(fp,'%d \n',3);
      fclose(fp);
%
      if (IPRM==1)
        fp = fopen(edge_flag_out, 'w');
        fprintf(fp,'UPDATE,N,0,0,1\n');
        fprintf(fp,'FLAG,I,1,1,0\n');
        fprintf(fp,'%d \n',3);
        fclose(fp);
      end
%
      command = (['mv ',edge_flag_in_bk,' ',edge_flag_in]);
      system(command);
    end
    pause(DELAY);
  %
  end
%-------------------------------------------------------------------------------
%end
%
end
%-------------------------------------------------------------------------------
% Internal functions
%-------------------------------------------------------------------------------
function [STATUS] = load_flag(file)
%
  if (exist(file,'file'))
    [ds,ps]=ffaaload(file,'mute');
    STATUS = ffa_get(ds,ps{2});
  else
    STATUS = -1;
  end
%
end
%-------------------------------------------------------------------------------
