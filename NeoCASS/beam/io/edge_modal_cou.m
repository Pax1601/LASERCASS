%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2008 - 2011 
% 
% Sergio Ricci (sergio.ricci@polimi.it)
%
% Politecnico di Milano, Dipartimento di Ingegneria Aerospaziale
% Via La Masa 34, 20156 Milano - ITALY
% 
% This file is part of NeoCASS Software (www.neocass.org)
%
% NeoCASS is free software; you can redistribute it and/or
% modify it under the terms of the GNU General Public
% License as published by the Free Software Foundation;
% either version 2, or (at your option) any later version.
%
% NeoCASS is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied
% warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
% PURPOSE.  See the GNU General Public License for more
% details.
%
% You should have received a copy of the GNU General Public
% License along with NeoCASS; see the file GNU GENERAL 
% PUBLIC LICENSE.TXT.  If not, write to the Free Software 
% Foundation, 59 Temple Place -Suite 330, Boston, MA
% 02111-1307, USA.
%

%
%***********************************************************************************************************************
%  SimSAC Project
%
%  SMARTCAD
%  Simplified Models for Aeroelasticity in Conceptual Aircraft Design  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Alessandro Degaspari <degaspari@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%	
%   Author: Luca Cavagna, DIAPM
%***********************************************************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     092211      1.0     L.Cavagna        Creation
%
%*******************************************************************************
%
% function [qbk, Qbk] = edge_modal_cou(edge_headfile, IPRM, Kmm, DAMP)
%
%   DESCRIPTION: Export modal coordinates to Edge FFA format for coupled 
%   aeroelastics (ISOOPT = 112)
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                edge_headfile  FILE       Edge CFI variable for IO filenames
%                IPRM           FILE       Edge IPRM variable (0,1) to define IO filenames
%                Kmm            real       matrix with modal stiffness
%                DAMP           real       relaxation factor
%                NQTRIM         int        number of iterations allowed
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                qbk            real       modal amplitudes history
%                Qbk            real       modal forces history
%
%    REFERENCES:
%
%*******************************************************************************
%



function [qbk, Qbk] = edge_modal_cou(edge_headfile, IPRM, Kmm, DAMP, NQTRIM)
%
if (IPRM==0)
  edge_flag_in  = [edge_headfile,'_FAE112.ainp'];
  edge_data_in  = [edge_headfile,'_DAE112.ainp'];
  edge_flag_out = [edge_headfile,'_FAE112.ainp'];
  edge_data_out = [edge_headfile,'_DAE112.ainp'];
else
  edge_flag_in  = [edge_headfile,'_FAE112.ainp_in'];
  edge_data_in  = [edge_headfile,'_DAE112.ainp_in'];
  edge_flag_out = [edge_headfile,'_FAE112.ainp_out'];
  edge_data_out = [edge_headfile,'_DAE112.ainp_out'];
end
edge_flag_in_bk = 'bk.ainp';
%-------------------------------------------------------------------------------
DELAY = 0.5;
KQ = 0;
nmodes = size(Kmm,1);
qprev = zeros(nmodes,1);
qbk = zeros(nmodes, NQTRIM);
Qbk = zeros(nmodes, NQTRIM);
%-------------------------------------------------------------------------------
  while ( KQ < NQTRIM )
  %
%
    [STATUS] = load_flag(edge_flag_out);
    if (STATUS == 2) % Edge has finished one time step
      KQ = KQ + 1;
      
%      [ds,ps] = ffaaload(edge_data_in, 'mute');
%      modalQ = ffa_get(ds,ps{1});
      modalQ = load_Q(edge_data_in);
      modalQ = modalQ';
      q = Kmm\modalQ;
      q = (1-DAMP) * q + (DAMP) * qprev;
      qprev = q;
      qbk(:,KQ) = q;
      Qbk(:,KQ) = modalQ;
%
%      ds = ffa_create('modal_state');
%      ds_q = ffa_create('modal_coordinate','RF',q');
%      [ds, ie] = ffa_putsub(ds, ds_q);
%
      write_q(q,edge_data_in)
%      ffa_dump(ds, edge_data_in);
%      command = ['ffab2a ', outfile,' ',edge_data_in];
%      system(command);
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
      save([edge_headfile,'_bk.mat'], 'qbk','Qbk');
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
    fp = fopen(file,'r');
    line = fgetl(fp);
    line = fgetl(fp);
    line = fgetl(fp);
    STATUS = int16(str2num(line));
    fclose(fp);
%    [ds,ps]=ffaaload(file,'mute');
%    STATUS = ffa_get(ds,ps{2});
  else
    STATUS = -1;
  end
%
end
%-------------------------------------------------------------------------------
function [Q] = load_Q(file)
%
  if (exist(file,'file'))
    fp = fopen(file,'r');
    line = fgetl(fp);
    line = fgetl(fp);
    [s1,s2]=strtok(line,',');
    [s1,s2]=strtok(s2,',');
    [s1,s2]=strtok(s2,',');
    n = str2num(s1);
    Q = [];
    nl=ceil(n/6);
    for k=1:nl
      line = fgetl(fp);
      Q = [Q, str2num(line)];
    end
    fclose(fp);
  end
%
end
%-------------------------------------------------------------------------------
function write_q(q, file)
%
  fp = fopen(file,'w');
  fprintf(fp,'modal_state,N,0,0,1\n');
  fprintf(fp,'modal_coordinate,RF,%d,1,0\n', length(q));
  fprintf(fp,'%e\n', q);
  fclose(fp);    
%
end
