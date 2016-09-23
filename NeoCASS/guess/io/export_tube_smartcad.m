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
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing
%
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS     PROGRAMMER       DESCRIPTION
%     080502      1.0      A. Da Ronch      Creation
%     090326      1.3.2    A. De Gaspari    Modification
%     091119      1.3.9    L. Travaglini    Modification
%     120502      2.1.237  L. Riccobene     Modification
%*******************************************************************************
%
function [file_name_conf, WREG, WREGCG] = export_model_smartcad(stick, geo, aircraft, ref, pdcylin, str,  filename_stick, filename_trim, nconf)
%
fid = 1;
%
fprintf(fid, '\n\t - Exporting stick model output file...');
%
headname = filename_stick(1:find(filename_stick=='.')-1);
file_name_conf = [headname,'_CONF',num2str(nconf),'.dat'];
% Update stick model
%
stick.ID.fuse    = stick.ID.fuse_thick;
stick.ID.tboomsr = stick.ID.tboomsr_thick;
stick.ID.tboomsl = stick.ID.tboomsl_thick;
stick.ID.winr    = stick.ID.winr_thick;
stick.ID.winl    = stick.ID.winl_thick;
stick.ID.vert    = stick.ID.vert_thick;
stick.ID.vert2   = stick.ID.vert2_thick;
stick.ID.horr    = stick.ID.horr_thick;
stick.ID.horl    = stick.ID.horl_thick;
stick.ID.canr    = stick.ID.canr_thick;
stick.ID.canl    = stick.ID.canl_thick;
stick.ID.win2r   = stick.ID.win2r_thick;
stick.ID.win2l   = stick.ID.win2l_thick;
%
stick.nodes.fuse    = stick.nodes.fuse_thick;
stick.nodes.winr    = stick.nodes.winr_thick;
stick.nodes.winl    = stick.nodes.winl_thick;
stick.nodes.tboomsr = stick.nodes.tboomsr_thick;
stick.nodes.tboomsl = stick.nodes.tboomsl_thick;
stick.nodes.winrC2  = stick.nodes.winrC2_thick;
stick.nodes.winlC2  = stick.nodes.winlC2_thick;
stick.nodes.vert    = stick.nodes.vert_thick;
stick.nodes.vert2   = stick.nodes.vert2_thick;
stick.nodes.horrC2  = stick.nodes.horrC2_thick;
stick.nodes.horlC2  = stick.nodes.horlC2_thick;
stick.nodes.canrC2  = stick.nodes.canrC2_thick;
stick.nodes.canlC2  = stick.nodes.canlC2_thick;
stick.nodes.win2rC2 = stick.nodes.win2rC2_thick;
stick.nodes.win2lC2 = stick.nodes.win2lC2_thick;
%
stick.PID.fuse   = stick.PID.fuse_thick;
stick.PID.tbooms = stick.PID.tbooms_thick;
stick.PID.wing   = stick.PID.wing_thick;
stick.PID.vert   = stick.PID.vert_thick;
stick.PID.vert2  = stick.PID.vert2_thick;
stick.PID.hori   = stick.PID.hori_thick;
stick.PID.wing2  = stick.PID.wing2_thick;
stick.PID.canr   = stick.PID.canr_thick;
%
stick.EID.fuse    = stick.EID.fuse_thick;
stick.EID.tboomsr = stick.EID.tboomsr_thick;
stick.EID.tboomsl = stick.EID.tboomsl_thick;
stick.EID.winr    = stick.EID.winr_thick;
stick.EID.winl    = stick.EID.winl_thick;
stick.EID.vert    = stick.EID.vert_thick;
stick.EID.vert2   = stick.EID.vert2_thick;
stick.EID.horr    = stick.EID.horr_thick;
stick.EID.horl    = stick.EID.horl_thick;
stick.EID.canr    = stick.EID.canr_thick;
stick.EID.canl    = stick.EID.canl_thick;
stick.EID.win2r   = stick.EID.win2r_thick;
stick.EID.win2l   = stick.EID.win2l_thick;
%
stick.link.Ma   = stick.link_thick.Ma_thick;
stick.link.RBE2 = stick.link_thick.RBE2;
stick.ID.extrar = stick.ID.extrar_thick;
stick.ID.extral = stick.ID.extral_thick;
pdcylin.stick.nwing_carryth = pdcylin.stick.nwing_carryth_coarse;

% Update geometry
%
% Fuselage
geo.fus.x_nodes     = geo.fus.x_nodes_thick;
geo.fus.x_nodes_1_2 = geo.fus.x_nodes_1_2_thick;
stick.fus.Lbeam     = stick.fus.Lbeam_thick;

% Wing
geo.wing.y_nodes     = geo.wing.y_nodes_thick;
geo.wing.y_nodes_1_2 = geo.wing.y_nodes_1_2_thick;
stick.wing.Lbeam    = stick.wing.Lbeam_thick;

% VT
if isequal(pdcylin.stick.model.vert,1)
    geo.vtail.y_nodes     = geo.vtail.y_nodes_thick;
    geo.vtail.y_nodes_1_2 = geo.vtail.y_nodes_1_2_thick;
    stick.vtail.Lbeam     = stick.vtail.Lbeam_thick;
end

% HT
if isequal(pdcylin.stick.model.horr,1)
  geo.htail.y_nodes     = geo.htail.y_nodes_thick;
  geo.htail.y_nodes_1_2 = geo.htail.y_nodes_1_2_thick;
  stick.htail.Lbeam     = stick.htail.Lbeam_thick;
end

% Canard
if isequal(pdcylin.stick.model.canr,1)
  geo.canard.y_nodes     = geo.canard.y_nodes_thick;
  geo.canard.y_nodes_1_2 = geo.canard.y_nodes_1_2_thick;
  stick.canr.Lbeam       = stick.canr.Lbeam_thick;
end

% Wing2
if isequal(pdcylin.stick.model.win2r,1)
  geo.wing2.y_nodes     = geo.wing2.y_nodes_thick;
  geo.wing2.y_nodes_1_2 = geo.wing2.y_nodes_1_2_thick;
  stick.wing2.Lbeam     = stick.wing2.Lbeam_thick;
end

% Tail booms
if isequal(stick.model.tboomsr,1)
  geo.tbooms.x_nodes = geo.tbooms.x_nodes_thick;
  stick.tbooms.Lbeam = stick.tbooms.Lbeam_thick;
  geo.tbooms.x_nodes_1_2 = geo.tbooms.x_nodes_1_2_thick;
end

%-------------------------------------------------------------------------------
% NON STRUCTURAL MASSES DETERMINATION
%-------------------------------------------------------------------------------
aircraft.weight_balance.COG(25,4,1) = aircraft.weight_balance.COG(25,4,1)*pdcylin.MassConf.Baggage(nconf);
aircraft.weight_balance.COG(24,4,1) = aircraft.weight_balance.COG(24,4,1)*pdcylin.MassConf.Pass(nconf);
aircraft.weight_balance.COG(18,4,1) = aircraft.weight_balance.COG(18,4,1)*pdcylin.MassConf.Wfuel(nconf);
aircraft.weight_balance.COG(19,4,1) = aircraft.weight_balance.COG(19,4,1)*pdcylin.MassConf.Cfuel(nconf);
pdcylin.MassConf.WfuelArrive = pdcylin.MassConf.WfuelArrive(nconf);
pdcylin.MassConf.WfuelStart  = pdcylin.MassConf.WfuelStart(nconf);
pdcylin.MassConf.Wfuel       = pdcylin.MassConf.Wfuel(nconf);
%
pdcylin.MassConf.BagArrive = pdcylin.MassConf.BagArrive(nconf);
pdcylin.MassConf.BagStart = pdcylin.MassConf.BagStart(nconf);
pdcylin.MassConf.Baggage = pdcylin.MassConf.Baggage(nconf);
%
pdcylin.MassConf.PaxArrive = pdcylin.MassConf.PaxArrive(nconf);
pdcylin.MassConf.PaxStart = pdcylin.MassConf.PaxStart(nconf);
pdcylin.MassConf.Pass = pdcylin.MassConf.Pass(nconf);
%
fprintf(fid, '\n\t - Determining non-structural masses...');
[str] = Add_NSM(fid, pdcylin, aircraft, geo, stick, str);
fprintf(fid, '\ndone.');
line = ['INCLUDE ', filename_stick];
% Write to file
if nconf == 1
%    
  fp  = fopen(filename_stick, 'w');
  fp2 = fopen(file_name_conf, 'w');
  fprintf(fp2, '%s', line);
  % PARAM fuselage pressure
  writeFUSE_DP2file(fid, fp, pdcylin, aircraft);
  % MAT1 card
  writeMAT12file(fid, fp, pdcylin, stick, aircraft);
  % GRID card
  writeGRID2file(fid, fp, stick, aircraft);
%  writeGRID2file(fid, fp2, stick, aircraft);
  % CBAR card
  [stick] = writeCBAR2file(fid, fp, stick, geo, aircraft);
  % RBE0
  [stick] = writeRBE02file(fid, fp, stick, geo, aircraft);
  % PBAR / PBARSMX card
  [geo, str, stick] = setup_PROPERTY_BAR_mod(fid, fp, geo, str, stick, aircraft, pdcylin, 1, 1);
  % CAERO1 card
  [stick] = writeCAERO2file(fid, fp, aircraft, stick, geo, 0, pdcylin.smartcad.caerob);
  % AELINK card
  writeAELINK2file(fid, fp, pdcylin, geo, stick, aircraft, 1);
  % SET1 card
  writeSET12file(fid, fp, stick, aircraft, pdcylin, 0);
  % SPLINE2 card
  switch(pdcylin.smartcad.spline_type)
    case 1
    % SPLINE1 beam card
      TCOND = pdcylin.smartcad.tcond;
      writeSPLINE12file(fid, fp, stick, aircraft, TCOND);
    case 2
      % SPLINE2 RBF card
      RMAX = pdcylin.smartcad.rmax ;
      TCOND = pdcylin.smartcad.tcond;
      writeSPLINE22file(fid, fp, stick, aircraft, RMAX, TCOND);
    % SPLINE3 MLS card
    case 3
      POLY = pdcylin.smartcad.poly;
      W = pdcylin.smartcad.weight;
      NP = pdcylin.smartcad.npoints;
      RMAX = pdcylin.smartcad.rmax ;
      TCOND = pdcylin.smartcad.tcond;
      writeSPLINE32file(fid, fp, stick, aircraft, POLY, W, NP, RMAX, TCOND);
  end
  % CONM2
  [str,IDSUPORT,stick] = Add_NSM_conc(fid, fp2, fp, pdcylin, aircraft, geo, stick, str, 0, 0);
  writeRIGIDlinkRBE2(fp, stick, 0);
  % TRIM card
  % AEROS card
  writeAEROS2file(fp, stick, aircraft, ref);%,SUPORT);
  fclose(fp);
  fprintf(fp2, '\n$ SUPORT FOR CURRENT CONFIGURATION\n');
  BULKdataSUPORT(fp2,IDSUPORT);
  fclose(fp2);
%
else
%
  fp2 = fopen(file_name_conf, 'w');
 % writeGRID2file(fid, fp2, stick, aircraft);
  fprintf(fp2, '%s', line);
%
  [stick] = setCBAR_mod(fid, stick, geo, aircraft);
  [geo, str, stick] = setup_PROPERTY_BAR_mod(fid, fp2, geo, str, stick, aircraft, pdcylin, 1, 0);
  [str,IDSUPORT,stick] = Add_NSM_conc(fid, fp2, '', pdcylin, aircraft, geo, stick, str, 0, 0);
  fprintf(fp2, '\n$ SUPORT FOR CURRENT CONFIGURATION\n');
  BULKdataSUPORT(fp2,IDSUPORT);
  fclose(fp2);
end
%
WREG = str.wing.regr.M;
WREGCG = str.wing.regr.CG;
%
fprintf('\n\ndone.\n');
