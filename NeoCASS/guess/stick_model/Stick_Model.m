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
%
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing
%
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
%   Author: <andreadr@kth.se>
%
%  Main function in generating stick model
%
% Called by:    guess_mod.m
%
% Calls:        Stick_Points.m, Stick_Nodes.m, symmXZ.m, Stick_ID.m
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080723      1.0     A. Da Ronch      Creation
%     091022      1.3.9   L. Travaglini    Modification
%*******************************************************************************
function [stick, geo] = Stick_Model(pdcylin, aircraft, geo)

%--------------------------------------------------------------------------
% Initialize variables
%--------------------------------------------------------------------------

% Basic stick model points for QC line
ptos_qc  = {'fuse', 'tbooms', 'winr', 'winl', 'vert', 'vert2', 'horr', 'horl', 'canr', 'canl', 'win2r', 'win2l'};
nptos_qc = numel(ptos_qc); 

% Basic stick model points on struct box middle elastic axis
ptos_C2    = {'winrC2', 'winlC2', 'horrC2', 'horlC2', 'vertC2', 'vert2C2', 'canrC2', 'canlC2', 'win2rC2', 'win2lC2'};
nptos_C2   = numel(ptos_C2);
stick.ptos = cell2struct(cell(nptos_qc + nptos_C2, 1), [ptos_qc ptos_C2]);

% Node distribution for QC line
nodes_qc       = {'fuse', 'tboomsr', 'tboomsl', 'winr', 'winl', 'vert', 'vert2', 'horr', 'horl', 'canr', 'canl', 'win2r', 'win2l'};
nodes_qc_thick = strcat(nodes_qc, '_thick');
nnodes_qc      = numel(nodes_qc);

% Node distribution for elastic axis
nodes_C2       = {'winrC2', 'winlC2', 'horrC2', 'horlC2', 'vertC2', 'vert2C2', 'canrC2', 'canlC2', 'win2rC2', 'win2lC2', 'extrar', 'extral'};
nodes_C2_thick = strcat(nodes_C2, '_thick');
nnodes_C2      = numel(nodes_C2);

stick.nodes    = cell2struct(cell(2*nnodes_qc+2*nnodes_C2, 1), [nodes_qc nodes_qc_thick nodes_C2 nodes_C2_thick]);

% Grid Identification Number
ID       = {'fuse', 'tboomsr', 'tboomsl', 'winr', 'winl', 'vert', 'vert2', 'horr', 'horl', 'canr', 'canl', 'win2r', 'win2l', 'extrar', 'extral'};
ID_thick = strcat(ID, '_thick');
nid      = numel(ID);
stick.ID = cell2struct(cell(2*nid, 1), [ID ID_thick]);
% Components in the stick model: boolean flag.
stick.model.fuse  = pdcylin.stick.model.fuse;
stick.model.winr  = pdcylin.stick.model.winr;
stick.model.vert  = pdcylin.stick.model.vert;
%
if pdcylin.stick.model.twin
  stick.model.vert2 = 1;
else
  stick.model.vert2 = 0;
end
    %
if aircraft.Horizontal_tail.present
    stick.model.horr = pdcylin.stick.model.horr;
else
    stick.model.horr =0;
end
%
stick.model.tboomsr = 0;
if (isfield(aircraft, 'Tailbooms') &&  aircraft.Tailbooms.present)
  stick.model.tboomsr = 1;
end
%
stick.model.winl    = 0; % DO NOT CHANGE, SET TO 1 IF SYMMETRY IS APPLIED
stick.model.horl    = 0; % DO NOT CHANGE, SET TO 1 IF SYMMETRY IS APPLIED
stick.model.canl    = 0;
stick.model.tboomsl = 0;

if aircraft.Canard.present
    stick.model.canr = pdcylin.stick.model.canr;
else
    stick.model.canr =0;
end

if aircraft.wing2.present  % to be removed when AcBuilder will be modified
    stick.model.win2r = 1; % pdcylin.stick.model.win2r;
else
    stick.model.win2r = 0;
end
stick.model.win2l = 0;     % DO NOT CHANGE, SET TO 1 IF SYMMETRY IS APPLIED

% Symmetry
stick.model.symmXZ = pdcylin.stick.model.symmXZ;

% Material Identification Number
mat       = {'fuse', 'tbooms', 'wing', 'vert', 'vert2', 'hori', 'canr', 'wing2'};
nmat      = numel(mat);
pid_thick = strcat(mat, '_thick');

% Material
stick.MAT1 = cell2struct(cell(nmat, 1), mat);

% Property Identification Number
stick.PID  = cell2struct(cell(2*nmat, 1), [mat pid_thick]);

% Element Identification Number
eid        = nodes_qc;
eid_thick  = strcat(eid, '_thick');
neid       = numel(eid); 
stick.EID  = cell2struct(cell(2*neid, 1), [eid eid_thick]);

% Slave nodes for aerodynamic surfaces
stick_slv_n   = ptos_qc;
nstick_slv_n  = numel(stick_slv_n);
stick_slv_ID  = nodes_qc;
nstick_slv_ID = numel(stick_slv_ID);

stick.slaves.nodes = cell2struct(cell(nstick_slv_n, 1), stick_slv_n);
stick.slaves.ID    = cell2struct(cell(nstick_slv_ID, 1), stick_slv_ID);

% Identification Number for SET1 card
stick.IDSET.fuse = [];
stick.IDSET.tboomsr = [];
stick.IDSET.tboomsl = [];
stick.IDSET.winr = [];
stick.IDSET.winl = [];
stick.IDSET.win2r = [];
stick.IDSET.win2l = [];
stick.IDSET.vert = [];
stick.IDSET.vert2 = [];
stick.IDSET.horr = [];
stick.IDSET.horl = [];
stick.IDSET.canr = [];
stick.IDSET.canl = [];

% Interpolation parameters used in Mechanical Properties determination
str_mech = {'Astick', 'I1stick', 'I2stick', 'Jstick', 'K1stick', 'K2stick'};
for k = 1:nptos_qc,
    str.(ptos_qc{k}) = cell2struct(cell(numel(str_mech), 1), str_mech);
end

% Chord-wise number of panels
stick.nx.wing_inboard  = pdcylin.stick.nx.wing_inboard;
stick.nx.wing_midboard = pdcylin.stick.nx.wing_midboard;
stick.nx.wing_outboard = pdcylin.stick.nx.wing_outboard;
if aircraft.winglet.present ==1 && aircraft.winglet.Span>0
    stick.nx.wing_winglet = pdcylin.stick.nx.wing_winglet;
end
stick.nx.vert_inboard  = pdcylin.stick.nx.vert_inboard;
stick.nx.vert_outboard = pdcylin.stick.nx.vert_outboard;
if aircraft.wing2.present
    stick.nx.wing2_inboard  = pdcylin.stick.nx.wing2_inboard;
    stick.nx.wing2_midboard = pdcylin.stick.nx.wing2_midboard;
    stick.nx.wing2_outboard = pdcylin.stick.nx.wing2_outboard;
end
if aircraft.Horizontal_tail.present
    stick.nx.hori_inboard  = pdcylin.stick.nx.hori_inboard;
    stick.nx.hori_outboard = pdcylin.stick.nx.hori_outboard;
end
if aircraft.Canard.present
    stick.nx.canr_inboard  = pdcylin.stick.nx.canard_inboard;
    stick.nx.canr_outboard = pdcylin.stick.nx.canard_outboard;
end

% Chord-wise number of panels for control surfaces
if aircraft.wing1.flap.present == 1
  stick.nx.sup_control.wing_inboard  = pdcylin.stick.nx.sup_control.wing_inboard;
  stick.nx.sup_control.wing_midboard = pdcylin.stick.nx.sup_control.wing_midboard;
else
  stick.nx.sup_control.wing_inboard  = 0;
  stick.nx.sup_control.wing_midboard = 0;
end
if aircraft.wing1.aileron.present == 1
  stick.nx.sup_control.wing_outboard = pdcylin.stick.nx.sup_control.wing_outboard;
else
  stick.nx.sup_control.wing_outboard = 0;
end

if aircraft.winglet.present ==1 && aircraft.winglet.Span>0
    stick.nx.sup_control.wing_winglet = pdcylin.stick.nx.sup_control.wing_winglet;
end
%
if aircraft.Vertical_tail.present
  if aircraft.Vertical_tail.Rudder.present == 1
    stick.nx.sup_control.vert_inboard  = pdcylin.stick.nx.sup_control.vert_inboard;
    stick.nx.sup_control.vert_outboard = pdcylin.stick.nx.sup_control.vert_outboard;
  else
    stick.nx.sup_control.vert_inboard  = 0;
    stick.nx.sup_control.vert_outboard = 0;
  end
end
%
if aircraft.Horizontal_tail.present
  if aircraft.Horizontal_tail.Elevator.present == 1
    stick.nx.sup_control.hori_inboard  = pdcylin.stick.nx.sup_control.hori_inboard;
    stick.nx.sup_control.hori_outboard = pdcylin.stick.nx.sup_control.hori_outboard;
  else
    stick.nx.sup_control.hori_inboard  = 0;
    stick.nx.sup_control.hori_outboard = 0;
  end
end
%
if aircraft.wing2.present
    stick.nx.sup_control.wing2_inboard  = pdcylin.stick.nx.sup_control.wing2_inboard;
    stick.nx.sup_control.wing2_midboard = pdcylin.stick.nx.sup_control.wing2_midboard;
    stick.nx.sup_control.wing2_outboard = pdcylin.stick.nx.sup_control.wing2_outboard;
end
%
if aircraft.Canard.present
  if aircraft.Canard.Elevator.present ==1
    stick.nx.sup_control.canard_inboard  = pdcylin.stick.nx.sup_control.canard_inboard;
    stick.nx.sup_control.canard_outboard = pdcylin.stick.nx.sup_control.canard_outboard;
  else
    stick.nx.sup_control.canard_inboard  = 0;
    stick.nx.sup_control.canard_outboard = 0;
  end
end

% Span-wise number of panels
stick.ny.wing_carryth = 0; %set in Stick_Points_Wing
stick.ny.wing_inboard  = pdcylin.stick.ny.wing_inboard;
stick.ny.wing_midboard = pdcylin.stick.ny.wing_midboard;
stick.ny.wing_outboard = pdcylin.stick.ny.wing_outboard;
if aircraft.winglet.present ==1 && aircraft.winglet.Span>0
    stick.ny.wing_winglet = pdcylin.stick.ny.wing_winglet;
end
stick.ny.vert_inboard  = pdcylin.stick.ny.vert_inboard;
stick.ny.vert_outboard = pdcylin.stick.ny.vert_outboard;
if aircraft.Horizontal_tail.present
    stick.ny.hori_carryth  = 0;
    stick.ny.hori_inboard  = pdcylin.stick.ny.hori_inboard;
    stick.ny.hori_outboard = pdcylin.stick.ny.hori_outboard;
end
if aircraft.wing2.present
    stick.ny.wing2_carryth = 0;
    stick.ny.wing2_inboard  = pdcylin.stick.ny.wing2_inboard;
    stick.ny.wing2_midboard = pdcylin.stick.ny.wing2_midboard;
    stick.ny.wing2_outboard = pdcylin.stick.ny.wing2_outboard;
end
if aircraft.Canard.present
    stick.ny.canr_carryth  = 0;
    stick.ny.canr_inboard  = pdcylin.stick.ny.canard_inboard;
    stick.ny.canr_outboard = pdcylin.stick.ny.canard_outboard;
end

% Identification Number for CAERO1 card
caero = {'winr', 'winl', 'vert', 'vert2', 'horr', 'horl', 'canr', 'canl', 'win2r', 'win2l'};
ncaer = numel(caero);
stick.IDCAERO1 = cell2struct(cell(ncaer, 1), caero);

% Number of panels per each CAERO1 panel
stick.nTOT.wing = [];
stick.nTOT.vert = [];
stick.nTOT.hori = [];
stick.nTOT.wing2 = [];
stick.nTOT.canr = [];

% Number of panels in x and y per each defined CAERO1 panel
stick.nx.wing = [];
stick.ny.wing = [];
stick.nx.vert = [];
stick.ny.vert = [];
stick.nx.hori = [];
stick.ny.hori = [];
stick.nx.wing2 = [];
stick.ny.wing2 = [];
stick.nx.canr  = [];
stick.ny.canr  = [];

stick.nx.sup_control.wing = [];
stick.nx.sup_control.vert = [];
stick.nx.sup_control.hori = [];
stick.nx.sup_control.wing2 = [];
stick.nx.sup_control.canr = [];

% Store coordinates of corners for each CAERO1 panel
stick.ptospanel.winr = [];
stick.ptospanel.winl = [];
stick.ptospanel.vert = [];
stick.ptospanel.vert2 = [];
stick.ptospanel.horr = [];
stick.ptospanel.horl = [];
stick.ptospanel.canr = [];
stick.ptospanel.canl = [];
stick.ptospanel.win2r = [];
stick.ptospanel.win2l = [];

% Beam orientation vector coordinates
stick.X1X2X3.fuse = [];
stick.X1X2X3.tboomsr = [];
stick.X1X2X3.tboomsl = [];
stick.X1X2X3.winr = [];
stick.X1X2X3.winl = [];
stick.X1X2X3.vert = [];
stick.X1X2X3.vert2 = [];
stick.X1X2X3.horr = [];
stick.X1X2X3.horl = [];
stick.X1X2X3.canr = [];
stick.X1X2X3.canl = [];
stick.X1X2X3.win2r = [];
stick.X1X2X3.win2l = [];

% Beam element vectors collected in tensor: xel = (:,:,1) yel = (:,:,2) zel = (:,:,3)
stick.CBAR.fuse = [];
stick.CBAR.tboomsr = [];
stick.CBAR.tboomsl = [];
stick.CBAR.winr = [];
stick.CBAR.winl = [];
stick.CBAR.vert = [];
stick.CBAR.vert2 = [];
stick.CBAR.horr = [];
stick.CBAR.horl = [];
stick.CBAR.canr = [];
stick.CBAR.canl = [];
stick.CBAR.win2r = [];
stick.CBAR.win2l = [];

% Beam element offset ** not used for wings and vertical tail since only fuse and HT are moved **
stick.OFFSET.fuse = zeros(3,1);
stick.OFFSET.hori = zeros(3,1);

% Beam length for structural elements
stick.fus.Lbeam = [];
stick.tbooms.Lbeam = [];
stick.wing.Lbeam = [];
stick.vtail.Lbeam = [];
stick.htail.Lbeam = [];
stick.wing2.Lbeam = [];
stick.canr.Lbeam = [];

stick.fus.Lbeam_thick   = [];
stick.tbooms.Lbeam_thick   = [];
stick.wing.Lbeam_thick  = [];
stick.vtail.Lbeam_thick = [];
stick.htail.Lbeam_thick = [];
stick.wing2.Lbeam_thick = [];
stick.canr.Lbeam_thick = [];

% Stress recovery coefficients
stick.PBAR.fuse.str_rec_coef.Coord = [];
stick.PBAR.fuse.str_rec_coef.C     = [];
stick.PBAR.fuse.str_rec_coef.D     = [];

stick.PBAR.tbooms.str_rec_coef.Coord = [];
stick.PBAR.tbooms.str_rec_coef.C     = [];
stick.PBAR.tbooms.str_rec_coef.D     = [];

stick.PBAR.wing.str_rec_coef.C     = [];
stick.PBAR.wing.str_rec_coef.D     = [];
stick.PBAR.wing2.str_rec_coef.C     = []; %?
stick.PBAR.wing2.str_rec_coef.D     = []; %?


%--------------------------------------------------------------------------
% Calculate points to define each single structure in the aircraft model
%
[stick, geo] = Stick_Points(aircraft, stick, geo);
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Calculate node distribution in each single structure
%
[stick, geo] = Stick_Nodes( stick, geo, pdcylin);
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% SYMMETRY STATEMENT
%
if isequal(stick.model.symmXZ, 1)
    stick = symmXZ(stick, aircraft);
end
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Generate ID, MAT1, PID, EID fields
%
stick = Stick_ID(stick, aircraft);
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Define extra used-defined links
%
stick = Stick_Extra_Link(stick, aircraft);
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Define link between different structures
%
stick = Link_StructsT(stick,geo,aircraft);
%
%--------------------------------------------------------------------------
