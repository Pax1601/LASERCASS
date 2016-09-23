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
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% Export all the files necessary for NASTRAN SOL 101/145/200 
%
% Input: main NASTRAN file to create (without extension)
%        xml file to load with aircraft geometry
%        input_file: ASCII .m file with input parameters (optional)
%        If provided, FEMGEN will skip user batch interface
%
function wing_fem(outfilename, xml_file, input_file)
%
fid = 1;
close all;
%
dotpos = find(outfilename=='.');
if ~isempty(dotpos)
  outfilename = outfilename(1:dotpos(end)-1);
end
%
%-------------------------------------------------------------------------------
% geometry
%
if ~exist(xml_file,'file')
  fprintf(fid,'\nFile %s not found.', xml_file);
  return
end
%
fprintf(fid,'\nXML main file: %s.', xml_file);
%
[foil, PANE, CAERO1, aircraft, geo] =  get_geo_wing(xml_file);
figure(1); close; figure(1); hold on; axis equal;
%
naer = size(foil,2)-1;
% 
for k=1:naer+1
  plot3(foil{k}(:,1),foil{k}(:,2),foil{k}(:,3),'-');
end
for k=1:naer
  offset = (k-1)*4 + 1;
  plot3(PANE(1,offset:offset+3),PANE(2,offset:offset+3),PANE(3,offset:offset+3),'-');
end
plot3(PANE(1,[1,4]),PANE(2,[1,4]),PANE(3,[1,4]),'-g');
doffset = mean(PANE(2,[1,2]))/2;
text(mean(PANE(1,[1,4])),mean(PANE(2,[1,4]))+doffset,'1');
for k=1:naer
  offset = (k-1)*4 + 1;
  plot3(PANE(1,offset+1:offset+2),PANE(2,offset+1:offset+2),PANE(3,offset+1:offset+2),'-g');
  text(mean(PANE(1,offset+1:offset+2)),mean(PANE(2,offset+1:offset+2))+doffset,num2str(k+1));
end
%
H = figure(1);
saveas(H,'wing_geo.fig','fig');
%
%-------------------------------------------------------------------------------
% input
%
if isempty(input_file)
  okinp = 0;
  while okinp == 0
    fprintf(fid,'\n\n- INPUT DATA\n');
    fprintf(fid,'\n\n  LAYOUT\n');
    ok = 0;
    while ok==0
    %
      okw = 1;
      span_frac = zeros(naer, 2);
    % skip CT outboard chord
      for k=[1,3:naer+1]
        fprintf(fid, '\nSection %d:', k); 
        span_frac(k,1) = input('\n        Fore spar sector: '); 
        span_frac(k,2) = input('        Aft spar sector: '); 
      end
%
    MIN = 0.01*aircraft.wing1.span*0.5;
    if aircraft.wing1.spanwise_kink1>MIN
      dist = aircraft.wing1.spanwise_kink1*geo.wing.b/2;
      C = geo.wing.CR_kink1;
      LAMBDA = geo.wing.lambdaQC_inboard;
    else
      if aircraft.wing1.spanwise_kink2>MIN
        dist = aircraft.wing1.spanwise_kink1*geo.wing.b/2;
        C = geo.wing.CR_kink2;
        LAMBDA = geo.wing.lambdaQC_midboard;
      else
        dist = geo.wing.b/2;
        C = geo.wing.CT;
        LAMBDA = geo.wing.lambdaQC_outboard;
      end
    end
     [fore_spar, aft_spar] = root_spar(geo.wing.CRp, C, geo.wing.CR, ...
                    span_frac(1,1), span_frac(1,2), ...
                    span_frac(3,1), span_frac(3,2), ...
                    geo.fus.R, dist, LAMBDA);
     span_frac(2,1) = fore_spar; span_frac(2,2) = aft_spar;
     span_frac(1,1) = fore_spar; span_frac(1,2) = aft_spar;
    %  fprintf(fid, '\nGiven values:\n'); 
    %  disp(span_frac);
      if (~isempty(find(span_frac>1)))
        fprintf(fid,'\nValues higher than 1 given.\n');
        okw = 0;
      end
      if (~isempty(find(span_frac<0)))
        fprintf(fid,'\nValues less than 0 given.\n');
        okw = 0;
      end
      okw = draw_wbox(foil, PANE, span_frac);
      if okw~=0
        ok = input('OK to proceed? Yes = 1, No = 0: ');
        if ok~=1
          ok = 0;
        end
      else
        ok = 0;
      end
    %
    end
    %
    ok = 0;
    while ok==0
      rib_type= [];
      answer = input('\n        Ribs alignment (1: freestream, 2: wing box axis): '); 
      ok = input('OK to proceed? Yes = 1, No = 0: ');
      switch(answer)
        case(1)
         rib_type = 'wind';
        case(2)
         rib_type = 'ax';
        otherwise
          fprintf(fid, '\n### Warning: unknown option.\n'); 
          ok = 0;
      end
      if ok~=1
        ok = 0;
      end
    end
    ok = 0;
    while ok==0
      str_type= [];
      answer = input('\n        Stringers layout (1: fixed, 2: variable): '); 
      ok = input('OK to proceed? Yes = 1, No = 0: ');
      switch(answer)
        case(1)
         str_type = 'fix';
        case(2)
         str_type = 'var';
        otherwise
          fprintf(fid, '\n### Warning: unknown option.\n'); 
          ok = 0;
      end
      if ok~=1
        ok = 0;
      end
    end
    %
    ok = 0;
    str_up_pitch   = 0;
    str_low_pitch  = 0;
    nstr_up_start  = 0;
    nstr_low_start = 0;
    %
    if (strcmp(str_type,'var'))
      while ok==0
        str_up_pitch = 0;
        str_up_pitch = input('\n        Upper stringers pitch: '); 
        str_low_pitch= 0;
        str_low_pitch = input('\n        Lower stringers pitch: '); 
        ok = input('OK to proceed? Yes = 1, No = 0: ');
        if ok~=1
          ok = 0;
        end
      end
    else
      while ok==0
        nstr_up_start = 0;
        nstr_up_start = input('\n        Number of upper stringers: '); 
        nstr_low_start = 0;
        nstr_low_start = input('\n        Number of lower stringers: '); 
        ok = input('OK to proceed? Yes = 1, No = 0: ');
        if ok~=1
          ok = 0;
        end
      end
    end
    %
    ok = 0;
    while ok==0
      nrib_span = zeros(naer,1);
      for k=1:naer
        fprintf(fid, '\nBay %d:', k); 
        nrib_span(k) = input('\n        Number of ribs: '); 
      end
      ok = input('OK to proceed? Yes = 1, No = 0: ');
      if ok~=1
        ok = 0;
      end
    end
  %
    ok = 0;
    while ok==0
      rib_thick = input('\n        Ribs thickness: '); 
      ok = input('OK to proceed? Yes = 1, No = 0: ');
      if ok~=1
        ok = 0;
      end
    end
    %
    %-------------------------------------------------------------------------------
    % input
    %
    fprintf(fid,'\n\n  MESH\n');
    ok = 0;
    while ok==0
      nelem_btwn_rib = 0;
      nelem_btwn_rib = input('\n        Number of spanwise elements between ribs: '); 
      ok = input('OK to proceed? Yes = 1, No = 0: ');
      if ok~=1
        ok = 0;
      end
    end
    %
    ok = 0;
    while ok==0
      nelem_btwn_str= 0;
      nelem_btwn_str = input('\n        Number of elements between stringers: '); 
      ok = input('OK to proceed? Yes = 1, No = 0: ');
      if ok~=1
        ok = 0;
      end
    end
    %
    ok = 0;
    while ok==0
      nelem_fore_spar = 0;
      nelem_fore_spar = input('\n        Number of elements along fore spar: '); 
      nelem_aft_spar= 0;
      nelem_aft_spar = input('\n        Number of elements along aft spar: '); 
      ok = input('OK to proceed? Yes = 1, No = 0: ');
      if ok~=1
        ok = 0;
      end
    end
    %
    ok = 0;
    while ok==0
      nelem_rib= 0;
      nelem_rib = input('\n        Number of radial strips of elements within ribs: '); 
      ok = input('OK to proceed? Yes = 1, No = 0: ');
      if ok~=1
        ok = 0;
      end
    end
    %
    fprintf(fid,'\n\n  PROPERTIES DEFINITION\n');
    ok = 0;
    while ok==0
      answer = input('\n        Material (1: Aluminum, 2: carbon fiber): '); 
      ok = input('OK to proceed? Yes = 1, No = 0: ');
      switch(answer)
        case(1)
         mat_type = 1;
        case(2)
         mat_type = 2;
        otherwise
          fprintf(fid, '\n### Warning: unknown option.\n'); 
          ok = 0;
      end
      if ok~=1
        ok = 0;
      end
    end
    %
    % create param struct
    param.fore_spar = span_frac(:,1);
    param.aft_spar  = span_frac(:,2);
    param.nrib_span = nrib_span;
    param.rib_thick = rib_thick;
    param.nelem_btwn_rib = nelem_btwn_rib;
    param.nstr_up_start=nstr_up_start;
    param.nstr_low_start=nstr_low_start;
    param.nelem_btwn_str = nelem_btwn_str;
    param.nelem_fore_spar = nelem_fore_spar;
    param.nelem_aft_spar=nelem_aft_spar;
    param.nelem_rib=nelem_rib;
    param.rib_type=rib_type;
    param.str_up_pitch=str_up_pitch;
    param.str_low_pitch=str_low_pitch;
    param.str_type=str_type;
    param.mat_type=mat_type;
    param.foil=foil;
    %
    %
    %-------------------------------------------------------------------------------
    % input
    %
    guess_model = [];
    guess_filename = [];
    X0 = [0.0 0.0 0.0];
    %
    % initialize solution from guess or input
    %
    ok = 0;
    while ok==0
      okw = 1;
      answer = input('\n        Use GUESS file to initialize data? Yes = 1, No = 0: '); 
      ok = input('OK to proceed? Yes = 1, No = 0: ');
      if ok~=1
        ok = 0;
      end
      if ok
        switch(answer)
          case(1)
            guess_filename = input('\n        Guess filename (between ''): ');
            if ~exist(guess_filename,'file')
              fprintf(fid,'\nFile %s not found.', guess_filename);
              okw=0;
            else
              load(guess_filename);
              if guess_model.pdcylin.wing.kcon>=9
                fprintf(fid, '\n        Solution will be interpolated from GUESS solution (kcon>=9).'); 
              else
                fprintf(fid, '\n        An optimization problem will be run to recover structural parameters from GUESS stiffness distribution.'); 
              end
            end
          case(0)
            guess_model = [];
            X0(1) = input('\n        Skin thickness: '); 
            X0(2) = input('\n        Spar thickness: '); 
            X0(3) = input('\n        Stringer area: '); 
          otherwise
            fprintf(fid, '\n### Warning: unknown option.\n'); 
            okw = 0;
        end
        if (okw==0)
          ok = 0;
        end
      end
    end
    %
    % Link variables
    %
    offset = 0;
    nsec = 0;
    fprintf(fid, '\nCreate links for design variables');
    for k=1:naer
      ok = 0;
      while ok == 0
        okw = 1;
        fprintf(fid, '\nBay %d:', k);
        ntot_set = nrib_span(k)+1; 
        fprintf(fid, '\nNumber of zones: %d', ntot_set);
        fprintf(fid, '\nSets ID:');
        set_id = [offset : offset + ntot_set-1];
        disp(set_id);
        nset = input('        Number of sets to create: '); 
        if nset > nrib_span(k)+1
          okw = 0;
          fprintf(fid, '\nToo many sets required.');
        end
        if nset == 1
          nsec = nsec+1;
          section(nsec).prop = set_id;
        else
          for j=1:nset
            nsec = nsec+1;
            list = input('\n        List ID to use (use [], for example [0 1]): ');
            for i=1:length(list)
              if (isempty(find(set_id == list(i))))
                okw = 0;
                fprintf(fid, '\nUnknown set given.');
              end
            end
            section(nsec).prop = list;
          end
        end
        %
        ok = input('OK to proceed? Yes = 1, No = 0: ');
        if ok~=1
          ok = 0;
        end
        if (okw==0)
          ok = 0;
        end
        if ok==1
          offset = offset + ntot_set;
        else
          nsec = nsec - nset;
        end
      end
    end
    fprintf(fid,'\nInput data for structural model concluded.\n');
    okinp = input('OK to proceed? Yes = 1, No = 0: ');
    if okinp~=1
      okinp = 0;
    end
    if (okinp==0)
      figure(1); close;
      open('wing_geo.fig'); view(2); axis equal; hold on;
    end
  end % input section
  %-------------------------------------------------------------------------------
  % export mesh file
  inp.outfilename = outfilename;
  inp.param = param;
  inp.guess_model = guess_model;
  inp.guess_filename = guess_filename;
  inp.X0 = X0;
  inp.section = section;
  %
  ribs_data = femgen(outfilename, param, guess_model, X0, section);
  %
  H = figure(1);
  saveas(H,'wing_fem.fig','fig');
  bkcaero.PANE = PANE;
  bkcaero.CAERO1 = CAERO1;
  save('caero.mat','bkcaero');
  aero_mesh = aero_model(PANE, CAERO1.CFRAC, CAERO1.SFRAC, CAERO1.CPOS, CAERO1.NAME, ...
    ribs_data,nrib_span,[]);
  inp.aero_mesh = aero_mesh;
  %
  save('input.mat','inp');
  export_input_file(param, inp, [outfilename,'.m']);
  % to run this function separately:
  % load ribs_data.mat
  % load caero.mat
  %aero_model(bkcaero.PANE, bkcaero.CAERO1.CFRAC, bkcaero.CAERO1.SFRAC, bkcaero.CAERO1.CPOS, bkcaero.CAERO1.NAME, ribs_data,inp.param.nrib_span);
  %
else
  bkcaero.PANE = PANE;
  bkcaero.CAERO1 = CAERO1;
  save('caero.mat','bkcaero');
  read_input_file(input_file, outfilename);

end
end