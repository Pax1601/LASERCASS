function [ribs_data] = femgen(outname, input, guess_model, X0, section)
%
figure_plot1=1;
figure_plot2=0;
if (figure_plot1)
  figure(1); view(2);%close; figure(1); hold on; axis equal;
end
if (figure_plot2)
  figure(2); close; figure(2);  hold on; axis equal;
end
%
% number of sectors
n_sec = length(input.aft_spar);
%-------------------------------------------------------------------------------
% wing box 
x_inf=cell(1,n_sec);
y_inf=cell(1,n_sec);
z_inf=cell(1,n_sec);
%
x_sup=cell(1,n_sec);
y_sup=cell(1,n_sec);
z_sup=cell(1,n_sec);
%
vertex_x=cell(1,n_sec);
vertex_y=cell(1,n_sec);
vertex_z=cell(1,n_sec);
%
x_air_min=cell(1,n_sec);
y_air_min=cell(1,n_sec);
z_air_min=cell(1,n_sec);
%
x_air_max=cell(1,n_sec);
y_air_max=cell(1,n_sec);
z_air_max=cell(1,n_sec);
%
for i=1:n_sec
  [x_inf{i},y_inf{i},z_inf{i},x_sup{i},y_sup{i},z_sup{i},...
  vertex_x{i},vertex_y{i},vertex_z{i},...
  x_air_min{i},y_air_min{i},z_air_min{i},...
  x_air_max{i},y_air_max{i},z_air_max{i}]=...
    airfoil_box(input.foil{i},input.fore_spar(i),input.aft_spar(i),i);
end
%-------------------------------------------------------------------------------
% elastic axis points
ae=cell(1,n_sec);
for i=1:n_sec
    ae{i}=[sum(vertex_x{i})/4 sum(vertex_y{i})/4 sum(vertex_z{i})/4];
end
%-------------------------------------------------------------------------------
% plot
if figure_plot2==1
  figure(2); hold on;
  for i=1:n_sec
    plot3(x_inf{i},y_inf{i},z_inf{i},'k*');
    plot3(x_sup{i},y_sup{i},z_sup{i},'k*');
%
    plot3(vertex_x{i},vertex_y{i},vertex_z{i},'r*');
%
    plot3(x_air_min{i},y_air_min{i},z_air_min{i},'c*');
    plot3(x_air_max{i},y_air_max{i},z_air_max{i},'c*');
%
    plot3(ae{i}(1),ae{i}(2),ae{i}(3),'gs');
  end
  axis equal
end

%-------------------------------------------------------------------------------
% sectors geometry
% number of sectors
stretch = n_sec-1;
%
lungh_ae=zeros(1,stretch);
delta_ae=zeros(1,stretch);
%
c_rib=cell(1,n_sec);
%
lungh_LE=zeros(1,stretch);
lungh_TE=zeros(1,stretch);
%
vers_ae=zeros(stretch,3); 
vers_LE=zeros(stretch,3);
vers_TE=zeros(stretch,3);
%
for i=1:stretch
% ae length
  lungh_ae(i)=sqrt((ae{i+1}(1)-ae{i}(1))^2+(ae{i+1}(2)-ae{i}(2))^2+(ae{i+1}(3)-ae{i}(3))^2);
% ae cosines
  vers_ae(i,:)=[ae{i+1}(1)-ae{i}(1) ae{i+1}(2)-ae{i}(2) ae{i+1}(3)-ae{i}(3)]/lungh_ae(i);     
%  if i==2
%    offset = 0.2;
%    lungh_ae(i)=sqrt((ae{i+1}(1)-ae{i}(1)-offset)^2+...
%                     (ae{i+1}(2)-ae{i}(2))^2+(ae{i+1}(3)-ae{i}(3))^2);
%    vers_ae(i,:)=[ae{i+1}(1)-ae{i}(1)-offset ...
%                  ae{i+1}(2)-ae{i}(2) ae{i+1}(3)-ae{i}(3)]/lungh_ae(i);      
%  end
  lungh_LE(i)=sqrt((x_air_min{i+1}-x_air_min{i})^2+...
                   (y_air_min{i+1}-y_air_min{i})^2+(z_air_min{i+1}-z_air_min{i})^2);
  vers_LE(i,:)=[x_air_min{i+1}-x_air_min{i} ...
                y_air_min{i+1}-y_air_min{i} z_air_min{i+1}-z_air_min{i}]/lungh_LE(i); 
  lungh_TE(i)=sqrt((x_air_max{i+1}-x_air_max{i})^2+...
                   (y_air_max{i+1}-y_air_max{i})^2+(z_air_max{i+1}-z_air_max{i})^2);
  vers_TE(i,:)=[x_air_max{i+1}-x_air_max{i} ...
                y_air_max{i+1}-y_air_max{i} z_air_max{i+1}-z_air_max{i}]/lungh_TE(i);
% rib spacing
  delta_ae(i)=lungh_ae(i)/(input.nrib_span(i)+1);
% rib centers
  for j=1:input.nrib_span(i)
    c_rib{i}(j,:)=j*delta_ae(i)*vers_ae(i,:)+ae{i};
  end
end
%-------------------------------------------------------------------------------
% ribs
% modify cosines if ribs lay along wind
if strcmpi(input.rib_type,'wind')==1
  for i=1:stretch
    vers_wind = [0 1 vers_ae(i,3)];
    mag = norm(vers_wind);
    vers_ae(i,:)=vers_wind./mag;
  end
end
%
if figure_plot2==1
  figure(2)
  for i=1:stretch
    for j=1:input.nrib_span(i)
      plot3(c_rib{i}(j,1),c_rib{i}(j,2),c_rib{i}(j,3),'k*')
    end
  end
end
%
[rib_vertex,rib_sup,rib_inf] = rib_point_position(stretch,input.nrib_span,x_inf,y_inf,z_inf,...
                                 x_sup,y_sup,z_sup,vertex_x,vertex_y,vertex_z,c_rib);
if figure_plot2==1
  figure(2)
  for i=1:stretch
    for j=1:input.nrib_span(i)
      plot3(rib_vertex{i}{j}(:,1),rib_vertex{i}{j}(:,2),rib_vertex{i}{j}(:,3),'*')
      plot3(rib_sup{i}{j}(:,1),rib_sup{i}{j}(:,2),rib_sup{i}{j}(:,3),'*')
      plot3(rib_inf{i}{j}(:,1),rib_inf{i}{j}(:,2),rib_inf{i}{j}(:,3),'*')
      axis equal
    end
  end
end
%-------------------------------------------------------------------------------
% stringers
for i=1:n_sec
  [x_corr_sup{i},y_corr_sup{i},z_corr_sup{i},x_corr_inf{i},y_corr_inf{i},z_corr_inf{i},...
    ncorr_sup(i),ncorr_inf(i)]=stiffener_position_var(vertex_x{i},vertex_y{i},vertex_z{i},...
    x_sup{i},y_sup{i},z_sup{i},x_inf{i},y_inf{i},z_inf{i},input.nstr_up_start,...
    input.nstr_low_start,input.str_up_pitch,input.str_low_pitch,input.str_type);                                               
end
% stringers along ribs
for i=1:stretch
  for j=1:input.nrib_span(i)
    [rib_x_corr_sup{i}{j},rib_y_corr_sup{i}{j},rib_z_corr_sup{i}{j},rib_x_corr_inf{i}{j},...
    rib_y_corr_inf{i}{j},rib_z_corr_inf{i}{j},rib_ncorr_sup(i,j),rib_ncorr_inf(i,j)]=...
    stiffener_position_var_rib(rib_vertex{i}{j}(:,1)',rib_vertex{i}{j}(:,2)',rib_vertex{i}{j}(:,3)',...
    rib_sup{i}{j}(:,1)',rib_sup{i}{j}(:,2)',rib_sup{i}{j}(:,3)',rib_inf{i}{j}(:,1)',...
    rib_inf{i}{j}(:,2)',rib_inf{i}{j}(:,3)',input.nstr_up_start,input.nstr_low_start,input.str_up_pitch,input.str_low_pitch,input.str_type);          
  end
end
if figure_plot2==1
  figure(2);
  for i=1:n_sec
    plot3(x_corr_sup{i},y_corr_sup{i},z_corr_sup{i},'ko')
    plot3(x_corr_inf{i},y_corr_inf{i},z_corr_inf{i},'ko')
  end
  for i=1:stretch
    for j=1:input.nrib_span(i)
      plot3(rib_x_corr_sup{i}{j},rib_y_corr_sup{i}{j},rib_z_corr_sup{i}{j},'ko')
      plot3(rib_x_corr_inf{i}{j},rib_y_corr_inf{i}{j},rib_z_corr_inf{i}{j},'ko')
    end
  end
end 
%
coord_sect_nodes=cell(n_sec,1);
nnode_sup=zeros(1,n_sec);
nnode_inf=zeros(1,n_sec);
nnode_longant=zeros(1,n_sec);
nnode_longpost=zeros(1,n_sec);
for i=1:n_sec
  [coord_sect_nodes{i},nnode_sup(i),nnode_inf(i),nnode_longant(i),nnode_longpost(i)]=sect_nodes_position...
  (vertex_x{i},vertex_y{i},vertex_z{i},x_sup{i},y_sup{i},z_sup{i},...
  x_inf{i},y_inf{i},z_inf{i},ncorr_sup(i),ncorr_inf(i),input.nelem_fore_spar,input.nelem_aft_spar,input.nelem_btwn_str,input.str_up_pitch,input.str_low_pitch,input.str_type);                                    
end
%
if figure_plot1==1
  figure(1); hold on;
  for i=1:n_sec
    plot3(coord_sect_nodes{i}(:,1),coord_sect_nodes{i}(:,2),coord_sect_nodes{i}(:,3),'ro')
  end
  axis equal
end
% ribs nodes
for i=1:stretch
  for j=1:input.nrib_span(i)
    [coord_rib_nodes{i}{j},size_rib_nodes{i}{j}] = rib_nodes_position(rib_vertex{i}{j}(:,1)',...
    rib_vertex{i}{j}(:,2)',rib_vertex{i}{j}(:,3)',rib_sup{i}{j}(:,1)',rib_sup{i}{j}(:,2)',...
    rib_sup{i}{j}(:,3)',rib_inf{i}{j}(:,1)',rib_inf{i}{j}(:,2)',rib_inf{i}{j}(:,3)',...
    input.str_up_pitch,input.str_low_pitch,input.nelem_btwn_str,input.nelem_fore_spar,input.nelem_aft_spar,input.str_type,ncorr_sup(i),ncorr_inf(i));
  end
end
% ribs nodes rotation
size_sect_nodes=cell(1,n_sec);sect_vertex=cell(1,n_sec);
for i=1:n_sec
  size_sect_nodes{i}=[nnode_sup(i);nnode_longpost(i);nnode_inf(i);nnode_longant(i)];
  sect_vertex{i}=[vertex_x{i}' vertex_y{i}' vertex_z{i}'];
end
[coord_rib_nodes,size_rib_nodes,ver1_rot,ver2_rot,ver3_rot,ver4_rot]=...
  rib_nodes_rotation(coord_rib_nodes,size_rib_nodes,coord_sect_nodes,...
  size_sect_nodes,rib_vertex,sect_vertex,vers_ae,c_rib,stretch,input.nrib_span);
if figure_plot1==1
  figure(1)
  for i=1:stretch
    for j=1:input.nrib_span(i)
      plot3(coord_rib_nodes{i}{j}(:,1),coord_rib_nodes{i}{j}(:,2),coord_rib_nodes{i}{j}(:,3),'bo')
    end
  end
  axis equal
end
% spanwise nodes
coord_nodes=nodes_position(coord_sect_nodes,coord_rib_nodes,size_sect_nodes,size_rib_nodes,...
                           input.nelem_btwn_rib,stretch,input.nrib_span);
if figure_plot1==1
  figure(1)
  for i=1:stretch
    for j=1:input.nrib_span(i)+1
      for w=1:input.nelem_btwn_rib 
        plot3(coord_nodes{i}{j}{w}(:,1),coord_nodes{i}{j}{w}(:,2),coord_nodes{i}{j}{w}(:,3),'r.')
      end
    end
  end
  axis equal
end
% vertices
vert1=struct([]);vert2=struct([]);vert3=struct([]);vert4=struct([]);
for i=1:length(vertex_x)-1
  vert1(end+1).coord=[vertex_x{i}(1) vertex_y{i}(1) vertex_z{i}(1)];
  vert2(end+1).coord=[vertex_x{i}(2) vertex_y{i}(2) vertex_z{i}(2)];
  vert3(end+1).coord=[vertex_x{i}(3) vertex_y{i}(3) vertex_z{i}(3)];
  vert4(end+1).coord=[vertex_x{i}(4) vertex_y{i}(4) vertex_z{i}(4)];
  for j=1:length(rib_vertex{i})
    vert1(end+1).coord=ver1_rot{i}{j};
    vert2(end+1).coord=ver2_rot{i}{j};
    vert3(end+1).coord=ver3_rot{i}{j};
    vert4(end+1).coord=ver4_rot{i}{j};       
  end
end
vert1(end+1).coord=[vertex_x{length(vertex_x)}(1) vertex_y{length(vertex_x)}(1) ...
                    vertex_z{length(vertex_x)}(1)];
vert2(end+1).coord=[vertex_x{length(vertex_x)}(2) vertex_y{length(vertex_x)}(2) ...
                    vertex_z{length(vertex_x)}(2)];
vert3(end+1).coord=[vertex_x{length(vertex_x)}(3) vertex_y{length(vertex_x)}(3) ...
                    vertex_z{length(vertex_x)}(3)];
vert4(end+1).coord=[vertex_x{length(vertex_x)}(4) vertex_y{length(vertex_x)}(4) ...
                    vertex_z{length(vertex_x)}(4)];
%-------------------------------------------------------------------------------
% GRID cards
[grid_coord,spc_nodes,vert1,vert2,vert3,vert4] = ...
    grid_generator(coord_sect_nodes,coord_nodes,input.nelem_btwn_rib,stretch,input.nrib_span,size_sect_nodes,...
    size_rib_nodes,vert1,vert2,vert3,vert4);  
nnodes_tot=size(grid_coord,1);
%-------------------------------------------------------------------------------
% CQUAD4 cards
[n_baie,nelem_tot] = quad4_generator(grid_coord,size_sect_nodes,size_rib_nodes,input.nelem_btwn_rib,...
                                      stretch,input.nrib_span);
%-------------------------------------------------------------------------------
% initialize solution
%-------------------------------------------------------------------------------
n_rib_station=(n_baie/input.nelem_btwn_rib)+1;
%
if ~isempty(guess_model)
%
  nodey = [0; cumsum(guess_model.stick_model.wing.Lbeam_thick)];
%
  if guess_model.pdcylin.wing.kcon>=9
%
    try
    [start_val, span_pos] = starting_point_interp(stretch,input.nrib_span,...
                 nodey,input.nelem_btwn_rib,coord_nodes,size_rib_nodes,...
                 size_sect_nodes,guess_model.str_prop.wing.ttorq,...
                 guess_model.str_prop.wing.twbs,guess_model.str_prop.wing.Aeq);
    catch
    [start_val, span_pos] = starting_point_interp(stretch,input.nrib_span,...
                 nodey,input.nelem_btwn_rib,coord_nodes,size_rib_nodes,...
                 size_sect_nodes,guess_model.str_prop.wing.skin.tskin,...
                 guess_model.str_prop.wing.web.tw,guess_model.str_prop.wing.skin.Astr);
    end
  else
    nrcth   = guess_model.pdcylin.stick.nwing_carryth;
    I1 = meancouple(guess_model.str_prop.wing.I1);
    I2 = meancouple(guess_model.str_prop.wing.I2);
    I12 = zeros(length(I1),1);
    J = meancouple(guess_model.str_prop.wing.J);
    %
    Area = zeros(length(I1),1);
    Area(nrcth+1:end) = guess_model.str_prop.wing.mbox./(guess_model.pdcylin.wing.dsw*guess_model.stick_model.wing.Lbeam_thick(nrcth+1:end));
    Area(1:nrcth) = Area(nrcth+1);
    nodey_1_2 = meancouple(nodey);
    [start_val, span_pos] = starting_point_opt(stretch,input.nrib_span,Area,I1,I2,I12,J,...
                 nodey_1_2,input.nelem_btwn_rib,coord_nodes,size_rib_nodes,...
                 coord_sect_nodes,size_sect_nodes,input.nelem_btwn_str);
    span_pos = span_pos(1:end-1);
  end

  save('start_val.mat','start_val','span_pos');

else
  start_val = zeros(n_rib_station,3);
  start_val(:,1) = X0(1);
  start_val(:,2) = X0(2);
  start_val(:,3) = X0(3);
end
%-------------------------------------------------------------------------------
% export PBEAM
[A_pbeam,pbeam_id]=pbeam_generator(n_rib_station,start_val);
%-------------------------------------------------------------------------------
% export PSHELL
if input.mat_type==1
    pshell_id = pshell_generator(n_rib_station,start_val);
elseif input.mat_type==2
    pcomp_id = pcomp_generator(n_rib_station);
end
%-------------------------------------------------------------------------------
% export CBEAM
nelem_tot = beam_generator(coord_sect_nodes,coord_nodes,size_sect_nodes,size_rib_nodes,...
              stretch,input.nrib_span,input.nelem_btwn_str,nelem_tot,input.nelem_btwn_rib,start_val(:,3));
%-------------------------------------------------------------------------------
% export ribs (grids and elements)
if input.mat_type==2
    pshell_id=pcomp_id;
end
[n_rib,rib_id,spc_nodes,fuse_nodes,spar_v,rib_centre]=rib_generator(grid_coord,size_sect_nodes,size_rib_nodes,...
    stretch,input.nrib_span,input.nelem_btwn_str,input.nelem_btwn_rib,nelem_tot,nnodes_tot,input.nelem_rib,...
    input.mat_type,spc_nodes, input.rib_thick);
if figure_plot1==1
  figure(1)
  for i=1:size(rib_centre,2)
    plot3(rib_centre(i).coord(:,1),rib_centre(i).coord(:,2),rib_centre(i).coord(:,3),'k*')
  end
  axis equal
end
%-------------------------------------------------------------------------------
% export MAT
mat_generator(input.mat_type);
%-------------------------------------------------------------------------------
% export MAIN file for SOL 101, 145, 200
main_generator_101([outname,'_101.dat'], input.mat_type);
main_generator_145([outname,'_145.dat'], input.mat_type);
main_generator_200([outname,'_200.dat'], input.mat_type);
%-------------------------------------------------------------------------------
%Funzione per la generazione delle DESVAR di nastran
if input.mat_type==1
    desvar_generator(section,start_val);
elseif input.mat_type==2
    desvar_CF_generator(section,start_val);
end

%Funzione per la generazione delle DVPREL1 di nastran
if input.mat_type==1
    dvprel1_generator(section,n_rib_station);
elseif input.mat_type==2
    dvprel1_CF_generator(section,n_rib_station);
end

%Funzione per la generazione delle DVPREL2 di nastran
dvprel2_generator(section,n_rib_station);

%Funzione per la generazione delle DRESP1 di nastran
if input.mat_type==1
    dresp1_generator(pbeam_id,pshell_id,rib_id);
elseif input.mat_type==2
    dresp1_CF_generator(pbeam_id,pshell_id);
end

%Funzione per la generazione delle DRESP2 di nastran
if input.mat_type==2
    dresp2_CF_generator(section);
end

%Funzione per la generazione delle DCONSTR di nastran
if input.mat_type==1
    dconstr_generator(section);
elseif input.mat_type==2
    dconstr_CF_generator(section);
end

%Funzione per la generazione dei vincoli di buckling
if input.mat_type==1
    analytical_buckling(section);
elseif input.mat_type==2
    analytical_buckling_CF(section);
end
% symmetry constraints
spc_sym(spc_nodes);
% create fuse constraints
fuse_spar = 2 + input.nrib_span(1);
spc_fuse(fuse_nodes{1}, fuse_nodes{2}, spar_v);
%
% export blank file for RBE3 points
%
fout=fopen('RBE3.dat','w');
fclose(fout);
ribs_data.C = rib_centre;
ribs_data.V1 = vert1;
ribs_data.V2 = vert2;
ribs_data.V3 = vert3;
ribs_data.V4 = vert4;
ribs_data.vers_ae = vers_ae;
save('ribs_data.mat', 'ribs_data');
%RBE3_generator(rib_centre,vert1,vert2,vert3,vert4)
%




