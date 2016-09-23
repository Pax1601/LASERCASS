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

function varargout = ChEcK(varargin)
% CHECK M-file for ChEcK.fig
%      CHECK, by itself, creates a new CHECK or raises the existing
%      singleton*.
%
%      H = CHECK returns the handle to a new CHECK or the handle to
%      the existing singleton*.
%
%      CHECK('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CHECK.M with the given input arguments.
%
%      CHECK('Property','Value',...) creates a new CHECK or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ChEcK_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ChEcK_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ChEcK

% Last Modified by GUIDE v2.5 05-Mar-2012 14:40:32

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ChEcK_OpeningFcn, ...
                   'gui_OutputFcn',  @ChEcK_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before ChEcK is made visible.
function ChEcK_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ChEcK (see VARARGIN)

% Choose default command line output for ChEcK
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
if gui_param.guess.model.guessstd
    set(handles.MassConf,'Enable','off');
end

% UIWAIT makes ChEcK wait for user response (see UIRESUME)
% uiwait(handles.figure1);

global geo_model
if geo_model.aircraft.Horizontal_tail.present == 1  && geo_model.aircraft.Horizontal_tail.Elevator.present == 1
    set(handles.Htail,'Enable','on');
end
if geo_model.aircraft.Vertical_tail.present == 1 && geo_model.aircraft.Vertical_tail.Rudder.present == 1
    set(handles.Vtail,'Enable','on');
end
if geo_model.aircraft.Canard.present == 1 && geo_model.aircraft.Canard.Elevator.present == 1
    set(handles.Canardcheckbox3,'Enable','on');
end
% --- Outputs from this function are returned to the command line.
function varargout = ChEcK_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

 
% --- Executes on button press in Aerodynamic.
function Aerodynamic_Callback(hObject, eventdata, handles)
% hObject    handle to Aerodynamic (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global geo_model
global beam_model
beam_model = geo_model.beam_model;
plot_control_aelink2(2);
view([-37.5 30]); 
axis equal
clear beam_model
% --- Executes on button press in structural.
function structural_Callback(hObject, eventdata, handles)
% hObject    handle to structural (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global geo_model
color = ['ork';...
         'ogk';...
         'obk';...
         'ock';...
         'omk';...
         'oyk';...
         'owk';...
         'vrk';...
         'vgk';...
         'vbk';...
         'vck';...
         'vmk';...
         'vyk';...
         'vwk';...
         '<rk';...
         '<gk';...
         '<bk';...
         '<ck';...
         '<mk';...
         '<yk';...
         '<wk';...
         'drk';...
         'dgk';...
         'dbk';...
         'dck';...
         'dmk';...
         'dyk';...
         'dwk';...
         '^rk';...
         '^gk'];
legenda{1} = 'Wing';
legenda{2} = 'Wing2';
legenda{3} = 'Htail';
legenda{4} = 'Vtail';
legenda{5} = 'Fuselage';
legenda{6} = 'Landing gear';
legenda{7} = 'PowerPlant 1';
legenda{8} = 'PowerPlant 2';
legenda{9} = 'Aux L.G. 2';
legenda{10} = 'Vtail2';
legenda{11} = 'Canard';
legenda{12} = 'Tbooms';
legenda{13} = '';
legenda{14} = '';
legenda{15} = '';
legenda{16} = '';
legenda{17} = 'Furniture';
legenda{18} = 'Wing Tanks';
legenda{19} = 'Centre F. T.';
legenda{20} = 'Auxiliary T.';
legenda{21} = 'Interior';
legenda{22} = 'Pilots';
legenda{23} = 'Crew';
legenda{24} = 'Passengers';
legenda{25} = 'Bagagge';
legenda{26} = '';
legenda{27} = '';
legenda{28} = '';
legenda{29} = '';
legenda{30} = '';
legenda{31} = 'Stick';
legenda{32} = 'Spar';



IND = find(geo_model.WB(:,4,1)~=0);
%
fig = figure(2);
close(fig);
fig = figure(2);
subplot(1,2,1)
hold on;grid on;
for i = 1 :length(IND)
    plot3(geo_model.WB(IND(i),1),geo_model.WB(IND(i),2),geo_model.WB(IND(i),3),color(IND(i),1),'markersize',10,'markerfacecolor',color(IND(i),2),'markeredgecolor',color(IND(i),3),'linewidth',2);
    legenda{IND(i)} = [legenda{IND(i)} ,' = ', num2str(geo_model.WB(IND(i),4)) ,' Kg'];
end
% wing
plot3(geo_model.stick.nodes.winrC2(1,:),geo_model.stick.nodes.winrC2(2,:),geo_model.stick.nodes.winrC2(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
plot3(geo_model.geo.wing.xx+geo_model.geo.wing.rs*0.5,geo_model.geo.wing.yy,geo_model.geo.wing.zz,'k--','linewidth',2)

% H = legend(legenda([IND;31;32]),'location','WestOutside');
% set(H,'fontsize',14)
% set(H,'interpreter','latex')
% clear H
plot3(geo_model.stick.nodes.winlC2(1,:),geo_model.stick.nodes.winlC2(2,:),geo_model.stick.nodes.winlC2(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])

plot3(geo_model.geo.wing.xx-geo_model.geo.wing.rs*0.5,geo_model.geo.wing.yy,geo_model.geo.wing.zz,'k--','linewidth',2)
plot3(geo_model.geo.wing.xx+geo_model.geo.wing.rs*0.5,-geo_model.geo.wing.yy,geo_model.geo.wing.zz,'k--','linewidth',2)
plot3(geo_model.geo.wing.xx-geo_model.geo.wing.rs*0.5,-geo_model.geo.wing.yy,geo_model.geo.wing.zz,'k--','linewidth',2)
% fuse
plot3(geo_model.stick.nodes.fuse(1,:),geo_model.stick.nodes.fuse(2,:),geo_model.stick.nodes.fuse(3,:),'k-s','linewidth',1,'markersize',8,'markerfacecolor',[0.5,0.5,0.5])

% Htail
if geo_model.aircraft.Horizontal_tail.present == 1
    plot3(geo_model.stick.nodes.horrC2(1,:),geo_model.stick.nodes.horrC2(2,:),geo_model.stick.nodes.horrC2(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.stick.nodes.horlC2(1,:),geo_model.stick.nodes.horlC2(2,:),geo_model.stick.nodes.horlC2(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.geo.htail.xx+geo_model.geo.htail.rs*0.5,geo_model.geo.htail.yy,geo_model.geo.htail.zz,'k--','linewidth',2)
    plot3(geo_model.geo.htail.xx-geo_model.geo.htail.rs*0.5,geo_model.geo.htail.yy,geo_model.geo.htail.zz,'k--','linewidth',2)
    plot3(geo_model.geo.htail.xx+geo_model.geo.htail.rs*0.5,-geo_model.geo.htail.yy,geo_model.geo.htail.zz,'k--','linewidth',2)
    plot3(geo_model.geo.htail.xx-geo_model.geo.htail.rs*0.5,-geo_model.geo.htail.yy,geo_model.geo.htail.zz,'k--','linewidth',2)
end

% Vtail
if geo_model.aircraft.Vertical_tail.present == 1
    plot3(geo_model.stick.nodes.vert(1,:),geo_model.stick.nodes.vert(2,:),geo_model.stick.nodes.vert(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.geo.vtail.xx+geo_model.geo.vtail.rs*0.5,geo_model.geo.vtail.yy,geo_model.geo.vtail.zz,'k--','linewidth',2)
    plot3(geo_model.geo.vtail.xx-geo_model.geo.vtail.rs*0.5,geo_model.geo.vtail.yy,geo_model.geo.vtail.zz,'k--','linewidth',2)
    if geo_model.aircraft.Vertical_tail.Twin_tail == 1
        plot3(geo_model.stick.nodes.vert2(1,:),geo_model.stick.nodes.vert2(2,:),geo_model.stick.nodes.vert2(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
        plot3(geo_model.geo.vtail.xx+geo_model.geo.vtail.rs*0.5,-geo_model.geo.vtail.yy,geo_model.geo.vtail.zz,'k--','linewidth',2)
        plot3(geo_model.geo.vtail.xx-geo_model.geo.vtail.rs*0.5,-geo_model.geo.vtail.yy,geo_model.geo.vtail.zz,'k--','linewidth',2)
    end
end

% Canard
if geo_model.aircraft.Canard.present == 1
    plot3(geo_model.stick.nodes.canrC2(1,:),geo_model.stick.nodes.canrC2(2,:),geo_model.stick.nodes.canrC2(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.stick.nodes.canlC2(1,:),geo_model.stick.nodes.canlC2(2,:),geo_model.stick.nodes.canlC2(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.geo.canard.xx+geo_model.geo.canard.rs*0.5,geo_model.geo.canard.yy,geo_model.geo.canard.zz,'k--','linewidth',2)
    plot3(geo_model.geo.canard.xx-geo_model.geo.canard.rs*0.5,geo_model.geo.canard.yy,geo_model.geo.canard.zz,'k--','linewidth',2)
    plot3(geo_model.geo.canard.xx+geo_model.geo.canard.rs*0.5,-geo_model.geo.canard.yy,geo_model.geo.canard.zz,'k--','linewidth',2)
    plot3(geo_model.geo.canard.xx-geo_model.geo.canard.rs*0.5,-geo_model.geo.canard.yy,geo_model.geo.canard.zz,'k--','linewidth',2)
end

% tbooms
if geo_model.aircraft.Tailbooms.present == 1
    plot3(geo_model.stick.nodes.tboomsr(1,:),geo_model.stick.nodes.tboomsr(2,:),geo_model.stick.nodes.tboomsr(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.stick.nodes.tboomsl(1,:),geo_model.stick.nodes.tboomsl(2,:),geo_model.stick.nodes.tboomsl(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
end

view([-37.5 30]);
axis equal
H = title('Exported model');
set(H,'fontsize',14); 
set(H,'interpreter','latex')
clear H
subplot(1,2,2)

hold on;grid on;
for i = 1 :length(IND)
    plot3(geo_model.WB(IND(i),1),geo_model.WB(IND(i),2),geo_model.WB(IND(i),3),color(IND(i),1),'markersize',10,'markerfacecolor',color(IND(i),2),'markeredgecolor',color(IND(i),3),'linewidth',2);
end
% wing
plot3(geo_model.stick.nodes.winrC2_thick(1,:),geo_model.stick.nodes.winrC2_thick(2,:),geo_model.stick.nodes.winrC2_thick(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
plot3(geo_model.geo.wing.xx+geo_model.geo.wing.rs*0.5,geo_model.geo.wing.yy,geo_model.geo.wing.zz,'k--','linewidth',2)
% H = legend(legenda([IND(1:meta);31;32]),'location','SouthOutside','orientation','horizontal');
plot3(geo_model.stick.nodes.winlC2_thick(1,:),geo_model.stick.nodes.winlC2_thick(2,:),geo_model.stick.nodes.winlC2_thick(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])

plot3(geo_model.geo.wing.xx-geo_model.geo.wing.rs*0.5,geo_model.geo.wing.yy,geo_model.geo.wing.zz,'k--','linewidth',2)
plot3(geo_model.geo.wing.xx+geo_model.geo.wing.rs*0.5,-geo_model.geo.wing.yy,geo_model.geo.wing.zz,'k--','linewidth',2)
plot3(geo_model.geo.wing.xx-geo_model.geo.wing.rs*0.5,-geo_model.geo.wing.yy,geo_model.geo.wing.zz,'k--','linewidth',2)
% fuse
plot3(geo_model.stick.nodes.fuse_thick(1,:),geo_model.stick.nodes.fuse_thick(2,:),geo_model.stick.nodes.fuse_thick(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])

% Htail
if geo_model.aircraft.Horizontal_tail.present == 1
    plot3(geo_model.stick.nodes.horrC2_thick(1,:),geo_model.stick.nodes.horrC2_thick(2,:),geo_model.stick.nodes.horrC2_thick(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.stick.nodes.horlC2_thick(1,:),geo_model.stick.nodes.horlC2_thick(2,:),geo_model.stick.nodes.horlC2_thick(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.geo.htail.xx+geo_model.geo.htail.rs*0.5,geo_model.geo.htail.yy,geo_model.geo.htail.zz,'k--','linewidth',2)
    plot3(geo_model.geo.htail.xx-geo_model.geo.htail.rs*0.5,geo_model.geo.htail.yy,geo_model.geo.htail.zz,'k--','linewidth',2)
    plot3(geo_model.geo.htail.xx+geo_model.geo.htail.rs*0.5,-geo_model.geo.htail.yy,geo_model.geo.htail.zz,'k--','linewidth',2)
    plot3(geo_model.geo.htail.xx-geo_model.geo.htail.rs*0.5,-geo_model.geo.htail.yy,geo_model.geo.htail.zz,'k--','linewidth',2)
end

% Vtail
if geo_model.aircraft.Vertical_tail.present == 1
    plot3(geo_model.stick.nodes.vert_thick(1,:),geo_model.stick.nodes.vert_thick(2,:),geo_model.stick.nodes.vert_thick(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.geo.vtail.xx+geo_model.geo.vtail.rs*0.5,geo_model.geo.vtail.yy,geo_model.geo.vtail.zz,'k--','linewidth',2)
    plot3(geo_model.geo.vtail.xx-geo_model.geo.vtail.rs*0.5,geo_model.geo.vtail.yy,geo_model.geo.vtail.zz,'k--','linewidth',2)
    if geo_model.aircraft.Vertical_tail.Twin_tail == 1
        plot3(geo_model.stick.nodes.vert2_thick(1,:),geo_model.stick.nodes.vert2_thick(2,:),geo_model.stick.nodes.vert2_thick(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
        plot3(geo_model.geo.vtail.xx+geo_model.geo.vtail.rs*0.5,-geo_model.geo.vtail.yy,geo_model.geo.vtail.zz,'k--','linewidth',2)
        plot3(geo_model.geo.vtail.xx-geo_model.geo.vtail.rs*0.5,-geo_model.geo.vtail.yy,geo_model.geo.vtail.zz,'k--','linewidth',2)
    end
end

% Canard
if geo_model.aircraft.Canard.present == 1
    plot3(geo_model.stick.nodes.canrC2_thick(1,:),geo_model.stick.nodes.canrC2_thick(2,:),geo_model.stick.nodes.canrC2_thick(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.stick.nodes.canlC2_thick(1,:),geo_model.stick.nodes.canlC2_thick(2,:),geo_model.stick.nodes.canlC2_thick(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.geo.canard.xx+geo_model.geo.canard.rs*0.5,geo_model.geo.canard.yy,geo_model.geo.canard.zz,'k--','linewidth',2)
    plot3(geo_model.geo.canard.xx-geo_model.geo.canard.rs*0.5,geo_model.geo.canard.yy,geo_model.geo.canard.zz,'k--','linewidth',2)
    plot3(geo_model.geo.canard.xx+geo_model.geo.canard.rs*0.5,-geo_model.geo.canard.yy,geo_model.geo.canard.zz,'k--','linewidth',2)
    plot3(geo_model.geo.canard.xx-geo_model.geo.canard.rs*0.5,-geo_model.geo.canard.yy,geo_model.geo.canard.zz,'k--','linewidth',2)
end

% tbooms
if geo_model.aircraft.Tailbooms.present == 1
    plot3(geo_model.stick.nodes.tboomsr_thick(1,:),geo_model.stick.nodes.tboomsr_thick(2,:),geo_model.stick.nodes.tboomsr_thick(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.stick.nodes.tboomsl_thick(1,:),geo_model.stick.nodes.tboomsl_thick(2,:),geo_model.stick.nodes.tboomsl_thick(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
end
% axis equal
view([-37.5 30]);
axis equal
H = title('Internal model');
set(H,'fontsize',14); 
set(H,'interpreter','latex')
clear H


% subplot(1,3,3)
% for i = 1 :length(IND)
%     ff(i) = plot3(geo_model.WB(IND(i),1),geo_model.WB(IND(i),2),geo_model.WB(IND(i),3),color(IND(i),1),'markersize',10,'markerfacecolor',color(IND(i),2),'markeredgecolor',color(IND(i),3),'linewidth',2);
%     legenda{IND(i)} = [legenda{IND(i)} ,' = ', num2str(geo_model.WB(IND(i),4)) ,' Kg'];
% end 
% ff(i+1) = plot3(geo_model.stick.nodes.winrC2(1,:),geo_model.stick.nodes.winrC2(2,:),geo_model.stick.nodes.winrC2(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
% ff(i+2) = plot3(geo_model.geo.wing.xx+geo_model.geo.wing.rs*0.5,geo_model.geo.wing.yy,geo_model.geo.wing.zz,'k--','linewidth',2)
% 
% 
% H = legend(legenda([IND;31;32]),'location','WestOutside');
% clear ff
set(fig, 'Visible','on', 'Name', 'NeoCASS - Model plot', 'NumberTitle','off');
 


% --- Executes on button press in Aeroelastic.
function Aeroelastic_Callback(hObject, eventdata, handles)
% hObject    handle to Aeroelastic (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global geo_model
global beam_model
beam_model = geo_model.beam_model;
plot_beam_model(2);
color = ['ork';...
         'ogk';...
         'obk';...
         'ock';...
         'omk';...
         'oyk';...
         'owk';...
         'vrk';...
         'vgk';...
         'vbk';...
         'vck';...
         'vmk';...
         'vyk';...
         'vwk';...
         '<rk';...
         '<gk';...
         '<bk';...
         '<ck';...
         '<mk';...
         '<yk';...
         '<wk';...
         'drk';...
         'dgk';...
         'dbk';...
         'dck';...
         'dmk';...
         'dyk';...
         'dwk';...
         '^rk';...
         '^gk'];
     
legenda{1} = 'Wing';
legenda{2} = 'Wing2';
legenda{3} = 'Htail';
legenda{4} = 'Vtail';
legenda{5} = 'Fuselage';
legenda{6} = 'Landing gear';
legenda{7} = 'PowerPlant 1';
legenda{8} = 'PowerPlant 2';
legenda{9} = 'Aux L.G. 2';
legenda{10} = 'Vtail2';
legenda{11} = 'Canard';
legenda{12} = 'Tbooms';
legenda{13} = '';
legenda{14} = '';
legenda{15} = '';
legenda{16} = '';
legenda{17} = 'Furniture';
legenda{18} = 'Wing Tanks';
legenda{19} = 'Centre F. T.';
legenda{20} = 'Auxiliary T.';
legenda{21} = 'Interior';
legenda{22} = 'Pilots';
legenda{23} = 'Crew';
legenda{24} = 'Passengers';
legenda{25} = 'Bagagge';
legenda{26} = '';
legenda{27} = '';
legenda{28} = '';
legenda{29} = '';
legenda{30} = ''; 
legenda{31} = 'Stick';
legenda{32} = 'Spar';

IND = find(geo_model.WB(:,4,1)~=0);
axis normal
hold on;grid on
for i = 1 :length(IND)
    ff(i)= plot3(geo_model.WB(IND(i),1),geo_model.WB(IND(i),2),geo_model.WB(IND(i),3),color(IND(i),1),'markersize',10,'markerfacecolor',color(IND(i),2),'markeredgecolor',color(IND(i),3),'linewidth',2);
    legenda{IND(i)} = [legenda{IND(i)} ,' = ', num2str(geo_model.WB(IND(i),4),'%.2g') ,' Kg'];
end
% wing
ff(i+1) = plot3(geo_model.stick.nodes.winrC2(1,:),geo_model.stick.nodes.winrC2(2,:),geo_model.stick.nodes.winrC2(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5]);
ff(i+2) = plot3(geo_model.geo.wing.xx+geo_model.geo.wing.rs*0.5,geo_model.geo.wing.yy,geo_model.geo.wing.zz,'k--','linewidth',2);

 H = legend(ff,legenda([IND;31;32]),'location','WestOutside');


% plot3(geo_model.stick.nodes.winrC2(1,:),geo_model.stick.nodes.winrC2(2,:),geo_model.stick.nodes.winrC2(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
plot3(geo_model.stick.nodes.winlC2(1,:),geo_model.stick.nodes.winlC2(2,:),geo_model.stick.nodes.winlC2(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
% plot3(geo_model.geo.wing.xx+geo_model.geo.wing.rs*0.5,geo_model.geo.wing.yy,geo_model.geo.wing.zz,'k--','linewidth',2)
plot3(geo_model.geo.wing.xx-geo_model.geo.wing.rs*0.5,geo_model.geo.wing.yy,geo_model.geo.wing.zz,'k--','linewidth',2)
plot3(geo_model.geo.wing.xx+geo_model.geo.wing.rs*0.5,-geo_model.geo.wing.yy,geo_model.geo.wing.zz,'k--','linewidth',2)
plot3(geo_model.geo.wing.xx-geo_model.geo.wing.rs*0.5,-geo_model.geo.wing.yy,geo_model.geo.wing.zz,'k--','linewidth',2)
% fuse
plot3(geo_model.stick.nodes.fuse(1,:),geo_model.stick.nodes.fuse(2,:),geo_model.stick.nodes.fuse(3,:),'k-s','linewidth',1,'markersize',8,'markerfacecolor',[0.5,0.5,0.5])

% Htail
if geo_model.aircraft.Horizontal_tail.present == 1
    plot3(geo_model.stick.nodes.horrC2(1,:),geo_model.stick.nodes.horrC2(2,:),geo_model.stick.nodes.horrC2(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.stick.nodes.horlC2(1,:),geo_model.stick.nodes.horlC2(2,:),geo_model.stick.nodes.horlC2(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.geo.htail.xx+geo_model.geo.htail.rs*0.5,geo_model.geo.htail.yy,geo_model.geo.htail.zz,'k--','linewidth',2)
    plot3(geo_model.geo.htail.xx-geo_model.geo.htail.rs*0.5,geo_model.geo.htail.yy,geo_model.geo.htail.zz,'k--','linewidth',2)
    plot3(geo_model.geo.htail.xx+geo_model.geo.htail.rs*0.5,-geo_model.geo.htail.yy,geo_model.geo.htail.zz,'k--','linewidth',2)
    plot3(geo_model.geo.htail.xx-geo_model.geo.htail.rs*0.5,-geo_model.geo.htail.yy,geo_model.geo.htail.zz,'k--','linewidth',2)
end

% Vtail
if geo_model.aircraft.Vertical_tail.present == 1
    plot3(geo_model.stick.nodes.vert(1,:),geo_model.stick.nodes.vert(2,:),geo_model.stick.nodes.vert(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.geo.vtail.xx+geo_model.geo.vtail.rs*0.5,geo_model.geo.vtail.yy,geo_model.geo.vtail.zz,'k--','linewidth',2)
    plot3(geo_model.geo.vtail.xx-geo_model.geo.vtail.rs*0.5,geo_model.geo.vtail.yy,geo_model.geo.vtail.zz,'k--','linewidth',2)
    if geo_model.aircraft.Vertical_tail.Twin_tail == 1
        plot3(geo_model.stick.nodes.vert2(1,:),geo_model.stick.nodes.vert2(2,:),geo_model.stick.nodes.vert2(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
        plot3(geo_model.geo.vtail.xx+geo_model.geo.vtail.rs*0.5,-geo_model.geo.vtail.yy,geo_model.geo.vtail.zz,'k--','linewidth',2)
        plot3(geo_model.geo.vtail.xx-geo_model.geo.vtail.rs*0.5,-geo_model.geo.vtail.yy,geo_model.geo.vtail.zz,'k--','linewidth',2)
    end
end

% Canard
if geo_model.aircraft.Canard.present == 1
    plot3(geo_model.stick.nodes.canrC2(1,:),geo_model.stick.nodes.canrC2(2,:),geo_model.stick.nodes.canrC2(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.stick.nodes.canlC2(1,:),geo_model.stick.nodes.canlC2(2,:),geo_model.stick.nodes.canlC2(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.geo.canard.xx+geo_model.geo.canard.rs*0.5,geo_model.geo.canard.yy,geo_model.geo.canard.zz,'k--','linewidth',2)
    plot3(geo_model.geo.canard.xx-geo_model.geo.canard.rs*0.5,geo_model.geo.canard.yy,geo_model.geo.canard.zz,'k--','linewidth',2)
    plot3(geo_model.geo.canard.xx+geo_model.geo.canard.rs*0.5,-geo_model.geo.canard.yy,geo_model.geo.canard.zz,'k--','linewidth',2)
    plot3(geo_model.geo.canard.xx-geo_model.geo.canard.rs*0.5,-geo_model.geo.canard.yy,geo_model.geo.canard.zz,'k--','linewidth',2)
end

% tbooms
if geo_model.aircraft.Tailbooms.present == 1
    plot3(geo_model.stick.nodes.tboomsr(1,:),geo_model.stick.nodes.tboomsr(2,:),geo_model.stick.nodes.tboomsr(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
    plot3(geo_model.stick.nodes.tboomsl(1,:),geo_model.stick.nodes.tboomsl(2,:),geo_model.stick.nodes.tboomsl(3,:),'k-s','linewidth',1,'markersize',6,'markerfacecolor',[0.5,0.5,0.5])
end
view([-37.5 30]);
clear beam_model
axis equal


% --- Executes on button press in Htail.
function Htail_Callback(hObject, eventdata, handles)
% hObject    handle to Htail (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Htail


% --- Executes on button press in Vtail.
function Vtail_Callback(hObject, eventdata, handles)
% hObject    handle to Vtail (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Vtail


% --- Executes on button press in Canardcheckbox3.
function Canardcheckbox3_Callback(hObject, eventdata, handles)
% hObject    handle to Canardcheckbox3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Canardcheckbox3


% --- Executes on button press in Run.
function Run_Callback(hObject, eventdata, handles)
% hObject    handle to Run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global geo_model NMAX EPS

NMAX = str2double(get(handles.edit1,'String')); % handles.edit1 = edit_nmax
EPS = str2double(get(handles.edit2,'String')); % handles.edit1 = edit_eps

geo_model.run = 1;
geo_model.aircraft.Horizontal_tail.Allmovable = get(handles.Htail,'Value');
geo_model.aircraft.Vertical_tail.Allmovable = get(handles.Vtail,'Value');
geo_model.aircraft.Canard.Allmovable = get(handles.Canardcheckbox3,'Value');
if ~isfield(geo_model.pdcylin,'MassConf')
    geo_model.pdcylin.MassConf = [];
end
close ChEcK
% close(1)


% --- Executes on button press in Exit.
function Exit_Callback(hObject, eventdata, handles)
% hObject    handle to Exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global geo_model
geo_model.run = 0;
close ChEcK
% close(1)


% --- Executes on button press in MassConf.
function MassConf_Callback(hObject, eventdata, handles)
% hObject    handle to MassConf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global geo_model

uiwait(MassConfiguration)





function edit_nmax_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function edit_nmax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_eps_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit_eps_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
