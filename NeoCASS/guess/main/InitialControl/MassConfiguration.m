%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2008 - 2013
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

function varargout = MassConfiguration(varargin)
% MASSCONFIGURATION M-file for MassConfiguration.fig
%      MASSCONFIGURATION, by itself, creates a new MASSCONFIGURATION or raises the existing
%      singleton*.
%
%      H = MASSCONFIGURATION returns the handle to a new MASSCONFIGURATION or the handle to
%      the existing singleton*.
%
%      MASSCONFIGURATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MASSCONFIGURATION.M with the given input arguments.
%
%      MASSCONFIGURATION('Property','Value',...) creates a new MASSCONFIGURATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MassConfiguration_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MassConfiguration_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MassConfiguration

% Last Modified by GUIDE v2.5 08-Nov-2013 11:19:48

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @MassConfiguration_OpeningFcn, ...
                   'gui_OutputFcn',  @MassConfiguration_OutputFcn, ...
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


% --- Executes just before MassConfiguration is made visible.
function MassConfiguration_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MassConfiguration (see VARARGIN)

% Choose default command line output for MassConfiguration
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes MassConfiguration wait for user response (see UIRESUME)
% uiwait(handles.figure1);
set(handles.SelVal,'Enable','off');


% --- Outputs from this function are returned to the command line.
function varargout = MassConfiguration_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function Nconf_Callback(hObject, eventdata, handles)
% hObject    handle to Nconf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Nconf as text
%        str2double(get(hObject,'String')) returns contents of Nconf as a double
nconf = str2double(get(handles.Nconf,'string'));
if ~isnan(nconf) && nconf ~= 0
   set(handles.SelVal,'Enable','on');
else
    set(handles.SelVal,'Enable','off'); 
end
% --- Executes during object creation, after setting all properties.
function Nconf_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Nconf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in SelVal.
function SelVal_Callback(hObject, eventdata, handles)
% hObject    handle to SelVal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global geo_model
global Mass_conf

if isfield(geo_model.pdcylin,'MassConf')
    geo_model.pdcylin = rmfield(geo_model.pdcylin,'MassConf');
end

nconf = str2double(get(handles.Nconf,'string'));
cont = 1;
for i = 1 : nconf
   Mass_conf.N = i;
   uiwait(SelectMassValue)
   if Mass_conf.set == 1 % all values set
       geo_model.pdcylin.MassConf.Pass(cont) = Mass_conf.Pass/100;
       geo_model.pdcylin.MassConf.PaxStart(cont) = Mass_conf.PaxStart/100;
       geo_model.pdcylin.MassConf.PaxArrive(cont) = Mass_conf.PaxArrive/100;
       geo_model.pdcylin.MassConf.Baggage(cont) = Mass_conf.Baggage/100;
       geo_model.pdcylin.MassConf.BagStart(cont) = Mass_conf.BagStart/100;
       geo_model.pdcylin.MassConf.BagArrive(cont) = Mass_conf.BagArrive/100;
       geo_model.pdcylin.MassConf.Cfuel(cont) = Mass_conf.Cfuel/100;
       geo_model.pdcylin.MassConf.Wfuel(cont) = Mass_conf.Wfuel/100;
       geo_model.pdcylin.MassConf.WfuelStart(cont) = Mass_conf.WfuelStart/100;
       geo_model.pdcylin.MassConf.WfuelArrive(cont) = Mass_conf.WfuelArrive/100;
       geo_model.pdcylin.MassConf.Load(cont).data = Mass_conf.Load;
       cont = cont+1;       
   end
end

if ~isempty(geo_model.LoadConditions) && isfield(geo_model.pdcylin,'MassConf')
    Nconf = length(geo_model.pdcylin.MassConf.Pass);
    ind = zeros(Nconf,1);
   for j = 1 :  Nconf
       ind(j) = isempty(geo_model.pdcylin.MassConf.Load(j).data);
   end
   if all(ind)
      f = warndlg(sprintf('No mass configuration has a specified load condition \nmass configuration 1 is set with all avaible load conditions '), 'Data missing'); 
      geo_model.pdcylin.MassConf.Load(1).data = geo_model.LoadConditions;
   end
end

close MassConfiguration


% --- Executes on button press in Back.
function Back_Callback(hObject, eventdata, handles)
% hObject    handle to Back (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

close MassConfiguration