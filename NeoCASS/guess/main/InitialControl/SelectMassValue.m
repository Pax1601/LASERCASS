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

function varargout = SelectMassValue(varargin)
% SELECTMASSVALUE M-file for SelectMassValue.fig
%      SELECTMASSVALUE, by itself, creates a new SELECTMASSVALUE or raises the existing
%      singleton*.
%
%      H = SELECTMASSVALUE returns the handle to a new SELECTMASSVALUE or the handle to
%      the existing singleton*.
%
%      SELECTMASSVALUE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SELECTMASSVALUE.M with the given input arguments.
%
%      SELECTMASSVALUE('Property','Value',...) creates a new SELECTMASSVALUE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SelectMassValue_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SelectMassValue_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SelectMassValue

% Last Modified by GUIDE v2.5 08-Nov-2013 11:19:48

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @SelectMassValue_OpeningFcn, ...
    'gui_OutputFcn',  @SelectMassValue_OutputFcn, ...
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


% --- Executes just before SelectMassValue is made visible.
function SelectMassValue_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SelectMassValue (see VARARGIN)

% Choose default command line output for SelectMassValue
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SelectMassValue wait for user response (see UIRESUME)
% uiwait(handles.figure1);
global geo_model
global Mass_conf

set(handles.NConf,'string',num2str(Mass_conf.N));

if isempty(geo_model.LoadConditions)
    set(handles.LoadConditions,'Enable','off');
else
    set(handles.LoadConditions,'Enable','on');
    set(handles.LoadConditions,'String',num2str(geo_model.LoadConditions'));%); num2str([1 2 3]')
    set(handles.LoadConditions,'Min',0);
    set(handles.LoadConditions,'Max',1+length(geo_model.LoadConditions));% length(geo_model.LoadConditions));
end

set(handles.arrive,'string',num2str(geo_model.aircraft.fuel.Outboard_fuel_tank_span*100));

Aaft = pi/2 *( 2*geo_model.aircraft.fuselage.a0_aft^2 + geo_model.aircraft.fuselage.a1_aft^2 + geo_model.aircraft.fuselage.b1_aft^2 );
% radius at aft section
Raft = sqrt( Aaft/pi );

set(handles.start,'string',num2str(floor(Raft*2/geo_model.aircraft.wing1.span*100)));

% --- Outputs from this function are returned to the command line.
function varargout = SelectMassValue_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in LoadConditions.
function LoadConditions_Callback(hObject, eventdata, handles)
% hObject    handle to LoadConditions (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns LoadConditions contents as cell array
%        contents{get(hObject,'Value')} returns selected item from LoadConditions


% --- Executes during object creation, after setting all properties.
function LoadConditions_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LoadConditions (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in OK.
function OK_Callback(hObject, eventdata, handles)
% hObject    handle to OK (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Mass_conf
global geo_model

Mass_conf.Pass = str2double(get(handles.Passengers,'string'));
Mass_conf.PaxStart = str2double(get(handles.startP,'string'));
Mass_conf.PaxArrive = str2double(get(handles.arriveP,'string'));
%
Mass_conf.Baggage = str2double(get(handles.Baggage,'string'));
Mass_conf.BagStart = str2double(get(handles.startB,'string'));
Mass_conf.BagArrive = str2double(get(handles.arriveB,'string'));
%
Mass_conf.Cfuel = str2double(get(handles.Ctank,'string'));
Mass_conf.Wfuel = str2double(get(handles.Wtank,'string'));
Mass_conf.WfuelStart = floor(str2double(get(handles.start,'string')));
Mass_conf.WfuelArrive = str2double(get(handles.arrive,'string'));


if strcmp(get(handles.LoadConditions,'Enable') , 'on')
    Mass_conf.Load = str2num(get(handles.LoadConditions,'string'));
    Mass_conf.Load = Mass_conf.Load(get(handles.LoadConditions,'Value'));
else
    Mass_conf.Load = [];
end

Aaft = pi/2 *( 2*geo_model.aircraft.fuselage.a0_aft^2 + geo_model.aircraft.fuselage.a1_aft^2 + geo_model.aircraft.fuselage.b1_aft^2 );
% radius at aft section
Raft = sqrt( Aaft/pi );

if isnan(Mass_conf.Pass) || isnan(Mass_conf.Baggage) || isnan(Mass_conf.Cfuel) || isnan(Mass_conf.Wfuel) || isnan(Mass_conf.WfuelStart) || isnan(Mass_conf.WfuelArrive)
    f = errordlg('All data must be numbers', 'Data error');
elseif Mass_conf.WfuelArrive>geo_model.aircraft.fuel.Outboard_fuel_tank_span*100
    f = warndlg(['wing fuel must be in wing tank, so "To" must be <= to ',num2str(geo_model.aircraft.fuel.Outboard_fuel_tank_span*100)], 'Data error');
elseif Mass_conf.WfuelArrive<=Mass_conf.WfuelStart
    f = warndlg('start point must be lower than end poin', 'Data error');
elseif Mass_conf.WfuelStart<floor(Raft*2/geo_model.aircraft.wing1.span*100)
     f = warndlg(['wing fuel must be in wing tank, so "From" must be >= to ',num2str(floor(Raft*2/geo_model.aircraft.wing1.span*100))], 'Data error');
else
    
    Mass_conf.set = 1;
    close SelectMassValue
end




% --- Executes on button press in discard.
function discard_Callback(hObject, eventdata, handles)
% hObject    handle to discard (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Mass_conf
Mass_conf.set = 0;

close SelectMassValue

function Ctank_Callback(hObject, eventdata, handles)
% hObject    handle to Ctank (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ctank as text
%        str2double(get(hObject,'String')) returns contents of Ctank as a double


% --- Executes during object creation, after setting all properties.
function Ctank_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ctank (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Wtank_Callback(hObject, eventdata, handles)
% hObject    handle to Wtank (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Wtank as text
%        str2double(get(hObject,'String')) returns contents of Wtank as a double


% --- Executes during object creation, after setting all properties.
function Wtank_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Wtank (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Passengers_Callback(hObject, eventdata, handles)
% hObject    handle to Passengers (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Passengers as text
%        str2double(get(hObject,'String')) returns contents of Passengers as a double


% --- Executes during object creation, after setting all properties.
function Passengers_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Passengers (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Baggage_Callback(hObject, eventdata, handles)
% hObject    handle to Baggage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Baggage as text
%        str2double(get(hObject,'String')) returns contents of Baggage as a double


% --- Executes during object creation, after setting all properties.
function Baggage_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Baggage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function start_Callback(hObject, eventdata, handles)
% hObject    handle to start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of start as text
%        str2double(get(hObject,'String')) returns contents of start as a double


% --- Executes during object creation, after setting all properties.
function start_CreateFcn(hObject, eventdata, handles)
% hObject    handle to start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function arrive_Callback(hObject, eventdata, handles)
% hObject    handle to arrive (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of arrive as text
%        str2double(get(hObject,'String')) returns contents of arrive as a double


% --- Executes during object creation, after setting all properties.
function arrive_CreateFcn(hObject, eventdata, handles)
% hObject    handle to arrive (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function startP_Callback(hObject, eventdata, handles)
% hObject    handle to startP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of startP as text
%        str2double(get(hObject,'String')) returns contents of startP as a double


% --- Executes during object creation, after setting all properties.
function startP_CreateFcn(hObject, eventdata, handles)
% hObject    handle to startP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function arriveP_Callback(hObject, eventdata, handles)
% hObject    handle to arriveP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of arriveP as text
%        str2double(get(hObject,'String')) returns contents of arriveP as a double


% --- Executes during object creation, after setting all properties.
function arriveP_CreateFcn(hObject, eventdata, handles)
% hObject    handle to arriveP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function startB_Callback(hObject, eventdata, handles)
% hObject    handle to startB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of startB as text
%        str2double(get(hObject,'String')) returns contents of startB as a double


% --- Executes during object creation, after setting all properties.
function startB_CreateFcn(hObject, eventdata, handles)
% hObject    handle to startB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function arriveB_Callback(hObject, eventdata, handles)
% hObject    handle to arriveB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of arriveB as text
%        str2double(get(hObject,'String')) returns contents of arriveB as a double


% --- Executes during object creation, after setting all properties.
function arriveB_CreateFcn(hObject, eventdata, handles)
% hObject    handle to arriveB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
