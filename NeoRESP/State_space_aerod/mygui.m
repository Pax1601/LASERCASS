function varargout = mygui(varargin)
% =========================================================================
%                                                                     mygui
% =========================================================================
%
% Description: graphical user interface for the Improved Rational Matrix 
%              Fraction Approximation program.
%
% -------------------------------------------------------------------------
%
%
%   Copyright (C) 2012 Paolo Mantegazza   <mantegazza@aero.polimi.it>
%   Copyright (C) 2012 Matteo Ripepi      <ripepi@aero.polimi.it>
%  
%   This program is free software; you can redistribute it and/or
%   modify it under the terms of the GNU General Public License as
%   published by the Free Software Foundation; either version 3 of the
%   License, or (at your option) any later version.
%  
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%  
%   You should have received a copy of the GNU General Public License
%   along with this program; if not, write to the Free Software
%   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
%  
%   Ref: Ripepi M., Mantegazza P., 'An improved matrix fraction approximation of
%        aerodynamic transfer matrices', AIAA journal, submitted for publication.
%
%
% =========================================================================
%
% MYGUI MATLAB code for mygui.fig
%      MYGUI, by itself, creates a new MYGUI or raises the existing
%      singleton*.
%
%      H = MYGUI returns the handle to a new MYGUI or the handle to
%      the existing singleton*.
%
%      MYGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MYGUI.M with the given input arguments.
%
%      MYGUI('Property','Value',...) creates a new MYGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before mygui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to mygui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help mygui

% Last Modified by GUIDE v2.5 26-Sep-2012 12:06:48

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @mygui_OpeningFcn, ...
                   'gui_OutputFcn',  @mygui_OutputFcn, ...
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


% --- Executes just before mygui is made visible.
function mygui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to mygui (see VARARGIN)

% Choose default command line output for mygui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes mygui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = mygui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

set(handles.figure1,'Name','Improved Matrix Fraction Approximation')


% --- Executes on button press in calculate.
function calculate_Callback(hObject, eventdata, handles)
% hObject    handle to calculate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%--------------------------------------------------------------------------
%                              USER INPUT DATA
%--------------------------------------------------------------------------
% Identification options

% MFD order
opt{1} = str2double( get(handles.mfdorder,'String') );

% MFD algorithm
if get(handles.kron,'Value'),     opt{2} = 1;
elseif get(handles.det,'Value'),  opt{2} = 2;
elseif get(handles.lin,'Value'),  opt{2} = 3;
end

% left or right MFD
if get(handles.left,'Value'),      opt{3} = 'lmfd';
elseif get(handles.right,'Value'), opt{3} = 'rmfd';
end
opt{4} = 2;  % ro parameter in treating discrete gusts using the RMFD 

% weights parameters
opt{5} = str2double( get( handles.weight ,'string') );  % weight value W^2 

% Levenberg-Marquardt parameters
tau     = str2double( get( handles.tau ,'string') ); 
tolg    = str2double( get( handles.tolg ,'string') );
tolx    = str2double( get( handles.tolx ,'string') );
maxiter = str2double( get( handles.maxiter ,'string') );
optsLM = [tau tolg tolx maxiter]; 

% Model order reduction algorithm
if get(handles.balance,'Value'),    algROM = 'balance'; 
elseif get(handles.schur,'Value'),  algROM = 'schur';
elseif get(handles.hankel,'Value'), algROM = 'hankel';
elseif get(handles.bst,'Value'),    algROM = 'bst';
end

% stabilizations parameters
eigsopt.threshold = str2double( get( handles.threshold ,'string') ); 
eigsopt.bound     = str2double( get( handles.boundvalue ,'string') ); 

% eig shifting method
if get(handles.flip,'Value'),      eigsopt.type = 'flip';
elseif get(handles.bound,'Value'), eigsopt.type = 'bound';
end

% eigensolution recovering method
if get(handles.modal,'Value'),     eigsopt.method = 'eigshift';
elseif get(handles.place,'Value'), eigsopt.method = 'polesplace';
end


%--------------------------------------------------------------------------
%               Improved matrix fraction approximation
% -------------------------------------------------------------------------

% get input and output filename
inputdatafilename = get( handles.inputfile ,'String');
outputfilename = get( handles.outputfile ,'String');

% load aerodynamic transfer matrix and reduced frequencies
if  ~exist(inputdatafilename,'file') && ~exist( strcat(inputdatafilename,'.mat'),'file')
    fprintf(1,'Warning: input file not found. Select a valid file name.\n');
    return
end
load(inputdatafilename,'-mat');

% load (if exist) previous stable solutions MFD matrices
if get( handles.restartflag, 'Value' )
    restart = get( handles.restartfile ,'String');
else
    restart = [ ];
end

% execute the improved matrix fraction approximation
solution = improvedMFDfun(k,Ha,opt,optsLM,eigsopt,algROM,restart);

% save data
save(outputfilename)
% -------------------------------------------------------------------------


% --- Executes on button press in cancel.
function cancel_Callback(hObject, eventdata, handles)
% hObject    handle to cancel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

error('Operation terminated by user.')


%  MFD type
function left_Callback(hObject, eventdata, handles)
function right_Callback(hObject, eventdata, handles)

%  MFD algorithms
function kron_Callback(hObject, eventdata, handles)
function det_Callback(hObject, eventdata, handles)
function lin_Callback(hObject, eventdata, handles)

% Model order reduction
function balance_Callback(hObject, eventdata, handles)
function hankel_Callback(hObject, eventdata, handles)
function schur_Callback(hObject, eventdata, handles)
function bst_Callback(hObject, eventdata, handles)

% Eigensolution recovering method
function modal_Callback(hObject, eventdata, handles)
function place_Callback(hObject, eventdata, handles)

% Eigenvalue shifting mode
function flip_Callback(hObject, eventdata, handles)
function bound_Callback(hObject, eventdata, handles)


function tau_Callback(hObject, eventdata, handles)
function tau_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tau (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function tolg_Callback(hObject, eventdata, handles)
function tolg_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tolg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function tolx_Callback(hObject, eventdata, handles)
function tolx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tolx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function maxiter_Callback(hObject, eventdata, handles)
function maxiter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maxiter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function mfdorder_Callback(hObject, eventdata, handles)
function mfdorder_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mfdorder (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function threshold_Callback(hObject, eventdata, handles)
function threshold_CreateFcn(hObject, eventdata, handles)
% hObject    handle to threshold (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function weight_Callback(hObject, eventdata, handles)
function weight_CreateFcn(hObject, eventdata, handles)
% hObject    handle to weight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function boundvalue_Callback(hObject, eventdata, handles)
function boundvalue_CreateFcn(hObject, eventdata, handles)
% hObject    handle to boundvalue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function outputfile_Callback(hObject, eventdata, handles)
function outputfile_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outputfile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in outputbrowse.
function outputbrowse_Callback(hObject, eventdata, handles)
% hObject    handle to outputbrowse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[filename, pathname, filterindex] = uiputfile('.mat', 'Select File for Save Output' );
if isequal(filename,0) || isequal(pathname,0)
    filename = [ ]; pathname = [ ];
end
set(handles.outputfile,'String',strcat(pathname,filename));


function inputfile_Callback(hObject, eventdata, handles)
function inputfile_CreateFcn(hObject, eventdata, handles)
% hObject    handle to inputfile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in inputbrowse.
function inputbrowse_Callback(hObject, eventdata, handles)
% hObject    handle to inputbrowse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[filename, pathname, filterindex] = uigetfile('.mat', 'Select Input File' );
if isequal(filename,0) || isequal(pathname,0)
    filename = [ ]; pathname = [ ];
end
set(handles.inputfile,'String',strcat(pathname,filename));


% --- Executes when figure1 is resized.
function figure1_ResizeFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in restartflag.
function restartflag_Callback(hObject, eventdata, handles)
% hObject    handle to restartflag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of restartflag
if get(hObject,'Value')
    set(handles.browserestart,'Enable','on');
    set(handles.restartfile,'Enable','on');
else
    set(handles.browserestart,'Enable','off');
    set(handles.restartfile,'Enable','off');
end

% --- Executes on button press in browserestart.
function browserestart_Callback(hObject, eventdata, handles)
% hObject    handle to browserestart (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[filename, pathname, filterindex] = uigetfile('.mat', 'Select Restart File' );
if isequal(filename,0) || isequal(pathname,0)
    filename = [ ]; pathname = [ ];
end
set(handles.restartfile,'string',strcat(pathname,filename));

function restartfile_Callback(hObject, eventdata, handles)
function restartfile_CreateFcn(hObject, eventdata, handles)
% hObject    handle to restartfile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
