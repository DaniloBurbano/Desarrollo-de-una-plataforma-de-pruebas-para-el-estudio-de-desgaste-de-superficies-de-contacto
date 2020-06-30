function varargout = FuncionMotor(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                    'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @FuncionMotor_OpeningFcn, ...
                   'gui_OutputFcn',  @FuncionMotor_OutputFcn, ...
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


% --- Executes just before FuncionMotor is made visible.
function FuncionMotor_OpeningFcn(hObject, eventdata, handles, varargin)
global Velocidad_Motor Velocidad_Rueda;
Velocidad_Motor=[];
Velocidad_Rueda=[];
disp(Velocidad_Motor);
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to FuncionMotor (see VARARGIN)

% Choose default command line output for FuncionMotor
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes FuncionMotor wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = FuncionMotor_OutputFcn(hObject, eventdata, handles) 

% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



% --- Executes on button press in Conectar.


% --- Executes when selected object is changed in Estado_del_Motor.
function Estado_del_Motor_SelectionChangedFcn(hObject, eventdata, handles)

global ar sx  dutty_cicle t_alto t_bajo;
estado=get(hObject,'String');


alto=get(handles.alto,'string');
t_alto=str2double(alto);

bajo=get(handles.bajo,'string');
t_bajo=str2double(bajo);

dutty_cicle = 5;
switch estado
    case 'Apagado'
      writeDigitalPin(ar, 'D10', 0);  
      writeDigitalPin(ar, 'D11', 0);
      sx=3;

    case 'Encendido'
      writeDigitalPin(ar, 'D10', 0);  
      writeDigitalPin(ar, 'D11', 1);
      sx=2;  
          
    case 'Atras'
      writeDigitalPin(ar, 'D10', 1);  
      writeDigitalPin(ar, 'D11', 0);
      sx=1;
      
    case 'Secuencia de Activaci?n'
      writeDigitalPin(ar, 'D10', 0);  
      writeDigitalPin(ar, 'D11', 1);
      sx=2;
      

end



% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)

global ar sensor1 sensor2  sx t;
s=get(hObject,'Value');
s=round(s);
d=double(s*0.01);
writePWMDutyCycle(ar,'D9',abs(d-0.00));
set(handles.Velocidad,'String',s);
%***********************************
motor=read(sensor1,1);
rueda=read(sensor2,1);
vibracion=readVoltage(ar, 'A0')*1023/5;
vrads_motor=double(motor)*2*pi/5.1;
vrads_rueda=double(rueda)*2*pi/5.1;   
set(handles.SensorMotor,'String',vrads_motor);
set(handles.SensorRueda,'String',vrads_rueda);
vms_motor=double(motor)*0.208/5.1;
vms_rueda=double(rueda)*1.57/5.1;
set(handles.SensorMotor2,'String',vms_motor);
set(handles.SensorRueda2,'String',vms_rueda);
set(handles.Sensorvibracion,'String',vibracion);

while get(hObject,'Value')
    s=get(hObject,'Value');
    s=round(s);
    d=double(s*0.01);
    writePWMDutyCycle(ar,'D9',abs(d-0.02));
    motor=read(sensor1,1);
    rueda=read(sensor2,1);
    vibracion=readVoltage(ar, 'A0')*1023/5;
    vrads_motor=double(motor)*2*pi/5.1;   % velocidad en rad/s
    vrads_rueda=double(rueda)*2*pi/5.1;   % velocidad en rad/s
    set(handles.SensorMotor,'String',vrads_motor);
    set(handles.SensorRueda,'String',vrads_rueda);
    vms_motor=double(motor)*0.208/5.1;        % velocidad en m/s
    vms_rueda=double(rueda)*1.57/5.1;        % velocidad en m/s
    set(handles.SensorMotor2,'String',vms_motor);
    set(handles.SensorRueda2,'String',vms_rueda);
    set(handles.Sensorvibracion,'String',vibracion);
    pause(0.5);
end


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%% CONECTAR

function conectar1_Callback(hObject, eventdata, handles)
global ar sensor1 sensor2;
e=get(hObject,'Value');
handles.e=e;
if handles.e==1
    ar=arduino('COM9','Uno')
    configurePin(ar,'D9','PWM');
%   CONFIGURACION I2C
    sensor1=i2cdev(ar,1)
    sensor2=i2cdev(ar,2)
    write(sensor1,1);
    write(sensor2,1);
    %pause(1);
    set(handles.conectar1,'String','Desconectar');
    disp(e)
    
else
    set(handles.conectar1,'String','Conectar');
    clear ar;
    clear sensor1;
    disp('desconectar')
end
% hObject    handle to conectar1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of conectar1



% --- Executes on button press in CapturarDatos.
function CapturarDatos_Callback(hObject, eventdata, handles)
global ar sensor1 sensor2 Vibracion Voltaje Velocidad_Motor Velocidad_Rueda sx t dutty_cicle t_alto t_bajo hora_inicio hora_final;

% hObject    handle to CapturarDatos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if strcmp(get(handles.CapturarDatos,'String'),'Iniciar')
    set(handles.CapturarDatos,'String','Terminar');
    hora_inicio=datestr(now);
    %clear Velocidad_Motor;
    Velocidad_Motor=[];
    Velocidad_Rueda=[];
    Vibracion=[];
    Voltaje=[];
    i=1;
    t=0;
    b=0;
    tic;
    while get(hObject,'Value')
        motor=read(sensor1,1);
        rueda=read(sensor2,1);
        vrads_motor=double(motor)*2*pi/5.1;
        vrads_rueda=double(rueda)*2*pi/5.1;
        Velocidad_Motor(i,1)=vrads_motor;
        Velocidad_Rueda(i,1)=vrads_rueda;
        Vibracion(i,1)=readVoltage(ar, 'A0')*1023/5;
        disp('vibracion:');
        disp(Vibracion);
        set(handles.SensorMotor,'String',vrads_motor);
        set(handles.SensorRueda,'String',vrads_rueda);
        vms_motor=double(motor)*0.208/5.1; % 0.208 distancia en m recorrida por el motor en 1 vuelta
        vms_rueda=double(rueda)*1.57/5.1;  % 5.1 es una constante para para comunicacion i2c
        set(handles.SensorMotor2,'String',vms_motor);
        set(handles.SensorRueda2,'String',vms_rueda);
        set(handles.Sensorvibracion,'String',Vibracion(i,1));
        Velocidad_Motor(i,2)=vms_motor;
        Velocidad_Rueda(i,2)=vms_rueda;
        timer=toc;
        Velocidad_Motor(i,3)=timer;
        Velocidad_Rueda(i,3)=Velocidad_Motor(i,3);
        
        %********** Genera la se?al cuadrada **********
        if sx==2
            if (timer-t)>dutty_cicle                
                t=timer;
                if b==1
                    dutty_cicle = t_alto;
                    writeDigitalPin(ar, 'D10', 0);  
                    writeDigitalPin(ar, 'D11', 1);
                    b=0;
                    Voltaje(i,1)=1;
                else
                    dutty_cicle = t_bajo;
                    writeDigitalPin(ar, 'D10', 0);  
                    writeDigitalPin(ar, 'D11', 0);
                    b=1;
                    Voltaje(i,1)=0;
                end                
            end
        end
        %********************************************
        
        i=i+1
        pause(0.5);       
    end
    
    
else
    set(handles.CapturarDatos,'String','Iniciar');
    disp('******************************')
    hora_final=datestr(now);
   
end
    

% --- Executes on button press in GuardarDatos.
function GuardarDatos_Callback(hObject, eventdata, handles)
global Velocidad_Motor Velocidad_Rueda hora_final hora_inicio Vibracion Voltaje;
pregunta = {'Ingrese el nombre del archivo:'};
titulo = 'Guardar';
sugerencia = {'prueba.xlsx'};
Nombre = inputdlg(pregunta,titulo,[1,50],sugerencia);
Nombre=Nombre{1};
parametros =[{'Velocidad [rad/s]'},{'Velocidad [m/s]'},{'Tiempo [s]'}];
xlswrite(Nombre,{'Motor'},1,'A1');
xlswrite(Nombre,parametros,1,'A2');
xlswrite(Nombre,Velocidad_Motor,1,'A3');
xlswrite(Nombre,{'Rueda'},1,'D1');
xlswrite(Nombre,parametros,1,'D2');
xlswrite(Nombre,Velocidad_Rueda,1,'D3');
xlswrite(Nombre,{'Vibracion'},1,'G2');
xlswrite(Nombre,Vibracion,1,'G3');
xlswrite(Nombre,{'Voltaje'},1,'H2');
xlswrite(Nombre,Voltaje,1,'H3');
hora_inicio={hora_inicio};
hora_final={hora_final};
xlswrite(Nombre,{'Hora de inicio'},1,'I2');
xlswrite(Nombre,{'Hora de final'},1,'J2');
xlswrite(Nombre,hora_inicio,1,'I3');
xlswrite(Nombre,hora_final,1,'J3');
disp('***************************************')
disp('Guardado')



function alto_Callback(hObject, eventdata, handles)
% hObject    handle to alto (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of alto as text
%        str2double(get(hObject,'String')) returns contents of alto as a double


% --- Executes during object creation, after setting all properties.
function alto_CreateFcn(hObject, eventdata, handles)
% hObject    handle to alto (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function bajo_Callback(hObject, eventdata, handles)
% hObject    handle to bajo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bajo as text
%        str2double(get(hObject,'String')) returns contents of bajo as a double


% --- Executes during object creation, after setting all properties.
function bajo_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bajo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
