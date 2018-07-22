function varargout = matlab_playground_gui(varargin)
% MATLAB_PLAYGROUND_GUI MATLAB code for matlab_playground_gui.fig
%      MATLAB_PLAYGROUND_GUI, by itself, creates a new MATLAB_PLAYGROUND_GUI or raises the existing
%      singleton*.
%
%      H = MATLAB_PLAYGROUND_GUI returns the handle to a new MATLAB_PLAYGROUND_GUI or the handle to
%      the existing singleton*.
%
%      MATLAB_PLAYGROUND_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MATLAB_PLAYGROUND_GUI.M with the given input arguments.
%
%      MATLAB_PLAYGROUND_GUI('Property','Value',...) creates a new MATLAB_PLAYGROUND_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before matlab_playground_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to matlab_playground_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help matlab_playground_gui

% Last Modified by GUIDE v2.5 14-Jun-2018 19:55:38

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @matlab_playground_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @matlab_playground_gui_OutputFcn, ...
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

% center points of circles (x1, y1) (x2, y2)
% radius od circles r1, r2
function points=intersection_points_of_circles( x1, y1, x2, y2, r1, r2 )
points = zeros(1,4); % intersections are [points(1), points(2)] and [points(3), points(4)]

d = sqrt( (x1-x2)^2 + (y1-y2)^2 ); %distance between center points
d1 = ((r1^2) + (d^2) - (r2^2))/(2.0*d); %distance from (x1, y1) to the intersection from line (x1,y1)->(x2,y2) and the line between the intersection points of the circles 

h_squared = (r1^2) - (d1^2);
if h_squared >= 0.0
    h = sqrt( (r1^2) - (d1^2) );

    xm = x1 + (x2-x1)*(d1/d);
    ym = y1 + (y2-y1)*(d1/d);

    xp = (y1-y2)*(h/d);
    yp = (y1-y2)*(h/d);

    points(1) = xm + xp;
    points(2) = ym + yp;
    points(3) = xm - xp;
    points(4) = ym - yp;
end


% --- Executes just before matlab_playground_gui is made visible.
function matlab_playground_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to matlab_playground_gui (see VARARGIN)

% Choose default command line output for matlab_playground_gui
handles.output = hObject;

handles.serialPort = hObject;
handles.serialPortOpen = false;

global length_of_history;
length_of_history = 50;
global hist_dist;
hist_dist = zeros([4,length_of_history]);
global hist_qual;
hist_qual = zeros([4,length_of_history]);
global hist_botx;
hist_botx = zeros([1,length_of_history]);
global hist_boty;
hist_boty = zeros([1,length_of_history]);
global hist_botx_filtered;
hist_botx_filtered = zeros([1,length_of_history]);
global hist_boty_filtered;
hist_boty_filtered = zeros([1,length_of_history]);
global position_quality_hist;
position_quality_hist = zeros([1,length_of_history]);
global hist_cw;
hist_cw = zeros([1,length_of_history]);
global hist_ccw;
hist_ccw = zeros([1,length_of_history]);
global hist_ir_qual;
hist_ir_qual = zeros([1,length_of_history]);
global hist_ir_offset;
hist_ir_offset = zeros([1,length_of_history]); 
global debug_collector;
debug_collector = zeros([1,2048]);
global current_receiving_waveform;
current_receiving_waveform = 0;
global time_of_last_message_s;
time_of_last_message_s = now*24*60*60;
global length_of_debug_array;
length_of_debug_array = 0;

% predraw playing area
global beaconX;
global beaconY;
global fixed_beacon_offset;
fixed_beacon_offset = 50;
wall_th = 22;
beaconX = [0,                                   1000-fixed_beacon_offset ,  -1000+fixed_beacon_offset];
beaconY = [1500+2*wall_th+fixed_beacon_offset, -1500-2*wall_th-fixed_beacon_offset , -1500-2*wall_th-fixed_beacon_offset];
playing_area = [1000,1500; 1000,-1500; -1000,-1500; -1000,1500; 1000,1500];
hold(handles.axesPlayground, 'off');
plot(handles.axesPlayground, playing_area(:,2), playing_area(:,1),'k');
hold(handles.axesPlayground, 'on');
xlim(handles.axesPlayground,[-1600,1600]); 
ylim(handles.axesPlayground,[-1100,1100]);
xlabel(handles.axesPlayground, '.                                          Y in mm');
ylabel(handles.axesPlayground, 'X in mm');
%set(handles.axesPlayground, 'XTick', [-1500, -1000, -500, 0, 500, 1000, 1500]);
%set(handles.axesPlayground, 'YTick', [-1000, -500, 0, 500, 1000]);
set(handles.axesPlayground, 'YDir','reverse');

grid(handles.axesPlayground, 'on');
%grid(handles.axesPlayground, 'minor');

nr_of_points_per_dRadius = 36;
global circle_template;
circle_template = zeros([2,nr_of_points_per_dRadius+1]);
for i=1:nr_of_points_per_dRadius+1
    phi = 2*pi*i/nr_of_points_per_dRadius;
    circle_template(1,i) = cos(phi);
    circle_template(2,i) = sin(phi);
end

% global dRadius_plot_handle;
% dRadius_plot_handle(1) = plot(handles.axesPlayground, circle_template(1,:), circle_template(2,:), 'r', 'DisplayName', 'Master');
% dRadius_plot_handle(2) = plot(handles.axesPlayground, circle_template(1,:), circle_template(2,:), 'g', 'DisplayName', 'Slave1');
% dRadius_plot_handle(3) = plot(handles.axesPlayground, circle_template(1,:), circle_template(2,:), 'b', 'DisplayName', 'Slave2');
% dRadius_plot_handle(4) = plot(handles.axesPlayground, circle_template(1,:), circle_template(2,:), '--m', 'DisplayName', 'Coop');

global circle_plot_handle;
circle_plot_handle(1) = rectangle('Parent', handles.axesPlayground, 'Position', [0,0,0,0], 'Curvature', 1.0, 'EdgeColor', 'r');
circle_plot_handle(2) = rectangle('Parent', handles.axesPlayground, 'Position', [0,0,0,0], 'Curvature', 1.0, 'EdgeColor', 'g');
circle_plot_handle(3) = rectangle('Parent', handles.axesPlayground, 'Position', [0,0,0,0], 'Curvature', 1.0, 'EdgeColor', 'b');
circle_plot_handle(4) = rectangle('Parent', handles.axesPlayground, 'Position', [0,0,0,0], 'Curvature', 1.0, 'EdgeColor', 'm', 'LineStyle', '--');

global position_plot_handle;
position_plot_handle(1) = plot(handles.axesPlayground, hist_boty, hist_botx,  'Color', [1.0 1.0 1.0], 'DisplayName', 'Robot Path');
position_plot_handle(2) = plot(handles.axesPlayground, [0 0], [0 0], 'o', 'Color', [0.5 0.5 0.5], 'DisplayName', 'Robot Position');
position_plot_handle(3) = plot(handles.axesPlayground, hist_boty_filtered, hist_botx_filtered,  'w', 'DisplayName', 'Robot Path filtered');
position_plot_handle(4) = plot(handles.axesPlayground, 0, 0, 'o', 'Color', 'k', 'DisplayName', 'Robot Position filtered');
position_plot_handle(5) = plot(handles.axesPlayground, 0, 0, 'o', 'Color', 'm', 'DisplayName', 'Coop Position');

global microphone_position;
microphone_position = plot(handles.axesPlayground, [0,0,0,0,0], [0,0,0,0,0], 'Color', 'k');

%predraw distance history
hold(handles.axesSignals, 'off');
global dist_plot_handle;
dist_plot_handle(1) = plot(handles.axesSignals, hist_dist(1,:), 'r', 'DisplayName', 'Master'); 
hold(handles.axesSignals, 'on');
dist_plot_handle(2) = plot(handles.axesSignals, hist_dist(2,:), 'g', 'DisplayName', 'Slave1');
dist_plot_handle(3) = plot(handles.axesSignals, hist_dist(3,:), 'b', 'DisplayName', 'Slave2');
dist_plot_handle(4) = plot(handles.axesSignals, hist_dist(3,:), 'y', 'DisplayName', 'AngleSig');
xlim(handles.axesSignals, [-1,50]);
xlabel(handles.axesSignals, 'Sample nr');
ylabel(handles.axesSignals, 'Distance in mm');
grid(handles.axesSignals, 'on');

%predraw quality history
hold(handles.axesQuality, 'off');
global quality_plot_handle;
quality_plot_handle(1) = plot(handles.axesQuality, hist_qual(1,:),'r', 'DisplayName', 'Master');  
hold(handles.axesQuality, 'on');
quality_plot_handle(2) = plot(handles.axesQuality, hist_qual(2,:),'g', 'DisplayName', 'Slave1'); 
quality_plot_handle(3) = plot(handles.axesQuality, hist_qual(3,:),'b', 'DisplayName', 'Slave2'); 
quality_plot_handle(4) = plot(handles.axesQuality, hist_qual(4,:),'y', 'DisplayName', 'AngleSig'); 
xlim(handles.axesQuality, [-1,50]);
%ylim(handles.axesQuality, [0,200]);
ylabel(handles.axesQuality, 'Signal Qualiy');
grid(handles.axesQuality, 'on');

%predraw ir history axes
hold(handles.axesIrSync, 'off');
global ir_plot_handle;
ir_plot_handle(1) = plot(handles.axesIrSync, hist_ir_qual(:), 'c', 'DisplayName', 'IR_Quality');
hold(handles.axesIrSync, 'on');
ir_plot_handle(2) = plot(handles.axesIrSync, hist_ir_offset(:), 'm', 'DisplayName', 'IR_Offset');
grid(handles.axesIrSync, 'on');
ylim(handles.axesIrSync, [-5,5]);
xlim(handles.axesIrSync, [0,length_of_history]);

% Static legend
hold(handles.axesLegend, 'on');
plot(handles.axesLegend, [0], [0], 'r', 'DisplayName', 'Master');
plot(handles.axesLegend, [0], [0], 'g', 'DisplayName', 'Slave1');
plot(handles.axesLegend, [0], [0], 'b', 'DisplayName', 'Slave2');
plot(handles.axesLegend, [0], [0], 'y', 'DisplayName', 'AngleSig');
plot(handles.axesLegend, [0], [0], 'k', 'DisplayName', 'Robot Location');
legend(handles.axesLegend, 'show', 'Location', 'northoutside');

hold(handles.axesLegendIr, 'on');
plot(handles.axesLegendIr, [0], [0], 'c', 'DisplayName', 'IR-Quality');
plot(handles.axesLegendIr, [0], [0], 'm', 'DisplayName', 'IR-Offset');
legend(handles.axesLegendIr, 'show', 'Location', 'northoutside');

drawnow

global list_of_waveforms;
list_of_waveforms = {'1-MIC1' '2-MIC2' '3-MIC3' '4-MIC4'...
                     '5-MIC1_SPECTRUM' '6-MIC2_SPECTRUM' '7-MIC3_SPECTRUM' '8-MIC4_SPECTRUM'...
                     '9-Signal_Strengths'...
                     '10-Master_Correlation' '11-Slave1_Correlation' '12-Slave2_Correlation' '13-Angle_Correlation'...
                     '14-IR' '15-IR_Spectrum' '16-IR_Correlation' ...
                     '17-IR_Correlation_Lower' '18-IR_Correlation_Uper' '17-None' ...
                     '20-WF_MIC1_MASTER_CORRELATION_128' ...
                     '21-WF_MIC2_MASTER_CORRELATION_128' ...
                     '22-WF_MIC3_MASTER_CORRELATION_128' ...
                     '23-WF_MIC4_MASTER_CORRELATION_128' ...
                     '24-WF_MIC1_SLAVE1_CORRELATION_128' ...
                     '25-WF_MIC2_SLAVE1_CORRELATION_128' ...
                     '26-WF_MIC3_SLAVE1_CORRELATION_128' ...
                     '27-WF_MIC4_SLAVE1_CORRELATION_128' ...
                     '28-WF_MIC1_SLAVE2_CORRELATION_128' ...
                     '29-WF_MIC2_SLAVE2_CORRELATION_128' ...
                     '30-WF_MIC3_SLAVE2_CORRELATION_128' ...
                     '31-WF_MIC4_SLAVE2_CORRELATION_128' ...
                     '32-WF_MIC1_COOP3_CORRELATION_128' ...
                     '33-WF_MIC2_COOP3_CORRELATION_128' ...
                     '34-WF_MIC3_COOP3_CORRELATION_128' ...
                     '35-WF_MIC4_COOP3_CORRELATION_128' ...
                     '36-WF_MIC1_COOP4_CORRELATION_128' ...
                     '37-WF_MIC2_COOP4_CORRELATION_128' ...
                     '38-WF_MIC3_COOP4_CORRELATION_128' ...
                     '39-WF_MIC4_COOP4_CORRELATION_128' ...
                     '40-WF_MIC1_COOP5_CORRELATION_128' ...
                     '41-WF_MIC2_COOP5_CORRELATION_128' ...
                     '42-WF_MIC3_COOP5_CORRELATION_128' ...
                     '43-WF_MIC4_COOP5_CORRELATION_128' ...
                     '44-WF_MIC1_COOP6_CORRELATION_128' ...
                     '45-WF_MIC2_COOP6_CORRELATION_128' ...
                     '46-WF_MIC3_COOP6_CORRELATION_128' ...
                     '47-WF_MIC4_COOP6_CORRELATION_128' ...
                     '48-WF_DIST_CORRELATION_3x128' ...
                     '49-WF_SNAPSHOT'};

global angle_figure_enabled;
angle_figure_enabled = 0;     

global store_message_for_snapshot;
store_message_for_snapshot = false;
global snapshot_beacon_message;
snapshot_beacon_message = '';

set(handles.popupmenuWaveform, 'String', list_of_waveforms);

global distance_matrix;
distance_matrix = zeros(4,7);

global unambiguity_matrix;
unambiguity_matrix = zeros(4,7);

global correlation_matrix;
correlation_matrix = zeros(4,7);

global guihandle;
guihandle = handles;






% Update handles structure
guidata(hObject, handles);

% UIWAIT makes matlab_playground_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = matlab_playground_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in buttonSerialConnect.
function buttonSerialConnect_Callback(hObject, eventdata, handles)
% hObject    handle to buttonSerialConnect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

instrreset; % update matlab port list
fprintf('Open Serial Port ..\n');
portName = get(handles.editPortName,'String');

handles.serialPort = serial(portName, 'BaudRate', 115200);
handles.serialPort.InputBufferSize = 1000*1000;
handles.serialPort.Terminator = 'LF';
handles.serialPort.BytesAvailableFcnMode = 'terminator';
handles.serialPort.BytesAvailableFcn ={@serialReceiveLine, hObject};
fopen(handles.serialPort);
handles.serialPortOpen = true;

guidata(hObject, handles);
fprintf('Port Open\n')

function serialReceiveLine (s, event, hObject)
if ischar(s)
  line = s;
else
  line = fscanf(s);
end

if line(1) == 'a'
    % strean contains a debug array value
    values = sscanf(line, 'a%i[%i]=%f\n');
    if length(values) == 3
        waveform = values(1);
        index = values(2)+1;
        value = values(3);
        global debug_collector;
        if index == 0
            global current_receiving_waveform;
            current_receiving_waveform = waveform;
            for i=1:2048
                debug_collector(i) = 0;
            end
        end
                
        debug_collector(index) = value;
        global length_of_debug_array;
        length_of_debug_array = index;
    end
    
elseif line(1) == 'm'
    if line(2) == '1'
        % microphone position debug output decoding
        %                           1      2      3      4     5      6      7      8
        values = sscanf(line, 'm1x=%i m1y=%i m2x=%i m2y=%i m3x=%i m3y=%i m4x=%i m4y=%i \n');
        if length(values) == 8
            global microphone_position;
            set(microphone_position, 'XData', [values(2)/10, values(4)/10, values(6)/10, values(8)/10, values(2)/10,],...
                                     'YData', [values(1)/10, values(3)/10, values(5)/10, values(7)/10, values(1)/10,]);
        end
    else
        % microphone matrix decoding
        values = sscanf(line, 'mic%i s=%i d=%i u=%i c=%i\n');
        if length(values) == 5    
            global distance_matrix;
            global unambiguity_matrix;
            global correlation_matrix;
            distance_matrix(values(1) , values(2)+1) = values(3)/10;
            unambiguity_matrix(values(1) , values(2)+1) = values(4);
            correlation_matrix(values(1) , values(2)+1) = values(5);
            
            if (values(1) == 4)&(values(2) == 6)
                handles = guidata(hObject);
                dist_draw_selection = get(handles.popupmenuRangingDisplay, 'Value');
                if dist_draw_selection > 1.6
                    microphone = floor(dist_draw_selection-0.5);
                    ranges = distance_matrix(microphone,1:3)
                    global beaconX;
                    global beaconY;
                    global circle_plot_handle;
                    
                    for i=1:3
                        radius = max(distance_matrix(microphone,i),10);
                        position = [(beaconY(i)-radius),(beaconX(i)-radius),(2.0*radius),(2.0*radius)];
                        set(circle_plot_handle(i), 'Position', position); 
                    end
                    global coop_y;
                    global coop_x;
                    radius = max(distance_matrix(microphone,4), 10);
                    set(circle_plot_handle(4), 'Position', [(coop_y-radius) (coop_x-radius) (2*radius) (2*radius)]);
                end
            end    
        end
    end
    
elseif line(1) == 'u'
    % update debug array plot
    global debug_collector;
    handles = guidata(hObject);
    current_receiving_waveform = get(handles.popupmenuWaveform, 'Value');
    
    if get(handles.checkboxHold, 'Value') == 1.0
        hold(handles.axesDebugWaveform, 'on');
    else
        hold(handles.axesDebugWaveform, 'off');
    end
    line
    
    global store_message_for_snapshot;
    store_message_for_snapshot = true;

    
    global list_of_waveforms;
    display_name = char(list_of_waveforms(current_receiving_waveform));
    
    if      (current_receiving_waveform == 1) || ...
            (current_receiving_waveform == 2) || ...
            (current_receiving_waveform == 3) || ...
            (current_receiving_waveform == 4) || ...
            (current_receiving_waveform == 15)
        %plot signal over time    
        timestep = 1/160000;
        time_axis = (0:2047)*timestep;
        
        length(time_axis)
        length(debug_collector)
        
        plot(handles.axesDebugWaveform, time_axis, debug_collector, 'DisplayName', display_name);
        xlabel(handles.axesDebugWaveform, 'time in s');
        ylabel(handles.axesDebugWaveform, 'amplitude');      
        
    elseif  (current_receiving_waveform == 5) || ...
            (current_receiving_waveform == 6) || ...
            (current_receiving_waveform == 7) || ...
            (current_receiving_waveform == 8) || ...
            (current_receiving_waveform == 15)
        %plot spectrum, calculate magnitude and phase and plot over freuency
        real = debug_collector(1:2:2048);
        imag = debug_collector(2:2:2048);
        magnitude = sqrt(real.^2 + imag.^2);
        bin_width = 160000/2048;
        frequency_axis = (0:1023)*bin_width;
        plot(handles.axesDebugWaveform, frequency_axis/1000, magnitude, 'DisplayName', display_name);
        xlabel(handles.axesDebugWaveform, 'frequency in kHz');
    
    elseif (current_receiving_waveform == 36) %correlation debug
        
        Master_Correlation = debug_collector(  1:128);
        Slave1_Correlation = debug_collector(129:256);
        Slave2_Correlation = debug_collector(257:384);
        plot(handles.axesDebugWaveform, Master_Correlation, 'DisplayName', 'Master Correlation');
        hold(handles.axesDebugWaveform, 'on');
        plot(handles.axesDebugWaveform, Slave1_Correlation, 'DisplayName', 'Slave1 Correlation');
        plot(handles.axesDebugWaveform, Slave2_Correlation, 'DisplayName', 'Slave2 Correlation');
        xlabel(handles.axesDebugWaveform, 'sample');
        
        
        correlation_128 = zeros(3,128);     
        correlation_128(1,:) = Master_Correlation;
        correlation_128(2,:) = Slave1_Correlation;
        correlation_128(3,:) = Slave2_Correlation;
        
        global beaconX;
        global beaconY;
        step_size = 50;
        c_plot = zeros((2000/step_size), (3000/step_size));
        i = 1;
        for x_step = 0:(2000/step_size)
            X = x_step * step_size -1000;
            for y_step = 0:(3000/step_size)
                Y = y_step * step_size -1500;
              
                correlation_product = 1;
                for beacon = 1:3
                    d = sqrt( (X-beaconX(beacon))^2 + (Y-beaconY(beacon))^2 );
                    index = ((d+481)/33);
                    correlation = 1;
                    if index > 0
                        if index < 128
                            correlation = spline(0:127, correlation_128(beacon,:), index);
                        end
                    end
                    correlation_product = correlation_product*correlation;
                end
                
                c_plot(x_step+1,y_step+1) = correlation_product;
                i = i+1;
            end
        end
        
        length(-1500:step_size:1500)
        size(c_plot,2)
        length(-1000:step_size:1000)
        size(c_plot,1)
        
        
        figure;
        daspect([1 1 1]);
        contourf( -1500:step_size:1500, -1000:step_size:1000, c_plot );
        grid();
        xlabel('Y in mm');
        ylabel('X in mm');
        
    else
        %plot raw, e.g. for correlation
        global length_of_debug_array;
        plot(handles.axesDebugWaveform, debug_collector(1:length_of_debug_array), 'DisplayName', display_name);
        xlabel(handles.axesDebugWaveform, 'sample');
    end
    grid(handles.axesDebugWaveform, 'on');
    legend(handles.axesDebugWaveform,'-DynamicLegend');
    
    
elseif line(1) == 'd'
    %measure update-rate
    global time_of_last_message_s;
    time_s = now*24*60*60;
    interval = time_s-time_of_last_message_s;
    time_of_last_message_s = time_s;
    update_rate = 1/interval
    
    global store_message_for_snapshot;
    if store_message_for_snapshot == true
        global snapshot_beacon_message;
        snapshot_beacon_message = line;
        store_message_for_snapshot = false;
    end
    
    handles = guidata(hObject);
    if get(handles.checkboxSaveStream, 'Value') == 1.0
        global stream_file_handle;      
        fprintf(stream_file_handle, line);
    end
    
    %                          1    2     3     4     5     6     7     8    9    10    11    12    13    14     15  16    17   18   19
    values = sscanf(line, 'd0=%i u0=%i d1=%i u1=%i d2=%i u2=%i d3=%i u3=%i d4=%i u4=%i d5=%i u5=%i d6=%i u6=%i x=%i y=%i ax=%i ay=%i cx=%i cy=%i IRQ=%i IR_o=%i IRC=%i OD=%i s=%i*');
    length(values)
    fprintf('plot')
    if length(values) == 25
        if ischar(s) || (s.BytesAvailable < 300)
            distances = zeros([4,1]);
            qualities = zeros([4,1]);
            for i = 1:4
               distances(i) = values(i*2-1) *0.1;
               qualities(i) = values(i*2);
            end

            distances(4) = distances(4)-125;
            
            x_from_uC = values(15)  *0.1;
            y_from_uC = values(16) *0.1;
            
            alt_x_from_uC = values(17) *0.1;
            alt_y_from_uC = values(18) *0.1;

            best_sender = 1;%values(11)+1;
            angle_beacon_to_beacon = 0;%values(12);
            angle_robot_to_beacon = 0;%values(13);
            angle_absolute = 0;%values(14);
            dist_cw = 0;%values(15);
            dist_ccw = 0;%values(16);

            distances
            qualities;
            
            global coop_y;
            global coop_x;
            coop_x = values(13+6);
            coop_y = values(14+6);

            ir_quality = values(15+6);
            ir_offset = values(16+6);
            ir_correlation_max = values(17+6);
            synchronization_offset = values(18+6);
            status_flags = values(19+6);
            
            
            
            
            global length_of_history;
            global hist_dist;
            global hist_qual;
            global hist_cw;
            global hist_ccw;
            global hist_ir_qual;
            global hist_ir_offset;
            global position_plot_handle; 
            
            set(position_plot_handle(5), 'XData', coop_y, 'YData', coop_x);

            hist_dist = hist_dist(:,2:end);
            hist_qual = hist_qual(:,2:end);
            hist_cw   = hist_cw(2:end);
            hist_ccw  = hist_ccw(2:end);
            hist_cw(length_of_history)  = dist_cw;
            hist_ccw(length_of_history) = dist_ccw;
            hist_ir_qual   = hist_ir_qual(2:end);
            hist_ir_offset = hist_ir_offset(2:end);
            hist_ir_qual(length_of_history) = ir_quality/100.0;
            hist_ir_offset(length_of_history) = ir_offset;
            for i = 1:4
                hist_dist(i,length_of_history) = distances(i);
                hist_qual(i,length_of_history) = qualities(i);
            end

            %handles = guidata(hObject);
            %hold(handles.axesSignals, 'off');
            color = ['r', 'g', 'b', 'y'];
            beacon_names = ['Master';'Slave1';'Slave2';'Angle '];
            global dist_plot_handle;
            for i=1:4
                %plot(handles.axesSignals,hist_dist(i,:),'Color',color(i),'DisplayName',beacon_names(i,:));
                set(dist_plot_handle(i), 'YData', hist_dist(i,:));
                %if i<2
                %    hold(handles.axesSignals, 'on');
                %end
            end
            %hold(handles.axesQuality, 'off');
            global quality_plot_handle;
            for i=1:4
                %plot(handles.axesQuality,hist_qual(i,:),'Color',color(i),'DisplayName',beacon_names(i,:));                     
                set(quality_plot_handle(i), 'YData', hist_qual(i,:));
                %if i<2
                %    hold(handles.axesQuality, 'on');
                %end
            end 
            
            global ir_plot_handle;
            set(ir_plot_handle(1), 'YData', hist_ir_qual);
            set(ir_plot_handle(2), 'YData', hist_ir_offset);

            global beaconX;
            global beaconY;
            global circle_plot_handle;
            
            dist_draw_selection = get(handles.popupmenuRangingDisplay, 'Value');
            if get(handles.popupmenuRangingDisplay, 'Value') == 1.0
                for i=1:3
                    radius = max(distances(i),10);
                    position = [(beaconY(i)-radius),(beaconX(i)-radius),(2.0*radius),(2.0*radius)];
                    set(circle_plot_handle(i), 'Position', position); 
                end
                radius = max(distances(4),10);
                set(circle_plot_handle(4), 'Position', [(coop_y-radius) (coop_x-radius) (2*radius) (2*radius)]);
            end
            
            %caclulate robot position
            
            quality_thres = -1.0;
            if (qualities(1)>quality_thres) && (qualities(2)>quality_thres) && (qualities(3)>quality_thres)

                x1 = beaconX(1);
                y1 = beaconY(1);
                x2 = beaconX(2);
                y2 = beaconY(2);
                x3 = beaconX(3);
                y3 = beaconY(3);
                d1 = distances(1);
                d2 = distances(2);
                d3 = distances(3);
                q1 = qualities(1);
                q2 = qualities(2);
                q3 = qualities(3);
% 
%                 botx = 0;
%                 boty = 0;
%                 iterations = 0;
%                 max_iterations = 20;
% 
%                 while iterations < max_iterations
%                     iterations = iterations +1;
%                     err_x0  = trilateration_error(botx   ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
%                     err_x1  = trilateration_error(botx+1 ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
%                     err_x2  = trilateration_error(botx+2 ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
% 
%                     err_dx_01 = err_x1 - err_x0; %1st derivation
%                     err_dx_12 = err_x2 - err_x1;           
%                     err_dx2 = err_dx_12-err_dx_01; %2nd derivation   
%                     new_botx = botx - err_dx_01/err_dx2;
% 
%                     err_y0  = trilateration_error(botx ,boty   ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
%                     err_y1  = trilateration_error(botx ,boty+1 ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
%                     err_y2  = trilateration_error(botx ,boty+2 ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
% 
%                     err_dy_01 = err_y1 - err_y0; %1st derivation
%                     err_dy_12 = err_y2 - err_y1;       
%                     err_dy2 = err_dy_12-err_dy_01; %2nd derivation 
%                     new_boty = boty - err_dy_01/err_dy2;
% 
%                     if sqrt( (botx-new_botx)^2 + (boty-new_boty)^2 ) < 0.05
%                         iterations
%                         iterations = max_iterations;
%                     end
%                     botx = new_botx;
%                     boty = new_boty;
%                 end
%     
%                 
                botx = x_from_uC;
                boty = y_from_uC;
                
                global hist_botx;
                global hist_boty;
                
                %<swap botxy with altbotxy if alt is closer to old position
                if (botx ~= alt_x_from_uC)||(boty ~= alt_y_from_uC)
                    d_pos = sqrt( (botx-hist_botx(end))^2 + (boty-hist_boty(end))^2 );
                    d_alt_pos = sqrt( (alt_x_from_uC-hist_botx(end))^2 + (alt_y_from_uC-hist_boty(end))^2 );
                    
                    if (d_alt_pos < d_pos)
                        temp_x = botx;
                        temp_y = boty;
                        botx = alt_x_from_uC;
                        boty = alt_y_from_uC;
                        alt_x_from_uC = temp_x;
                        alt_y_from_uC = temp_y;
                    end
                end
                %</swap>
                
                hist_botx = hist_botx(2:end);
                hist_boty = hist_boty(2:end);
                hist_botx(length_of_history) = botx;
                hist_boty(length_of_history) = boty;
                %plot(handles.axesPlayground, hist_boty, hist_botx, 'k');
                %plot(handles.axesPlayground, boty, botx, 'o');

                
                set(position_plot_handle(1), 'XData', hist_boty, 'YData', hist_botx);
                set(position_plot_handle(2), 'XData', [boty alt_y_from_uC], 'YData', [botx alt_x_from_uC]);
% 
% 
% 
%                 %sprintf('botx=%d x_from_uC=%d boty=%d y_from_uC=%d', botx, x_from_uC, boty, y_from_uC)
% 
%     % Angle estimation using closest beacon             
%     %             plot(handles.axesPlayground, [boty beaconY(best_sender)], [botx beaconX(best_sender)], strcat(':',color(best_sender)));  
%     %             
%     %             dx = botx - beaconX(best_sender);
%     %             dy = boty - beaconY(best_sender);
%     %             
%     %             angle_robot_to_beacon = atan2( dy, dx )
%     %             
%     %             viewX = botx + 500*cos(angle_absolute);
%     %             viewY = boty + 500*sin(angle_absolute);
%     %             plot(handles.axesPlayground, [boty viewY], [botx viewX], ':k');        
%     %             
%     %             hold(handles.axesAngle, 'off');
%     %             plot(handles.axesAngle, hist_cw, ':k');
%     %             hold(handles.axesAngle, 'on');
%     %             plot(handles.axesAngle, hist_ccw, '--k');
%     %             plot(handles.axesAngle,hist_dist(best_sender,:),'Color',color(best_sender));
% 
%                 %color plot
% 
%     %             x_grid = -1000:50:1000; 
%     %             y_grid = -1500:50:1500;
%     %             size_x = length(x_grid);
%     %             size_y = length(y_grid);
%     %             
%     %             error_map = zeros([size_x,size_y]); 
%     %             
%     %             for x = 1:size_x
%     %                 for y = 1: size_y
%     %                     error_map(x,y) = trilateration_error(x_grid(x) ,y_grid(y),x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
%     %                 end
%     %             end
%     %             pcolor(error_map);
%    
%             

                %%% <Position filtering>

                distance_quality = min( [max(q1, q2), max(q2, q3), max(q1, q3)] ); %pair with q >= distancequality availabe
                distance_quality = min( distance_quality, 50);

                trilat_error = trilateration_error(botx ,boty   ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
                position_quality = distance_quality / (trilat_error + 10)
                
                global position_quality_hist;
                position_quality_hist = position_quality_hist(2:end);
                position_quality_hist(length_of_history) = position_quality;

                position_quality_threshold = 0.001;%(50.0 / 20.0);
                max_filter_length = 5;

                filter_length = 0;
                quality_sum = 0;
                while (quality_sum <= position_quality_threshold) && (filter_length < max_filter_length);
                    filter_length = filter_length+1;
                    quality_sum = quality_sum + position_quality_hist(end-filter_length+1); 
                end

                time_weighted_quality_sum = 0;
                pos_x_accu = 0;
                pos_y_accu = 0;
                for i = 1:filter_length
                    quality_factor = position_quality_hist(end-i+1)/(i^2);
                    pos_x_accu = pos_x_accu + (hist_botx(end-i+1) * quality_factor);
                    pos_y_accu = pos_y_accu + (hist_boty(end-i+1) * quality_factor);
                    
                    time_weighted_quality_sum = time_weighted_quality_sum + quality_factor;
                end
                botx_filtered = pos_x_accu / time_weighted_quality_sum;
                boty_filtered = pos_y_accu / time_weighted_quality_sum;

                global hist_botx_filtered;
                global hist_boty_filtered;
                hist_botx_filtered = hist_botx_filtered(2:end);
                hist_boty_filtered = hist_boty_filtered(2:end);
                hist_botx_filtered(length_of_history) = botx_filtered;
                hist_boty_filtered(length_of_history) = boty_filtered;

                set(position_plot_handle(3), 'XData', hist_boty_filtered, 'YData', hist_botx_filtered);
                set(position_plot_handle(4), 'XData', boty_filtered, 'YData', botx_filtered);

               
                
                %%% </Position filtering>
            
                
                
                
                
            end;
            
            global guihandle;
                
            %status_flags = values(17);

            sync_states = {'Synchronizing' 'Synchronized' 'Master'};
            ir_sig_qual_states = {'Bad' 'Good'};
            
            
            sync_state = char(sync_states( bitand(status_flags,1+2)+1 ));
            ir_state = char(ir_sig_qual_states( (bitand(status_flags,4) ~= 0)+1));
            
            set(guihandle.textStatusDebug, 'String', '');
            text = sprintf('botx=%i\nboty=%i\nir_cor_m=%i\nso=%f\nsync=%s\nir_sig=%s\nstd(d0)=%4.2f',...
                            botx, boty, ir_correlation_max,synchronization_offset/1000.0, sync_state, ir_state, std(hist_dist(1,:)));
            set(guihandle.textStatusDebug, 'String', text);
            
            drawnow;

        end %if s.BytesAvailable == 0
    end %if values ~= []

elseif line(1) == 'r' %information about the reduced spectrum, used to calculate the angle
    % d[signal_nr][mic_nr]         1    2      3    4   
    line
    values = sscanf(line, 'r d00=%i u=%i d01=%i u=%i d02=%i u=%i d03=%i u=%i d10=%i u=%i d11=%i u=%i d12=%i u=%i d13=%i u=%i d20=%i u=%i d21=%i u=%i d22=%i u=%i d23=%i u=%i d30=%i u=%i d31=%i u=%i d32=%i u=%i d33=%i u=%i \n');
    values;
    
    r_distance = zeros(4,4);
    r_unambigu = zeros(4,4);
    
    for sender=1:4
        for mic=1:4
            r_distance(sender,mic)=values( 2* (((sender-1)*4) + (mic-1))+1 );
            r_unambigu(sender,mic)=values( 2* (((sender-1)*4) + (mic-1))+2 );
        end
    end
    
    s1_dist = r_distance(1,:);
    s1_unam = r_unambigu(1,:);
    
    [~,los_mic] = max(s1_unam); %get microphone with line of sight and best signal
    if s1_unam(los_mic) > 30 %only evaluate qngle if line of sight quality > 300%
        s1_unam(los_mic) = 0;
        
        nr_of_microphones = 4;
        mic_n = los_mic-1;
        if mic_n < 1
            mic_n = nr_of_microphones; 
        end
        
        diff_n = 0;
        quality_n = s1_unam(mic_n);
          
        if quality_n > 20           
            diff_n = s1_dist(mic_n) - s1_dist(los_mic);      
            quality_n = s1_unam(mic_n);
        end
        
        mic_p = los_mic+1;
        if mic_p > nr_of_microphones
            mic_p = 1; 
        end
        
        diff_p = 0;
        quality_p = s1_unam(mic_p);
          
        if quality_p > 20
            diff_p = s1_dist(mic_p) - s1_dist(los_mic);
            
            quality_p = s1_unam(mic_p);
           
        end
          
        
        %####################<tested in plot_angle_info.m>    
        
        d = 52; %distance between microphones
        mic_px = [-d/2, d/2, d/2, -d/2, -d/2];
        mic_py = [-d/2,-d/2, d/2,  d/2, -d/2];
        
        if diff_p >= d
            diff_p = d * sign(diff_p);
            quality_p = 0;
        end
        if diff_p < -15
            diff_p = -15;
            message='diff_p < 15 !!!'
            quality_p = 0;
        end
            
        if diff_n >= d
            diff_n = d * sign(diff_n);
            quality_n = 0;
        end
        if diff_n < -15
            diff_n = -15;
            message='diff_n < -15 !!!'
            quality_n = 0;
        end
        
        %mic in positive direction
        alpha_p = acos( diff_p / d );
        %alpha_p_deg = alpha_p * 360/(2*pi);

        lin = [-cos(alpha_p)*diff_p; -sin(alpha_p)*diff_p]; 
        rot = (pi/2)*(mic_p-2);
        rotmatrix = [[ cos(rot),-sin(rot)];[sin(rot), cos(rot)]];
        d_rot = rotmatrix*lin;

        dxp = d_rot(1);
        dyp = d_rot(2);

        lin_p = d_rot;
        
        %mic in negative direction  
        alpha_n = acos( diff_n / d );
        %alpha_n_deg = alpha_n * 360/(2*pi);

        lin = [sin(alpha_n)*diff_n; cos(alpha_n)*diff_n]; 
        rot = (pi/2)*(mic_n-2);
        rotmatrix = [[ cos(rot),-sin(rot)];[sin(rot), cos(rot)]];
        d_rot = rotmatrix*lin;

        dxn = d_rot(1);
        dyn = d_rot(2); 

        lin_n = d_rot;
        
        %############### linear combination

        if norm(lin_n) ~= 0
            lin_n = (lin_n / norm(lin_n)) * quality_n * sign(diff_n);
        end
        if norm(lin_p) ~= 0
            lin_p = (lin_p / norm(lin_p)) * quality_p * sign(diff_p);
        end
        lin_tot = lin_n + lin_p;

        zoom = d / norm(lin_tot);
        lin_tot = lin_tot*zoom;
        lin_n = lin_n*zoom;
        lin_p = lin_p*zoom;
        %end_of_</tested in plot_angle_info.m>        
           
        global angle_figure_enabled;
        if angle_figure_enabled == 1 
            global angle_plot_handles;
            %triangle in poisitve directrion
            set(angle_plot_handles(1), 'XData', [mic_px(mic_p), mic_px(mic_p)+dxp], 'YData', [mic_py(mic_p), mic_py(mic_p)+dyp]);  
            set(angle_plot_handles(2), 'String', int2str(diff_p), 'Position', [mic_px(mic_p)+dxp/2, mic_py(mic_p)+dyp/2]); 
            set(angle_plot_handles(3), 'XData', [mic_px(los_mic), mic_px(mic_p)+dxp], 'YData', [mic_py(los_mic), mic_py(mic_p)+dyp]);

            %triangle in negative directrion
            set(angle_plot_handles(4), 'XData', [mic_px(mic_n), mic_px(mic_n)+dxn], 'YData', [mic_py(mic_n), mic_py(mic_n)+dyn]);  
            set(angle_plot_handles(5), 'String', int2str(diff_n), 'Position', [mic_px(mic_n)+dxn/2, mic_py(mic_n)+dyn/2]); 
            set(angle_plot_handles(6), 'XData', [mic_px(los_mic), mic_px(mic_n)+dxn], 'YData', [mic_py(los_mic), mic_py(mic_n)+dyn]);

            %central geometric addition    
            set(angle_plot_handles(7), 'XData', [0, lin_tot(1)], 'YData', [0, lin_tot(2)]);
            set(angle_plot_handles(8), 'XData', [0, lin_p(1)], 'YData', [0, lin_p(2)]);
            set(angle_plot_handles(9), 'XData', [lin_tot(1), lin_p(1)], 'YData', [lin_tot(2), lin_p(2)]);                       
        end 
    end  
else
    line
end



function error = trilateration_error(botx,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3)
act_d1 = sqrt( (x1-botx)^2 + (y1-boty)^2 );
act_d2 = sqrt( (x2-botx)^2 + (y2-boty)^2 );
act_d3 = sqrt( (x3-botx)^2 + (y3-boty)^2 );

error = abs( abs((act_d1-d1)^2)*q1 + abs((act_d2-d2)^2)*q2 + abs((act_d3-d3)^2)*q3 );




% --- Executes on button press in buttonDisconnect.
function buttonDisconnect_Callback(hObject, eventdata, handles)
% hObject    handle to buttonDisconnect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fprintf('Trying to close Port ..');
fclose(handles.serialPort);
handles.serialPortOpen = false;
guidata(hObject, handles);
fprintf('Port closed\n');


% --- Executes on button press in pushbuttonViewPlaygroundTab.
function pushbuttonViewPlaygroundTab_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonViewPlaygroundTab (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.uipanelPlayground,'Visible','On');
set(handles.uipanelWaveform,'Visible','Off');

% --- Executes on button press in pushbuttonViewWaveform.
function pushbuttonViewWaveform_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonViewWaveform (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.uipanelPlayground,'Visible','Off');
set(handles.uipanelWaveform,'Visible','On');


function editPortName_Callback(hObject, eventdata, handles)
% hObject    handle to editPortName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editPortName as text
%        str2double(get(hObject,'String')) returns contents of editPortName as a double


% --- Executes during object creation, after setting all properties.
function editPortName_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editPortName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonSetDACoutput.
function pushbuttonSetDACoutput_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonSetDACoutput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1


% --- Executes on button press in checkboxMIC2.
function checkboxMIC2_Callback(hObject, eventdata, handles)
% hObject    handle to checkboxMIC2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkboxMIC2


% --- Executes on button press in checkboxMIC3.
function checkboxMIC3_Callback(hObject, eventdata, handles)
% hObject    handle to checkboxMIC3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkboxMIC3


% --- Executes on button press in checkboxMIC4.
function checkboxMIC4_Callback(hObject, eventdata, handles)
% hObject    handle to checkboxMIC4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkboxMIC4


% --- Executes on button press in pushbuttonSetDacOutput.
function pushbuttonSetDacOutput_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonSetDacOutput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.serialPortOpen == true
    
    fwrite(handles.serialPort, 45); %enable debug interface
    pause(0.2);
    
    dac_channel = get(handles.popupmenuDacChannel, 'Value');
    waveform_nr = get(handles.popupmenuWaveform, 'Value');
    message = dac_channel*64+waveform_nr;
    fwrite(handles.serialPort, message);
else
    fprintf('Error: Serial Port not open');
end    
guidata(hObject, handles);

% --- Executes on button press in pushbuttonPlotWaveform.
function pushbuttonPlotWaveform_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonPlotWaveform (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.serialPortOpen == true
    waveform_nr = get(handles.popupmenuWaveform, 'Value')
    message = 3*64+waveform_nr;
    fwrite(handles.serialPort, message);
else
    fprintf('Error: Serial Port not open');
end    
guidata(hObject, handles);

% --- Executes on selection change in popupmenuDacChannel.
function popupmenuDacChannel_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenuDacChannel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenuDacChannel contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenuDacChannel


% --- Executes during object creation, after setting all properties.
function popupmenuDacChannel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenuDacChannel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenuWaveform.
function popupmenuWaveform_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenuWaveform (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenuWaveform contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenuWaveform


% --- Executes during object creation, after setting all properties.
function popupmenuWaveform_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenuWaveform (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenuColour.
function popupmenuColour_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenuColour (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenuColour contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenuColour


% --- Executes during object creation, after setting all properties.
function popupmenuColour_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenuColour (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkboxHold.
function checkboxHold_Callback(hObject, eventdata, handles)
% hObject    handle to checkboxHold (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkboxHold



function editScale_Callback(hObject, eventdata, handles)
% hObject    handle to editScale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editScale as text
%        str2double(get(hObject,'String')) returns contents of editScale as a double


% --- Executes during object creation, after setting all properties.
function editScale_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editScale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonShowAngle.
function pushbuttonShowAngle_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonShowAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global angle_figure_enabled;
angle_figure_enabled = 1;
%angle_plot_handle = plot([-100, 100], [-100,100] );

figure;
daspect([1 1 1])
set(gca,'NextPlot','add')
d = 52; %distance between microphones
mic_px = [-d/2, d/2, d/2, -d/2, -d/2];
mic_py = [-d/2,-d/2, d/2,  d/2, -d/2];
mic_name = ['1', '2', '3', '4'];
nr_of_microphones = 4;

plot( mic_px, mic_py, 'black' );
plot( mic_px .*2, mic_py .*2, 'white');
for i=1:nr_of_microphones
    text( mic_px(i), mic_py(i), mic_name(i) );
end

%triangle in poisitve directrion 
global angle_plot_handles;
angle_plot_handles(1) = plot( [0, d], [0, 0], 'blue' );
angle_plot_handles(2) = text( 0, 0, '', 'Color', 'blue'); 
angle_plot_handles(3) = plot( [0, 0], [d, 0], 'Color', [0.5, 0.5, 0.5]  );

%triangle in negative directrion
angle_plot_handles(4) = plot( [0, -d], [0, 0], 'green' );
angle_plot_handles(5) = text( 0, 0, '', 'Color', 'green'); 
angle_plot_handles(6) = plot( [0, 1], [-d, 0], 'Color', [0.5, 0.5, 0.5]  );

%central geometric addition
angle_plot_handles(7) = plot( [0, 0], [-d, 0], 'red');
angle_plot_handles(8) = plot( [0, 1], [0, 0], 'blue');
angle_plot_handles(9) = plot( [0, 0], [-d, 0], 'green');

set(angle_plot_handles(1), 'XData', [0,d], 'YData', [0,d]);  

% --- Executes on button press in pushbuttonDebug.
function pushbuttonDebug_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonDebug (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global angle_plot_handles;
set(angle_plot_handles(1), 'XData', [0,-52], 'YData', [0,52]);  


% --- Executes on button press in pushbuttonSaveWF.
function pushbuttonSaveWF_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonSaveWF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 global length_of_debug_array;
 global debug_collector;
 save_data = debug_collector(1:length_of_debug_array);
 uisave('save_data');
 
 
 
        


% --- Executes on button press in pushbuttonSaveSnapshot.
function pushbuttonSaveSnapshot_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonSaveSnapshot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global debug_collector;
microphone_samplerate = 160000;
microphone_signals = [debug_collector((2048*0)+1:(2048*1));
                     debug_collector((2048*1)+1:(2048*2));
                     debug_collector((2048*2)+1:(2048*3));
                     debug_collector((2048*3)+1:(2048*4))];

global beaconX;
global beaconY;
fixed_speaker_position = [beaconX(1),beaconY(1);...
                          beaconX(2),beaconY(2);...
                          beaconX(3),beaconY(3) ];                      
                      
local_microphone_position = [-70,-70;...
                              70,-70;...
                              70, 70;...
                             -70, 70];

speaker_samplerate = 320000;
load('speaker_signals');

fixed_speaker_signal_ids = [1,2,3];
coop_beacon_signal_ids = [4];

global snapshot_beacon_message;
beacon_message = snapshot_beacon_message;
beacon_message_description = ['d0={distance in mm of signal 1 (fixed beacon)}',char(10),...
                              'u0={signal quality of signal 1}',char(10),...
                              'd1={distance in mm of signal 2 (fixed beacon)}',char(10),...
                              'u1={signal quality of signal 2}',char(10),...
                              'd2={distance in mm of signal 3 (fixed beacon)}',char(10),...
                              'u2={signal quality of signal 3}',char(10),...
                              'd3={distance in mm of signal 4 (mobile coop beacon)}',char(10),...
                              'u3={signal quality of signal 4}',char(10),...
                              'x={x position of beacon in mm}',char(10),...
                              'y={y position of beacon in mm}',char(10),...
                              'ax={alternative x position of beacon if signal is ambiguous',char(10),...
                              'ay={alternative y position of beacon if signal is ambiguous',char(10),...
                              'cx={x position in mm of mobile cooperative beacon}',char(10),...
                              'cy={x position in mm of mobile cooperative beacon}',char(10)];
                              
uisave({'microphone_signals',...
        'local_microphone_position','microphone_samplerate',...
        'speaker_signals',...
        'fixed_speaker_position', 'speaker_samplerate',...
        'fixed_speaker_signal_ids','coop_beacon_signal_ids'...
        'beacon_message'});


% --- Executes on button press in checkboxSaveStream.
function checkboxSaveStream_Callback(hObject, eventdata, handles)
% hObject    handle to checkboxSaveStream (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkboxSaveStream
global stream_file_handle;
if get(hObject,'Value') == 1.0
    filename = uiputfile
    stream_file_handle = fopen(filename,'a');
else
    fclose(stream_file_handle);
end


% --- Executes on button press in pushbuttonReplayStream.
function pushbuttonReplayStream_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonReplayStream (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
filename = uigetfile
stream_file_handle = fopen(filename,'r');
line = 'start';
while ~strcmp(line,'')
    line = fgetl(stream_file_handle);
    serialReceiveLine (line, 0, hObject);
    drawnow
    pause(0.05);
end


%line = ['d0=20169 u0=41 d1=13068 u1=41 d2=18318 u2=43 d3=0 u3=0  x=14317 y=1733 ax=-4205 ay=-3786 cx=0 cy=0 IRQ=262 IR_o=2 IRC=589 OD=0 s=4'];


% --- Executes on button press in pushbuttonSaveAxesPlayground.
function pushbuttonSaveAxesPlayground_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonSaveAxesPlayground (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fig_save = figure;
copyobj(handles.axesPlayground, fig_save);


% --- Executes on button press in checkboxNRFDebugStream.
function checkboxNRFDebugStream_Callback(hObject, eventdata, handles)
% hObject    handle to checkboxNRFDebugStream (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkboxNRFDebugStream
if handles.serialPortOpen == true
    if get(hObject,'Value') == 1.0
        fwrite(handles.serialPort, 1); %enable nrf packet content debug output
    else
        fwrite(handles.serialPort, 2); %disable nrf packet content debug output
    end
end    
guidata(hObject, handles);



% --- Executes on button press in checkboxMicPosDebug.
function checkboxMicPosDebug_Callback(hObject, eventdata, handles)
% hObject    handle to checkboxMicPosDebug (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkboxMicPosDebug
if handles.serialPortOpen == true
    if get(hObject,'Value') == 1.0
        fwrite(handles.serialPort, 3); %enable microphone position debug output
    else
        fwrite(handles.serialPort, 4); %disable microphone position debug output
    end
end    
guidata(hObject, handles);

% --- Executes on button press in checkboxMicMatrixDebug.
function checkboxMicMatrixDebug_Callback(hObject, eventdata, handles)
% hObject    handle to checkboxMicMatrixDebug (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkboxMicMatrixDebug
if handles.serialPortOpen == true
    if get(hObject,'Value') == 1.0
        fwrite(handles.serialPort, 5); %enable us ranging matrix debug output
    else
        fwrite(handles.serialPort, 6); %disable us ranging matrix debug output
    end
end    
guidata(hObject, handles);



% --- Executes on selection change in popupmenuRangingDisplay.
function popupmenuRangingDisplay_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenuRangingDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenuRangingDisplay contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenuRangingDisplay


% --- Executes during object creation, after setting all properties.
function popupmenuRangingDisplay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenuRangingDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
