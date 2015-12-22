function varargout = DropletGUI(varargin)
% DROPLETGUI MATLAB code for DropletGUI.fig
%      DROPLETGUI, by itself, creates a new DROPLETGUI or raises the existing
%      singleton*.
%
%      H = DROPLETGUI returns the handle to a new DROPLETGUI or the handle to
%      the existing singleton*.
%
%      DROPLETGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DROPLETGUI.M with the given input arguments.
%
%      DROPLETGUI('Property','Value',...) creates a new DROPLETGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before DropletGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to DropletGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help DropletGUI

% Last Modified by GUIDE v2.5 22-Dec-2015 16:28:27

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @DropletGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @DropletGUI_OutputFcn, ...
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


% --- Executes just before DropletGUI is made visible.
function DropletGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to DropletGUI (see VARARGIN)

% Choose default command line output for DropletGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes DropletGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = DropletGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global FileName address
global idx %Index of video file
global i %Index of frame in (idx)th video file
global angle cropLeft cropRight cropUp cropDown Crop
[FileName,address] = uigetfile('MultiSelect','on');
if ischar(FileName)                                                         %% To recignize each cell in array
    FileName = {FileName};
end
idx = 1; i = 1; %Inititate i and idx
vid=VideoReader([address,FileName{idx}]); %Video object
img = read(vid,i);
if (isempty(angle) == 0)                                                    %% ==0: is not empty, == 1: is empty
    img = imrotate(img,angle);
end
axes(handles.axes1);                                                        
if (Crop == 1)                                                              
    img = img(cropUp:cropDown,cropLeft:cropRight);
end
imshow(img);


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global i
global idx
global FileName
global address
global angle
global cropLeft
global cropRight
global cropUp
global cropDown
global Crop
vid=VideoReader([address,FileName{idx}]); %Video object
hObject.Max = vid.NumberOfFrames; %Maximum Value as the total number of frames in the (idx)th video file
hObject.SliderStep = [1/(vid.NumberofFrames - 1) 5/(vid.NumberofFrames)];   
i = round(hObject.Value);
img = read(vid,i);
if (isempty(angle) == 0)
    img = imrotate(img,angle);
end
axes(handles.axes1);
if (Crop == 1)                                                              
    img = img(cropUp:cropDown,cropLeft:cropRight);
end
imshow(img);

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global idx
global i
global FileName
global address
global angle
global cropLeft
global cropRight
global cropUp
global cropDown
global Crop
hObject.Value = floor(hObject.Value);

if (hObject.Value > length(FileName))
    hObject.Value = length(FileName);
end
hObject.Max = length(FileName);
if (length(FileName) == 1)
    hObject.Max = 2;
end
if (length(FileName) == 1)
    hObject.SliderStep = [0.99 0.99];
else
    hObject.SliderStep = [1/(length(FileName) - 1) 2/(length(FileName) - 1)];
end
hObject.Value
if (hObject.Value > length(FileName))
    hObject.Value = length(FileName);
end
if (length(FileName) == 1)
    hObject.Value = 1;
end
idx = hObject.Value;
axes(handles.axes1);
vid=VideoReader([address,FileName{idx}]); %Video object
img = read(vid,i);
if (isempty(angle) == 0)
    img = imrotate(img,angle);
end
if (Crop == 1)
    img = img(cropUp:cropDown,cropLeft:cropRight);
end
imshow(img);


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


%Angle
function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double
global FileName
global address
global idx
global i
global angle
global cropLeft
global cropRight
global cropUp
global cropDown
global Crop
angle = str2num(hObject.String);
vid=VideoReader([address,FileName{idx}]); %Video object
img = read(vid,i);
if (isempty(angle) == 0)
    img = imrotate(img,angle);
end
axes(handles.axes1);
if (Crop == 1)
    img = img(cropUp:cropDown,cropLeft:cropRight);
end
imshow(img);

% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton2. cropLeft
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global cropLeft
axes(handles.axes1);
[x,y] = ginput(1);                                                          %% Graphical one input point from mouse. 
cropLeft = round(x);

% --- Executes on button press in pushbutton3. cropUp
function pushbutton4_Callback(hObject, eventdata, handles) 
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global cropRight
axes(handles.axes1);
[x,y] = ginput(1);
cropRight = round(x);

% --- Executes on button press in pushbutton4.
function pushbutton3_Callback(hObject, eventdata, handles) %cropUp
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global cropUp
[x,y] = ginput(1);
cropUp = round(y);

% --- Executes on button press in pushbutton5.
function pushbutton2_Callback(hObject, eventdata, handles) %cropDown
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global cropDown
[x,y] = ginput(1);
cropDown = round(y);


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles) %Crop
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1
global Crop
Crop = hObject.Value;


function edit2_Callback(hObject, eventdata, handles) %Scale
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double
global scale %micrometer per pixel
scale = str2num(hObject.String);

% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles) %Frame Rate
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double
global fps
fps = str2num(hObject.String);

% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton6. Initiate Sequence Button
% (Main Tracking Code)
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global FileName
global address
global scale
global fps
global cropUp cropDown cropLeft cropRight pop threshold
global data
global Crop
global angle smooth
global se se1 Size
global minax 
if (isempty(pop))
    pop = 1;
end
if (isempty(threshold))
    threshold = 0.4;
end
for idx = 1:length(FileName)
    h = msgbox(['Processing Video File ',int2str(idx),' of ',int2str(length(FileName))]);
    drawnow                                                                 %% Update figure windows and process callbacks
    color = [{'red'},{'green'},{'blue'},{'black'},{'magenta'}];
    passed = 0;
    Area = {[]};
    delta = {[]};
    majax = {[]};
    minax = {[]};
    X = {[]};
    clear v
    v = VideoWriter([address,FileName{idx},'Tracked.avi']);
    v.FrameRate = 5;
    open(v);
    clear Im
    vid=VideoReader([address,FileName{idx}]); %Video object
    numFrames = vid.NumberOfFrames;
    n=numFrames;
    
for i = 1:n
    Im(:,:,(i - 1) + 1) = read(vid,i);
end

flag = 0; 
%imshow(Im(:,:,1));
% [x,y] = ginput(4);
% cropLeft = x(1);
% cropRight = x(2);
% cropUp = y(3);
% cropDown = y(4);
for i = 1:size(Im,3)
    %i
%     im = imread([address,FileName{i}]);
%     im = rgb2gray(im);
    if (isempty(cropDown))
        cropDown = size(Im,1);
    end
    im = Im(:,:,i);
    im = mat2gray(im);
    im = imrotate(im,angle);
    if (Crop == 1)
        im = im(cropUp:cropDown,cropLeft:cropRight);
    end
    if (isempty(smooth) == 1)
        smooth = sqrt(2);
    end
    
    if (isempty(se) == 1)
        se = strel('disk',1);
    end
    if (isempty(se1) == 1)
        se1 = strel('disk',5);
    end
    if (pop == 1)
        im = wiener2(im,[5,5]);                                                 %Applies wiener filter before thresholding
        ed = edge(im,'canny',[],smooth);
        ed = imdilate(ed,se);
        bw = imfill(ed,'holes');
        bw = imerode(bw,se);
    end
    if (pop == 2)
        bw = im2bw(im,threshold);
        bw = imfill(imcomplement(bw),'holes');
    end
    
    if (isempty(Size) == 1)
        Size = 2000;
    end
    bw = bwareaopen(bw,Size);
    bw = imopen(bw,se1);
    bw = imclearborder(bw);
    %a = regionprops(bw,'Area');
    edg = bwperim(bw);
    %imwrite(bw,[address,int2str(i),'.tif']);
    stats = regionprops(bw,'Centroid','MajorAxisLength','MinorAxisLength','Area');      %% Measure properties of image region
    im = im + double(edg);
    imwrite(bw,[address,int2str(i),'bw.tif']);
    ref = [];
    ascInd = [];
        if (i == 1)
            flag = 0;
        end
        
        if (size(stats,1) ~= 0)
            for z = 1:size(stats,1)
                
            ref(z) = stats(z).Centroid(2);
            end
            refSort = sort(ref);
            if (i ~= 1 && exist('top'))
                if (min(ref) > top )
                    passed = passed + 1;
                end
            end
            top = min(ref);
            for k = 1:length(refSort)
                ascInd(k) = find(ref == refSort(k));
            end
        CNew = [];
        for p = 1:size(stats,1)
            im = insertMarker(im,[stats(ascInd(p)).Centroid(1),stats(ascInd(p)).Centroid(2)],'Color',color(mod((p+passed),5)+1),'size',10);
            CNew(p,:) = stats(ascInd(p)).Centroid;
            A(p) = stats(ascInd(p)).Area*scale^2;
            Majax(p) = stats(ascInd(p)).MajorAxisLength*scale;
            Minax(p) = stats(ascInd(p)).MinorAxisLength*scale;
            Delta(p) = (Majax(p) - Minax(p))/(Majax(p) + Minax(p));
            if (flag == 1)
            
            if (p + passed > length(majax))
%                 speed{p + passed} = [];
                Area{p + passed} = [];
                delta{p + passed} = [];
%                 speed{p + passed} = [];
                majax{p + passed} = [];
                minax{p + passed} = [];
                X{p + passed} = [];
            else
                %speed{p + passed} = [speed{p + passed},Speed(p)];
                Area{p + passed} = [Area{p + passed},A(p)];
                delta{p + passed} = [delta{p + passed},Delta(p)];
                majax{p + passed} = [majax{p + passed},Majax(p)];
                minax{p + passed} = [minax{p + passed},Minax(p)];
                X{p + passed} = [X{p + passed},(size(im,1) - CNew(p,2))*scale];
            end
            end
        end
        flag = 1;
        COld = CNew;
        im = mat2gray(im);
        else flag = 0; COld = [];
        end
        writeVideo(v,im);
    %%%%%%%%%%%
    
    %%%%%%%%%%%
    
end

for l = 1:length(X)
    xx{l} = (1:length(X{l}))/fps;
end

if (isempty(Area) == 0)
data(idx).area = Area(~cellfun('isempty',Area));                            %% cellfun('imsempty'): make 0 or 1 --> to eliminate [null] array
data(idx).delta = delta(~cellfun('isempty',delta));
data(idx).majax = majax(~cellfun('isempty',delta));
data(idx).minax = minax(~cellfun('isempty',minax));
data(idx).X = X(~cellfun('isempty',X));
data(idx).XX = xx;
end

%%%%%%%%%%%%
close(v);
close(h);
end
set(handles.text10,'String',['(',int2str(length(FileName)),')']);          %% Set object properties: handle text10 in GUI to present drop number
set(handles.text11,'String',['(',int2str(length(data(1).X)),')']);         %% Set object properties: handle text11 in GUI to present video number


function edit4_Callback(hObject, eventdata, handles) %Matrix Viscosity
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double
global viscC
viscC = str2num(hObject.String);

% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles) %Droplet Viscosity
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double
global viscD
viscD = str2num(hObject.String);

% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles) %Fitting
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global fps scale xlimit lengthx 
global data datafit
global Ndrop NVideo
global viscC viscD
global p1 p2 p3 p4 p5
global z1 z2
visc = [viscD,viscC];
if (isempty(Ndrop) == 1)
    Ndrop = 1;
end
if (isempty(NVideo) == 1)
    NVideo = 1;
end
X = data(NVideo).X{Ndrop};
lengthx = length(X);
XX = (1:length(X))/fps;
if (isempty(xlimit) == 1 || xlimit > length(X) - 2)
    xlimit = length(X) - 2;
end
axes(handles.axes2);                                                        %% write in the axes2 figure until axes6
plot(X(1:xlimit),XX(1:xlimit),'.'); xlabel('X'); ylabel('Time');
if (isempty(p1) == 0)
    Xf = csaps(XX, X, p1, XX);    %% Cubic smoothing spline.
    axes(handles.axes2);                                                    
    hold on 
    plot(Xf(1:xlimit + 2),XX(1:xlimit + 2));
    hold off
else Xf = X;
end
axis([min(X) max(X)  min(XX) max(XX)]);
majax = data(NVideo).majax{Ndrop};
axes(handles.axes6);
plot(1:xlimit + 2,majax(1:xlimit + 2),'.');
if (isempty(p2) == 0)
    Max = csaps(1:length(majax), majax, p2, 1:length(majax));
    axes(handles.axes6);
    hold on 
    plot(1:xlimit + 2,Max(1:xlimit + 2));
    hold off
end
axis([0 length(majax) min(majax) max(majax)]);
minax = data(NVideo).minax{Ndrop};
axes(handles.axes7);
plot(1:xlimit + 2,minax(1:xlimit + 2),'.');
if (isempty(p3) == 0)
    Mix = csaps(1:length(minax), minax, p3, 1:length(minax));
    axes(handles.axes7);
    hold on 
    plot(1:xlimit + 2,Mix(1:xlimit + 2));
    hold off
end
axis([0 length(minax) min(minax) max(minax)]);
if (isempty(p2) == 1 || isempty(p3) == 1)
    Max = majax;
    Mix = minax;
end

delta = (Max - Mix)./(Max + Mix);
[epsilonDot,z1,z2,speed] = intTsn(delta,Mix,Max,Xf,visc);    
axes(handles.axes8);
plot(Xf(1:xlimit + 1),speed(1:xlimit + 1),'.');                            
if (isempty(p4) == 0)
    speedf = csaps(Xf(1:length(speed)), speed, p4, Xf(1:length(speed)));
    axes(handles.axes8);
    hold on 
    plot(Xf(1:xlimit + 1),speedf(1:xlimit + 1));
    hold off
end
axis([min(Xf) max(Xf) min(speed) max(speed)]);
if (isempty(xlimit) == 1 || xlimit > length(epsilonDot))                    
    xlimit = length(epsilonDot);
end
axes(handles.axes3);
plot(X(1:xlimit),epsilonDot(1:xlimit),'.'); xlabel('X'); ylabel('epsilonDot');     
if (isempty(p5) == 0)
    epsilonf = csaps(Xf(1:length(epsilonDot)), epsilonDot, p5, Xf(1:length(epsilonDot)));
    axes(handles.axes3)
    hold on 
    plot(Xf(1:xlimit),epsilonf(1:xlimit));
    hold off
end
axis([min(Xf) max(Xf) min(epsilonDot) max(epsilonDot)]);
if (isempty(p5) == 1)
    epsilonf = epsilonDot;
end
axes(handles.axes4);
plot(X(1:xlimit + 2),delta(1:xlimit + 2),'.'); xlabel('X'); ylabel('Deformation');
axis([min(X) max(X) min(delta) max(delta)]);
axes(handles.axes5);
coeff = polyfit(z2(1:xlimit),z1(1:xlimit),1);
slope = coeff(1);
plot(z2(1:xlimit),z1(1:xlimit),'.');
hold on 
plot(z2(1:xlimit),polyval(coeff,z2(1:xlimit)));
hold off
axis([min(z1) max(z2) 0 max(z1)]);                                             %Easier to see plots when xlimit is changed
set(handles.text16,'String',['Slope = ',num2str(slope)]);                   %% To handle of text16 in GUI
datafit.area = data(NVideo).area{Ndrop};
datafit.minax = Mix;
datafit.majax = Max;
datafit.X = Xf;
datafit.epsilonDot = epsilonf;
datafit.z1 = z1;
datafit.z2 = z2(1:length(z1));


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit6_Callback(hObject, eventdata, handles) %Ndrop
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double
global Ndrop
global data
global NVideo
if (isempty(NVideo) == 1)
    NVideo = 1;
end
Ndrop = str2num(hObject.String);
set(handles.text11,'String',int2str(length(data(NVideo).X)));


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit7_Callback(hObject, eventdata, handles) %NVideo
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double
global NVideo
global data
NVideo = str2num(hObject.String);
set(handles.text11,'String',int2str(length(data(NVideo).X)));

% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function text10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function text11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function edit8_Callback(hObject, eventdata, handles) %p1
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double
global p1
p1 = str2num(hObject.String);



% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles) %p2
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double
global p2
p2 = str2num(hObject.String);

% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles) %p3
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double
global p3
p3 = str2num(hObject.String);

% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double
global p4
p4 = str2num(hObject.String);

% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double
global p5
p5 = str2num(hObject.String);

% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton9. Export to excel
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global datafit 
global data
global address
global Ndrop NVideo
global idx
global FileName
if (isempty(Ndrop) == 1)
    Ndrop = 1;
end
if (isempty(NVideo) == 1)
    NVideo = 1;
end
%%% Messagebox
if (isempty(datafit) == 0)
    a = datafit.area;
    min = datafit.minax;
    maj = datafit.minax;
    X = datafit.X;
    Z1 = datafit.z1;
    Z2 = datafit.z2;
    a_raw = data.area{Ndrop};
    min_raw = data.minax{Ndrop};
    maj_raw = data.minax{Ndrop};
    X_raw = data.X{Ndrop};
    time = data.XX{Ndrop};
    h = msgbox('Processing');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],{'a'},'Sheet1','A1:A1');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],a','Sheet1','A2');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],{'min'},'Sheet1','B1:B1');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],min','Sheet1','B2');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],{'maj'},'Sheet1','C1:C1');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],maj','Sheet1','C2');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],{'X'},'Sheet1','D1:D1');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],X','Sheet1','D2');    
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],{'z1'},'Sheet1','E1:E1');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],Z1','Sheet1','E2');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],{'z2'},'Sheet1','F1:F1');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],Z2','Sheet1','F2');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],{'min_raw'},'Sheet1','G1:G1');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],min_raw','Sheet1','G2');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],{'maj_raw'},'Sheet1','H1:H1');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],maj_raw','Sheet1','H2');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],{'X_raw'},'Sheet1','I1:I1');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],X_raw','Sheet1','I2');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],{'time'},'Sheet1','J1:J1');
    xlswrite([address,['Data(',FileName{idx},'Tracked).xls']],time','Sheet1','J2');
    close(h);
else msgbox('Please process your data before exporting');
    savefig(DropletGUI,[address,['ScreenShot(',FileName{idx},'Tracked).fig']]);
end


% --- Executes on button press in pushbutton10. %Play Video button
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global address
global NVideo
global FileName
if (isempty(NVideo) == 1)
    NVideo = 1;
end
implay([address,FileName{NVideo},'Tracked.avi']);                           %% Play the video


% --- Executes on button press in pushbutton11. Display binary image
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global address FileName pop threshold
global idx i 
global angle
global im
global Crop cropUp cropDown cropLeft cropRight se se1 Size smooth
if isempty(threshold)
    threshold = 0.4;
end
    vid=VideoReader([address,FileName{idx}]); %Video object
    img = read(vid,i);
    if (isempty(angle) == 0)
        img = imrotate(img,angle);
    end
    axes(handles.axes9);
    if (Crop == 1)
        img = img(cropUp:cropDown,cropLeft:cropRight);
    end
    im = img;
    clear img
    im = mat2gray(im);
    if (isempty(smooth) == 1)
        smooth = sqrt(2);
    end
    
    if (isempty(se) == 1)
        se = strel('disk',1);
    end
    if (isempty(se1) == 1)
        se1 = strel('disk',5);
    end
    if (pop == 1)
        im = wiener2(im,[5,5]);                                                 %% Applies wiener filter before edge detection
        ed = edge(im,'canny',[],smooth);                                        %% edge detection with canny.
        ed = imdilate(ed,se);
        bw = imfill(ed,'holes');
        bw = imerode(bw,se);
    end
    if (pop == 2)
        bw = im2bw(im,threshold);                                               %%Binary thresholding for black and white image
        bw = imfill(imcomplement(bw),'holes');
    end
    bw = imclearborder(bw);
    if (isempty(Size) == 1)
        Size = 2000;
    end
    bw = bwareaopen(bw,Size);
    bw = imopen(bw,se1);
    bw = imclearborder(bw);
    imshow(bw);



function edit15_Callback(hObject, eventdata, handles) %Dilation
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit15 as text
%        str2double(get(hObject,'String')) returns contents of edit15 as a double
global se
se = strel('disk',str2num(hObject.String));


% --- Executes during object creation, after setting all properties.
function edit15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit16_Callback(hObject, eventdata, handles) %Opening
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit16 as text
%        str2double(get(hObject,'String')) returns contents of edit16 as a double
global se1
se1 = strel('disk',str2num(hObject.String));

% --- Executes during object creation, after setting all properties.
function edit16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit17_Callback(hObject, eventdata, handles) %Size
% hObject    handle to edit17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit17 as text
%        str2double(get(hObject,'String')) returns contents of edit17 as a double
global Size
Size = str2num(hObject.String);

% --- Executes during object creation, after setting all properties.
function edit17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function text16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on slider movement. Cropping Data
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

global xlimit
global lengthx
if (hObject.Value > lengthx)
    hObject.Value = lengthx - 1;
end
if (isempty(lengthx) == 0)
    hObject.Max = lengthx - 2;
end
hObject.SliderStep = [1/(hObject.Max - 1) 2/(hObject.Max - 1)];
xlimit = round(hObject.Value);


% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit18_Callback(hObject, eventdata, handles)
% hObject    handle to edit18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit18 as text
%        str2double(get(hObject,'String')) returns contents of edit18 as a double
global smooth
smooth = str2num(hObject.String);

% --- Executes during object creation, after setting all properties.
function edit18_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2
global pop
pop = hObject.Value;

% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit19_Callback(hObject, eventdata, handles)
% hObject    handle to edit19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit19 as text
%        str2double(get(hObject,'String')) returns contents of edit19 as a double
global threshold
threshold = str2num(hObject.String);

% --- Executes during object creation, after setting all properties.
function edit19_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
