global fps 
global scale
global Tracked
[FileName,address] = uigetfile('MultiSelect','on');
if ischar(FileName)
    FileName = {FileName};
end
scale = 1/0.49 * 1e-6; % meter per pixel
fps = 10000; %frames per second
viscD = 0.008;
viscC = 0.0215;
rotate = -90; %Counterclockwise
for idx = 1:length(FileName)
    
    drawnow;
    color = [{'red'},{'green'},{'blue'},{'black'},{'magenta'}];
    passed = 0;
    Area = {[]};
    delta = {[]};
    majax = {[]};
    minax = {[]};
    speed = {[]};
    X = {[]};
    clear v
    v = VideoWriter([address,FileName{idx},'Tracked.avi']);
    v.FrameRate = 5;
    open(v);
    clear Im
    vid=VideoReader([address,FileName{idx}]); %Video object
    numFrames = vid.NumberOfFrames;
    n=numFrames;
    cropUp = 450;
    cropDown = [];
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
    im = imrotate(im,rotate);
    im = im(cropUp:cropDown,:);
%     im = adapthisteq(im);
%     im = adapthisteq(im);
%     im = wiener2(im,[5,5]);
    ed = edge(im);
    se = strel('disk',1);
    se1 = strel('disk',5);
    ed = imdilate(ed,se);
    bw = imfill(ed,'holes');
    bw = imerode(bw,se);
    bw = imclearborder(bw);
    bw = bwareaopen(bw,1500);
    bw = imopen(bw,se1);
    bw = imclearborder(bw);
    %a = regionprops(bw,'Area');
    edg = bwperim(bw);
    %imwrite(bw,[address,int2str(i),'.tif']);
    stats = regionprops(bw,'Centroid','MajorAxisLength','MinorAxisLength','Area');
    im = im + double(edg);
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
            if (i ~= 1)
                if (min(ref) > top)
                    passed = passed + 1;
                end
            end
            top = min(ref);
            for k = 1:length(refSort)
                ascInd(k) = find(ref == refSort(k));
            end
        CNew = [];
        for p = 1:size(stats,1)
            %im = insertMarker(im,[stats(ascInd(p)).Centroid(1),stats(ascInd(p)).Centroid(2)],'Color',color(mod((p+passed),5)+1),'size',10);
            CNew(p,:) = stats(ascInd(p)).Centroid;
            A(p) = stats(ascInd(p)).Area*scale^2;
            Majax(p) = stats(ascInd(p)).MajorAxisLength*scale;
            Minax(p) = stats(ascInd(p)).MinorAxisLength*scale;
            Delta(p) = (Majax(p) - Minax(p))/(Majax(p) + Minax(p));
            if (flag == 1)
%             if (p <= size(COld,1))
%                 disp(p,:) = CNew(p,:) - COld(p,:);
%                 dist = sqrt(disp(p,1)^2 + disp(p,2)^2)*scale;
%                 Speed(p) = dist/fps;
%             else disp(p,1:2) = NaN;
%                 Speed(p) = NaN;
%             end
            
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
                X{p + passed} = [X{p + passed},size(im,1) - CNew(p,2)];
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
% xlswrite([address,'data'],{'Major Axis'},FileName{idx},'A1:A1');
% xlswrite([address,'data'],majax',FileName{idx},'A2');
% xlswrite([address,'data'],{'Minor Axis'},FileName{idx},'B1:B1');
% xlswrite([address,'data'],minax',FileName{idx},'B2');
% xlswrite([address,'data'],{'Delta'},FileName{idx},'C1:C1');
% xlswrite([address,'data'],delta',FileName{idx},'C2');
% xlswrite([address,'data'],{'Speed'},FileName{idx},'D1:D1');
% xlswrite([address,'data'],speed',FileName{idx},'D2');
% xlswrite([address,'data'],{'Area'},FileName{idx},'E1:E1');
% xlswrite([address,'data'],area',FileName{idx},'E2');

%%%%%%%%%%%% Fitting
% for j = 1:length(X)
%     if (isempty(X{j}) == 0)
%     X{j} = sgolayfilt(X{j},5,floor(length(X{j})/5)*2 + 1);
%     p = polyfit(1:length(majax{j}),majax{j},9);
%     majax{j} = polyval(p,1:length(majax{j}));
%     p = polyfit(1:length(minax{j}),minax{j},9);
%     minax{j} = polyval(p,1:length(minax{j}));
%     delta{j} = (majax{j} - minax{j})./(majax{j} + minax{j});
%     end
% end
%%%%%%%%%%%%
close(v);
visc = [0.008,0.0215];
T = [];
vArea = []; vMinax = []; vMajax = []; vSpeed = []; vDelta = [];
for t = 1:length(Area)
    vArea = [vArea,Area{t}];
    vMinax = [vMinax,minax{t}];
    vMajax = [vMajax,majax{t}];
%     vSpeed = [vSpeed,speed{t}];
    vDelta = [vDelta,delta{t}];
end


end