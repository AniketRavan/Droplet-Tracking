[FileName,address] = uigetfile('MultiSelect','on');
if ischar(FileName)
    FileName = {FileName};
end
for idx = 1:length(FileName)
    clear v
    v = VideoWriter([address,FileName{idx},'Tracked.avi']);
    v.FrameRate = 5;
    open(v);
    clear Im
    vid=VideoReader([address,FileName{idx}]); %video object
    numFrames = vid.NumberOfFrames;
    n=numFrames;
    crop = 180; 
for i = 1:5:n
    Im(:,:,(i - 1)/5 + 1) = read(vid,i);
end
scale = 1/0.5067; % micrometer per pixel
fps = 5000;
flag = 0; 

for i = 1:size(Im,3)
%     im = imread([address,FileName{i}]);
%     im = rgb2gray(im);
    im = Im(crop:size(Im,1),:,i);
    im = mat2gray(im);
    ed = edge(im);
    bw = imfill(ed,'holes');
    bw = bwareaopen(bw,800);
    bw = imclearborder(bw);
    %a = regionprops(bw,'Area');
    edg = bwperim(bw);
    %overlap = double(edg) + im;
    stats = regionprops(bw,'Centroid','MajorAxisLength','MinorAxisLength','Area');
    if (size(stats,1) ~= 0)
        CNew = stats.Centroid;
        if (flag == 1)
            disp(:,i) = CNew - COld;
            dist(i) = sqrt(disp(1,i)^2 + disp(2,i)^2)*scale;
            speed(i) = dist(i)/fps;
        end
        COld = CNew;
        majax(i) = stats.MajorAxisLength*scale;
        minax(i) = stats.MinorAxisLength*scale;
        delta(i) = (majax(i) - minax(i))/(majax(i) + minax(i));
        area(i) = stats.Area*scale^2;
        flag = 1;
        im = im + double(edg);
        im = mat2gray(im);
        im = insertMarker(im,[CNew(1,1),CNew(1,2)]);
    else flag = 0;
    end
    writeVideo(v,im);
    %%%%%%%%%%%
    
    %%%%%%%%%%%
    
end
xlswrite([address,'data'],{'Major Axis'},FileName{idx},'A1:A1');
xlswrite([address,'data'],majax',FileName{idx},'A2');
xlswrite([address,'data'],{'Minor Axis'},FileName{idx},'B1:B1');
xlswrite([address,'data'],minax',FileName{idx},'B2');
xlswrite([address,'data'],{'Delta'},FileName{idx},'C1:C1');
xlswrite([address,'data'],delta',FileName{idx},'C2');
xlswrite([address,'data'],{'Speed'},FileName{idx},'D1:D1');
xlswrite([address,'data'],speed',FileName{idx},'D2');
xlswrite([address,'data'],{'Area'},FileName{idx},'E1:E1');
xlswrite([address,'data'],area',FileName{idx},'E2');

%%%%%%%%%%%%

%%%%%%%%%%%%
close(v);
idx
end