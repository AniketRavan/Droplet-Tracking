clc
clear all
close all

[FileName,address] = uigetfile('MultiSelect','on');
if ischar(FileName)
    FileName = {FileName};
end

delta_matrix=cell(1,length(FileName)); r_star_matrix=cell(1,length(FileName));

for idx = 1:length(FileName)
    clear v
    v = VideoWriter([address,FileName{idx},'Tracked.avi']);
    v.FrameRate = 5;
    open(v);
    clear Im
    vid=VideoReader([address,FileName{idx}]); %video object
    numFrames = vid.NumberOfFrames;
    n=numFrames;
    cropUp = 180;            % Crop upperlimit
    cropDown = [];           % Crop lowerlimit
for i = 1:5:n
    Im(:,:,(i - 1)/5 + 1) = read(vid,i);
end
scale = 1/1.066; % micrometer per pixel
fps = 5000;      % Frame per Second
flag = 0; 
w1 = 100;       % [um] channel width


for i = 1:size(Im,3)
%     im = imread([address,FileName{i}]);
%     im = rgb2gray(im);
if isempty(cropDown) 
   cropDown = size(Im,1);
end
    im = Im(cropUp:cropDown,:,i);
    im = mat2gray(im);
    im = adapthisteq(im); %Contrast enhancement
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
        r_star(i) = 2*(sqrt(area(i)/pi))/w1;
        flag = 1;
        im = im + double(edg);
        im = mat2gray(im);
        im = insertMarker(im,[CNew(1,1),CNew(1,2)]);
    else flag = 0;
    end
    writeVideo(v,im);
%%%%%%%%%%%%%%%%%%%%%%   modify  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%    end    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
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
xlswrite([address,'data'],{'R_star'},FileName{idx},'F1:F1');
xlswrite([address,'data'],r_star',FileName{idx},'F2');

%%%%%%%%%%%%%%%%%%%%%%   modify  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta_matrix{idx}=delta';    r_star_matrix{idx}=r_star';
%%%%%%%%%%%%%%%%%%%%%%    end    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close(v);
idx
end

%%%%%%%%%%%%%%%%%%%%%%   modify  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
for j =1:length(FileName)
    dm(j)=max(delta_matrix{j});
    rsm(j)=max(r_star_matrix{j});
end
plot(rsm,dm,'o');
xlabel('R*'); ylabel('delta max');
hgsave (figure(1), 'figure1');
%%%%%%%%%%%%%%%%%%%%%%    end    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
