clc;
clear;
close all;
idFeature = 1;
timeFeature = 4; % in milliseconds from some reference time
trajFeatures = 5:6;  % x and y
vehicleFeatures = 14:20; % various useful things, like origin
Start_ind=1;
timeToCover = 7;  % seconds
timesteps = floor(timeToCover * 1000 / 100);
num_rej=0;
region=3;
band=4;
section=1;
LocationTime= zeros(0,timesteps,length(trajFeatures)); %vehicle X time X pos
vehicleMatrix = zeros(0,length(vehicleFeatures));  %vehicle X info
nTooShortTrajects = 0;
flag=0;
flag1=0;
v_num=1;
temp = importdata('trajectories-0830am-0845am.txt');
i=0;
j=0;
while (Start_ind<length(temp(:,1)))
    Vehicle_info=find(temp(:,idFeature)==temp(Start_ind,idFeature));
    Temp1=temp(Vehicle_info,:);
    Start_ind= Start_ind+length(Vehicle_info);   %next time starting point
%     ind1=find(Temp1(:,17)==region);
%     ind2=mode(Temp1(:,19)==band);
    ind=(Temp1(:,17)==region).*(Temp1(:,19)==band);
    if(max(ind)==1)
        j=j+1;
    end
    ts=find(ind==1);
    if (~isempty(ts)&((ts(end)-timesteps )>0))
        Temp1=Temp1((ts(end)-timesteps ):ts(end),:);
    else
        Temp1=[];
    end
%     if (~isempty(ind1))
%         Temp1=Temp1;
%         if(mode(Temp1(1:end,19)==band))
%             Temp1=Temp1;
%         else
%             Temp1=[];
%            
%         end
%    end
    if (length(Temp1)>=timesteps)
            Temp1=Temp1(1:timesteps,:);
    else
        num_rej=num_rej+1;
        flag1=1;
    end
    if(flag1==0)
        LocationTime(v_num,:,:)=Temp1(:,trajFeatures);
        Temp2=mode(Temp1(:,vehicleFeatures));
        vehicleMatrix(v_num,:)=Temp2;
        v_num=v_num+1;
    end
    flag1=0;
    i=i+1;
end
Temp2=load('data_s3_SB.mat');
X=Temp2.timeMatrix(:,:,1);
Y=Temp2.timeMatrix(:,:,2);
Z=Temp2.vehicleMatrix;
Temp3=load('segment_s3sb_3.mat');
if(section==1)
    X=Temp3.timeMatrix(:,1:70,1);
    Y=Temp3.timeMatrix(:,1:70,2);
elseif (section==2)
    X=Temp3.timeMatrix(:,71:140,1);
    Y=Temp3.timeMatrix(:,71:140,2);
end
Temp4=load('s3sb_3_rotated.mat');
if(section==1)
    X=Temp4.timeMatrix(:,1:70,1);
    Y=Temp4.timeMatrix(:,1:70,2);
elseif (section==2)
    X=Temp4.timeMatrix(:,71:140,1);
    Y=Temp4.timeMatrix(:,71:140,2);
end
Temp5=load('segment_s3sb_3_rotated.mat');
if(section==1)
    X=Temp5.timeMatrix(:,1:70,1);
    Y=Temp5.timeMatrix(:,1:70,2);
elseif (section==2)
    X=Temp5.timeMatrix(:,71:140,1);
    Y=Temp5.timeMatrix(:,71:140,2);
end
    
kk=1;

while(kk<=size(X,1)) 
    if (Y(kk,1)<155 || Y(kk,end)>120)
       X(kk,:)=[];
       Y(kk,:)=[];
       %code(kk)=[];
    %   Z(kk,:)=[]; 
    else 
        kk=kk+1;
    end
end


% while(kk<=size(X,1)) 
%     if (Y(kk,1)<160 || Y(kk,end)>80)
%        X(kk,:)=[];
%        Y(kk,:)=[];
%        Z(kk,:)=[]; 
%     else 
%         kk=kk+1;
%     end
% end
% while(kk<=size(X,1))
%     if (Y(kk,end)<95)
%        X(kk,:)=[];
%        Y(kk,:)=[];
%        Z(kk,:)=[]; 
%     else 
%         kk=kk+1;
%     end
% end

X1=X;
Y1=Y;
for i=1:length(X)
    plot(X(i,:),Y(i,:));
    hold on
end
x=LocationTime(:,:,1);
y=LocationTime(:,:,2);
figure();
for i=1:length(x)
    plot(x(i,:),y(i,:))
    hold on
end
x=X;
y=Y;

% expectation maximization algorihthem 
sigma=2; %ft to be checked on the figure
sigma_x=std2(x); %ft to be checked on the figure
sigma_y=std2(y);
M=18; %number of clusters
K= timesteps; % number of points per trajectories wich defines the number of distribution multiplicant in 
%initilize the mean at each time step 
% this must get much smarter than this
[X1,Y1 ,Mux ,Muy ] = Cluster_init(X1,Y1,timesteps,M);


%figure();
% for kk=1:12
%     Mx=conv(Mux(:,kk),[1/4 1/4 1/4 1/4]);
%     My=conv(Muy(:,kk),[1/4 1/4 1/4 1/4]);
%     Mx(1:3)=Mux(1:3,kk);
%     My(1:3)=Muy(1:3,kk);
%     Mux(:,kk)=Mx(1:70);
%     Muy(:,kk)=My(1:70);
%     plot(Mux(1:end,kk),Muy(1:end,kk))
%     hold on
% end
%TO BE CHECKED


for kk=1:length(x)
     for ii=1:M
        for jj=1:70
            list_of_xi{kk,jj,ii}=[];
            %prob{jj,ii}=[];
        end
     end
end
for j=1:100
    for i=1:length(x)
        x_temp=x(i,:); %trajectory i x pos
        x_mean(j,:,:)=Mux; %mean for any iteration over time and clusters
        x_temp=x_temp';
        x_mean_temp(:,:)= x_mean(j,:,:);
        tcx=bsxfun(@minus,x_temp,x_mean_temp);% culomnwise subtraction to get the trajectory/cluster exponent
        y_temp=y(i,:); %trajectory i x pos
        y_mean(j,:,:)=Muy; %mean for any iteration over time and clusters
        y_temp=y_temp';
        y_mean_temp(:,:)= y_mean(j,:,:);
        tcy=bsxfun(@minus,y_temp,y_mean_temp);% culomnwise subtraction to get the trajectory/cluster exponent
        [tc, list_of_xi]=distance_to_line(x_temp, y_temp,x_mean_temp, y_mean_temp,M,list_of_xi,i);%.^2/(2*sigma^2);
        %[tc list_of_xi]=distance_to_point(x_temp, y_temp,x_mean_temp, y_mean_temp,M,list_of_xi,sigma,i);%.^2/(2*sigma^2);
      %  tc=tcx.^2+tcy.^2;
        Pmk(j,i,:,:)=exp(-1/(2*sigma^2)*(tc.^2)); % probability of each data point (k) on trajectory i belog to cluster m where m is column #
        
      %  Pmk(j,i,:,:)=exp(-1/(2*sigma_x^2)*(dvx.^2)).*exp(-1/(2*sigma_y^2)*(dvy.^2)); % probability of each data point (k) on trajectory i belog to cluster m where m is column #
       
        temp_p(:,:)=Pmk(j,i,:,:);
        Pm=prod(temp_p);
        eta(j,i)=1/sum(Pm);
        ECim(j,i,:)=eta(j,i)*Pm;        
    end
    Prob_c(:,:)=ECim(j,:,:);
    save('loc_segment_s3sb_rotated_sec_1.mat','x_mean_temp','y_mean_temp','Prob_c','sigma','M','timesteps');
    %maximization step
%     for i=1:length(x)
%         for m=1:M
%             for k=1:size(x,2)
%                 if (list_of_xi)
%                 end
%             end
%         end
%     end
figure()
for lll=1:M
    
    plot(x_mean_temp(:,lll),y_mean_temp(:,lll))
    hold on
end
    for m=1:M
        for k=1:size(x,2)
            temppn_x=0;
            temppd_x=0;
            temppn_y=0;
            temppd_y=0;
            for i=1:length(x)
                
                if(~isempty(list_of_xi{i,k,m}))
                    tempp_x=ECim(j,i,m)*list_of_xi{i,k,m}(:,1);
                    temppn_x=temppn_x+sum(tempp_x);
                    temppd_x=temppd_x+length(tempp_x)*ECim(j,i,m);
                    tempp_y=ECim(j,i,m)*list_of_xi{i,k,m}(:,2);
                    temppn_y=temppn_y+sum(tempp_y);
                    temppd_y=temppd_y+length(tempp_y)*ECim(j,i,m);
                    
                end
            end
            Mux(k,m)=temppn_x/temppd_x;
            Muy(k,m)=temppn_y/temppd_y;
        end
    end
    %Mux(k,m)=;
    %Muy=sort(Muy);
end
for kk=1:M
Mx=conv(Mux(:,kk),[1/4 1/4 1/4 1/4]);
My=conv(Muy(:,kk),[1/4 1/4 1/4 1/4]);
Mx(1:3)=Mux(1:3,kk);
My(1:3)=Muy(1:3,kk);
Mx=Mx(1:70);
My=My(1:70);
plot(Mx(4:end),My(4:end))
hold on
end


