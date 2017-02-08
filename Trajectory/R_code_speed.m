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
section=1;
region=3;
band=4;
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
Temp3=load('segment_s3sb_3_rotated.mat');
if(section==1)
    X=Temp3.timeMatrix(:,1:70,1);
    Y=Temp3.timeMatrix(:,1:70,2);
elseif (section==2)
    X=Temp3.timeMatrix(:,71:140,1);
    Y=Temp3.timeMatrix(:,71:140,2);
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
% while(kk<=size(X,1)) this first data 'data_s3_SB.mat
%     if (Y(kk,1)<980 || Y(kk,end)>690)
%        X(kk,:)=[];
%        Y(kk,:)=[];
%        Z(kk,:)=[]; 
%     else 
%         kk=kk+1;
%     end
% end
% while(kk<=size(X,1))
%     if (Y(kk,1)<180 ||X(kk,1)<60)
%        X(kk,:)=[];
%        Y(kk,:)=[];
%        Z(kk,:)=[]; 
%     else 
%         kk=kk+1;
%     end
% end
while(kk<=size(X,1)) 
    if (Y(kk,1)<1 || Y(kk,end)>80)
       X(kk,:)=[];
       Y(kk,:)=[];
       code(kk)=[];
    %   Z(kk,:)=[]; 
    else 
        kk=kk+1;
    end
end
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
x_ro= bsxfun(@minus,x,x(:,1)); 
y_ro= bsxfun(@minus,y,y(:,1)); 
xt2=x_ro(:,2:end);
xt1=x_ro(:,1:end-1);
vx=(xt2-xt1)/0.1;
yt2=y_ro(:,2:end);
yt1=y_ro(:,1:end-1);
vy=(yt2-yt1)/0.1;


% expectation maximization algorihthem 
sigma_x=std2(vx); %ft to be checked on the figure
sigma_y=std2(vy);
M=24; %number of clusters
K= timesteps-1; % number of points per trajectories wich defines the number of distribution multiplicant in 
%initilize the mean at each time step 
% this must get much smarter than this
for ll=1:K
    mkvx=mean(vx(:,ll));
    mkvy=mean(vy(:,ll));
    sig_vx=std(vx(:,ll));
    sig_vy=std(vy(:,ll));
    [indx1 nx1]=find(vx(:,ll)>mkvx+6*sig_vx);
    if (~isempty(indx1))
         vx(indx1,ll)=(vx(indx1-1,ll)+vx(indx1+1,ll))/2;
    end
    [indx2 nx2]=find(vx(:,ll)<mkvx-6*sig_vx);
    if (~isempty(indx2))
         vx(indx2,ll)=(vx(indx2-1,ll)+vx(indx2+1,ll))/2;
    end
    [indy1 ny1]=find(vy(:,ll)>mkvy+6*sig_vy);
    if (~isempty(indy1))
        vy(indy1,ll)=(vy(indy1-1,ll)+vy(indy1+1,ll))/2;
    end
    [indy2 ny2]=find(vy(:,ll)<mkvy-6*sig_vy);
    if (~isempty(indy2))
        vy(indy2,ll)=(vy(indy2-1,ll)+vy(indy2+1,ll))/2;
    end
    
end

for  k=1:K
    [r1 val1]=hist(vx(:,k),48);
    [r2 val2]=hist(vy(:,k),48);
    [s1 ind1]=sort(r1,'descend');
    [s2 ind2]=sort(r2,'descend');
    Muvx(k,:)=sort(val1(ind1(1:M)));
    Muvy(k,:)=sort(val2(ind2(1:M)));%*ones(1,5);    
end
%TO BE CHECKED

% %EM E-step E[Cim | TEHTA[j],]
% for j=1:10000 %number of EM loops
%     for i=1:length(x) % number of trajectories
%         for m=1:M % number of clusters
%             for k=1:timpsteps % number of timpe step in each trajectory 
%                 
%             end
%         end        
%     end
% end
% 
% Ec(i,m)=

for kk=1:length(vx)
     for ii=1:M
        for jj=1:69
            list_of_vi{kk,jj,ii}=[];
        end
     end
end
for j=1:100
    for i=1:length(vx)
        vx_temp=vx(i,:); %trajectory i x pos
        vx_mean(j,:,:)=Muvx; %mean for any iteration over time and clusters
        vx_temp=vx_temp';
        vx_mean_temp(:,:)= vx_mean(j,:,:);
        tcvx=bsxfun(@minus,vx_temp,vx_mean_temp);% culomnwise subtraction to get the trajectory/cluster exponent
        vy_temp=vy(i,:); %trajectory i x pos
        vy_mean(j,:,:)=Muvy; %mean for any iteration over time and clusters
        vy_temp=vy_temp';
        vy_mean_temp(:,:)= vy_mean(j,:,:);
        tcvy=bsxfun(@minus,vy_temp,vy_mean_temp);% culomnwise subtraction to get the trajectory/cluster exponent
        [dvx dvy list_of_vi]=distance_to_point(vx_temp, vy_temp,vx_mean_temp, vy_mean_temp,M,list_of_vi,sigma_x,sigma_y,i);%.^2/(2*sigma^2);
        %[tc list_of_xi]=distance_to_point(x_temp, y_temp,x_mean_temp, y_mean_temp,M,list_of_xi,sigma,i);%.^2/(2*sigma^2);
      %  tc=tcx.^2+tcy.^2;
        Pmk(j,i,:,:)=exp(-1/(2*sigma_x^2)*(dvx.^2)).*exp(-1/(2*sigma_y^2)*(dvy.^2)); % probability of each data point (k) on trajectory i belog to cluster m where m is column #
        temp_p(:,:)=Pmk(j,i,:,:);
        Pm=prod(temp_p);
        eta(j,i)=1/sum(Pm);
        ECim(j,i,:)=eta(j,i)*Pm;   
        
    end
    
figure()

for lll=1:M
    x_mean_temp(1,lll)=0;
    y_mean_temp(1,lll)=0;
    for jjj=1:length(vx_mean_temp(:,lll))
        x_mean_temp(jjj+1,lll)=x_mean_temp(jjj,lll)+0.1*vx_mean_temp(jjj,lll);
        y_mean_temp(jjj+1,lll)=y_mean_temp(jjj,lll)+0.1*vy_mean_temp(jjj,lll);
    end
    plot(x_mean_temp(:,lll),y_mean_temp(:,lll))
    hold on
end
Prob_c(:,:)=ECim(j,:,:);
save('speed_segment_s3sb_rotated_sec_1.mat','x_mean_temp','y_mean_temp','Prob_c','vx_mean_temp','vy_mean_temp','sigma_x','sigma_y','M','timesteps');
    for m=1:M
        for k=1:size(vx,2)
            temppn_vx=0;
            temppd_vx=0;
            temppn_vy=0;
            temppd_vy=0;
            for i=1:length(vx)  
                if(~isempty(list_of_vi{i,k,m}))
                    tempp_vx=ECim(j,i,m)*list_of_vi{i,k,m}(:,1);
                    temppn_vx=temppn_vx+sum(tempp_vx);
                    temppd_vx=temppd_vx+length(tempp_vx)*ECim(j,i,m);
                    tempp_vy=ECim(j,i,m)*list_of_vi{i,k,m}(:,2);
                    temppn_vy=temppn_vy+sum(tempp_vy);
                    temppd_vy=temppd_vy+length(tempp_vy)*ECim(j,i,m);          
                end
            end
            Muvx(k,m)=temppn_vx/temppd_vx;
            Muvy(k,m)=temppn_vy/temppd_vy;
        end
    end
    %Mux(k,m)=;
    %Muy=sort(Muy);
end
