function [x,y ,Mux ,Muy ]= Cluster_init(x,y,timesteps,M)

sigma=2; %ft to be checked on the figure
sigma_x=std2(x); %ft to be checked on the figure
sigma_y=std2(y);
K= timesteps; % number of points per trajectories wich defines the number of distribution multiplicant in 
%initilize the mean at each time step 
% this must get much smarter than this
counter=0;
for ll=1:K
    mkx=mean(x(:,ll));
    mky=mean(y(:,ll));
    sig_x=std(x(:,ll));
    sig_y=std(y(:,ll));
    [indx1 nx1]=find(x(2:end-1,ll)>mkx+3*sig_x);
    [indx2 nx2]=find(x(2:end-1,ll)<mkx-3*sig_x);
    
    [indy1 ny1]=find(y(2:end-1,ll)>mky+3*sig_y);
    [indy2 ny2]=find(y(2:end-1,ll)<mky-3*sig_y);
    
    %if ((indy1>1)&&(indy1<(size(x,1)))&&(indy2>1)&&(indy2<(size(x,1)))&&(indx1>1)&&(indx1<(size(x,1)))&&(indx2>1)&&(indx2<(size(x,1))))
        if (~isempty(indx1))
          %  if ((indx1>1)&&(indx1<(size(x,1))))
                 x(indx1,ll)=(x(indx1-1,ll)+x(indx1+1,ll))/2;
                 counter=counter+1;
          %  end
        end
        
        if (~isempty(indx2))
            %if ((indx2>1)&&(indx2<(size(x,1))))
                 x(indx2,ll)=(x(indx2-1,ll)+x(indx2+1,ll))/2;
                         counter=counter+1;
            %end
        end
        
        if (~isempty(indy1))
           % if ((indy1>1)&&(indy1<(size(x,1))))
                y(indy1,ll)=(y(indy1-1,ll)+y(indy1+1,ll))/2;
                        counter=counter+1;
            %end
            
        end
        
        if (~isempty(indy2))
          %  if ((indy2>1)&&(indy<(size(x,1))))
                y(indy2,ll)=(y(indy2-1,ll)+y(indy2+1,ll))/2;
                        counter=counter+1;
          %  end
        end
    %end
     
    
end
x(1,:)=[];
y(1,:)=[];
x(end,:)=[];
y(end,:)=[];
l=1;
for  k=1:10:timesteps+1
%for k=1:70
    if (k>1)
        k=k-1;
    end
    [r1 val1]=hist(x(:,k),20);
    [r2 val2]=hist(y(:,k),20);
    [s1 ind1]=sort(r1,'descend');
    [s2 ind2]=sort(r2,'descend');
    tempx(l,:)=sort(val1(ind1(1:M)));
    Mux(k,:)=tempx(l,:);
    tempy(l,:)=sort(val2(ind2(1:M)));
    Muy(k,:)=tempy(l,:);%*ones(1,5);
    l=l+1;
   
end
l=1;
for k=1:10:timesteps-1
    if (k>1)
        k=k-1;
        r=10;
        q=11;
    else
        r=9;
        q=10;
    end
    for j=1:M
        Mux(k:k+r,j) = linspace(Mux(k,j),Mux(k+r,j),q);
        Muy(k:k+r,j) = interp1(tempx(l:l+1,j),tempy(l:l+1,j),Mux(k:k+r,j),'spline');
    end  
    l=l+1;
end
figure();
for j=1:M
    plot(tempx(:,j),tempy(:,j),'o',Mux(:,j),Muy(:,j),':.');
    hold on
end