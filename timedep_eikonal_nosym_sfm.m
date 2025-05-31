clear all
path(path,'C:/Users/User/Documents/tan/elte_alkmat_msc/socialForce/toolbox_fast_marching');
path(path,'C:/Users/User/Documents/tan/elte_alkmat_msc/socialForce/toolbox_fast_marching/toolbox');

disc_wall=0;%0:wall is given by lines, 1: wall given as a mask of the discretized space
%no bresenams algorithm
eikonal=1;

load('room_for_soc_forc_mod_0_10.mat')%room, see make_discrete_room.m

if disc_wall==1
    %{
    A=ones(40);
    A([2,39],2:39)=0;
    A(2:39,2)=0;
    A([2:15,30:39],39)=0;
    A([15,30],20:39)=0;
    A(4,4)=0;
    room.wall_map=~A;
    %}
else %cont wall, should be same as the discrete for the eikonal
    %load('room_for_soc_forc_mod_0_10.mat')%room, see make_discrete_room.m
    room.walls=[0,10,10,10;...%x_startp,y_startp,x_endp,y_endp
    10,10,10,0;...
    0,0,0,10;...
    10,0,0,0];
    room.walls_init=room.walls(:,1:2);
    room.walls_end=room.walls(:,3:4);
    
    %u_s,n_s fix
    room.u_s=room.walls(:,3:4)-room.walls(:,1:2);
    room.lengths=vecnorm(room.u_s,2,2);

    room.n_s=zeros(size(room.u_s));
    room.n_s(:,1:2:end)=-room.u_s(:,2:2:end);
    room.n_s(:,2:2:end)=room.u_s(:,1:2:end);
end

num_of_ppl=6;
init_pos=randi([1,9],1,2*num_of_ppl);%[randi([2,38],1,2*num_of_ppl)];%[1,0,1,0,randi([2,15],1,2*(num_of_ppl-2))];%[-1.4,1,-1,1];%[1,1,-1,1];%[2,2,randi([-10,10],1,2*(num_of_ppl-1))];
init_vel=[0,-1,randi([0,10],1,2*(num_of_ppl-1))];%[1,0,1,0];%[-1,0.001,1,0];%[-1,-1,randi([0,10],1,2*(num_of_ppl-1))];
save_tf=1;

for ind1=1:2:2*num_of_ppl 
    ppl_tmp=init_pos(ind1+[0,1]);
    for ind2=ind1+2:2*(num_of_ppl-1)
        if sum(ppl_tmp==init_pos(ind2+[0,1]))==2
           init_pos(ind1+[0,1])=randi([1,9],1,2);
        end
    end
end


which_goal=[1,2,2,1,1,2];%,1,1,2];
num_of_goals=max(which_goal);
goals{1}=[1,7;...%2,10,38,38,27;...
       1,7];%1,38,38,3,39];
goals{2}=[9;...
          9];
clear options;
options.nb_iter_max = Inf;
for ind1=1:num_of_goals
    [Dist_fncs{ind1},~] = perform_fast_marching(double(~room.wall_map), (1/room.resolution)* goals{ind1}, options);
end
%}
%{
load('test_ppl_362206_socforcmodel.mat')
disc_wall=0;
num_of_ppl=onerun.num_of_ppl;

init_pos=onerun.init_pos;
init_vel=onerun.init_vel;
goals=onerun.ppl_goal;

save_tf=0;
%}

t_0=0;
t_max=20;
step_size=1/200;
t=t_0:step_size:t_max;
num_of_time_grid=length(t);

%in genral we have x''=f(x,x'),, we rewrite it to 4N dim ODE, without
%changing the names of the variables

%The exact ode depends on the geometry(through the wall forces) and on the
%number of people
%thus we need to make th RHS adaptable.

v_0=2;%wanted speed
tau=1/2;%relaxation time [s]
v_max=2*v_0;%maximal speed
effective_angle_of_sight=pi/2;

%parameters for the forces
A_i=20;
B_i=0.2;
lambda_i=0;
r_ij=0.5;
k=100;
kappa=100;

tic
if disc_wall==1
    [y,forces]=exp_euler_discrete_wall(num_of_time_grid,step_size,[init_pos,init_vel],goals,v_max,v_0,room,eikonal,Dist_fncs,which_goal);
    simple_plot(y,forces,goals,num_of_ppl,r_ij,room,disc_wall,eikonal,which_goal)
    
else
    [y,forces]=exp_euler_cont_wall(num_of_time_grid,step_size,[init_pos,init_vel],goals,v_max,v_0,room,eikonal,Dist_fncs,which_goal);
    simple_plot(y,forces,goals,num_of_ppl,r_ij,room,disc_wall,eikonal,which_goal)
end
toc



%movie_plot(y,t,num_of_ppl)

%create_plots(t,y,num_of_ppl)

if save_tf==1
    onerun.ppl_goal=goals;
    onerun.init_pos=init_pos;
    onerun.init_vel=init_vel;
    onerun.num_of_ppl=num_of_ppl;
    rand_num=randi(1000000);
    file_name='test_ppl_'+string(rand_num)+'_socforcmodel.mat';
    save(file_name,'onerun')
    
    disp(['initvals were saved as ', file_name])
end

function [y,forces_k]=exp_euler_discrete_wall(N,h,init,ppl_goal,v_max,v_0,room,eikonal,Dist_fncs,which_goal)
    num_of_ppl=length(init)/4;
    y=zeros(N,length(init));%approxes
    %save forces
    forces_k=zeros(N,length(init)/2,3);

    y(1,:)=init;
    
    for j=1:N-1 %timesteps
        B=find_closest_disc_wall_coords(y(j,1:2*num_of_ppl),room);%closest wall coordinates
        forces_k(j,:,:)=h*forceparts_calc(y(j,1:2*num_of_ppl),ppl_goal,y(j,2*num_of_ppl+1:end),B,eikonal,room,Dist_fncs,which_goal);
        %forces_k(j,:,:)=f1(j,:,:);
        y(j+1,:)=y(j,:)+[h*cutoff_fnc(y(j,2*num_of_ppl+1:end),v_max,v_0),sum(forces_k(j,:,:),3)];%id fnc in cutoff
    end

end

function [y,forces_k]=exp_euler_cont_wall(N,h,init,ppl_goal,v_max,v_0,room,eikonal,Dist_fncs,which_goal)
    num_of_ppl=length(init)/4;
    y=zeros(N,length(init));
    %save forces
    forces_k=zeros(N,length(init)/2,3);
    
    y(1,:)=init;
       
    for j=1:N-1
        B=find_closest_cont_wall_coords(y(j,1:2*num_of_ppl),room);%closest wall coordinates
        forces_k(j,:,:)=h*forceparts_calc(y(j,1:2*num_of_ppl),ppl_goal,y(j,2*num_of_ppl+1:end),B,eikonal,room,Dist_fncs,which_goal);
        %forces_k(j,:,:)=f1(j,:,:);
        y(j+1,:)=y(j,:)+[h*cutoff_fnc(y(j,2*num_of_ppl+1:end),v_max,v_0),sum(forces_k(j,:,:),3)];%id fnc in cutoff
    end

end

function forces_one_time=forceparts_calc(X,goals,V,B,eikonal,room,Dist_fncs,which_goal)
    v_0=2;%wanted speed
    tau=1/2;%relaxation time [s]
    v_max=2*v_0;%maximal speed
    effective_angle_of_sight=pi/2;
    
    %parameters for the forces
    A_i=20;
    B_i=0.2;
    lambda_i=0;
    r_ij=0.5;
    k=100;
    kappa=100;

    num_of_coords=size(X,2);
    num_of_ppl=num_of_coords/2;

    more_than_one=1;
    
    X_tmp=zeros(1,num_of_ppl);

    kartz_s=2;

    g=1;
    h=1;

    res=room.resolution;

    X_tmp=zeros(1,num_of_ppl);
    if more_than_one==0
        ppl_goal=reshape(cell2mat(goals(which_goal)),1,num_of_coords);
        X_tmp=sqrt((X(1:2:num_of_coords)-ppl_goal(1:2:num_of_coords)).^2+(X(2:2:num_of_coords)-ppl_goal(2:2:num_of_coords)).^2);
    else
        for ind1=1:num_of_ppl
            %goals{which_goal}
            [minval_X,min_ind_X]=min(sqrt(sum((X(2*ind1-1:2*ind1)'-goals{which_goal(ind1)}).^2)));
            X_tmp(ind1)=minval_X;
        end
    end

    forces_one_time=zeros(num_of_coords,3);

    %ppl_goal=repmat(goals',[1,num_of_ppl]);%in the case when they all have the same goal
    
    %{
    X_tmp=sqrt((X(1:2:num_of_coords)-ppl_goal(1:2:num_of_coords)).^2+(X(2:2:num_of_coords)-ppl_goal(2:2:num_of_coords)).^2);
    %}
    X_norm(1:2:num_of_coords)=X_tmp;
    X_norm(2:2:num_of_coords)=X_tmp;
    
    
    V_norm_double=zeros(1,num_of_coords);
    V_norm=sqrt(V(1:2:num_of_coords).^2+V(2:2:num_of_coords).^2);
    V_norm_double(1:2:num_of_coords)=V_norm;
    V_norm_double(2:2:num_of_coords)=V_norm;

    %room.X_coords(round((1/res)*X_r)+1)
    
    forces_at_goal=0;
    goal_dist_cutoff_val=0.5;

    indices_tmp=1:num_of_ppl;

    if forces_at_goal==1
        not_at_goal=true(size(X_tmp));
        not_at_goal_double=true(1,2*num_of_ppl);
    else
        not_at_goal=X_tmp>goal_dist_cutoff_val;
        not_at_goal_double=true(1,2*num_of_ppl);
        not_at_goal_double(1:2:2*num_of_ppl)=not_at_goal;
        not_at_goal_double(2:2:2*num_of_ppl)=not_at_goal;
    end
    
    f_dir=zeros(1,num_of_coords);
    
    if kartz_s==0 % equiv 1
        for ind1=1:max(which_goal)
            ppl_mask=zeros(1,num_of_coords);
            ppl_mask(1:2:num_of_coords)= not_at_goal &(which_goal==ind1);
            ppl_mask(2:2:end)=ppl_mask(1:2:num_of_coords);
            if sum(ppl_mask)~=0
                ip = compute_geodesic_modified(Dist_fncs{ind1}, (1/res)*reshape(X(logical(ppl_mask)),2,[]),res);
                %%%%%%%%%
                ip=ip./vecnorm(ip);
                %%%%%%%%%
                f_dir(logical(ppl_mask))=1/tau*(reshape(ip,1,[])*v_0-V(logical(ppl_mask)));%dir force
            end
        end
    elseif kartz_s==1 % where a person is, the eikonal eqn inherits the speed (doesnt look out for different goals

        X_r=res*round(1/res*X);
        max_val=10-res;
        min_val=0+res;
        X_r = max(min(X_r, max_val), min_val);
        ind_tmp=reshape(round((1/res)*X_r)+1,2,num_of_ppl);
        for_room_dist_fnc=double(~room.wall_map); %eikonal with one and walls lhs, we change where there are ppl:
        for_room_dist_fnc(sub2ind(size(for_room_dist_fnc),ind_tmp(1,:),ind_tmp(2,:)))=1./V_norm; % where a person is, the eikonal eqn inherits the speed
        %for_room_dist_fnc(sub2ind(size(for_room_dist_fnc),ind_tmp(1,:),ind_tmp(2,:)))=max(1,1./V_norm);%
        %where a person is, the eikonal eqn inherits the speed but only can
        %make one slower there

        clear options;
        options.nb_iter_max = Inf;
        [Dist_fncs_1,~] = perform_fast_marching(double(for_room_dist_fnc), (1/room.resolution)*cell2mat(goals), options);
        
        ip = compute_geodesic_modified(Dist_fncs_1, (1/res)*reshape(X(not_at_goal_double),2,[]),res);
        ip=ip./vecnorm(ip);
        %%%%%%%%%
        f_dir(not_at_goal_double)=1/tau*(reshape(ip,1,[])*v_0-V(not_at_goal_double));%dir force

    elseif kartz_s==2 % where a person is, the eikonal eqn inherits the speed (doesnt look out for different goals

        X_r=res*round(1/res*X);
        max_val=10-res;
        min_val=0+res;
        X_r = max(min(X_r, max_val), min_val);
        ind_tmp=reshape(round((1/res)*X_r)+1,2,num_of_ppl);
        for indt=1:max(which_goal)
            for_room_dist_fnc=double(~room.wall_map); %eikonal with one and walls lhs, we change where there are ppl:

            ppl_mask=not_at_goal &(which_goal==indt);
            ppl_mask_double=zeros(1,num_of_coords);
            ppl_mask_double(1:2:num_of_coords)= ppl_mask;
            ppl_mask_double(2:2:end)=ppl_mask;
            if sum(ppl_mask)~=0

                for_room_dist_fnc(sub2ind(size(for_room_dist_fnc),ind_tmp(1,ppl_mask),ind_tmp(2,ppl_mask)))=1./V_norm(ppl_mask); % where a person is, the eikonal eqn inherits the speed
                %for_room_dist_fnc(sub2ind(size(for_room_dist_fnc),ind_tmp(1,ppl_mask),ind_tmp(2,ppl_mask)))=max(1,1./V_norm(ppl_mask));%
                %where a person is, the eikonal eqn inherits the speed but only can
                %make one slower there
                clear options;
                options.nb_iter_max = Inf;
                [Dist_fncs_1,~] = perform_fast_marching(double(for_room_dist_fnc), (1/room.resolution)*goals{indt}, options);
                %for_room_dist_fnc(sub2ind(size(for_room_dist_fnc),ind_tmp(1,:),ind_tmp(2,:)))=1./V_norm;
            
                ip = compute_geodesic_modified(Dist_fncs_1, (1/res)*reshape(X(logical(ppl_mask_double)),2,[]),res);
                %%%%%%%%%
                ip=ip./vecnorm(ip);
                %%%%%%%%%
                f_dir(logical(ppl_mask_double))=1/tau*(reshape(ip,1,[])*v_0-V(logical(ppl_mask_double)));%dir force
            end
        end
    elseif kartz_s==3 % what is in the paper from kartz (looks out for different goals

        X_r=res*round(1/res*X);
        max_val=10-res;
        min_val=0+res;
        X_r = max(min(X_r, max_val), min_val);
        ind_tmp=reshape(round((1/res)*X_r)+1,2,num_of_ppl);
        for indt=1:max(which_goal)
            for_room_dist_fnc=double(~room.wall_map); %eikonal with one and walls lhs, we change where there are ppl:

            ppl_mask=not_at_goal &(which_goal==indt);
            ppl_mask_double=zeros(1,num_of_coords);
            ppl_mask_double(1:2:num_of_coords)= ppl_mask;
            ppl_mask_double(2:2:end)=ppl_mask;
            if sum(ppl_mask)~=0
                ip = compute_geodesic_modified(Dist_fncs{indt}, (1/res)*reshape(X(logical(ppl_mask_double)),2,[]),res);
                %%%%%%%%%
                ip=ip./vecnorm(ip);
        
                for_room_dist_fnc(sub2ind(size(for_room_dist_fnc),ind_tmp(1,ppl_mask),ind_tmp(2,ppl_mask)))=1+max(0,g*(1+(h/v_0)*(V(ppl_mask_double)*ip'))); % where a person is, the eikonal eqn inherits the speed
                %for_room_dist_fnc(sub2ind(size(for_room_dist_fnc),ind_tmp(1,ppl_mask),ind_tmp(2,ppl_mask)))=max(1,1./V_norm(ppl_mask));%
                %where a person is, the eikonal eqn inherits the speed but only can
                %make one slower there
                clear options;
                options.nb_iter_max = Inf;
                [Dist_fncs_new,~] = perform_fast_marching(double(for_room_dist_fnc), (1/room.resolution)*goals{indt}, options);
                %for_room_dist_fnc(sub2ind(size(for_room_dist_fnc),ind_tmp(1,:),ind_tmp(2,:)))=1./V_norm;
            
                ip = compute_geodesic_modified(Dist_fncs_new, (1/res)*reshape(X(logical(ppl_mask_double)),2,[]),res);
                %%%%%%%%%
                ip=ip./vecnorm(ip);
                %%%%%%%%%
                f_dir(logical(ppl_mask_double))=1/tau*(reshape(ip,1,[])*v_0-V(logical(ppl_mask_double)));%dir force
            end
        end
    end

    
    forces_one_time(:,1)=f_dir;%dir force
    
    %
    %wall force making:
    d_ib=zeros(1,num_of_coords);
    d_ib(1:2:num_of_coords)=sqrt((X(1:2:num_of_coords)-B(1:2:num_of_coords)).^2+(X(2:2:num_of_coords)-B(2:2:num_of_coords)).^2);
    d_ib(2:2:num_of_coords)=d_ib(1:2:num_of_coords);
    %normed vector from now checked person to its closest wall coords
    n_ib=(X-B)./d_ib;
    %angle dependency part (between the wall and the direction of the movement of the person)
    angles_all_tmp=-n_ib.*(V./V_norm_double);
    angles_ib_all=angles_all_tmp(1:2:num_of_coords)+angles_all_tmp(2:2:num_of_coords);%dotprods
    lambda_part_wall=zeros(1,num_of_coords);
    lambda_part_wall(1:2:num_of_coords)=lambda_i+(1-lambda_i)*1/2*(1+angles_ib_all);
    lambda_part_wall(2:2:num_of_coords)=lambda_part_wall(1:2:num_of_coords);
    %wall force
    %f_w=A_i*exp((r_ij/2-d_ib)/B_i).*n_ib.*lambda_part_wall;
    forces_one_time(:,3)=A_i*exp((r_ij/2-d_ib)/B_i).*n_ib.*lambda_part_wall;
    %}

    %f_g=zeros(1,2*num_of_ppl);
    %if we dont want to add the body forces to the soc model but work with
    %it separately:
    %f_k=zeros(1,2*num_of_ppl);%body force
    f_soc=zeros(1,num_of_coords);%soc frce
    
    for p1=indices_tmp(not_at_goal)%1:num_of_ppl
        x_p1=X(2*p1+[-1,0]);%coords of the used person
        %b_p1=B(2*p1+[-1,0]);%coords of the used persons closes wall
        %v_p1=V(2*p1+[-1,0]);
        v_p1_normed_vect=repmat(V(2*p1+[-1,0])/norm(V(2*p1+[-1,0])),1,num_of_ppl-1);
        %coords of all the other person, dim: 1 x 2*(num_of_ppl-1)
        X_others=X(reshape(2*cat(2,1:p1-1,p1+1:num_of_ppl)+[-1,0]',1,2*(num_of_ppl-1)));
        %vectors from all the other people pointing to the now checked person
        X_ij=repmat(x_p1,1,(num_of_ppl-1))-X_others;
        %d_ij=sqrt(X_ij(1:2:end).^2+X_ij(2:2:end).^2);
        %norms of the above, doubled
        d_ij_double=reshape(repmat(sqrt(X_ij(1:2:end).^2+X_ij(2:2:end).^2),2,1),1,2*(num_of_ppl-1));
        %normed vectors from all the other people pointing to the now checked person
        n_ij=X_ij./d_ij_double;
        %{
        %wall force making:
        d_ib=sqrt(sum((x_p1-b_p1).^2));
        %normed vector from now checked person to its closest wall coords
        %LATER PUT THIS OUTSIDE OF THE FOR CYCLE
        n_ib=(x_p1-b_p1)/d_ib;
        %angle dependency part (between the wall and the direction of the movement of the person)
        angles_ib=sum(-n_ib.*(V(2*p1+[-1,0])./sqrt(sum((V(2*p1+[-1,0])).^2))));
        lambda_part_wall=lambda_i+(1-lambda_i)*1/2*(1+angles_ib);
        %wall force
        f_wall_of_p1=A_i*exp((r_ij/2-d_ib)/B_i).*n_ib.*lambda_part_wall;
        %}
        %soc force making:
        %angle dependency part (between the person and others)
        angles_ij=reshape(repmat(sum(reshape(-n_ij.*v_p1_normed_vect,2,[])),2,1),1,2*(num_of_ppl-1));
        lambda_part_soc=lambda_i+(1-lambda_i)*1/2*(1+angles_ij);
        f_ppl_of_p1=A_i*exp((r_ij-d_ij_double)/B_i).*n_ij.*lambda_part_soc;
        %f_w(2*p1+[-1,0])=f_wall_of_p1;
        
        %body forces
        f_k_part=k*max(0,r_ij-d_ij_double).*n_ij;
        %if we dont want to add the body forces to the soc model but work with
        %it separately:
        %f_k(2*p1+[-1,0])=[sum(f_k_part(1:2:2*(num_of_ppl-1))),sum(f_k_part(2:2:2*(num_of_ppl-1)))];
        %soc force
        f_soc(2*p1+[-1,0])=f_soc(2*p1+[-1,0])+[sum(f_ppl_of_p1(1:2:2*(num_of_ppl-1))),sum(f_ppl_of_p1(2:2:2*(num_of_ppl-1)))]+[sum(f_k_part(1:2:2*(num_of_ppl-1))),sum(f_k_part(2:2:2*(num_of_ppl-1)))];
    end

    forces_one_time(:,2)=f_soc;
end

%{
%this in an f function later
X=[9     8     1     0    -6    -7    -2    -6     0    -3];%init_pos;
V=[ 0    -1     3     7     7     5     7     7     1     1];%init_vel;
B=find_closest_wall_coords(X,room);%closest wall coordinates

X_tmp=sqrt((X(1:2:2*num_of_ppl)-ppl_goal(1:2:2*num_of_ppl)).^2+(X(2:2:2*num_of_ppl)-ppl_goal(2:2:2*num_of_ppl)).^2);
X_norm(1:2:2*num_of_ppl)=X_tmp;
X_norm(2:2:2*num_of_ppl)=X_tmp;

V_norm=zeros(1,2*num_of_ppl);
V_norm(1:2:2*num_of_ppl)=sqrt(V(1:2:2*num_of_ppl).^2+V(2:2:2*num_of_ppl).^2);
V_norm(2:2:2*num_of_ppl)=V_norm(1:2:2*num_of_ppl);

f_dir=1/tau*((ppl_goal-X)./X_norm*v_0-V);%dir force
f_soc=zeros(1,2*num_of_ppl);%soc frce
%f_g=zeros(1,2*num_of_ppl);
%if we dont want to add the body forces to the soc model but work with
%it separately:
%f_k=zeros(1,2*num_of_ppl);%body force
%f_w=zeros(1,2*num_of_ppl);%wall force

%wall force making:
d_ib=zeros(1,2*num_of_ppl);
d_ib(1:2:2*num_of_ppl)=sqrt((X(1:2:2*num_of_ppl)-B(1:2:2*num_of_ppl)).^2+(X(2:2:2*num_of_ppl)-B(2:2:2*num_of_ppl)).^2);
d_ib(2:2:2*num_of_ppl)=d_ib(1:2:2*num_of_ppl);
%normed vector from now checked person to its closest wall coords
n_ib=(X-B)./d_ib;

%angle dependency part (between the wall and the direction of the movement of the person)
angles_all_tmp=-n_ib.*(V./V_norm);
angles_ib_all=angles_all_tmp(1:2:2*num_of_ppl)+angles_all_tmp(2:2:2*num_of_ppl);%dotprods
lambda_part_wall=zeros(1,2*num_of_ppl);
lambda_part_wall(1:2:2*num_of_ppl)=lambda_i+(1-lambda_i)*1/2*(1+angles_ib_all);
lambda_part_wall(2:2:2*num_of_ppl)=lambda_part_wall(1:2:2*num_of_ppl);
%wall force
f_w=A_i*exp((r_ij/2-d_ib)/B_i).*n_ib.*lambda_part_wall;

for p1=1:num_of_ppl%indices_tmp(not_at_goal)%1:num_of_ppl 
    x_p1=X(2*p1+[-1,0]);%coords of the used person
    b_p1=B(2*p1+[-1,0]);%coords of the used persons closes wall
    %v_p1=V(2*p1+[-1,0]);
    v_p1_normed_vect=repmat(V(2*p1+[-1,0])/norm(V(2*p1+[-1,0])),1,num_of_ppl-1);
    %coords of all the other person, dim: 1 x 2*(num_of_ppl-1)
    X_others=X(reshape(2*cat(2,1:p1-1,p1+1:num_of_ppl)+[-1,0]',1,2*(num_of_ppl-1)));
    %vectors from all the other people pointing to the now checked person
    X_ij=repmat(x_p1,1,(num_of_ppl-1))-X_others;
    %d_ij=sqrt(X_ij(1:2:end).^2+X_ij(2:2:end).^2);
    %norms of the above, doubled
    d_ij_double=reshape(repmat(sqrt(X_ij(1:2:end).^2+X_ij(2:2:end).^2),2,1),1,2*(num_of_ppl-1));
    %normed vectors from all the other people pointing to the now checked person
    n_ij=X_ij./d_ij_double;
    %{
    %wall force making:
    d_ib=sqrt(sum((x_p1-b_p1).^2));
    %normed vector from now checked person to its closest wall coords
    %LATER PUT THIS OUTSIDE OF THE FOR CYCLE
    n_ib=(x_p1-b_p1)/d_ib;
    %angle dependency part (between the wall and the direction of the movement of the person)
    angles_ib=sum(-n_ib.*(V(2*p1+[-1,0])./sqrt(sum((V(2*p1+[-1,0])).^2))));
    lambda_part_wall=lambda_i+(1-lambda_i)*1/2*(1+angles_ib);
    %wall force
    f_wall_of_p1=A_i*exp((r_ij/2-d_ib)/B_i).*n_ib.*lambda_part_wall;
    %soc force making:
    %angle dependency part (between the person and others)
    angles_ij=reshape(repmat(sum(reshape(-n_ij.*v_p1_normed_vect,2,[])),2,1),1,2*(num_of_ppl-1));
    lambda_part_soc=lambda_i+(1-lambda_i)*1/2*(1+angles_ij);
    f_ppl_of_p1=A_i*exp((r_ij-d_ij_double)/B_i).*n_ij.*lambda_part_soc;
    f_w(2*p1+[-1,0])=f_wall_of_p1;
    %}
    %body forces
    f_k_part=k*max(0,r_ij-d_ij_double).*n_ij;
    %if we dont want to add the body forces to the soc model but work with
    %it separately:
    %f_k(2*p1+[-1,0])=[sum(f_k_part(1:2:2*(num_of_ppl-1))),sum(f_k_part(2:2:2*(num_of_ppl-1)))];
    %soc force
    f_soc(2*p1+[-1,0])=f_soc(2*p1+[-1,0])+[sum(f_ppl_of_p1(1:2:2*(num_of_ppl-1))),sum(f_ppl_of_p1(2:2:2*(num_of_ppl-1)))]+[sum(f_k_part(1:2:2*(num_of_ppl-1))),sum(f_k_part(2:2:2*(num_of_ppl-1)))];
end
%}

function min_wall_coords=find_closest_cont_wall_coords(x,room)
    num_of_ppl=size(x,2)/2;
    ap_1=-repmat(room.walls_init,1,num_of_ppl)+x;
    t=ap_1.*repmat(room.u_s,1,num_of_ppl);%QP.*n_s
    tt=(t(:,1:2:end)+t(:,2:2:end))./(vecnorm(room.u_s,2,2).^2);%vetitett lekozelebbi vektor hossza, dim: # faldarabok x #személyek
    
    tt(tt>1)=1;
    tt(tt<0)=0;
    tt_double=zeros(size(tt,1),2*size(tt,2));
    tt_double(:,1:2:end)=tt;
    tt_double(:,2:2:end)=tt;
    
    us=repmat(room.u_s,1,num_of_ppl).*tt_double+repmat(room.walls_init,1,num_of_ppl);
    
    n_u=repmat(x,size(room.u_s,1),1)-(us);
    length_n_u=sqrt(n_u(:,1:2:size(n_u,2)).^2+n_u(:,2:2:size(n_u,2)).^2);
    %n_u-k hossza 
    [~,min_ind]=min(length_n_u,[],1);
    min_ind_double=zeros(1,2*size(min_ind,2));
    min_ind_double(1:2:end)=min_ind;
    min_ind_double(2:2:end)=min_ind;
    linindices=sub2ind([size(room.walls,1),2*num_of_ppl],min_ind_double,1:2*num_of_ppl);        %closest_vectors=n_u(linindices)+room.walls_init([min_ind],:)';
    %closest_vectors
    %min_wall_coords=closest_vectors(:)';
    %n_u2=tt2(linindices).*room.n_s([min_ind],:)';
    min_wall_coords=us(linindices);%y(j,1:2*num_of_ppl)-n_u2(:)';
end

function min_wall_coords=find_closest_disc_wall_coords(x,room)
        num_of_ppl=size(x,2)/2;
        vv=reshape(repmat(x,size(room.wall_coords,1),1)-repmat(room.wall_coords,1,num_of_ppl),size(room.wall_coords,1),2,[]);
        [~,min_lincoords]=min(squeeze(vecnorm(vv,2,2)));
        min_wall_coords=room.wall_coords([min_lincoords],:)';

        min_wall_coords=reshape(min_wall_coords,1,2*num_of_ppl);
end

function vect=cutoff_fnc(vect,v_max,v_0)%cannot be implemented by symbolic fnc...
    vect_norm=zeros(1,size(vect,2));
    vect_norm(1:2:end)=sqrt(vect(1:2:end).^2+vect(2:2:end).^2);
    vect_norm(2:2:end)=vect_norm(1:2:end);
    indxs_tmp=(vect_norm>=v_max);
    vals_to_cut_off=vect(indxs_tmp);
    %vals_to_cut_off./reshape(repmat(vecnorm(reshape(vals_to_cut_off,2,[])),2,1),1,[])
    vect(indxs_tmp)=v_max*(vals_to_cut_off./reshape(repmat(vecnorm(reshape(vals_to_cut_off,2,[])),2,1),1,[]));   
end
%}
function y=direction_dependence(e,f,effective_angle_of_sight,f_norm)
    y=ones(1,size(e,1));

    %{
    %for one vector
    if e*f>=vecnorm(f)*cos(effective_angle_of_sight)
        y=1;
    end
    %}
    %for multiple vectors
    y(sum(reshape(e.*f,2,[]))<f_norm(f)*cos(effective_angle_of_sight))=1;
end
        
function y_rk4=rk4_method(N,h,init)
    y_rk4=zeros(length(init),N);
    y_rk4(:,1)=init;

    for i=1:N
    y_rk4(:,i+1)=rk4_v(0,y_rk4(:,i),h);
    end   
end

function y_n=rk4_v(t,y,h)
    k1=f_v(t,y);
    k2=f_v(t+h*1/2,y+h*1/2*k1);
    k3=f_v(t+h*1/2,y+h*1/2*k2);
    k4=f_v(t+h,y+h*k3);
    y_n=y+1/6*h*(k1+2*k2+2*k3+k4);
end

function simple_plot(y,forces,ppl_goal,num_of_ppl,r_ij,room,disc_wall,eikonal,which_goal)
    figure;
    %xlim([-10,10]);
    %ylim([-10,10]);
    plot(y(1:end,1:2:size(y,2)/2),y(1:end,2:2:size(y,2)/2),'LineWidth',1.5)
    hold on;
    %{
    xline(10)
    xline(-10)
    yline(10)
    yline(-10)
    %}
    %xlim([-10.5,10.5])
    %ylim([-10.5,10.5])
    xlim([-1,11])
    ylim([-1,11])

    %plot walls - if we would do w/out the for cycle i.e. with vectors,
    %there would between the ending a starting points (not just the
    %starting and ending points)
    %
    if disc_wall==0
        for ind_plot=1:1:size(room.walls,1)
            line([room.walls(ind_plot,1),room.walls(ind_plot,3)],[room.walls(ind_plot,2),room.walls(ind_plot,4)])
        end
    else 
        xline(0)
        xline(10)
        yline(0)
        yline(10)
    end
    %}
    %imagesc([room.wall_map]')
    %plot circles
    plot_step=50;
   
    y_tmp=[reshape(y(1:plot_step:end,1:2:size(y,2)/2),[],1),reshape(y(1:plot_step:end,2:2:size(y,2)/2),[],1)];

    which_goal_double(1:2:2*num_of_ppl)=which_goal;
    which_goal_double(2:2:2*num_of_ppl)=which_goal;
    for ind1=1:max(which_goal)
        l=find(which_goal==ind1);
        y_tmp_goal=[reshape(y(1:plot_step:end,l*2-1),[],1),reshape(y(1:plot_step:end,l*2),[],1)];
        viscircles(y_tmp_goal,r_ij/2*ones(1,size(y_tmp_goal,1)),'Color',rand(1,3));
    end
    
    %plot acceleration vectors
    %{
    y_vel_tmp=[reshape(y(1:50:end,size(y,2)/2+1:2:size(y,2)),[],1),reshape(y(1:50:end,size(y,2)/2+2:2:size(y,2)),[],1)];
    quiver(y_tmp(:,1),y_tmp(:,2),y_vel_tmp(:,1),y_vel_tmp(:,2),'Color','r','LineWidth',1.6)
    %}

    %plot force vectors
    %f_dir, goal
    %{
    dir_force_tmp=[reshape(forces(1:50:end,1:2:size(y,2)/2,1),[],1),reshape(forces(1:50:end,2:2:size(y,2)/2,1),[],1)];
    quiver(y_tmp(:,1),y_tmp(:,2),dir_force_tmp(:,1)+2*1/100*y_vel_tmp(:,1),dir_force_tmp(:,2)+2*1/100*y_vel_tmp(:,2),'Color','k','LineWidth',1.6)
    quiver(y_tmp(:,1),y_tmp(:,2),dir_force_tmp(:,1),dir_force_tmp(:,2),'Color','g','LineWidth',1.6)
    %}
    %f_wall
    wall_force_tmp=[reshape(forces(1:plot_step:end,1:2:size(y,2)/2,3),[],1),reshape(forces(1:plot_step:end,2:2:size(y,2)/2,3),[],1)];
    quiver(y_tmp(:,1),y_tmp(:,2),wall_force_tmp(:,1),wall_force_tmp(:,2),'Color','g','LineWidth',1.6)
    %f_soc
    soc_force_tmp=[reshape(forces(1:plot_step:end,1:2:size(y,2)/2,2),[],1),reshape(forces(1:plot_step:end,2:2:size(y,2)/2,2),[],1)];
    quiver(y_tmp(:,1),y_tmp(:,2),soc_force_tmp(:,1),soc_force_tmp(:,2),'Color','r','LineWidth',1.6)

    %plot room
    %plot(room.wall_X,room.wall_Y,'*')
    %colorbar;
end

function movie_plot(y,t,num_of_ppl)
    figure;
    xlim([-1,1]);
    ylim([-1,1]);
    %hold on;
    for ind=1:size(y,1)
        plot(y(ind,1:2:2*num_of_ppl),y(ind,2:2:2*num_of_ppl),'s','MarkerSize',5,'MarkerEdgeColor','red');
        pause(0.01)
    end
end

function create_plots(t,y,num_of_ppl)
    figure;

    for ind=1:2*num_of_ppl
        subplot(2*num_of_ppl,2,ind)
        plot(t,y(:,ind),'LineWidth',1.3)
        title(['x',num2str(ind)])
        subplot(2*num_of_ppl,2,2*num_of_ppl+ind)
        plot(t,y(:,2*num_of_ppl+ind),'LineWidth',1.3)
        title(['v',num2str(ind)])
    end

end