clear variables
%close all
clc

%Social Force model is an ODE model based on newtons second law (ma=mx''=F), but
%with non-physical forces too. 
%x=(x1,x2) are the coordinates of the person (we assume 2d space), thus for
%N people, the ODE is 2N dimensional. There are 3 forces, namely
%(i) Force which drives people to their goal
%(ii) Force which drives people away the "walls"
%(iii) Force that drives people away (or possibly to) other people
% + more specific models generally modeled by adding extra forces
%If we incorporate the geometry of the room, we have to solve an auxilary
%problem to find the direction to the goals. This can be modeled by the
%eikonal eqn - in the case of simple shortest (but possibly not quickest)
%distance search, this is not coupled to the ODE.

% It might be useful to save the people separately - like if we want to
% differentiate between them (i.e. not only save a long x coordiantes
% array)
% we will save this as cell array of structs

%
num_of_ppl=5;
disc_wall=0;%0:wall is given by lines, 1: wall given as a mask of the discretized space


init_pos=[1,0,1,0,randi([-9,9],1,2*(num_of_ppl-2))];%[-1.4,1,-1,1];%[1,1,-1,1];%[2,2,randi([-10,10],1,2*(num_of_ppl-1))];

%check whether two person have the same init_pos
%circshift can't be vectorized....:(
for ind1=1:2:2*num_of_ppl 
    ppl_tmp=init_pos(ind1+[0,1]);
    for ind2=ind1+2:2*(num_of_ppl-1)
        if sum(ppl_tmp==init_pos(ind2+[0,1]))==2
           init_pos(ind1+[0,1])=randi([-9,9],1,2);
        end
    end
end


ppl_goal=[4,2,randi([0,9],1,2*(num_of_ppl-1))];%[1,1,1,1];%[-1,1,1,1];%randi([0,10],1,2*num_of_ppl);
init_vel=[0,-1,randi([0,10],1,2*(num_of_ppl-1))];%[1,0,1,0];%[-1,0.001,1,0];%[-1,-1,randi([0,10],1,2*(num_of_ppl-1))];
save_tf=1;
%}

%{
load('test_ppl_924839_socforcmodel.mat')
disc_wall=0;
num_of_ppl=onerun.num_of_ppl;

init_pos=onerun.init_pos;
%init_pos= [    2     2    -1   -9    -6     3     8    -2    9     0     8     1    -8     9];
%ppl_goal=[4,2,randi([0,9],1,2*(num_of_ppl-1))];%[1,1,1,1];%[-1,1,1,1];%randi([0,10],1,2*num_of_ppl);
%ppl_goal=[     4     2     0     5     0     1     6     8     9     5     9     6     5     3];
%init_vel=[0,-1,randi([0,10],1,2*(num_of_ppl-1))];%[1,0,1,0];%[-1,0.001,1,0];%[-1,-1,randi([0,10],1,2*(num_of_ppl-1))];
init_vel=onerun.init_vel;
ppl_goal=onerun.ppl_goal;

save_tf=0;
%}
%{
for ind1=1:2:2*num_of_ppl 
    ppl_tmp=init_pos(ind1+[0,1]);
    for ind2=ind1+2:2*(num_of_ppl-1)
        if sum(ppl_tmp==init_pos(ind2+[0,1]))==2
           init_pos(ind1+[0,1])=randi([-9,9],1,2);
        end
    end
end
%}
%ppl=struct('index',num2cell(1:num_of_ppl),'extra_attr',[],'a_coords',[],'v_coords',[],'x_coords',[]);   %empty cell of structs 
if disc_wall==1
    load('room_for_soc_forc_mod.mat')%room, see make_discrete_room.m
else
    room.walls=[-10,10,10,10;...%x_startp,y_startp,x_endp,y_endp
    10,10,10,-10;...
    -10,-10,-10,10;...
    10,-10,-10,-10];
    room.walls_init=room.walls(:,1:2);
    room.walls_end=room.walls(:,3:4);
    
    %u_s,n_s fix
    room.u_s=room.walls(:,3:4)-room.walls(:,1:2);
    room.lengths=vecnorm(room.u_s,2,2);

    room.n_s=zeros(size(room.u_s));
    room.n_s(:,1:2:end)=-room.u_s(:,2:2:end);
    room.n_s(:,2:2:end)=room.u_s(:,1:2:end);
end


t_0=0;
t_max=10;
step_size=1/200;
t=t_0:step_size:t_max;
num_of_time_grid=length(t);

%in genral we have x''=f(x,x'),, we rewrite it to 4N dim ODE, without
%changing the names of the variables

%The exact ode depends on the geometry(through the wall forces) and on the
%number of people
%thus we need to make th RHS adaptable. I plan to do it with symbolic
%functions, and than convert them (only once) to general matlab functions, thus it wont
%really make the code slower
%Create Array of Symbolic Variables - it just nicer and imo easier to write
%the general RHS

v_0=2;
tau=1/2;%relaxation time [s]
v_max=2*v_0;
effective_angle_of_sight=pi/2;

A_i=20;
B_i=0.2;
lambda_i=0;
r_ij=0.5;
k=100;
kappa=100;

%A = sym("a",[1 2*num_of_ppl]);
%or can be defined by syms a [1 10] if we dont want them in an array struct
X = sym("x",[1 2*num_of_ppl]);
V = sym("v",[1 2*num_of_ppl]);
B = sym("b",[1 2*num_of_ppl]);%closest wall coordinates
X_tmp=sqrt((X(1:2:2*num_of_ppl)-ppl_goal(1:2:2*num_of_ppl)).^2+(X(2:2:2*num_of_ppl)-ppl_goal(2:2:2*num_of_ppl)).^2);

%not_at_goal=X_tmp>1;
%not_at_goal_double(1:2:2*num_of_ppl)=not_at_goal;
%not_at_goal_double(2:2:2*num_of_ppl)=not_at_goal;

X_norm(1:2:2*num_of_ppl)=X_tmp;
X_norm(2:2:2*num_of_ppl)=X_tmp;

V_tmp=sqrt(V(1:2:2*num_of_ppl).^2+V(2:2:2*num_of_ppl).^2);
V_norm(1:2:2*num_of_ppl)=V_tmp;
V_norm(2:2:2*num_of_ppl)=V_tmp;

f_dir=1/tau*((ppl_goal-X)./X_norm*v_0-V);
%f_dir=f_dir.*ones(size(f_dir));
f_soc=sym(zeros(1,2*num_of_ppl));
f_g=sym(zeros(1,2*num_of_ppl));
f_k=sym(zeros(1,2*num_of_ppl));%body force
f_w=sym(zeros(1,2*num_of_ppl));

%f_2_ppl=A_i*exp((r_ij-)/B_i);
tic
%indices_tmp=1:num_of_ppl;
for p1=1:num_of_ppl%indices_tmp(not_at_goal)%1:num_of_ppl 
    x_p1=X(2*p1+[-1,0]);
    b_p1=B(2*p1+[-1,0]);
    %v_p1=V(2*p1+[-1,0]);
    v_p1_normed_vect=repmat(V(2*p1+[-1,0])/norm(V(2*p1+[-1,0])),1,num_of_ppl-1);
    
    X_others=X(reshape(2*cat(2,1:p1-1,p1+1:num_of_ppl)+[-1,0]',1,2*(num_of_ppl-1)));
    X_ij=repmat(x_p1,1,(num_of_ppl-1))-X_others;
    %d_ij=sqrt(X_ij(1:2:end).^2+X_ij(2:2:end).^2);
    d_ij_double=reshape(repmat(sqrt(X_ij(1:2:end).^2+X_ij(2:2:end).^2),2,1),1,2*(num_of_ppl-1));
    n_ij=X_ij./d_ij_double;
    f_k_part=k*max(0,r_ij-d_ij_double).*n_ij;
    %wall force
    d_ib=sqrt(sum((x_p1-b_p1).^2));
    n_ib=(x_p1-b_p1)/d_ib;
    angles_ib=sum(-n_ib.*(V(2*p1+[-1,0])./sqrt(sum((V(2*p1+[-1,0])).^2))));
    lambda_part_wall=lambda_i+(1-lambda_i)*1/2*(1+angles_ib);
    f_wall_of_p1=A_i*exp((r_ij/2-d_ib)/B_i).*n_ib.*lambda_part_wall;
    %soc force
    angles_ij=reshape(repmat(sum(reshape(-n_ij.*v_p1_normed_vect,2,[])),2,1),1,2*(num_of_ppl-1));
    lambda_part=lambda_i+(1-lambda_i)*1/2*(1+angles_ij);
    f_ppl_of_p1=A_i*exp((r_ij-d_ij_double)/B_i).*n_ij.*lambda_part;
    f_k(2*p1+[-1,0])=[sum(f_k_part(1:2:2*(num_of_ppl-1))),sum(f_k_part(2:2:2*(num_of_ppl-1)))];
    f_soc(2*p1+[-1,0])=f_soc(2*p1+[-1,0])+[sum(f_ppl_of_p1(1:2:2*(num_of_ppl-1))),sum(f_ppl_of_p1(2:2:2*(num_of_ppl-1)))]+[sum(f_k_part(1:2:2*(num_of_ppl-1))),sum(f_k_part(2:2:2*(num_of_ppl-1)))];
    f_w(2*p1+[-1,0])=f_wall_of_p1;
end
toc

%convert symbolic function to function handle
f_norm=matlabFunction(X_norm,"Vars",{X});% for the cutoff function
f_vect_soc=matlabFunction(f_soc,"Vars",{[X,V]});%{} instead of [] to have the inpts as arrays
f_x=matlabFunction(V,"Vars", {V});% id fnc
f_wall_vect=matlabFunction(f_w,"Vars",{[X,V,B]});
f_vect_dir=matlabFunction(f_dir,"Vars",{[X,V]});

%cutting it if velocity tto large:
%ll=piecewise(V_norm(1)<v_max,V(1:2),V_norm(1)>=v_max,V(1:2)./V_norm(1,2)*v_max);
%unfortunately matlabFunction cant do piecewise functions
%thus this part will be implemented as a matlab function


%plot_vector_field_one_person(-6,-6,6,6,0.5,F_goal)
tic
if disc_wall==1
    [y,forces]=exp_euler_discrete_wall(num_of_time_grid,step_size,[init_pos,init_vel],f_x,f_vect_dir,f_vect_soc,f_wall_vect,f_norm,v_max,v_0,room);
else
    [y,forces]=exp_euler_cont_wall(num_of_time_grid,step_size,[init_pos,init_vel],f_x,f_vect_dir,f_vect_soc,f_wall_vect,f_norm,v_max,v_0,room);
end
toc

simple_plot(y,forces,ppl_goal,num_of_ppl,r_ij,room,disc_wall)

%movie_plot(y,t,num_of_ppl)

%create_plots(t,y,num_of_ppl)

if save_tf==1
    onerun.ppl_goal=ppl_goal;
    onerun.init_pos=init_pos;
    onerun.init_vel=init_vel;
    onerun.num_of_ppl=num_of_ppl;
    rand_num=randi(1000000);
    file_name='test_ppl_'+string(rand_num)+'_socforcmodel.mat';
    save(file_name,'onerun')
    
    disp(['initvals were saved as ', file_name])
end


%numerical methods:
%we will need to define 'f'
function [y,forces_k]=exp_euler_discrete_wall(N,h,init,f_x,f_v_dir,f_v_soc,f_v_wall,f_norm,v_max,v_0,room)
    num_of_ppl=length(init)/4;
    y=zeros(N,length(init));
    %save forces
    %forces=zeros(N,length(init)/2,3);
    forces_k=zeros(N,length(init)/2,3);

    y(1,:)=init;

    for j=1:N-1
        %y(:,j+1)=y(:,j)+h*f_v(t,y(:,j));
        
        %find closest room coordinates

        vv=reshape(repmat(y(j,1:2*num_of_ppl),size(room.wall_coords,1),1)-repmat(room.wall_coords,1,num_of_ppl),size(room.wall_coords,1),2,[]);
        [min_vals,min_lincoords]=min(squeeze(vecnorm(vv,2,2)));
        min_wall_coords=room.wall_coords([min_lincoords,1],:)';

        %y(j+1,:)=y(j,:)+h*[cutoff_fnc(f_x(y(j,2*num_of_ppl+1:end)),f_norm,v_max,v_0),f_v_dir(y(j,:))+f_v_soc(y(j,:))+f_v_wall([y(j,:),min_wall_coords(:)'])];
        %forces(j,:,:)=[f_v_dir(y(j,:));f_v_soc(y(j,:));f_v_wall([y(j,:),min_wall_coords(:)'])];
        forces_k(j,:,1)=h*f_v_dir(y(j,:));
        forces_k(j,:,2)=h*f_v_soc(y(j,:));
        forces_k(j,:,3)=h*f_v_wall([y(j,:),min_wall_coords(:)']);
        y(j+1,:)=y(j,:)+[h*cutoff_fnc(f_x(y(j,2*num_of_ppl+1:end)),f_norm,v_max,v_0),sum(forces_k(j,:,:),3)];

        %f_x(y(j,2*num_of_ppl+1:end))
    end
end

function [y,forces_k]=exp_euler_cont_wall(N,h,init,f_x,f_v_dir,f_v_soc,f_v_wall,f_norm,v_max,v_0,room)
    r_ij=0.5;
    num_of_ppl=length(init)/4;
    y=zeros(N,length(init));
    %save forces
    %forces=zeros(N,length(init)/2,3);
    forces_k=zeros(N,length(init)/2,3);

    y(1,:)=init;

    for j=1:N-1
        %y(:,j+1)=y(:,j)+h*f_v(t,y(:,j));
        
        %find closest room coordinates
        %{
        %y(j,1:2*num_of_ppl)
        qp_1=(-repmat(room.walls_init,1,num_of_ppl)+y(j,1:2*num_of_ppl));
        %qp_2=(-repmat(room.walls_end,1,num_of_ppl)+y(j,1:2*num_of_ppl));
        %mins=min(sqrt(abs(qp_1(:,2:2:size(qp_1,2))).^2+abs(qp_1(:,1:2:size(qp_1,2))).^2),sqrt(abs(qp_2(:,2:2:size(qp_2,2))).^2+abs(qp_2(:,1:2:size(qp_2,2))).^2));
        %norm_prod=sqrt(qp_1(:,2:2:size(qp_1,2)).^2+qp_1(:,1:2:size(qp_1,2)).^2).*sqrt(qp_2(:,2:2:size(qp_2,2)).^2+qp_2(:,1:2:size(qp_2,2)).^2);
        %tmp_a=qp_1.*qp_2;
        %angles=(tmp_a(:,1:2:2*num_of_ppl)+tmp_a(:,2:2:2*num_of_ppl))./norm_prod;
        %angles>0
        t=qp_1.*repmat(room.n_s,1,num_of_ppl);%QP.*n_s
        tt=(t(:,1:2:end)+t(:,2:2:end))./(vecnorm(room.n_s,2,2).^2);
        [min_dist,min_ind]=min(abs(tt),[],1);
        linindices=sub2ind([size(room.walls,1),num_of_ppl],min_ind,1:num_of_ppl);
        

        n_u=tt(linindices).*room.n_s([min_ind],:)';%later the force should be changed that the inputs are the min(tt)*n_s vectors only....
                                              %or maybe write the line
                                              %point equation differently
        n_u
        min_wall_coords=y(j,1:2*num_of_ppl)-n_u(:)';
        min_wall_coords
        %}

        %
        %új
        ap_1=(-repmat(room.walls_init,1,num_of_ppl)+y(j,1:2*num_of_ppl));
        t=ap_1.*repmat(room.u_s,1,num_of_ppl);%QP.*n_s
        tt=(t(:,1:2:end)+t(:,2:2:end))./(vecnorm(room.u_s,2,2).^2);%vetitett lekozelebbi vektor hossza, dim: # faldarabok x #személyek
        %régi
        %t2=(-repmat(room.walls_init,1,num_of_ppl)+y(j,1:2*num_of_ppl)).*repmat(room.n_s,1,num_of_ppl);%QP.*n_s
        %tt2=(t2(:,1:2:end)+t2(:,2:2:end))./(vecnorm(room.n_s,2,2).^2);
        
        tt(tt>1)=1;
        tt(tt<0)=0;
        tt_double=zeros(size(tt,1),2*size(tt,2));
        tt_double(:,1:2:end)=tt;
        tt_double(:,2:2:end)=tt;
        
        us=repmat(room.u_s,1,num_of_ppl).*tt_double+repmat(room.walls_init,1,num_of_ppl);

        n_u=repmat(y(j,1:2*num_of_ppl),size(room.u_s,1),1)-(us);
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


        
        %}

        %vv=reshape(repmat(y(j,1:2*num_of_ppl),size(room.wall_coords,1),1)-repmat(room.wall_coords,1,num_of_ppl),size(room.wall_coords,1),2,[]);
        %[min_vals,min_lincoords]=min(squeeze(vecnorm(vv,2,2)));
        %min_wall_coords=room.wall_coords([min_lincoords,1],:)';

        %y(j+1,:)=y(j,:)+h*[cutoff_fnc(f_x(y(j,2*num_of_ppl+1:end)),f_norm,v_max,v_0),f_v_dir(y(j,:))+f_v_soc(y(j,:))+f_v_wall([y(j,:),min_wall_coords(:)'])];
        %forces(j,:,:)=[f_v_dir(y(j,:));f_v_soc(y(j,:));f_v_wall([y(j,:),min_wall_coords(:)'])];

        %vecnorm([y(j+1,1:2:2*num_of_ppl);y(j+1,2:2:2*num_of_ppl)]-[ppl_goal(1:2:end);ppl_goal(2:2:end)],2,1)<r_ij

        forces_k(j,:,1)=h*f_v_dir(y(j,:));
        forces_k(j,:,2)=h*f_v_soc(y(j,:));
        forces_k(j,:,3)=h*f_v_wall([y(j,:),min_wall_coords]);
        y(j+1,:)=y(j,:)+[h*cutoff_fnc(f_x(y(j,2*num_of_ppl+1:end)),f_norm,v_max,v_0),sum(forces_k(j,:,:),3)];

    end
end


function vect=cutoff_fnc(vect,f_norm,v_max,v_0)%cannot be implemented by symbolic fnc...
    indxs_tmp=f_norm(vect)>=v_max;
    vals_to_cut_off=vect(indxs_tmp);
    %vals_to_cut_off./reshape(repmat(vecnorm(reshape(vals_to_cut_off,2,[])),2,1),1,[])
    vect(indxs_tmp)=v_max*(vals_to_cut_off./reshape(repmat(vecnorm(reshape(vals_to_cut_off,2,[])),2,1),1,[]));
    
end

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

function simple_plot(y,forces,ppl_goal,num_of_ppl,r_ij,room,disc_wall)
    figure;
    xlim([-10,10]);
    ylim([-10,10]);
    plot(y(1:end,1:2:size(y,2)/2),y(1:end,2:2:size(y,2)/2),'LineWidth',1.5)
    hold on;
    %{
    xline(10)
    xline(-10)
    yline(10)
    yline(-10)
    %}
    xlim([-10.5,10.5])
    ylim([-10.5,10.5])

    %plot walls - if we would do w/out the for cycle i.e. with vectors,
    %there would between the ending a starting points (not just the
    %starting and ending points)
    if disc_wall==0
        for ind_plot=1:1:size(room.walls,1)
            line([room.walls(ind_plot,1),room.walls(ind_plot,3)],[room.walls(ind_plot,2),room.walls(ind_plot,4)])
        end
    else 
        xline(10)
        xline(-10)
        yline(10)
        yline(-10)
    end
    %plot circles
    plot_step=50;
    y_tmp=[reshape(y(1:plot_step:end,1:2:size(y,2)/2),[],1),reshape(y(1:plot_step:end,2:2:size(y,2)/2),[],1)];
    viscircles(y_tmp,r_ij/2*ones(1,size(y_tmp,1)),'Color','m');
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

    %plot goals

    plot(ppl_goal(1:2:2*num_of_ppl),ppl_goal(2:2:2*num_of_ppl),'s','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6]);
    
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

%{
function out=f_v(t,y)
    out=y;
end
%}
