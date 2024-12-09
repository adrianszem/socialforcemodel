clear variables
close all
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

%{
num_of_ppl=15;

ppl_goal=randi([0,9],1,2*num_of_ppl);%[1,1,1,1];%[-1,1,1,1];%randi([0,10],1,2*num_of_ppl);
init_pos=[2,2,randi([-9,9],1,2*(num_of_ppl-1))];%[-1.4,1,-1,1];%[1,1,-1,1];%[2,2,randi([-10,10],1,2*(num_of_ppl-1))];
init_vel=[-1,-1,randi([0,10],1,2*(num_of_ppl-1))];%[1,0,1,0];%[-1,0.001,1,0];%[-1,-1,randi([0,10],1,2*(num_of_ppl-1))];
%}

%
load('test_ppl_409843_socforcmodel.mat')

num_of_ppl=onerun.num_of_ppl;

ppl_goal=onerun.ppl_goal;
init_pos=onerun.init_pos;
init_vel=onerun.init_vel;
%}

ppl=struct('index',num2cell(1:num_of_ppl),'extra_attr',[],'a_coords',[],'v_coords',[],'x_coords',[]);   %empty cell of structs 
load('room_for_soc_forc_mod.mat')%room, see make_discrete_room.m
t_0=0;
t_max=10;
step_size=1/100;
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
X_tmp=sqrt((X(1:2:end)-ppl_goal(1:2:end)).^2+(X(2:2:end)-ppl_goal(2:2:end)).^2);
X_norm(1:2:2*num_of_ppl)=X_tmp;
X_norm(2:2:2*num_of_ppl)=X_tmp;

V_tmp=sqrt(V(1:2:end).^2+V(2:2:end).^2);
V_norm(1:2:2*num_of_ppl)=V_tmp;
V_norm(2:2:2*num_of_ppl)=V_tmp;

f=1/tau*((ppl_goal-X)./X_norm*v_0-V);
f_g=f;
f_k=f;%body force

%f_2_ppl=A_i*exp((r_ij-)/B_i);
tic
for p1=1:num_of_ppl
    %others=cat(2,1:p1-1,p1+1:num_of_ppl);
    x_p1=X(2*p1+[-1,0]);
    b_p1=B(2*p1+[-1,0]);
    v_p1=V(2*p1+[-1,0]);
    v_p1_normed_vect=repmat(V(2*p1+[-1,0])/norm(V(2*p1+[-1,0])),1,num_of_ppl-1);
    
    X_others=X(reshape(2*cat(2,1:p1-1,p1+1:num_of_ppl)+[-1,0]',1,2*(num_of_ppl-1)));
    X_ij=repmat(x_p1,1,(num_of_ppl-1))-X_others;
    d_ij=sqrt(X_ij(1:2:end).^2+X_ij(2:2:end).^2);
    d_ij_double=reshape(repmat(sqrt(X_ij(1:2:end).^2+X_ij(2:2:end).^2),2,1),1,2*(num_of_ppl-1));
    n_ij=X_ij./d_ij_double;
    f_k_part=k*max(0,r_ij-d_ij_double).*n_ij;
    angles_ij=reshape(repmat(sum(reshape(-n_ij.*v_p1_normed_vect,2,[])),2,1),1,2*(num_of_ppl-1));

    d_ib=sqrt(sum((x_p1-b_p1).^2));
    n_ib=(x_p1-b_p1)/d_ib;
    angles_ib=sum(-n_ib.*(V(2*p1+[-1,0])./sqrt(sum((V(2*p1+[-1,0])).^2))));
    lambda_part_wall=lambda_i+(1-lambda_i)*1/2*(1+angles_ib);
    f_wall_of_p1=A_i*exp((r_ij/2-d_ib)/B_i).*n_ib.*lambda_part_wall;

    lambda_part=lambda_i+(1-lambda_i)*1/2*(1+angles_ij);
    f_ppl_of_p1=A_i*exp((r_ij-d_ij_double)/B_i).*n_ij.*lambda_part;
    f_k(2*p1+[-1,0])=[sum(f_k_part(1:2:2*(num_of_ppl-1))),sum(f_k_part(2:2:2*(num_of_ppl-1)))];
    f(2*p1+[-1,0])=f(2*p1+[-1,0])+f_wall_of_p1+[sum(f_ppl_of_p1(1:2:2*(num_of_ppl-1))),sum(f_ppl_of_p1(2:2:2*(num_of_ppl-1)))]+[sum(f_k_part(1:2:2*(num_of_ppl-1))),sum(f_k_part(2:2:2*(num_of_ppl-1)))];
end
toc

%convert symbolic function to function handle
f_norm=matlabFunction(X_norm,"Vars",{X});% for the cutoff function
f_v=matlabFunction(f,"Vars",{[X,V,B]});%{} instead of [] to have the inpts as arrays
f_x=matlabFunction(V,"Vars", {V});% id fnc

%cutting it if velocity tto large:
%ll=piecewise(V_norm(1)<v_max,V(1:2),V_norm(1)>=v_max,V(1:2)./V_norm(1,2)*v_max);
%unfortunately matlabFunction cant do piecewise functions
%thus this part will be implemented as a matlab function


%plot_vector_field_one_person(-6,-6,6,6,0.5,F_goal)
tic
y=exp_euler(num_of_time_grid,step_size,[init_pos,init_vel],f_x,f_v,f_norm,v_max,v_0,room);
toc

simple_plot(y,ppl_goal,num_of_ppl,r_ij)

%movie_plot(y,t,num_of_ppl)

create_plots(t,y,num_of_ppl)

onerun.ppl_goal=ppl_goal;
onerun.init_pos=init_pos;
onerun.init_vel=init_vel;
onerun.num_of_ppl=num_of_ppl;
rand_num=randi(1000000);
file_name='test_ppl_'+string(rand_num)+'_socforcmodel.mat';
save(file_name,'onerun')

disp(['initvals were saved as ', file_name])



%numerical methods:
%we will need to define 'f'
function y=exp_euler(N,h,init,f_x,f_v,f_norm,v_max,v_0,room)
    num_of_ppl=length(init)/4;
    y=zeros(N,length(init));
    y(1,:)=init;

    for j=1:N-1
        %y(:,j+1)=y(:,j)+h*f_v(t,y(:,j));
        
        %find closest room coordinates

        vv=reshape(repmat(y(j,1:2*num_of_ppl),size(room.wall_coords,1),1)-repmat(room.wall_coords,1,num_of_ppl),size(room.wall_coords,1),2,[]);
        [min_vals,min_lincoords]=min(squeeze(vecnorm(vv,2,2)));
        min_wall_coords=room.wall_coords([min_lincoords,1],:)';

        y(j+1,:)=y(j,:)+h*[cutoff_fnc(f_x(y(j,2*num_of_ppl+1:end)),f_norm,v_max,v_0),f_v([y(j,:),min_wall_coords(:)'])];
        %f_x(y(j,2*num_of_ppl+1:end))
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

function simple_plot(y,ppl_goal,num_of_ppl,r_ij)
    figure;
    xlim([-10,10]);
    ylim([-10,10]);
    plot(y(1:end,1:2:size(y,2)/2),y(1:end,2:2:size(y,2)/2),'LineWidth',1.5)
    hold on;
    y_tmp=[reshape(y(1:50:end,1:2:size(y,2)/2),[],1),reshape(y(1:50:end,2:2:size(y,2)/2),[],1)];
    viscircles(y_tmp,r_ij*ones(1,size(y_tmp,1)));

    plot(ppl_goal(1:2:2*num_of_ppl),ppl_goal(2:2:2*num_of_ppl),'s','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6]);

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
