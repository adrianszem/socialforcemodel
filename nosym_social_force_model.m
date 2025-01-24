num_of_ppl=5;
disc_wall=0;%0:wall is given by lines, 1: wall given as a mask of the discretized space


init_pos=[1,0,1,0,randi([-9,9],1,2*(num_of_ppl-2))];%[-1.4,1,-1,1];%[1,1,-1,1];%[2,2,randi([-10,10],1,2*(num_of_ppl-1))];

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
save_tf=0;

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

%this in an f function later
X=init_pos;
V=init_vel;
B=find_closest_wall_coords(x,room);%closest wall coordinates
%{
X = sym("x",[1 2*num_of_ppl]);
V = sym("v",[1 2*num_of_ppl]);
B = sym("b",[1 2*num_of_ppl]);%closest wall coordinates
%}

X_tmp=sqrt((X(1:2:2*num_of_ppl)-ppl_goal(1:2:2*num_of_ppl)).^2+(X(2:2:2*num_of_ppl)-ppl_goal(2:2:2*num_of_ppl)).^2);
X_norm(1:2:2*num_of_ppl)=X_tmp;
X_norm(2:2:2*num_of_ppl)=X_tmp;

V_tmp=sqrt(V(1:2:2*num_of_ppl).^2+V(2:2:2*num_of_ppl).^2);
V_norm(1:2:2*num_of_ppl)=V_tmp;
V_norm(2:2:2*num_of_ppl)=V_tmp;

f_dir=1/tau*((ppl_goal-X)./X_norm*v_0-V);%dir force
f_soc=zeros(1,2*num_of_ppl);%soc frce
f_g=zeros(1,2*num_of_ppl);
f_k=zeros(1,2*num_of_ppl);%body force
f_w=zeros(1,2*num_of_ppl);%wall force

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
    %body forces
    f_k_part=k*max(0,r_ij-d_ij_double).*n_ij;
    f_k(2*p1+[-1,0])=[sum(f_k_part(1:2:2*(num_of_ppl-1))),sum(f_k_part(2:2:2*(num_of_ppl-1)))];
    %soc force
    f_soc(2*p1+[-1,0])=f_soc(2*p1+[-1,0])+[sum(f_ppl_of_p1(1:2:2*(num_of_ppl-1))),sum(f_ppl_of_p1(2:2:2*(num_of_ppl-1)))]+[sum(f_k_part(1:2:2*(num_of_ppl-1))),sum(f_k_part(2:2:2*(num_of_ppl-1)))];
    f_w(2*p1+[-1,0])=f_wall_of_p1;
end

function min_wall_coords=find_closest_wall_coords(x,room)
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