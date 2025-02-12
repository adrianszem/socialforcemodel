function sim_graph_objects=nosym_social_force_model_gui(room_config_datas,fig,uiax,newfigure_logical,run_as_movie_logical)
    init_pos=room_config_datas.person_coords;
    ppl_goal=room_config_datas.goal_coords;
    init_vel=room_config_datas.vel_coords;
    num_of_ppl=length(init_pos)/2;
    
    room.walls=room_config_datas.wall_coords;
    room.walls_init=room.walls(:,1:2);

    room.u_s=room.walls(:,3:4)-room.walls(:,1:2);
    room.n_s=zeros(size(room.u_s));
    room.n_s(:,1:2:end)=-room.u_s(:,2:2:end);
    room.n_s(:,2:2:end)=room.u_s(:,1:2:end);
    room.lengths=vecnorm(room.u_s,2,2);
    
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

    [y,forces]=exp_euler_cont_wall(num_of_time_grid,step_size,[init_pos,init_vel],ppl_goal,v_max,v_0,room,run_as_movie_logical,fig,uiax);
    
    %in the case, when we run a movie, after its done, we still want to see it:
    sim_graph_objects=simple_plot(y,step_size,forces,ppl_goal,num_of_ppl,r_ij,room,uiax,newfigure_logical);
    
    %movie_plot(y,t,num_of_ppl)

    %create_plots(t,y,num_of_ppl)
    
    disp('simulation ended')
    
    function [y,forces_k]=exp_euler_cont_wall(N,h,init,ppl_goal,v_max,v_0,room,run_as_movie_logical,fig,ax)
        if run_as_movie_logical==0
            d2 = uiprogressdlg(fig,'Title','Please Wait','Message','Simulating');
        elseif run_as_movie_logical==1
            plot_tmp=plot(uiax,[],[]);
        end


        r_ij=0.5;
        num_of_ppl=length(init)/4;
        y=zeros(N,length(init));
        %save forces
        forces_k=zeros(N,length(init)/2,3);
        
        y(1,:)=init;
    
        for j=1:N-1
            B=find_closest_cont_wall_coords(y(j,1:2*num_of_ppl),room);%closest wall coordinates
            forces_k(j,:,:)=h*forceparts_calc(y(j,1:2*num_of_ppl),ppl_goal,y(j,2*num_of_ppl+1:end),B);
            %forces_k(j,:,:)=f1(j,:,:);
            y(j+1,:)=y(j,:)+[h*cutoff_fnc(y(j,2*num_of_ppl+1:end),v_max,v_0),sum(forces_k(j,:,:),3)];%id fnc in cutoff

            if mod(j,30)==0 && run_as_movie_logical==1
                pause(0.001)
                delete(plot_tmp)
                plot_tmp(1)=plot(ax,y(j+1,1:2:2*num_of_ppl),y(j+1,2:2:2*num_of_ppl),'o',"MarkerSize",8,'MarkerFaceColor','b');
                plot_tmp(2)=quiver(ax,y(j+1,1:2:2*num_of_ppl),y(j+1,2:2:2*num_of_ppl),y(j+1,2*num_of_ppl+1:2:end),y(j+1,2*num_of_ppl+2:2:end),'Color','m','LineWidth',1.6,'AutoScale','off');
                %plot_tmp(3)=quiver(ax,y(j+1,1:2:2*num_of_ppl),y(j+1,2:2:2*num_of_ppl),1/h*forces_k(j,1:2:end,3),1/h*forces_k(j,2:2:end,3),'Color','g','LineWidth',1.6,'AutoScale','off');
                %plot_tmp(4)=quiver(ax,y(j+1,1:2:2*num_of_ppl),y(j+1,2:2:2*num_of_ppl),1/h*forces_k(j,1:2:end,2),1/h*forces_k(j,2:2:end,2),'Color','r','LineWidth',1.6,'AutoScale','off');
            elseif mod(j,200)==0 && run_as_movie_logical==0
                d2.Value = j/(N-1); 
            end
        end
        
        if run_as_movie_logical==0
            close(d2);
        elseif run_as_movie_logical==1
            delete(plot_tmp)
        end
    end
    
    function forces_one_time=forceparts_calc(X,ppl_goal,V,B)
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
        forces_one_time=zeros(num_of_coords,3);
    
        X_tmp=sqrt((X(1:2:num_of_coords)-ppl_goal(1:2:num_of_coords)).^2+(X(2:2:num_of_coords)-ppl_goal(2:2:num_of_coords)).^2);
        X_norm(1:2:num_of_coords)=X_tmp;
        X_norm(2:2:num_of_coords)=X_tmp;
        
        V_norm=zeros(1,num_of_coords);
        V_norm(1:2:num_of_coords)=sqrt(V(1:2:num_of_coords).^2+V(2:2:num_of_coords).^2);
        V_norm(2:2:num_of_coords)=V_norm(1:2:num_of_coords);
        
        forces_at_goal=0;
        goal_dist_cutoff_val=1;

        if forces_at_goal==1
            not_at_goal=true(size(X_tmp));
            not_at_goal_double=true(1,2*num_of_ppl);
        else
            not_at_goal=X_tmp>goal_dist_cutoff_val;
            not_at_goal_double=true(1,2*num_of_ppl);
            not_at_goal_double(1:2:2*num_of_ppl)=not_at_goal;
            not_at_goal_double(2:2:2*num_of_ppl)=not_at_goal;
        end
        

        num_of_ppl=num_of_coords/2;
        indices_tmp=1:num_of_ppl;
        f_dir=zeros(1,num_of_coords);
        f_dir(not_at_goal_double)=1/tau*((ppl_goal(not_at_goal_double)-X(not_at_goal_double))./X_norm(not_at_goal_double)*v_0-V(not_at_goal_double));%dir force
        forces_one_time(:,1)=f_dir;%dir force
    
        %wall force making:
        d_ib=zeros(1,num_of_coords);
        d_ib(1:2:num_of_coords)=sqrt((X(1:2:num_of_coords)-B(1:2:num_of_coords)).^2+(X(2:2:num_of_coords)-B(2:2:num_of_coords)).^2);
        d_ib(2:2:num_of_coords)=d_ib(1:2:num_of_coords);
        %normed vector from now checked person to its closest wall coords
        n_ib=(X-B)./d_ib;
        %angle dependency part (between the wall and the direction of the movement of the person)
        angles_all_tmp=-n_ib.*(V./V_norm);
        angles_ib_all=angles_all_tmp(1:2:num_of_coords)+angles_all_tmp(2:2:num_of_coords);%dotprods
        lambda_part_wall=zeros(1,num_of_coords);
        lambda_part_wall(1:2:num_of_coords)=lambda_i+(1-lambda_i)*1/2*(1+angles_ib_all);
        lambda_part_wall(2:2:num_of_coords)=lambda_part_wall(1:2:num_of_coords);
        %wall force
        %f_w=A_i*exp((r_ij/2-d_ib)/B_i).*n_ib.*lambda_part_wall;
        forces_one_time(:,3)=A_i*exp((r_ij/2-d_ib)/B_i).*n_ib.*lambda_part_wall;
    
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
            f_ppl_of_p1(~not_at_goal_double)=0;
            %f_w(2*p1+[-1,0])=f_wall_of_p1;
            
            %body forces
            f_k_part=k*max(0,r_ij-d_ij_double).*n_ij;
            f_k_part(~not_at_goal_double)=0;
            %if we dont want to add the body forces to the soc model but work with
            %it separately:
            %f_k(2*p1+[-1,0])=[sum(f_k_part(1:2:2*(num_of_ppl-1))),sum(f_k_part(2:2:2*(num_of_ppl-1)))];
            %soc force
            f_soc(2*p1+[-1,0])=f_soc(2*p1+[-1,0])+[sum(f_ppl_of_p1(1:2:2*(num_of_ppl-1))),sum(f_ppl_of_p1(2:2:2*(num_of_ppl-1)))]+[sum(f_k_part(1:2:2*(num_of_ppl-1))),sum(f_k_part(2:2:2*(num_of_ppl-1)))];
        end
    
        forces_one_time(:,2)=f_soc;
    end
    
    
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
            min_wall_coords=room.wall_coords([min_lincoords,1],:)';
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
    
    function sim_graph_objects=simple_plot(y,h,forces,ppl_goal,num_of_ppl,r_ij,room,uiax,newfigure_logical)
        sim_graph_objects=[];
        if newfigure_logical==1
            figure;
            grid minor;
            hold on;
            xlim([-10.5,10.5])
            ylim([-10.5,10.5])
            ax=gca;
            %plot walls - maybe make it nicer later
            for ind_plot=1:1:size(room.walls,1)
                sim_graph_objects(end+1)=plot(ax,[room.walls(ind_plot,1),room.walls(ind_plot,3)],[room.walls(ind_plot,2),room.walls(ind_plot,4)],'-ok','LineWidth',2,'MarkerFaceColor','k','MarkerSize',10);
            end
        else
            ax=uiax;
        end
        
        
        a1= plot(ax,y(1:end,1:2:size(y,2)/2),y(1:end,2:2:size(y,2)/2),'LineWidth',1.5);%people coords
    
        sim_graph_objects(end+1:end+num_of_ppl)=a1;
        
        %plot circles
        plot_step=20;
        y_tmp=[reshape(y(1:plot_step:end,1:2:size(y,2)/2),[],1),reshape(y(1:plot_step:end,2:2:size(y,2)/2),[],1)];
        %sim_graph_objects(end+1)=viscircles(ax,y_tmp,r_ij/2*ones(1,size(y_tmp,1)),'Color','m');
        %plot acceleration vectors
        %{
        y_vel_tmp=[reshape(y(1:50:end,size(y,2)/2+1:2:size(y,2)),[],1),reshape(y(1:50:end,size(y,2)/2+2:2:size(y,2)),[],1)];
        sim_graph_objects(end+1)=quiver(ax,y_tmp(:,1),y_tmp(:,2),y_vel_tmp(:,1),y_vel_tmp(:,2),'Color','r','LineWidth',1.6);
        %}
    
        %plot force vectors
        %f_dir, goal
        %{
        sim_graph_objects(end+1)=dir_force_tmp=[reshape(forces(1:50:end,1:2:size(y,2)/2,1),[],1),reshape(forces(1:50:end,2:2:size(y,2)/2,1),[],1)];
        sim_graph_objects(end+1)=quiver(ax,y_tmp(:,1),y_tmp(:,2),dir_force_tmp(:,1)+2*1/100*y_vel_tmp(:,1),dir_force_tmp(:,2)+2*1/100*y_vel_tmp(:,2),'Color','k','LineWidth',1.6);
        sim_graph_objects(end+1)=quiver(ax,y_tmp(:,1),y_tmp(:,2),dir_force_tmp(:,1),dir_force_tmp(:,2),'Color','g','LineWidth',1.6);
        %}
        %f_wall
        wall_force_tmp=[reshape(forces(1:plot_step:end,1:2:size(y,2)/2,3),[],1),reshape(forces(1:plot_step:end,2:2:size(y,2)/2,3),[],1)];
        sim_graph_objects(end+1)=quiver(ax,y_tmp(:,1),y_tmp(:,2),1/h*wall_force_tmp(:,1),1/h*wall_force_tmp(:,2),'Color','g','LineWidth',1.6,'AutoScaleFactor',0.2);
        %f_soc
        soc_force_tmp=[reshape(forces(1:plot_step:end,1:2:size(y,2)/2,2),[],1),reshape(forces(1:plot_step:end,2:2:size(y,2)/2,2),[],1)];
        sim_graph_objects(end+1)=quiver(ax,y_tmp(:,1),y_tmp(:,2),1/h*soc_force_tmp(:,1),1/h*soc_force_tmp(:,2),'Color','r','LineWidth',1.6,'AutoScaleFactor',0.2);
    
        %plot goals 
        sim_graph_objects(end+1)=plot(ax,ppl_goal(1:2:2*num_of_ppl),ppl_goal(2:2:2*num_of_ppl),'s','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6]);
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
end
