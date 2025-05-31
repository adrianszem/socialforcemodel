
fig=uifigure("Name", "Make Room");

%global globvar;
%globvar=[];
m = uimenu(fig,'Text','&File');
m_save = uimenu(m,'Text','&Save');
m_load = uimenu(m,'Text','&Load');
m_load.Accelerator = 'L';
m_save.Accelerator = 'S';

m = uimenu(fig,'Text','&Edit');
m_run = uimenu(m,'Text','&Run');
m_run_movie=uimenu(m,'Text','&Run movie');
m_delete_config = uimenu(m,'Text','&Delete config');
m_delete_sim = uimenu(m,'Text','&Delete simulation');
m_delete_all = uimenu(m,"Text",'&Delete all');

m_delete_config.Accelerator = 'D';
m_run.Accelerator = 'R';
m_run_movie.Accelerator = 'T';
m_delete_sim.Accelerator = 'Q';
m_delete_all.Accelerator = 'A';

g1=uigridlayout(fig,[5,3]);
g1.RowHeight={'1x',35,35,35,35};
g1.ColumnWidth={'1x','1x','fit'};

uiax = uiaxes(g1,"HitTest","off");
handle_init(fig);

btn_save=uibutton(g1,"Text",["Save as: _AddText_.mat",".mat"],"Tooltip", "If no persons are placed, it does not save an empty mtx, by the logic that in this case, one would probably use random distribution of persons");
editfld_save=uieditfield(g1);

btn_add_person=uibutton(g1,"state","Text",["Add person, velocity and then goal"],"Tooltip","");
btn_add_walls=uibutton(g1,"state","Text",["Add walls"],"Tooltip","");

btn_delete_person=uibutton(g1,"state","Text",["Delete person (or wall)"],"Tooltip","");

btn_add_people_rectangle=uibutton(g1,"state","Text",["Add _N_ people", "in the clicked rectangle"],"Tooltip","");
editfld_num_off_ppl_to_add=uieditfield(g1);

btn_del_all=uibutton(g1,"Text",["Delete all"]);

btn_hit_helper=uibutton(g1,"state","Text",["Marker hit helper"],"Tooltip","Only works for wall end points and goal points. If turned on the click doesnt get registered until one clicks on a marker");

btn_load=uibutton(g1,"Text",["Load as: _AddText_.mat",".mat"],"Tooltip"," ");
editfld_load=uieditfield(g1);

btn_run=uibutton(g1,"Text","Run sim");


uiax.Layout.Row=1;
uiax.Layout.Column=[1,3];
btn_save.Layout.Row=4;
btn_save.Layout.Column=2;
editfld_save.Layout.Row=4;
editfld_save.Layout.Column=3;

btn_add_person.Layout.Row=2;
btn_add_person.Layout.Column=2;
btn_add_walls.Layout.Row=2;
btn_add_walls.Layout.Column=3;


btn_add_people_rectangle.Layout.Row=3;
btn_add_people_rectangle.Layout.Column=2;
editfld_num_off_ppl_to_add.Layout.Row=3;
editfld_num_off_ppl_to_add.Layout.Column=3;

btn_del_all.Layout.Row=2;
btn_del_all.Layout.Column=1;

btn_hit_helper.Layout.Row=3;
btn_hit_helper.Layout.Column=1;

btn_load.Layout.Row=5;
btn_load.Layout.Column=2;

editfld_load.Layout.Row=5;
editfld_load.Layout.Column=3;

btn_delete_person.Layout.Row=4;
btn_delete_person.Layout.Column=1;

btn_run.Layout.Row=5;
btn_run.Layout.Column=1;

xlim(uiax,[-10,10])
ylim(uiax,[-10,10])
hold(uiax,'on');
grid(uiax,'minor')

btn_add_person.ValueChangedFcn={@GeneratePeopleAndGoals_OnOff,uiax,btn_add_walls,btn_add_people_rectangle,btn_delete_person,fig};
btn_add_walls.ValueChangedFcn={@GenerateWalls_OnOff,uiax,btn_add_person,btn_add_people_rectangle,btn_delete_person,fig};
btn_add_people_rectangle.ValueChangedFcn={@GeneratePeoplesInRectangle_OnOff,uiax,editfld_num_off_ppl_to_add,btn_add_person,btn_add_walls,btn_delete_person,fig};
btn_delete_person.ValueChangedFcn={@DeletePerson_OnOff,uiax,btn_add_person,btn_add_walls,btn_add_people_rectangle,fig};

uiax.ButtonDownFcn={@mouseCallback,fig,btn_add_person,btn_add_walls,btn_add_people_rectangle,btn_hit_helper,btn_delete_person,editfld_num_off_ppl_to_add};
btn_save.ButtonPushedFcn={@saveData,editfld_save,fig};
btn_del_all.ButtonPushedFcn={@delConfig,fig};
btn_run.ButtonPushedFcn={@runSim,uiax,fig,0};

btn_load.ButtonPushedFcn={@loadData,editfld_load,fig,uiax};

m_load.MenuSelectedFcn = {@loadMenuSelected,fig,uiax};
m_save.MenuSelectedFcn = {@saveMenuSelected,fig};
m_run.MenuSelectedFcn = {@runSim,uiax,fig,0};
m_run_movie.MenuSelectedFcn={@runSim,uiax,fig,1};
m_delete_config.MenuSelectedFcn={@delConfig,fig};
m_delete_sim.MenuSelectedFcn={@delSim,uiax,fig};
m_delete_all.MenuSelectedFcn={@delAll,fig};

function mouseCallback(src, evnt,fig,person_button,wall_button,rect_button,marker_hit_helper_button,del_person_button,nmbr_of_rand_ppl)
    xyz = get(src, 'CurrentPoint');
    
    x = xyz(1,1);
    y = xyz(1,2);
    %disp([x,y])
    data=guidata(src);
    if person_button.Value==1
        if data.person_or_goal==0 %person coords
            data.ppl_graph_objects(end+1,1:5)=plot(src,x,y,'*k','ButtonDownFcn',{@MouseClickPerson,fig});
            data.ppl_graph_objects(end,3)=appviscircles(src,[x,y],0.25,'Color',"red");
            
            data.person_coords=[data.person_coords,x,y];
            data.person_or_goal=1;
            guidata(src,data);

        elseif data.person_or_goal==1 %velocity coords
            data.vel_coords=[data.vel_coords,x-data.person_coords(end-1),y-data.person_coords(end)];
            data.ppl_graph_objects(end,4)=quiver(src,data.person_coords(end-1),data.person_coords(end),data.vel_coords(end-1),data.vel_coords(end),'Color','g','LineWidth',1.6,'HitTest','off');
            data.person_or_goal=2;
            guidata(src,data);
        else %i.e. data.person_or_goal==2, goal coords
            if marker_hit_helper_button.Value==1 %in this case we do not click on the marker but on somewhere in the axis
                disp("marker not hit");          %thus an empty if is enought, data.person_or_goal stays 2
               
            else
            data.ppl_graph_objects(end,2)=plot(src,x,y,'*b',"MarkerSize",10,'ButtonDownFcn',{@MouseClickGoal,fig,nmbr_of_rand_ppl});
            data.goal_coords=[data.goal_coords,x,y];
            data.ppl_graph_objects(end,5)=line(src,[data.person_coords(end-1),data.goal_coords(end-1)],[data.person_coords(end),data.goal_coords(end)],'Color','magenta','LineStyle','--','HitTest','off');
            data.person_or_goal=0;
            guidata(src,data);
            end
        end
    elseif wall_button.Value==1
        if data.walls==0
            if marker_hit_helper_button.Value==1 %in this case we do not click on the marker but on somewhere in the axis
                disp("marker not hit");          %thus an empty if is enought
               
            else
            data.wall_graph_objects(end+1,1:3)=plot(src,x,y,'ok',"MarkerSize",10,'MarkerFaceColor','k','Tag',num2str(data.wall_tagger_ind),'ButtonDownFcn',{@MouseClickWall,fig});% a bit surprisingly, if i give ,wall_button.Value as the fnc input for the mouseclickwall fnc
                                                                                                                                                                         % it always gives back '1'
            data.wall_coords=[data.wall_coords;x,y,0,0];
            data.walls=1;
            guidata(src,data);
            end
        else
            if marker_hit_helper_button.Value==1 %in this case we do not click on the marker but on somewhere in the axis
                disp("marker not hit");          %thus an empty if is enought
               
            else
            data.wall_graph_objects(end,2)=plot(src,x,y,'ok',"MarkerSize",10,'MarkerFaceColor','k','Tag',num2str(data.wall_tagger_ind),'ButtonDownFcn',{@MouseClickWall,fig});
            data.wall_coords(end,3:4)=[x,y];
            data.wall_graph_objects(end,3)=line(src,[data.wall_coords(end,1),data.wall_coords(end,3)],[data.wall_coords(end,2),data.wall_coords(end,4)],'Color','black','LineWidth',2,'Tag',num2str(data.wall_tagger_ind),'HitTest','off');%@(h,e) disp(e.IntersectionPoint));
            data.wall_tagger_ind=data.wall_tagger_ind+1;
            data.walls=0;
            guidata(src,data);
            end
        end
    elseif rect_button.Value==1 %first point of rectangle

        if data.rect==0
            data.rect_coords(1:2)=[x,y];
            data.rect_point=plot(src,x,y,"og");
            data.rect=1;
            guidata(src,data);
        elseif data.rect==1 %second point of rectangle
            data.rect_coords(3:4)=[x,y];
            h=abs(data.rect_coords(1)-data.rect_coords(3));
            w=abs(data.rect_coords(2)-data.rect_coords(4));
            center_point=data.rect_coords(1,1:2)+1/2*(data.rect_coords(1,3:4)-data.rect_coords(1,1:2));
            rects=rectangle2(src,[center_point,h,w],'LineStyle','-.','HitTest','off');
            data.rectangles(end+1)=rects;
            
            new_ppl_coords=[(rand(1,str2num(nmbr_of_rand_ppl.Value))-1/2)*h;(rand(1,str2num(nmbr_of_rand_ppl.Value))-1/2)*w]+center_point';
            new_pplc_coords2=reshape(new_ppl_coords,1,[]);
            data.ppl_graph_objects(end+1,1:5)=plot(src,new_pplc_coords2(1:2:end),new_pplc_coords2(2:2:end),'*k','HitTest','off');%ppl_coords
            data.ppl_graph_objects(end,3)=appviscircles(src,new_ppl_coords',0.25*ones(1,str2num(nmbr_of_rand_ppl.Value)),'Color',"red");
            data.person_coords=[data.person_coords,new_pplc_coords2];
            data.vel_coords=[data.vel_coords,rand(1,2*str2num(nmbr_of_rand_ppl.Value))-1/2];
            data.rect=2;
            guidata(src,data);
        else %if data.rect==2 %goals (they only can have the same goals)
            if marker_hit_helper_button.Value==1 %in this case we do not click on the marker but on somewhere in the axis
                disp("marker not hit");          %thus an empty if is enought  
            else
            data.ppl_graph_objects(end,2)=plot(src,x,y,'*b','MarkerSize',10,'ButtonDownFcn',{@MouseClickGoal,fig,nmbr_of_rand_ppl});
            data.goal_coords=[data.goal_coords,repmat([x,y],1,str2num(nmbr_of_rand_ppl.Value))];
            tmp_plot_x=zeros(1,2*str2num(nmbr_of_rand_ppl.Value));
            tmp_plot_y=zeros(1,2*str2num(nmbr_of_rand_ppl.Value));
            tmp_plot_x(1:2:end)=data.person_coords(end-2*str2num(nmbr_of_rand_ppl.Value)+1:2:end);%x init coords
            tmp_plot_x(2:2:end)=x;%x goal coords
            tmp_plot_y(1:2:end)=data.person_coords(end-2*str2num(nmbr_of_rand_ppl.Value)+2:2:end);%y init coords
            tmp_plot_y(2:2:end)=y;%y goal koords
            data.ppl_graph_objects(end,5)=line(src,tmp_plot_x,tmp_plot_y,'Color','magenta','LineStyle','--','HitTest','off');
            delete(data.rect_point)
            data.rect_point=[];
            data.rect=0;
            
            guidata(src,data);
            end
        end
    %elseif data.del_person==1
       %here i'm not clicking on the uiax, but the marker to delete so the
       %action has to be in the MouseClickGoal function
    end
    
end

function GeneratePeoplesInRectangle_OnOff(src,event,uiax,editfld,otherstatebutton1,otherstatebutton2,otherstatebutton3,fig)
    if isempty(editfld.Value)
        uialert(fig,"First add the number of ppl!","Eyy!")
        src.Value=0;
        return;
    elseif  otherstatebutton1.Value==1 || otherstatebutton2.Value==1 || otherstatebutton3.Value==1
        uialert(fig,"Turn off the other buttons first","Eyy!")
        src.Value=0;
        return;
    end

    data=guidata(src);

    if src.Value==1
        uiax.HitTest=1;
        data.rect_button_val=1;
    else
        %delete the rectangles
        h=data.rectangles;
        delete(h);
        data.rect_coords=[];
        data.rectangles=[];
        data.rect_button_val=0;
        uiax.HitTest=0;
    end
    guidata(src,data);
end

function GeneratePeopleAndGoals_OnOff(src,event,uiax,otherstatebutton1,otherstatebutton2,otherstatebutton3,fig)
    if otherstatebutton1.Value==1 ||  otherstatebutton2.Value==1 || otherstatebutton3.Value==1
        uialert(fig,"Turn off the other buttons first","Eyy!")
        src.Value=0;
        return;
    end

    data=guidata(src);

    if src.Value==1
        uiax.HitTest=1;
        data.person_button_val=1;
    else
        if data.person_or_goal==1 || data.person_or_goal==2
            uialert(fig,"Choose a goal to the person first","Eyy!")
            src.Value=0;
            return;   
        end
        data.person_button_val=0;
        uiax.HitTest=0;
    end
    guidata(src,data);
end

function GenerateWalls_OnOff(src,event,uiax,otherstatebutton1,otherstatebutton2,otherstatebutton3,fig)

    if otherstatebutton1.Value==1 ||  otherstatebutton2.Value==1 || otherstatebutton3.Value==1
        uialert(fig,"Turn off the other buttons first","Eyy!")
        src.Value=0;
        return;
    end
    data=guidata(src);

    if src.Value==1
        uiax.HitTest=1;
        data.wall_button_val=1;
    else
        if data.walls==1
            uialert(fig,"Choose the end point of the wall first","Eyy!")
            src.Value=1;
            return;
            
        end
        data.wall_button_val=0;
        uiax.HitTest=0;
    end
    guidata(src,data);
end

function DeletePerson_OnOff(src,event,uiax,otherstatebutton1,otherstatebutton2,otherstatebutton3,fig)

    if otherstatebutton1.Value==1 ||  otherstatebutton2.Value==1 || otherstatebutton3.Value==1
        uialert(fig,"Turn off the other buttons first","Eyy!")
        src.Value=0;
        return;
    end

    if src.Value==1
        uiax.HitTest=1;
        data=guidata(src);
        data.del_person=1;
        guidata(src,data);
    else     
        uiax.HitTest=0;
        data=guidata(src);
        data.del_person=0;
        guidata(src,data);
    end
end

function loadData(src,event,editfld,fig,uiax)
    if isempty(editfld.Value)
        uialert(fig,"The data can not be loaded. Please Add a File Name!","No filename Added")
        return;
    end
    %check if the data under the given name exists
    if isfile(strcat(editfld.Value,'.mat'))==0
        uialert(fig,"There is no existing file with the given name!","Not a filename")
        return;
    end
    load(strcat(editfld.Value,'.mat')); %loads room_config_datas struct
    %room_config_datas=data_to_save;
    %check if the loaded data has the needed fields
    if sum(isfield(room_config_datas,{'person_coords','vel_coords','goal_coords','wall_coords'}))~=4
        uialert(fig,"The given file does not have a needed structure","Something wrong")
        return;
    end
    % we delete the existing data
    delConfig(src,event,fig);
    data=guidata(src); %we want to update everything tho, so no real
    %need for this, nevermind, we need it because the loaded data is
    %missing multiple fields

    %add wall elements - later we want to have the "lines" of a wall
    %independently as graphical objects, so I use for cycles
    data.wall_coords=room_config_datas.wall_coords; %no need update this also line byy line in a for cycle
    data.person_coords=room_config_datas.person_coords;
    data.vel_coords=room_config_datas.vel_coords;
    data.goal_coords=room_config_datas.goal_coords;

    for ind_wall=1:size(data.wall_coords,1)
        %it would be nicer to put this to an addWall function and
        %possibly use this function when we make the wall by clicking,
        %but the problems are (i) we want to show the start point marker
        %before choosing the end point and (ii) i cannot get the
        %handler once outside the function and once inside and update
        %it first inside and then outside 
         data.wall_graph_objects(end+1,1:3)=plot(uiax,data.wall_coords(ind_wall,1),data.wall_coords(ind_wall,2),'ok',"MarkerSize",10,'MarkerFaceColor','k','Tag',num2str(data.wall_tagger_ind),'ButtonDownFcn',{@MouseClickWall,fig});
         data.wall_graph_objects(end,2)=plot(uiax,data.wall_coords(ind_wall,3),data.wall_coords(ind_wall,4),'ok',"MarkerSize",10,'MarkerFaceColor','k','Tag',num2str(data.wall_tagger_ind),'ButtonDownFcn',{@MouseClickWall,fig});
         data.wall_graph_objects(end,3)=line(uiax,[data.wall_coords(ind_wall,1),data.wall_coords(ind_wall,3)],[data.wall_coords(ind_wall,2),data.wall_coords(ind_wall,4)],'Color','black','LineWidth',2,'Tag',num2str(data.wall_tagger_ind),'HitTest','off');
         data.wall_tagger_ind=data.wall_tagger_ind+1;
         %guidata(src,data);
    end

    %add people one-by-one so they are independent graphical objects
    for ind_ppl=1:2:size(data.person_coords,2)
        person_indices=ind_ppl+[0,1];
        data.ppl_graph_objects(end+1,1)=plot(uiax,data.person_coords(person_indices(1)),data.person_coords(person_indices(2)),'*k','ButtonDownFcn',{@MouseClickPerson,fig});
        data.ppl_graph_objects(end,3)=appviscircles(uiax,data.person_coords(person_indices),0.25,'Color',"red");
        data.ppl_graph_objects(end,4)=quiver(uiax,data.person_coords(person_indices(1)),data.person_coords(person_indices(2)),data.vel_coords(person_indices(1)),data.vel_coords(person_indices(2)),'Color','g','LineWidth',1.6,'HitTest','off');
        %later, i will probably shouldn't make a new marker for a goal, if that
        %goal already exists
        data.ppl_graph_objects(end,2)=plot(uiax,data.goal_coords(person_indices(1)),data.goal_coords(person_indices(2)),'*b',"MarkerSize",10,'Tag',num2str(data.wall_tagger_ind),'ButtonDownFcn',{@MouseClickGoal,fig});
        data.ppl_graph_objects(end,5)=line(uiax,[data.person_coords(person_indices(1)),data.goal_coords(person_indices(1))],[data.person_coords(person_indices(2)),data.goal_coords(person_indices(2))],'Color','magenta','LineStyle','--','HitTest','off');
    end
    guidata(src,data);
end

function loadMenuSelected(src,event,fig,uiax)
    file_name = uigetfile('*.mat');
    load(file_name); %loads room_config_datas struct
    %room_config_datas=data_to_save;
    %check if the loaded data has the needed fields
    if sum(isfield(room_config_datas,{'person_coords','vel_coords','goal_coords','wall_coords'}))~=4
        uialert(fig,"The given file does not have a needed structure","Something wrong")
        return;
    end
    % we delete the existing data
    delConfig(src,event,fig);
    data=guidata(src); %we want to update everything tho, so no real
    %need for this, nevermind, we need it because the loaded data is
    %missing multiple fields

    %add wall elements - later we want to have the "lines" of a wall
    %independently as graphical objects, so I use for cycles
    data.wall_coords=room_config_datas.wall_coords; %no need update this also line byy line in a for cycle
    data.person_coords=room_config_datas.person_coords;
    data.vel_coords=room_config_datas.vel_coords;
    data.goal_coords=room_config_datas.goal_coords;

    for ind_wall=1:size(data.wall_coords,1)
        %it would be nicer to put this to an addWall function and
        %possibly use this function when we make the wall by clicking,
        %but the problems are (i) we want to show the start point marker
        %before choosing the end point and (ii) i cannot get the
        %handler once outside the function and once inside and update
        %it first inside and then outside 
         data.wall_graph_objects(end+1,1:3)=plot(uiax,data.wall_coords(ind_wall,1),data.wall_coords(ind_wall,2),'ok',"MarkerSize",10,'MarkerFaceColor','k','Tag',num2str(data.wall_tagger_ind),'ButtonDownFcn',{@MouseClickWall,fig});
         data.wall_graph_objects(end,2)=plot(uiax,data.wall_coords(ind_wall,3),data.wall_coords(ind_wall,4),'ok',"MarkerSize",10,'MarkerFaceColor','k','Tag',num2str(data.wall_tagger_ind),'ButtonDownFcn',{@MouseClickWall,fig});
         data.wall_graph_objects(end,3)=line(uiax,[data.wall_coords(ind_wall,1),data.wall_coords(ind_wall,3)],[data.wall_coords(ind_wall,2),data.wall_coords(ind_wall,4)],'Color','black','LineWidth',2,'Tag',num2str(data.wall_tagger_ind),'HitTest','off');
         data.wall_tagger_ind=data.wall_tagger_ind+1;
         %guidata(src,data);
    end

    %add people one-by-one so they are independent graphical objects
    for ind_ppl=1:2:size(data.person_coords,2)
        person_indices=ind_ppl+[0,1];
        data.ppl_graph_objects(end+1,1)=plot(uiax,data.person_coords(person_indices(1)),data.person_coords(person_indices(2)),'*k','ButtonDownFcn',{@MouseClickPerson,fig});
        data.ppl_graph_objects(end,3)=appviscircles(uiax,data.person_coords(person_indices),0.25,'Color',"red");
        data.ppl_graph_objects(end,4)=quiver(uiax,data.person_coords(person_indices(1)),data.person_coords(person_indices(2)),data.vel_coords(person_indices(1)),data.vel_coords(person_indices(2)),'Color','g','LineWidth',1.6,'HitTest','off');
        %later, i will probably shouldn't make a new marker for a goal, if that
        %goal already exists
        data.ppl_graph_objects(end,2)=plot(uiax,data.goal_coords(person_indices(1)),data.goal_coords(person_indices(2)),'*b',"MarkerSize",10,'Tag',num2str(data.wall_tagger_ind),'ButtonDownFcn',{@MouseClickGoal,fig});
        data.ppl_graph_objects(end,5)=line(uiax,[data.person_coords(person_indices(1)),data.goal_coords(person_indices(1))],[data.person_coords(person_indices(2)),data.goal_coords(person_indices(2))],'Color','magenta','LineStyle','--','HitTest','off');
    end
    guidata(src,data);

    
end

function saveData(src,event,editfld,fig)
    data=guidata(src);
    %floor_field=mtx;
    if isempty(editfld.Value)
        uialert(fig,"The data can not be saved. Please Add a File Name!","No filename Added")
    else
        if isempty(data.goal_coords)==1 %no walls is ok rn
            uialert(fig,"The data can not be saved, since it is empty","no values to save")
        elseif data.person_or_goal==1 || data.person_or_goal==2
            uialert(fig,"Choose a goal to the person first","Eyy!")
            return;
        elseif data.walls==1
            uialert(fig,"Choose the endpoint of the wall first","Eyy!")
            return;
        else 
            room_config_datas=rmfield(data,{'person_or_goal','walls','rect','rect_coords','rectangles','rect_point','wall_graph_objects','ppl_graph_objects','wall_tagger_ind','person_button_val','rect_button_val','wall_button_val','del_person','sim_graph_objects'});
            save(editfld.Value,'room_config_datas'); %we save the people independently
        end
    end
 end
% % % 
function saveMenuSelected(src,event,fig)
    file_name = uiputfile('*.mat');
    if file_name~=0
        data=guidata(src);
        if isempty(data.goal_coords)==1 %no walls is ok rn
            uialert(fig,"The data can not be saved, since it is empty","no values to save")
        elseif data.person_or_goal==1 || data.person_or_goal==2
            uialert(fig,"Choose a goal to the person first","Eyy!")
            return;
        elseif data.walls==1
            uialert(fig,"Choose the endpoint of the wall first","Eyy!")
            return;
        else 
            room_config_datas=rmfield(data,{'person_or_goal','walls','rect','rect_coords','rectangles','rect_point','wall_graph_objects','ppl_graph_objects','wall_tagger_ind','person_button_val','rect_button_val','wall_button_val','del_person','sim_graph_objects'});
            save(file_name,'room_config_datas'); %we save the people independently
        end
    end
end

function delConfig(src,event,fig)
    data=guidata(src);
    delete(data.wall_graph_objects);
    delete(data.ppl_graph_objects)
    delete(data.rectangles);
    handle_init(fig);
end

function delSim(src,event,uiax,fig)
    data=guidata(src);
    disp('ahaa')
    delete(data.sim_graph_objects);
    data.sim_graph_objects=[];
    %delsim is also called before runSim (and in del all), but we only want to plot (again) the init
    %values when we specifically press on the delete simulation button
    if nargin==4
        for ind_ppl=1:2:size(data.person_coords,2)
            person_indices=ind_ppl+[0,1];
            data.ppl_graph_objects(end+1,1)=plot(uiax,data.person_coords(person_indices(1)),data.person_coords(person_indices(2)),'*k','ButtonDownFcn',{@MouseClickPerson,fig});
            data.ppl_graph_objects(end,3)=appviscircles(uiax,data.person_coords(person_indices),0.25,'Color',"red");
            data.ppl_graph_objects(end,4)=quiver(uiax,data.person_coords(person_indices(1)),data.person_coords(person_indices(2)),data.vel_coords(person_indices(1)),data.vel_coords(person_indices(2)),'Color','g','LineWidth',1.6,'HitTest','off');
            %later, i will probably shouldn't make a new marker for a goal, if that
            %goal already exists
            data.ppl_graph_objects(end,2)=plot(uiax,data.goal_coords(person_indices(1)),data.goal_coords(person_indices(2)),'*b',"MarkerSize",10,'Tag',num2str(data.wall_tagger_ind),'ButtonDownFcn',{@MouseClickGoal,fig});
            data.ppl_graph_objects(end,5)=line(uiax,[data.person_coords(person_indices(1)),data.goal_coords(person_indices(1))],[data.person_coords(person_indices(2)),data.goal_coords(person_indices(2))],'Color','magenta','LineStyle','--','HitTest','off');
        end
    end
    %}
    guidata(src,data);
end

function delAll(src,event,fig)
    delSim(src,event);
    delConfig(src,event,fig);
end

function handle_init(fig)
    data.person_or_goal=0;
    data.walls=0;
    data.wall_button_val=0;% a bit surprisingly, if i give ,wall_button.Value as the fnc input for the mouseclickwall fnc
                           % it always gives back '1' the value of the
                           % initialization, so thats why i need the value
                           % also as a handle
    data.person_button_val=0;
    data.rect_button_val=0;
    data.rect=0;
    data.del_person=0;
    data.wall_tagger_ind=1;
    %data.nmbr_of_rand_ppl=0;
    
    data.person_coords=[];
    data.vel_coords=[];
    data.goal_coords=[];
    data.wall_coords=[];
    data.rect_coords=[];
    %data.wall_tags=[];
    
    %we save the rectangles (graphical objects) to delete them when we turn of the generator state
    %button
    data.rectangles=[];
    data.rect_point=[];
    data.wall_graph_objects=[];%since we want to delete them possibly
    data.ppl_graph_objects=[];
    data.sim_graph_objects=[];
    
    guidata(fig,data);
end

function MouseClickWall(src,event,fig)
    disp("wall marker hit");
   
    %setGlobala1(event.Source)
    disp(event.IntersectionPoint)
    data=guidata(src);
    
    if data.wall_button_val==1 && data.walls==0
        data.wall_coords=[data.wall_coords;event.IntersectionPoint(1),event.IntersectionPoint(2),0,0];
        data.walls=1;
        %data.wall_graph_objects(end+1,1:3)=event.Source;
        data.wall_graph_objects(end+1,1:3)=plot(src.Parent,event.IntersectionPoint(1),event.IntersectionPoint(2),'ok',"MarkerSize",10,'MarkerFaceColor','k','Tag',num2str(data.wall_tagger_ind),'ButtonDownFcn',{@MouseClickWall,fig});
        guidata(src,data);
    elseif data.wall_button_val==1 && data.walls==1
        data.wall_coords(end,3:4)=[event.IntersectionPoint(1),event.IntersectionPoint(2)];
        %data.wall_graph_objects(end,2)=event.Source;
        data.wall_graph_objects(end,2)=plot(src.Parent,event.IntersectionPoint(1),event.IntersectionPoint(2),'ok',"MarkerSize",10,'MarkerFaceColor','k','Tag',num2str(data.wall_tagger_ind),'ButtonDownFcn',{@MouseClickWall,fig});

        data.wall_graph_objects(end,3)=line(src.Parent,[data.wall_coords(end,1),data.wall_coords(end,3)],[data.wall_coords(end,2),data.wall_coords(end,4)],'Color','black','LineWidth',2,'Tag',num2str(data.wall_tagger_ind),'HitTest','off');%@(h,e) disp(e.IntersectionPoint));
        data.wall_tagger_ind=data.wall_tagger_ind+1;
        data.walls=0;
        guidata(src,data);
    elseif data.wall_button_val==0 && data.del_person==1
        ind_to_del=(sum(data.wall_graph_objects==event.Source,2)==1);
        sum(data.wall_graph_objects==event.Source,2)
        delete(data.wall_graph_objects(ind_to_del,:));
        data.wall_graph_objects=data.wall_graph_objects(~(ind_to_del~=0),:);
        data.wall_coords=data.wall_coords(~(ind_to_del~=0),:);
        guidata(fig,data);
    end
end

function MouseClickGoal(src,event,fig,nmbr_of_rand_ppl)
    disp("goal marker hit");
    data=guidata(src);
    if data.person_button_val==1 && data.person_or_goal==2
        disp(event.IntersectionPoint)
        data.goal_coords=[data.goal_coords,event.Source.XData(1),event.Source.YData(1)];
        data.ppl_graph_objects(end,2)=plot(src.Parent,event.Source.XData(1),event.Source.YData(1),'*b',"MarkerSize",10,'ButtonDownFcn',{@MouseClickGoal,fig,nmbr_of_rand_ppl});
        data.person_or_goal=0;
        data.ppl_graph_objects(end,5)=line(src.Parent,[data.person_coords(end-1),data.goal_coords(end-1)],[data.person_coords(end),data.goal_coords(end)],'Color','magenta','LineStyle','--','HitTest','off');
        guidata(src,data);
    elseif data.rect_button_val==1 && data.rect==2
        disp(event.IntersectionPoint)
        
        disp(str2num(nmbr_of_rand_ppl.Value))
        data.goal_coords=[data.goal_coords,repmat([event.Source.XData(1),event.Source.YData(1)],1,str2num(nmbr_of_rand_ppl.Value))];
        tmp_plot_x=zeros(1,2*str2num(nmbr_of_rand_ppl.Value));
        tmp_plot_y=zeros(1,2*str2num(nmbr_of_rand_ppl.Value));
        tmp_plot_x(1:2:end)=data.person_coords(end-2*str2num(nmbr_of_rand_ppl.Value)+1:2:end);%x init coords
        tmp_plot_x(2:2:end)=event.IntersectionPoint(1);%x goal coords
        tmp_plot_y(1:2:end)=data.person_coords(end-2*str2num(nmbr_of_rand_ppl.Value)+2:2:end);%y init coords
        tmp_plot_y(2:2:end)=event.IntersectionPoint(2);%y goal koords
        data.ppl_graph_objects(end,5)=line(src.Parent,tmp_plot_x,tmp_plot_y,'Color','magenta','LineStyle','--','HitTest','off');
        delete(data.rect_point)
        data.rect_point=[];
        data.rect=0;
        guidata(src,data);
    elseif data.rect_button_val==0 &&data.person_button_val==0 && data.del_person==1
        %{
        %delete the marker 
        ind_to_del=data.ppl_graph_objects(:,2)==event.Source
        delete(data.ppl_graph_objects(ind_to_del,:));
        %delete data
        ind_num=find(ind_to_del==1)
        data.ppl_graph_objects
        data.ppl_graph_objects=data.ppl_graph_objects(~(ind_to_del~=0),:);
        data.ppl_graph_objects
        [1:2*(ind_num-1),(2*ind_num)+1:(size(data.person_coords,2)/2)]
        data.person_coords=data.person_coords([1:2*(ind_num-1),(2*ind_num)+1:size(data.person_coords,2)]);
        data.vel_coords=data.vel_coords([1:2*(ind_num-1),(2*ind_num)+1:size(data.person_coords,2)]);
        data.goal_coords=data.goal_coords([1:2*(ind_num-1),(2*ind_num)+1:size(data.person_coords,2)]);
        guidata(fig,data);
        %}
        %
        indices=(data.goal_coords(1:2:end)==event.Source.XData(1)) & (data.goal_coords(2:2:end)==event.Source.YData(1));
        indices_double=zeros(1,2*length(indices));
        ind_nums=find(indices==1);
        indices_double(1:2:end)=indices;
        indices_double(2:2:end)=indices;
        
        delete(data.ppl_graph_objects(ind_nums,:));
        data.ppl_graph_objects=data.ppl_graph_objects(~indices,:);
        data.person_coords=data.person_coords(~indices_double);
        data.vel_coords=data.vel_coords(~indices_double);
        data.goal_coords=data.goal_coords(~indices_double);
        guidata(fig,data);
        %}
        
    end
end

function MouseClickPerson(src,event,fig)
    disp("goal marker hit");
    data=guidata(src);
    if  data.del_person==1
        %
        %delete the marker 
        %ind_to_del=data.ppl_graph_objects(:,1)==event.Source
        delete(data.ppl_graph_objects(ind_to_del,:));
        %delete data
        %ind_num=find(ind_to_del==1)
        %data.ppl_graph_objects
        data.ppl_graph_objects=data.ppl_graph_objects(~(ind_to_del~=0),:);
        %data.ppl_graph_objects
        %[1:2*(ind_num-1),(2*ind_num)+1:(size(data.person_coords,2)/2)]
        data.person_coords=data.person_coords([1:2*(ind_num-1),(2*ind_num)+1:size(data.person_coords,2)]);
        data.vel_coords=data.vel_coords([1:2*(ind_num-1),(2*ind_num)+1:size(data.person_coords,2)]);
        data.goal_coords=data.goal_coords([1:2*(ind_num-1),(2*ind_num)+1:size(data.person_coords,2)]);
        guidata(fig,data);
        %}
        %{
        
        indices=(data.person_coords(1:2:end)==event.Source.XData(1)) & (data.person_coords(2:2:end)==event.Source.YData(1));
        indices_double=zeros(1,2*length(indices));
        ind_nums=find(indices==1);
        indices_double(1:2:end)=indices;
        indices_double(2:2:end)=indices;
        
        delete(data.ppl_graph_objects(ind_nums,:));
        data.ppl_graph_objects=data.ppl_graph_objects(~indices,:);
        data.person_coords=data.person_coords(~indices_double);
        data.vel_coords=data.vel_coords(~indices_double);
        data.goal_coords=data.goal_coords(~indices_double);
        guidata(fig,data);
            
        %}
    end
end

function runSim(src,event,uiax,fig,run_as_movie_logical)
    data=guidata(src);
    if data.rect_button_val==1 || data.person_button_val==1 || data.wall_button_val==1
        uialert(fig,"Finish the building","Eyy!")
            return;
    elseif isempty(data.goal_coords)==1 %no walls is ok rn
        uialert(fig,"The data can not be simulated, since it is empty","no values to run")
        return;
    end
    delete(data.ppl_graph_objects)
    data.ppl_graph_objects=[];
    guidata(fig,data);
    %similarly as in saveData
    room_config_datas=rmfield(data,{'person_or_goal','walls','rect','rect_coords','rectangles','rect_point','wall_graph_objects','ppl_graph_objects','wall_tagger_ind','person_button_val','rect_button_val','wall_button_val','sim_graph_objects'});
    newfigure_logical=0;
    delSim(src,event);

    %data.sim_graph_objects=social_force_model_gui(room_config_datas,fig,uiax,newfigure_logical,run_as_movie_logical);
    data.sim_graph_objects=nosym_social_force_model_gui(room_config_datas,fig,uiax,newfigure_logical,run_as_movie_logical);
    guidata(src,data);
end

%{
function setGlobala1(val)
    global a1
    a1
    a1 = val;
end

function setGlobala2(val)
    global a2
    a2 = val;
end

function r = getGlobala1
    global x
    r = x;
end
%}
