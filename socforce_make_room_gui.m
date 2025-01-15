clear all
close all
clc

fig=uifigure("Name", "Make Room");

g1=uigridlayout(fig,[5,3]);
g1.RowHeight={'1x',35,35,35,35};
g1.ColumnWidth={'1x','1x','fit'};

uiax = uiaxes(g1,"HitTest","off");
handle_init(fig);

btn=uibutton(g1,"Text",["Save as: _AddText_.mat",".mat"],"Tooltip", "If no persons are placed, it does not save an empty mtx, by the logic that in this case, one would probably use random distribution of persons");
lbl=uilabel(g1,"Text",["something1,", "something2"]);
editfld=uieditfield(g1);

btn2=uibutton(g1,"state","Text",["Add person, velocity and then goal"],"Tooltip","");
lbl2=uilabel(g1,"Text"," ");
btn3=uibutton(g1,"state","Text",["Add walls"],"Tooltip","");

btn4=uibutton(g1,"state","Text",["Add _N_ people", "in the clicked rectangle"],"Tooltip","");
editfld4=uieditfield(g1);

btn_del_all=uibutton(g1,"Text",["Delete all"]);

btn5=uibutton(g1,"state","Text",["Marker hit helper"],"Tooltip","Only works for wall end points and goal points. If turned on the click doesnt get registered until one clicks on a marker");

btn_load=uibutton(g1,"Text",["Load as: _AddText_.mat",".mat"],"Tooltip"," ");
editfld_load=uieditfield(g1);



uiax.Layout.Row=1;
uiax.Layout.Column=[1,3];
btn.Layout.Row=4;
btn.Layout.Column=2;
editfld.Layout.Row=4;
editfld.Layout.Column=3;
lbl.Layout.Row=3;
lbl.Layout.Column=1;

btn2.Layout.Row=2;
btn2.Layout.Column=2;
btn3.Layout.Row=2;
btn3.Layout.Column=3;
lbl2.Layout.Row=2;
lbl2.Layout.Column=1;

btn4.Layout.Row=3;
btn4.Layout.Column=2;
editfld4.Layout.Row=3;
editfld4.Layout.Column=3;

btn_del_all.Layout.Row=2;
btn_del_all.Layout.Column=1;

btn5.Layout.Row=3;
btn5.Layout.Column=1;

btn_load.Layout.Row=5;
btn_load.Layout.Column=2;

editfld_load.Layout.Row=5;
editfld_load.Layout.Column=3;


xlim(uiax,[-10,10])
ylim(uiax,[-10,10])
hold(uiax,'on');
grid(uiax,'minor')


btn2.ValueChangedFcn={@GeneratePeopleAndGoals_OnOff,uiax,btn3,btn4,fig};
btn3.ValueChangedFcn={@GenerateWalls_OnOff,uiax,btn2,btn4,fig};
btn4.ValueChangedFcn={@GeneratePeoplesInRectangle_OnOff,uiax,editfld4,btn2,btn3,fig};

uiax.ButtonDownFcn={@mouseCallback,btn2,btn3,btn4,btn5,editfld4};
btn.ButtonPushedFcn={@saveData,editfld,fig};
btn_del_all.ButtonPushedFcn={@del_all,fig};

btn_load.ButtonPushedFcn={@loadData,editfld_load,fig,uiax};

function mouseCallback(src, evnt,person_button,wall_button,rect_button,marker_hit_helper_button,nmbr_of_rand_ppl)
    xyz = get(src, 'CurrentPoint');
    
    x = xyz(1,1);
    y = xyz(1,2);
    %disp([x,y])
    data=guidata(src);
    if person_button.Value==1
        if data.person_or_goal==0 %person coords
            data.other_graph_objects(end+1)=appviscircles(src,[x,y],0.25,'Color',"red");
            data.other_graph_objects(end+1)=plot(src,x,y,'*k','HitTest','off');
            
            data.person_coords=[data.person_coords,x,y];
            data.person_or_goal=1;
            guidata(src,data);

        elseif data.person_or_goal==1 %velocity coords
            data.vel_coords=[data.vel_coords,x-data.person_coords(end-1),y-data.person_coords(end)];
            data.other_graph_objects(end+1)=quiver(src,data.person_coords(end-1),data.person_coords(end),data.vel_coords(end-1),data.vel_coords(end),'Color','g','LineWidth',1.6,'HitTest','off');
            data.person_or_goal=2;
            guidata(src,data);
        else %i.e. data.person_or_goal==2, goal coords
            if marker_hit_helper_button.Value==1 %in this case we do not click on the marker but on somewhere in the axis
                disp("marker not hit");          %thus an empty if is enought, data.person_or_goal stays 2
               
            else
            data.other_graph_objects(end+1)=plot(src,x,y,'*b',"MarkerSize",10,'ButtonDownFcn',{@MouseClickGoal,nmbr_of_rand_ppl});
            data.goal_coords=[data.goal_coords,x,y];
            data.other_graph_objects(end+1)=line(src,[data.person_coords(end-1),data.goal_coords(end-1)],[data.person_coords(end),data.goal_coords(end)],'Color','magenta','LineStyle','--','HitTest','off');
            data.person_or_goal=0;
            guidata(src,data);
            end
        end
    elseif wall_button.Value==1
        if data.walls==0
            if marker_hit_helper_button.Value==1 %in this case we do not click on the marker but on somewhere in the axis
                disp("marker not hit");          %thus an empty if is enought
               
            else
            data.other_graph_objects(end+1)=plot(src,x,y,'ok',"MarkerSize",10,'MarkerFaceColor','k','ButtonDownFcn',@MouseClickWall);
            data.wall_coords=[data.wall_coords;x,y,0,0];
            data.walls=1;
            guidata(src,data);
            end
        else
            if marker_hit_helper_button.Value==1 %in this case we do not click on the marker but on somewhere in the axis
                disp("marker not hit");          %thus an empty if is enought
               
            else
            data.other_graph_objects(end+1)=plot(src,x,y,'ok',"MarkerSize",10,'MarkerFaceColor','k','ButtonDownFcn',@MouseClickWall);
            data.wall_coords(end,3:4)=[x,y];
            data.other_graph_objects(end+1)=line(src,[data.wall_coords(end,1),data.wall_coords(end,3)],[data.wall_coords(end,2),data.wall_coords(end,4)],'Color','black','LineWidth',2,'HitTest','off');%@(h,e) disp(e.IntersectionPoint));
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
            data.other_graph_objects(end+1)=appviscircles(src,new_ppl_coords',0.25*ones(1,str2num(nmbr_of_rand_ppl.Value)),'Color',"red");
            data.other_graph_objects(end+1)=plot(src,new_pplc_coords2(1:2:end),new_pplc_coords2(2:2:end),'*k','HitTest','off');%ppl_coords
            data.person_coords=[data.person_coords,new_pplc_coords2];
            data.vel_coords=[data.vel_coords,rand(1,2*str2num(nmbr_of_rand_ppl.Value))-1/2];
            data.rect=2;
            guidata(src,data);
        else %if data.rect==2 %goals (they only can have the same goals)
            if marker_hit_helper_button.Value==1 %in this case we do not click on the marker but on somewhere in the axis
                disp("marker not hit");          %thus an empty if is enought  
            else
            data.other_graph_objects(end+1)=plot(src,x,y,'*b','MarkerSize',10,'ButtonDownFcn',{@MouseClickGoal,nmbr_of_rand_ppl});
            data.goal_coords=[data.goal_coords,repmat([x,y],1,str2num(nmbr_of_rand_ppl.Value))];
            tmp_plot_x=zeros(1,2*str2num(nmbr_of_rand_ppl.Value));
            tmp_plot_y=zeros(1,2*str2num(nmbr_of_rand_ppl.Value));
            tmp_plot_x(1:2:end)=data.person_coords(end-2*str2num(nmbr_of_rand_ppl.Value)+1:2:end);%x init coords
            tmp_plot_x(2:2:end)=x;%x goal coords
            tmp_plot_y(1:2:end)=data.person_coords(end-2*str2num(nmbr_of_rand_ppl.Value)+2:2:end);%y init coords
            tmp_plot_y(2:2:end)=y;%y goal koords
            data.other_graph_objects(end+1)=line(src,tmp_plot_x,tmp_plot_y,'Color','magenta','LineStyle','--','HitTest','off');
            delete(data.rect_point)
            data.rect_point=[];
            data.rect=0;
            
            guidata(src,data);
            end
        end

    end
    
end

function GeneratePeoplesInRectangle_OnOff(src,event,uiax,editfld,otherstatebutton1,otherstatebutton2,fig)
    if isempty(editfld.Value)
        uialert(fig,"First add the number of ppl!","Eyy!")
        src.Value=0;
        return;
    elseif  otherstatebutton1.Value==1 || otherstatebutton2.Value==1
        uialert(fig,"Turn off the other buttons first","Eyy!")
        src.Value=0;
        return;
    end

    if src.Value==1
        uiax.HitTest=1;
    else
        %delete the rectangles
        data=guidata(src);
        h=data.rectangles;
        delete(h);
        data.rect_coords=[];
        data.rectangles=[];
        guidata(src,data);
        uiax.HitTest=0;
    end

end

function GeneratePeopleAndGoals_OnOff(src,event,uiax,otherstatebutton1,otherstatebutton2,fig)
    if otherstatebutton1.Value==1 ||  otherstatebutton2.Value==1
        uialert(fig,"Turn off the other buttons first","Eyy!")
        src.Value=0;
        return;
    end

    if src.Value==1
        uiax.HitTest=1;
    else
        data=guidata(src);
        if data.person_or_goal==1 || data.person_or_goal==2
            uialert(fig,"Choose a goal to the person first","Eyy!")
            src.Value=0;
            return;
            
        end
        uiax.HitTest=0;
    end
end

function GenerateWalls_OnOff(src,event,uiax,otherstatebutton1,otherstatebutton2,fig)

    if otherstatebutton1.Value==1 ||  otherstatebutton2.Value==1
        uialert(fig,"Turn off the other buttons first","Eyy!")
        src.Value=0;
        return;
    end

    if src.Value==1
        uiax.HitTest=1;
    else
        data=guidata(src);
        if data.walls==1
            uialert(fig,"Choose the end point of the wall first","Eyy!")
            src.Value=1;
            return;
            
        end
        uiax.HitTest=0;
    end
end

function loadData(src,event,editfld,fig,uiax)
    if isempty(editfld.Value)
        uialert(fig,"The data can not be loaded. Please Add a File Name!","No filename Added")
        return;
    else
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
        del_all(src,event,fig);
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
             data.other_graph_objects(end+1)=plot(uiax,data.wall_coords(ind_wall,1),data.wall_coords(ind_wall,2),'ok',"MarkerSize",10,'MarkerFaceColor','k','ButtonDownFcn',@MouseClickWall);
             data.other_graph_objects(end+1)=plot(uiax,data.wall_coords(ind_wall,3),data.wall_coords(ind_wall,4),'ok',"MarkerSize",10,'MarkerFaceColor','k','ButtonDownFcn',@MouseClickWall);
             data.other_graph_objects(end+1)=line(uiax,[data.wall_coords(ind_wall,1),data.wall_coords(ind_wall,3)],[data.wall_coords(ind_wall,2),data.wall_coords(ind_wall,4)],'Color','black','LineWidth',2,'HitTest','off');
             %guidata(src,data);
        end

        %add people one-by-one so they are independent graphical objects
        for ind_ppl=1:2:size(data.person_coords,2)
            person_indices=ind_ppl+[0,1];
            data.other_graph_objects(end+1)=appviscircles(uiax,data.person_coords(person_indices),0.25,'Color',"red");
            data.other_graph_objects(end+1)=plot(uiax,data.person_coords(person_indices(1)),data.person_coords(person_indices(2)),'*k','HitTest','off');
            data.other_graph_objects(end+1)=quiver(uiax,data.person_coords(person_indices(1)),data.person_coords(person_indices(2)),data.vel_coords(person_indices(1)),data.vel_coords(person_indices(2)),'Color','g','LineWidth',1.6,'HitTest','off');
            %later, i will probably shouldn't make a new marker for a goal, if that
            %goal already exists
            data.other_graph_objects(end+1)=plot(uiax,data.goal_coords(person_indices(1)),data.goal_coords(person_indices(2)),'*b',"MarkerSize",10,'ButtonDownFcn',{@MouseClickGoal});
            data.other_graph_objects(end+1)=line(uiax,[data.person_coords(person_indices(1)),data.goal_coords(person_indices(1))],[data.person_coords(person_indices(2)),data.goal_coords(person_indices(2))],'Color','magenta','LineStyle','--','HitTest','off');
        end
        guidata(src,data);

    end



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
            room_config_datas=rmfield(data,{'person_or_goal','walls','rect','rect_coords','rectangles','rect_point','other_graph_objects'});
            save(editfld.Value,'room_config_datas'); %we save the people independently
        end
    end
end

function del_all(src,event,fig)
    data=guidata(src);
    delete(data.other_graph_objects);
    delete(data.rectangles);
    handle_init(fig);
end

function handle_init(fig)
    data.person_or_goal=0;
    data.walls=0;
    data.rect=0;
    %data.nmbr_of_rand_ppl=0;
    
    data.person_coords=[];
    data.vel_coords=[];
    data.goal_coords=[];
    data.wall_coords=[];
    data.rect_coords=[];
    
    %we save the rectangles (graphical objects) to delete them when we turn of the generator state
    %button
    data.rectangles=[];
    data.rect_point=[];
    data.other_graph_objects=[];%since we want to delete them possibly
    
    guidata(fig,data);
end

function MouseClickWall(src,event)
    disp("wall marker hit");
    disp(event.IntersectionPoint)
    data=guidata(src);
    if data.walls==0
        data.wall_coords=[data.wall_coords;event.IntersectionPoint(1),event.IntersectionPoint(2),0,0];
        data.walls=1;
        guidata(src,data);
    elseif data.walls==1
        data.wall_coords(end,3:4)=[event.IntersectionPoint(1),event.IntersectionPoint(2)];
        data.other_graph_objects(end+1)=line(src.Parent,[data.wall_coords(end,1),data.wall_coords(end,3)],[data.wall_coords(end,2),data.wall_coords(end,4)],'Color','black','LineWidth',2,'HitTest','off');%@(h,e) disp(e.IntersectionPoint));
        data.walls=0;
        guidata(src,data);
    end
end

function MouseClickGoal(src,event,nmbr_of_rand_ppl)
    disp("goal marker hit");
    data=guidata(src);
    if data.person_or_goal==2
        disp(event.IntersectionPoint)
        data.goal_coords=[data.goal_coords,event.IntersectionPoint(1),event.IntersectionPoint(2)];
        data.person_or_goal=0;
        data.other_graph_objects(end+1)=line(src.Parent,[data.person_coords(end-1),data.goal_coords(end-1)],[data.person_coords(end),data.goal_coords(end)],'Color','magenta','LineStyle','--','HitTest','off');
        guidata(src,data);
    elseif data.rect==2
        disp(event.IntersectionPoint)
        disp(str2num(nmbr_of_rand_ppl.Value))
        data.goal_coords=[data.goal_coords,repmat([event.IntersectionPoint(1),event.IntersectionPoint(2)],1,str2num(nmbr_of_rand_ppl.Value))];
        tmp_plot_x=zeros(1,2*str2num(nmbr_of_rand_ppl.Value));
        tmp_plot_y=zeros(1,2*str2num(nmbr_of_rand_ppl.Value));
        tmp_plot_x(1:2:end)=data.person_coords(end-2*str2num(nmbr_of_rand_ppl.Value)+1:2:end);%x init coords
        tmp_plot_x(2:2:end)=event.IntersectionPoint(1);%x goal coords
        tmp_plot_y(1:2:end)=data.person_coords(end-2*str2num(nmbr_of_rand_ppl.Value)+2:2:end);%y init coords
        tmp_plot_y(2:2:end)=event.IntersectionPoint(2);%y goal koords
        data.other_graph_objects(end+1)=line(src.Parent,tmp_plot_x,tmp_plot_y,'Color','magenta','LineStyle','--','HitTest','off');
        delete(data.rect_point)
        data.rect_point=[];
        data.rect=0;
        guidata(src,data);
    end
end