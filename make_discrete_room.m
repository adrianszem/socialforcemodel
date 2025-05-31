
%room is [-10,10,-10,10]
h=0.1; %resolution

xx=0:h:10;
yy=xx;%%(1:2:end);
[X,Y]=meshgrid(xx,yy);
wall_map=zeros(size(X,1)-2,size(X,2)-2);
wall_map=padarray(wall_map,[1,1],1,'both');


%find function somewaht slow, we will just make a linindex
%meshgrid(subindex could be ok also, but lin is better imo..
lin_indexes=1:size(X,1)*size(X,2);
lin_indexes=reshape(lin_indexes,size(X,1),size(X,2));
wall_indexes=lin_indexes((wall_map.*lin_indexes)>0);
room.wall_map=wall_map;
room.wall_indexes=wall_indexes;
room.X_coords=X(1,:);%X
room.Y_coords=Y(:,1)';%Y;
room.wall_X=X(wall_indexes);
room.wall_Y=Y(wall_indexes);
room.wall_coords=[X(wall_indexes),Y(wall_indexes)];
room.resolution=h;%redundant 
save('room_for_soc_forc_mod_0_10.mat',"room");
%we will need this later
%closest wall point from coord
coord=[1,1];
%coords=repmat(coord,size(room.wall_coords,1),1);
[min_val,min_ind]=min(vecnorm(repmat(coord,size(room.wall_coords,1),1)-room.wall_coords,2,2));
%min_wall_coords=room.wall_coords(min_ind,:);



