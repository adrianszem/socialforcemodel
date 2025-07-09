function ip = compute_geodesic_modified_less_2_nosavegrad(A, x,h)
%%

% compute_geodesic - extract a discrete geodesic in 2D and 3D
%
%   path = compute_geodesic(D,x,options);
%
%   D is the distance map.
%   x is the starting point.
%   path is the shortest path between x and the starting point y (wich should
%   satisfy D(y)=0).
%
%   Set options.method = 'discrete' if you want to use a pure discrete
%   gradient descent.compute
%
%   Copyright (c) 2007 Gabriel Peyre


%options.null = 0;
%method = getoptions(options, 'method', 'continuous');

%{
% Handle multiple starting points (existing code)
if size(x,1)>1 && size(x,2)>1
    % several geodesics
    if size(x,1)>3
        x = x'; % probably user mistake of dimensions
    end
    path = {};
    for i=1:size(x,2)
        path{end+1} = compute_geodesic_modified(D, x(:,i), options);
    end
    return;
end
%}
%ip = extract_path_2d(D,x,h);


% extract_path_2d - extract the shortest path using 
    %   a gradient descent.
    %
    %   path = extract_path_2d(D,end_point,options);
    %
    %   'D' is the distance function.
    %   'end_point' is ending point (should be integer). 
    %{
    if isfield(options, 'trim_path')
    trim_path = options.trim_path;
    else
        trim_path = 1;
    end
    
    if isfield(options, 'stepsize')
        stepsize = options.stepsize;
    else
        stepsize = 0.1;
    end
    if isfield(options, 'maxverts')
        maxverts = options.maxverts;
    else
        maxverts = 10000;
    end
    %}
    
    if size(x,1)~=2
        error('end_points should be of size 2xk');
    end
    
    % gradient computation
    %I = find(A==Inf);%walls
    %________________________________________
    %  THIS PART TAKES A LOT
    %I=A==Inf;
    %A(I) = max(max(A(~I)));%put largest values instead of inf
    %_______________
    %global grad;
    %grad=zeros([size(A),2]);
    %grad_logical=true(size(A));
    num_of_ppl=size(x,2);
    ppl_coords=floor(x);

    res=x-ppl_coords;


    ip=zeros(size(x));

    for ind1=1:num_of_ppl %with this goal...
        coords_tmp=ppl_coords(:,ind1);%2*ind1+[-1,0]);%left down
        %coords_tmp_lin=sub2ind(size(A),ppl_coords(1,ind1),ppl_coords(2,ind1));
        %tmp=coords_tmp';
        gridvals=coords_tmp+[0,1,0,1;0,0,1,1];
        %gridvals_lin=sub2ind(size(A),gridvals(1,:),gridvals(2,:));
        vals_for_intpol=zeros(4,2);
        for ind2=1:4
            val_tmp=gridvals(:,ind2);
            %if grad_logical(val_tmp(1),val_tmp(2))%isnan(grad(val_tmp(1),val_tmp(2),1))
                %ha nem peremen
                %grad(val_tmp(1),val_tmp(2),1)=-(A(val_tmp(1)+1,val_tmp(2))-A(val_tmp(1)-1,val_tmp(2)))/(2*h);
                %grad(val_tmp(1),val_tmp(2),2)=-(A(val_tmp(1),val_tmp(2)+1)-A(val_tmp(1),val_tmp(2)-1))/(2*h);
                tmp_x=-(A(val_tmp(1)+1,val_tmp(2))-A(val_tmp(1)-1,val_tmp(2)))/(2*h);
                tmp_y=-(A(val_tmp(1),val_tmp(2)+1)-A(val_tmp(1),val_tmp(2)-1))/(2*h);
                %grad_logical(val_tmp(1),val_tmp(2))=true;
            %end
            norm=sqrt(sum([tmp_x;tmp_y].^2));
            if norm
                tmp_xyn=[tmp_x,tmp_y]/norm;
            else
                tmp_xyn=[0,0];
            end
            %grad(val_tmp(1),val_tmp(2),:)=tmp_xyn;%[tmp_x,tmp_y]/sqrt(sum([tmp_x;tmp_y].^2));
            vals_for_intpol(ind2,:)=tmp_xyn;%grad(val_tmp(1),val_tmp(2),:);
        end
         %vals_for_intpol=grad(gridvals_lin);
         xx=res(1,ind1);
         yy=res(2,ind1);
         g = ( vals_for_intpol(1,:)*(1-xx)+vals_for_intpol(2,:)*xx )*(1-yy) + ( vals_for_intpol(3,:)*(1-xx)+vals_for_intpol(4,:)*xx )*yy;
         %g = g';
         ip(:,ind1) = g';

    end
end

