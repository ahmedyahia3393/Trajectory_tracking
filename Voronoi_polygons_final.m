
% Generate random points inside a bounding box
num_points = 100;
x = rand(num_points,1)*10;
y = rand(num_points,1)*10;
% Define polygonal obstacles
obstacles = {[1 1; 1 2; 2 2; 2 1], [5 5; 5 6; 6 6; 6 5]};

% Plot points and obstacles
figure;
hold on;
plot(x,y,'.','MarkerSize',20);
for i = 1:length(obstacles)
    obstacle = obstacles{i};
    patch(obstacle(:,1), obstacle(:,2), 'k');
end
xlim([0 10]);
ylim([0 10]);
axis equal;

% Remove Voronoi pts inside obstacles
for i = 1:length(obstacles)
    obstacle = obstacles{i};
    [in,on] = inpolygon(x,y,obstacle(:,1),obstacle(:,2));
    is_inside = in | on;
    x=x(~is_inside);
    y=y(~is_inside);
    x=[x ; obstacle(:,1)];
    y=[y ; obstacle(:,2)];
end
p=[x y];

% Generate Voronoi diagram
[v,c] = voronoin(p);

% Remove Voronoi vertices outside bounding box
is_outside = v(:,1)<0 | v(:,1)>10 | v(:,2)<0 | v(:,2)>10;
% v(:,1)(is_outside) = NaN;
% v(:,2)(is_outside) = NaN;
v(is_outside,:)=[];
index=find(is_outside);
while(~isempty(index))
    idx = index(1);
    eliminated_cells = 0;
    for j=1:length(c)
        if ~isempty(find(c{j-eliminated_cells}==idx,1))
            c(j-eliminated_cells)=[];
            eliminated_cells=eliminated_cells+1;
        else
            c{j-eliminated_cells}(c{j-eliminated_cells}>idx)=c{j-eliminated_cells}(c{j-eliminated_cells}>idx)-1;
        end
    end
    index(1)=[];
    index=index-1;
end

% Remove Voronoi edges inside obstacles
for i = 1:length(obstacles)
    obstacle = obstacles{i};
    [in,on] = inpolygon(v(:,1),v(:,2),obstacle(:,1),obstacle(:,2));
    is_inside = in | on;
    v(is_inside,:)=[];
    index=find(is_inside);
    while(~isempty(index))
        idx = index(1);
        eliminated_cells = 0;
        for j=1:length(c)
            if ~isempty(find(c{j-eliminated_cells}==idx,1))
                c(j-eliminated_cells)=[];
                eliminated_cells=eliminated_cells+1;
            else
                c{j-eliminated_cells}(c{j-eliminated_cells}>idx)=c{j-eliminated_cells}(c{j-eliminated_cells}>idx)-1;
            end
        end
        index(1)=[];
        index=index-1;
    end
end

% Plot Voronoi diagram
for j=1:length(c)
    plot([v(c{j},1)' v(c{j}(1),1)],[v(c{j},2)' v(c{j}(1),2)],'r');
end
