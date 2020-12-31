n = 6
m = 6
grid = inf(n, m);
grid(1, 1) = nan;
grid(1, 2) = nan;
grid(2, 1) = nan;
grid(2, 4) = nan;
grid(2, 6) = nan;
grid(3, 1) = nan;
grid(3, 3) = nan;
grid(3, 6) = nan;
grid(4, 2) = nan;
grid(4, 3) = nan;
grid(4, 5) = nan;
grid(4, 6) = nan;
parent = zeros(n, m);

start = 4
goal = 19

grid(start) = 0;
grid(goal) = 0;
prm_grid = grid

prm_grid = CheckObs(prm_grid)
num_node = 15;
vertex = [start];

%sampling
while length(vertex) < num_node+1
    index = randi(n*m);
    if FreeNode(grid, index) && ~ismember(index, vertex)
        vertex = [vertex, index];
        prm_grid(index) = 0;
    end
end
vertex = [vertex, goal]
prm_grid

%graph in terms of edges
edges = cell(length(vertex));

for i = 1:length(vertex)
    N = neighbor(prm_grid, vertex, vertex(i));
    for j = 1:length(N)
        if N(j) ~= vertex(i) && ~ismember(N(j), edges{vertex(i)}) && FreePath(prm_grid, vertex(i), vertex(j))
            edges{vertex(i)} = [edges{vertex(i)}; N(j)]; 
            edges{N(j)} = [edges{N(j)}; vertex(i)];
        end
    end
end

vertex
prm_grid
edges



function [row, col] = index_return(ind, n, m)
    if mod(ind, n) ~= 0
        row = mod(ind, n);
    else
        row = n;
    end
    col = ((ind - row)/n) + 1;
end

function [path] = dijkstra(grid)
    %final dijkstra
    queue = [];
    for i = 1:5
        queue = [queue, i];
    end
 			
    while length(queue) ~= 0
 
 	    ind = min_dist(dist, queue);
 
     	queue(find(queue==ind)) = [];
     
        for i = 1:5
            if grid(sub2ind(size(grid), ind, i)) ~= -1 %&& ismember(i, queue) 
                if dist(ind) + grid(sub2ind(size(grid), ind, i)) < dist(i)
     				dist(i) = dist(ind) + grid(sub2ind(size(grid), ind, i)); 
     				parent(i) = ind;
                 end
            end
        end
    end      
    path = [];
    parent_temp = parent;
    while isempty(path) || (path(1) ~= 1)
        path = [parent_temp(end), path];
        parent_temp(end) = [];
    end
end


function [min_index] = min_dist(dist, queue)
    minimum = inf;
    min_index = -1;
    for i = 1:length(dist) 
        if dist(i) < minimum && ismember(i, queue)
			minimum = dist(i);
			min_index = i; 
        end
    end
end

function [flag] = IsInGrid(n, m, i, j)
    flag = (i >= 1 && i <= m && j >= 1 && j <= n);
end

function [flag] = FreeNode(grid, node)
    if isnan(grid(node))
        flag = false;
    else
        flag = true;
    end
end

function [flag] = FreePath(grid, n1, n2)
    dim = size(grid);
    n = dim(1);
    m = dim(2);
    
    [n1x, n1y] = index_return(n1, n, m);
    [n2x, n2y] = index_return(n2, n, m);
    y1 = min(n1y, n2y);
    y2 = max(n1y, n2y);
    x1 = min(n1x, n2x);
    x2 = max(n1x, n2x);
    flag = true;
    if x1 ~= x2 && y1 ~= y2
        flag = false;
    elseif x1 ~= x2 && y1 == y2
        for i = x1:x2
            if isnan(grid((y1-1)*n+i)) || isinf(grid((y1-1)*n+i))
                flag = false;
                break;
            end
        end
    elseif x1 == x2 && y1 ~= y2
        for j = y1:y2
            if isnan(grid((j-1)*n + x1)) || isinf(grid((j-1)*n + x1))
                flag = false;
                break;
            end
        end
    end
end

function [grid] = CheckObs(grid)
    dim = size(grid);
    n = dim(1);
    m = dim(2);
    for i = 1:n
        for j = 1:m
            index = (j-1)*n + i;
            if index <= n*m - n && index > n && mod(n*m - index, n) ~= 5 && mod(n*m - index, n) ~= 0 && isnan(grid(index+1)) && isnan(grid(index-1)) && isnan(grid(index+n)) && isnan(grid(index-n))
                grid(i, j) = nan;
            elseif mod(n*m - index, n) == 5 && index ~= 1 && index ~= n*m - n + 1 && isnan(grid(index+1)) && isnan(grid(index-n)) && isnan(grid(index+n))
                grid(i, j) = nan;
            elseif mod(n*m - index, n) == 0 && index ~= n && index ~= n*m && isnan(grid(index-1)) && isnan(grid(index+1)) && isnan(grid(index-n))
                grid(i, j) = nan;
            elseif index < n && index > 1 && isnan(grid(index+1)) && isnan(grid(index-1)) && isnan(grid(index+n))
                grid(i, j) = nan;
            elseif index < n*m && index > n*m - n && isnan(grid(index+1)) && isnan(grid(index-1)) && isnan(grid(index-n))
                grid(i, j) = nan;
            elseif index == 1 && isnan(grid(index+1)) && isnan(grid(index+n))
                grid(i, j) = nan;
            elseif index == n && isnan(grid(index-1)) && isnan(grid(index+n))
                grid(i, j) = nan;
            elseif index == n*m-n+1 && isnan(grid(index+1)) && isnan(grid(index-n))
                grid(i, j) = nan;
            elseif index == n*m && isnan(grid(index-1)) && isnan(grid(index-n))
                grid(i, j) = nan;
            end
        end
    end
end

function cost = dist(grid, ind1, ind2)
    dim = size(grid);
    n = dim(1);
    m = dim(2);
    [ind1x,ind1y] = index_return(ind1, n, m);
    [ind2x,ind2y] = index_return(ind2, n, m);
    cost = abs(ind1x - ind2x) + abs(ind1y - ind2y);
end

function [neighbors] = neighbor(grid, vertex, ind)
    neighbors = [];
    for i = 1:length(vertex)
        if FreePath(grid, ind, vertex(i)) && ind ~= vertex(i) && dist(grid, ind, vertex(i)) < 3
            neighbors = [neighbors, vertex(i)];
        end
    end
end

