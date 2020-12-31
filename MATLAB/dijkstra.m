row = 5;
col = 5;
grid = [[0,3,-1,6,-1]; [5,0,1,-1,-1]; [-1,4,0,5,-1]; [1,-1,-1,0,6]; [-1,-1,-1,3,0]]	
dist = inf(1, row);
dist(1) = 0
parent = -1*ones(1, row)
queue = [];

%final dijkstra
queue = [];
for i = 1:row
    queue = [queue, i];
end
 			
while length(queue) ~= 0
 
 	ind = min_dist(dist, queue);
 
 	queue(find(queue==ind)) = [];
 
    for i = 1:row
        if grid(sub2ind(size(grid), ind, i)) ~= -1 %&& ismember(i, queue) 
            if dist(ind) + grid(sub2ind(size(grid), ind, i)) < dist(i)
 				dist(i) = dist(ind) + grid(sub2ind(size(grid), ind, i)); 
 				parent(i) = ind;
             end
        end
    end
end      

path = [];
parent_temp = parent
while isempty(path) || (path(1) ~= 1)
    path = [parent_temp(end), path];
    parent_temp(end) = [];
end

parent
dist
grid
path = [path, 5]

function [flag] = IsInGrid(n, m, i, j)
    flag = (i >= 1 && i <= m && j >= 1 && j <= n);
end

function [cost, index] = find_path(dij, a, b, n, m)
    if IsInGrid(n, m, a+1, b) && IsInGrid(n, m, a, b+1)
        el = [dij(a+1,b) dij(a,b+1)];
    elseif IsInGrid(n, m, a+1, b) && ~IsInGrid(n, m, a, b+1)
        el = [dij(a+1,b) dij(a+1,b)];
    elseif ~IsInGrid(n, m, a+1, b) && IsInGrid(n, m, a, b+1)
        el = [dij(a,b+1) dij(a,b+1)];
    end
    
    if el(1)>=0 && el(2)>=0
        [cost, index] = min(el);
    elseif el(1) >=0 && el(2) == -1
        cost = el(1);
        index = 1;
    elseif el(1) == -1 && el(2) >= 0 
        cost = el(2);
        index = 2;
    else 
        cost = -1;
        index = -1;
    end
    if index == 1
        index = (b-1)*n + a + 1;
    elseif index == 2
        index = (b)*n + a;
    else
        index = -1;
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


