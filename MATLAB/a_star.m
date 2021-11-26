n = 6
m = 6
grid = inf(n, m);
grid(4, 1) = 0;
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
parent = inf(n, m)
astar_temp = grid

start = 4
goal = 19


path = [start];
open = [start]
open_score = [heuristic(start, goal, n, m)]
closed = [];
f = astar_temp
g = astar_temp
g(start) = 0;
f(start) = heuristic(start, goal, n, m);

astar_temp = CheckObs(astar_temp)
if isinf(astar_temp(start)) || isnan(astar_temp(start))
    disp("Start pose cannot be accessed")
else
    cnt = 0;
    while open(end) ~= goal && cnt < 100
        [cost, ind] = min(open_score);
        current = open(ind);
        if (current == goal)
            break;
        else
            
            closed = [closed, current];
            neighbor = neighbors(current, n, m);
            for i = 1:length(neighbor)
                if isnan(astar_temp(neighbor(i)))
                    continue;
                else
                    cost = g(current) + heuristic(current, goal, n, m);
                    if parent(current) ~= neighbor(i) && ismember(neighbor(i), open) && cost < g(neighbor(i))
                        open(open==neighbor(i)) = []
                        open_score(find(open==neighbor(i))) = []
                        
                    elseif parent(current) ~= neighbor(i) && ismember(neighbor(i), closed) && cost < g(neighbor(i))
                        closed(closed==neighbor(i)) = [];
                    elseif parent(current) ~= neighbor(i) && ~ismember(neighbor(i), open) && ~ismember(neighbor(i), closed)
                        g(neighbor(i)) = g(current) + 1;
                        parent(neighbor(i)) = current;
                        open = [open, neighbor(i)];
                        f(neighbor(i)) = g(neighbor(i)) + heuristic(neighbor(i), goal, n, m);
                        open_score = [open_score, f(neighbor(i))];
                    end
                end
            end
            open(open == current) = [];
            open_score(find(open==current)) = [];
            
        end
        cnt = cnt+1;
    end
    path = [start];
    current = start;
    f(current) = inf;
    while (path(end)~=parent(goal))
        neighbor = neighbors(current, n, m);
        [select, index] = min(f(neighbor));
        for i = 1:length(neighbor)
            if ~isnan(neighbor(i)) && parent(neighbor(i) ~= 0) && (parent(neighbor(i)) == current)
                if ~(path == neighbor(index))
                    path = [path, neighbor(index)];
                end
            end
        end
        current = path(end);
        f(current) = inf;
    end
end

path = [path, goal]
    

open
open_score
parent
g
f
path

ab = neighbors(4, 6, 6)


x = [2, 4]
y = [1 2 3; 0 4 5]
[g,h ] = min(y(x))
x(h)

function [row, col] = index_return(ind, n, m)
    if mod(ind, n) ~= 0
        row = mod(ind, n);
    else
        row = n;
    end
    col = ((ind - row)/n) + 1;
end

function [flag] = IsInGrid(n, m, i, j)
    flag = (i >= 1 && i <= m && j >= 1 && j <= n);
end

function [neighbor] = neighbors(ind, n, m)
    neighbor = [];
    if mod(n*m - ind, n) >= 1
        neighbor = [neighbor, ind+1];
    end
    if mod(n*m - ind, n) ~= 5
        neighbor = [neighbor, ind-1];
    end
    if ind < n*m - n + 1
        neighbor = [neighbor, ind+n];
    end
    if ind > n
        neighbor = [neighbor, ind-n];
    end
end

function cost = heuristic(ind, goal, n, m)
    [ind1x,ind1y] = index_return(ind, n, m);
    [ind2x,ind2y] = index_return(goal, n, m);
    cost = abs(ind1x - ind2x) + abs(ind1y - ind2y);
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

