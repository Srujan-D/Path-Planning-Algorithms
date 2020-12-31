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
