n = 6
m = 6
grid = inf(n, m);
grid(19) = 0;
grid(1, 1) = nan;
grid(1, 2) = nan;
grid(2, 2) = nan;
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
grass_temp = grid
start = 4;
dest = 19;

hasinf = any(isinf(grass_temp(:)));
path = [start];
source = grass_temp(start);

flag = check_start(grass_temp, n, m, start) %for checking if start is in obstacle/surrounded by obstacle.
grass_temp = check_obs(grass_temp, n, m)

flag2 = false;
while hasinf && flag
    [grass_temp, parent, ~] = fire1(grass_temp, parent, n, m, start);
    [grass_temp, parent, flag1] = fire2(grass_temp, parent, n, m, start);
    hasinf = any(isinf(grass_temp(:)));
    flag2 = flag1;
end
flag2
grass_temp
parent

if (isinf(grass_temp(start)))
    path = [];
else
    while (parent(path(end)) ~= 0)
        if mod(n*m - start, n) >= 1 && grass_temp(start+1) < grass_temp(start)
            path = [path, start+1];
            start = start+1;
        elseif mod(n*m - start, n) ~= 5 && grass_temp(start-1) < grass_temp(start)
            path = [path, start-1];
            start = start-1;
        elseif (start <= n*m - n + 1) && grass_temp(start+n) < grass_temp(start)
            path = [path, start+n];
            start = start+n;
        elseif start <= n && grass_temp(start-n) < grass_temp(start)
            path = [path, start-n];
            start = start-n;
        end
    end
end
path = [path, dest]; 
path

function [grass_temp, parent, flag] = fire2(grass_temp, parent, n, m, start)
    flag = false;
    for i = n: -1: 1
        for j = m: -1: 1
            if ~isnan(grass_temp(i, j)) && (grass_temp(i, j) == inf)
                if i < n && j < m && (~isnan(grass_temp(i+1, j)) || ~isnan(grass_temp(i, j+1)))
                    grass_temp(i, j) = 1 + min(grass_temp(i+1, j), grass_temp(i, j+1));
                    parent(i, j) = min(grass_temp(i+1, j), grass_temp(i, j+1));
                elseif i == n && j < m && ~isnan(grass_temp(i, j+1))
                    grass_temp(i, j) = 1 + grass_temp(i, j+1);
                    parent(i, j) = grass_temp(i, j+1);
                elseif i < n && j == m && ~isnan(grass_temp(i+1, j))
                    grass_temp(i, j) = 1 + grass_temp(i+1, j);
                    parent(i, j) = grass_temp(i+1, j);
                end
            end
            if ((j-1)*n + i == start)
               flag = true;
            end
        end
    end
end

function [grass_temp, parent, flag] = fire1(grass_temp, parent, n, m, start)
    flag = false;
    for i = 1:n
        for j = 1:m
            if ~isnan(grass_temp(i, j)) && (grass_temp(i, j) == inf)
                if i > 1 && j > 1  && (~isnan(grass_temp(i-1, j)) || ~isnan(grass_temp(i, j-1)))
                    grass_temp(i, j) = 1 + min(grass_temp(i-1, j), grass_temp(i, j-1));
                    parent(i, j) = min(grass_temp(i-1, j), grass_temp(i, j-1));
                elseif i == 1 && j > 1 && ~isnan(grass_temp(i, j-1))
                    grass_temp(i, j) = 1 + grass_temp(i, j-1);
                    parent(i, j) = grass_temp(i, j-1);
                elseif i > 1 && j == 1 && ~isnan(grass_temp(i-1, j))
                    grass_temp(i, j) = 1 + grass_temp(i-1, j);
                    parent(i, j) = grass_temp(i-1, j);
                end
            end
            if ((j-1)*n + i == start)
               flag = true;
            end
        end
    end
end

function [flag] = check_start(grass_temp, n, m, start)
    flag= true;
    if start <= n*m - n && start > n && mod(n*m - start, n) ~= 5 && mod(n*m - start, n) ~= 0 && isnan(grass_temp(start+1)) && isnan(grass_temp(start-1)) && isnan(grass_temp(start+n)) && isnan(grass_temp(start-n))
        flag = false;
    elseif mod(n*m - start, n) == 5 && start ~= 1 && start ~= n*m - n + 1 && isnan(grass_temp(start+1)) && isnan(grass_temp(start-n)) && isnan(grass_temp(start+n))
        flag = flase;
    elseif mod(n*m - start, n) == 0 && start ~= n && start ~= n*m && isnan(grass_temp(start-1)) && isnan(grass_temp(start+1)) && isnan(grass_temp(start-n))
        flag = false;
    elseif start < n && start > 1 && isnan(grass_temp(start+1)) && isnan(grass_temp(start-1)) && isnan(grass_temp(start+n))
        flag = false;
    elseif start < n*m && start > n*m - n && isnan(grass_temp(start+1)) && isnan(grass_temp(start-1)) && isnan(grass_temp(start-n))
        flag = false;
    elseif start == 1 && isnan(grass_temp(start+1)) && isnan(grass_temp(start+n))
        flag = false;
    elseif start == n && isnan(grass_temp(start-1)) && isnan(grass_temp(start+n))
        flag = false;
    elseif start == n*m-n+1 && isnan(grass_temp(start+1)) && isnan(grass_temp(start-n))
        flag = false;
    elseif start == n*m && isnan(grass_temp(start-1)) && isnan(grass_temp(start-n))
        flag = false;
    end
end

function [grass_temp] = check_obs(grass_temp, n, m)
    for i = 1:n
        for j = 1:m
            start = (j-1)*n + i;
            if start <= n*m - n && start > n && mod(n*m - start, n) ~= 5 && mod(n*m - start, n) ~= 0 && isnan(grass_temp(start+1)) && isnan(grass_temp(start-1)) && isnan(grass_temp(start+n)) && isnan(grass_temp(start-n))
                grass_temp(i, j) = nan;
            elseif mod(n*m - start, n) == 5 && start ~= 1 && start ~= n*m - n + 1 && isnan(grass_temp(start+1)) && isnan(grass_temp(start-n)) && isnan(grass_temp(start+n))
                grass_temp(i, j) = nan;
            elseif mod(n*m - start, n) == 0 && start ~= n && start ~= n*m && isnan(grass_temp(start-1)) && isnan(grass_temp(start+1)) && isnan(grass_temp(start-n))
                grass_temp(i, j) = nan;
            elseif start < n && start > 1 && isnan(grass_temp(start+1)) && isnan(grass_temp(start-1)) && isnan(grass_temp(start+n))
                grass_temp(i, j) = nan;
            elseif start < n*m && start > n*m - n && isnan(grass_temp(start+1)) && isnan(grass_temp(start-1)) && isnan(grass_temp(start-n))
                grass_temp(i, j) = nan;
            elseif start == 1 && isnan(grass_temp(start+1)) && isnan(grass_temp(start+n))
                grass_temp(i, j) = nan;
            elseif start == n && isnan(grass_temp(start-1)) && isnan(grass_temp(start+n))
                grass_temp(i, j) = nan;
            elseif start == n*m-n+1 && isnan(grass_temp(start+1)) && isnan(grass_temp(start-n))
                grass_temp(i, j) = nan;
            elseif start == n*m && isnan(grass_temp(start-1)) && isnan(grass_temp(start-n))
                grass_temp(i, j) = nan;
            end
        end
    end
end
