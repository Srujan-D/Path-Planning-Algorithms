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
pot = zeros(n, m);
grid


start = 4
goal = 19
pmax = 10;
obs = [1, 2, 3, 7, 10, 15, 16, 20, 28, 31, 32, 33]

prm_grid = grid


prm_grid = CheckObs(prm_grid)
if isinf(prm_grid(start)) || isnan(prm_grid(start))
    disp("Start node invalid")
end

dim = size(grid);
n = dim(1);
m = dim(2);
sigma_x = n;
sigma_y = m;
sigma = sigma_x^2 + sigma_y^2;
syms x y

pot1 = 0;
for i = 1:length(obs)
        [obs_x, obs_y] = ind2sub(size(grid), obs(i));
        pot1 = pot1 + exp(-0.5*((((x - obs_x)^2) + (y - obs_y)^2)/(sigma))^i);
end
pot_r = pot1;
[goal_x, goal_y] = ind2sub(size(grid), goal);
pot_a = 1 - exp(-0.5*(((x - goal_x)^2) + (y - goal_y)^2)/(2*sigma));
pot_final = pot_r + pot_a;
U = pot_final;
diff(U, x);

pot = grid;
cnt = 0;
dist_tol = 0.5;
s = 0.05;
start = 4;

[i, j] = ind2sub(size(grid),start);
while start ~= goal
    neighbor = neighbors(grid, start);    %returns neighbors of start
    if isnan(grid(start)) || ~ismember(start, neighbor)
        continue;
    else
        start = start + (s*vpa(subs(pot_final,{x y}, {i j})));
        start = round(start)
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

function [neighbor] = neighbors(grid, ind)
    dim = size(grid);
    n = dim(1);
    m = dim(2);
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

function [pot] = attract_function(grid, point, goal)
    dim = size(grid);
    n = dim(1);
    m = dim(2);
    sigma_x = n;
    sigma_y = m;
    sigma = sigma_x^2 + sigma_y^2;
    [x, y] = ind2sub(size(grid), point);
    [goal_x,goal_y] = ind2sub(size(grid), goal);
    pot = 1 - exp(-0.5*(((x - goal_x)^2) + (y - goal_y)^2)/(2*sigma));
end



function [pot] = repel_function(grid, point, obs)
    dim = size(grid);
    n = dim(1);
    m = dim(2);
    sigma_x = n;
    sigma_y = m;
    sigma = sigma_x^2 + sigma_y^2;
    [x, y] = ind2sub(size(grid), point);
    for i = 1:length(obs)
        [obs_x, obs_y] = ind2sub(size(grid), obs(i));
        pot = pot + exp(-0.5*((((x - obs_x)^2) + (y - obs_y)^2)/(sigma))^i);
    end
end

