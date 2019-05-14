
function [done] = TSP(cities)
envmap = load('map1.txt');

close all;

%draw the environment
image(envmap'*0);
%cities = TSPplanner(envmap, cities);

for i = 1:length(cities)-1
    text(cities(i,1), cities(i,2), "city"+num2str(i));
end

start = [cities(1,1) cities(1,2)];
target = [cities(2,1) cities(2,2)];
[finalrobotx, finalroboty] = runtest('map1.txt', start, target);

for i = 3:length(cities)
    robotstart = [finalrobotx finalroboty];
    targetstart = [cities(i,1) cities(i,2)];
    [finalrobotx, finalroboty] = runtest('map1.txt', robotstart, targetstart);
end
    
done = 1;
end

