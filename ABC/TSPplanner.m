function [optimalpath] = TSPplanner(envmap, cities);
optimalpath = bco(envmap, length(cities), cities);
end

