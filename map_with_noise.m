function new_map = map_with_noise(old_map, deviation)
% new_map = MAP_WITH_NOISE(old_map, szoras)
%  Adds noise to a map
% Output:
%   new_map: map with noise
% Inputs:
%   old_map: original map
%   deviation: standard deviation of noise

new_map = old_map + randn(size(old_map))*deviation;

new_map(find(new_map < 0)) = 0;
new_map(find(new_map > 1)) = 1;