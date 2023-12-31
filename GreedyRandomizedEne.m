function [sol, maxLoad, Loads, linkEnergy] = GreedyRandomizedEne(nNodes, Links, T, sP, nSP, L, Lcap)
    nFlows = size(T, 1);
    % random order of flows 
    randFlows = randperm(nFlows);
    sol = zeros(1, nFlows);

    % iterate through each flow
    for flow = randFlows
        path_index = 0;
        best_Loads = inf;
        best_energy = inf;

        % test every path "possible" in a certain load
        for path = 1 : nSP(flow)
            % try the path for that flow
            sol(flow) = path;
            % calculate loads
            [Loads, linkEnergy] = calculateLinkLoadEnergy(nNodes, Links, T, sP, sol, L, Lcap);

            % check if the current load is better then bestLoad
            if linkEnergy < best_energy
                % change index of path and load
                path_index = path;
                best_Loads = Loads;
                best_energy = linkEnergy;
            end
        end
        sol(flow) = path_index;
    end
    Loads = best_Loads;
     maxLoad = 0; % Set a default value
    if ~isempty(Loads) && size(Loads, 2) >= 4 % Check if Loads has data and sufficient columns
        maxLoad = max(max(Loads(:, 3:4)));
    end
    linkEnergy = best_energy;
end