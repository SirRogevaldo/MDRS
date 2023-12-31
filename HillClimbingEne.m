function [sol, maxLoad, Loads, linkEnergy] = HillClimbingEne(nNodes, Links, T, sP, nSP, sol, Loads, linkEnergy, L, Lcap)
    nFlows = size(T,1);    
    % set the best local variables
    maxLoad = max(max(Loads(:, 3:4)));
    bestLocalLoad = maxLoad;
    bestLocalLoads = Loads;
    bestLocalSol = sol;
    bestLocalEnergy = linkEnergy;

    % Hill Climbing Strategy
    improved = true;
    while improved
        % test each flow
        for flow = 1 : nFlows
            % test each path of the flow
            for path = 1 : nSP(flow)
                if path ~= sol(flow)
                    % change the path for that flow
                    auxSol = sol;
                    auxSol(flow) = path;
                    % calculate loads
                    [auxLoads, auxLinkEnergy] = calculateLinkLoadEnergy(nNodes, Links, T, sP, auxSol, L, Lcap);
                    auxMaxLoad = max(max(auxLoads(:, 3:4)));
                    % check if the current load is better then start load
                    if auxLinkEnergy < bestLocalEnergy
                        bestLocalLoad = auxMaxLoad;
                        bestLocalLoads = auxLoads;
                        bestLocalSol = auxSol;
                        bestLocalEnergy = auxLinkEnergy;
                    end
                end
            end
        end

        if bestLocalEnergy < linkEnergy
            maxLoad = bestLocalLoad;
            Loads = bestLocalLoads;
            sol = bestLocalSol;
            linkEnergy = bestLocalEnergy;
        else 
            improved = false;
        end
    end
end