%% Mini-Project 2: TRAFFIC ENGINEERING OF TELECOMMUNICATION NETWORKS
% Tiago Pedrosa (93389), Lucas Pinto (98500)
clear all, close all, clc
% ------------------------------------------------------------------
% Variables
load('InputDataProject2.mat')
T = [T1; T2];
nFlows = size(T,1);
nNodes = size(Nodes,1);
nLinks = size(Links,1);
v = 2e5;    % speed of light on fibers 
D = L/v;    % propagation delay on each direction of each link
Link_cap = 100;     % Link capacity in Gbps
Node_cap = 1000;    % Router throughput capacity in Gbps
k = 2;

sP = cell(1,nFlows);
nSP = zeros(1, nFlows);

for f = 1:nFlows
   [shortestPath, totalCost] = kShortestPath(L,T(f,1),T(f,2),k);
   sP{f} = shortestPath;
   nSP(f) = length(totalCost);
end

timeLimit= 60;
bestLoad= inf; % best = inf, worst = 0
contador= 0;
somador= 0;
bestLinkEne = inf;
maxLoad = inf;
t = tic;

while toc(t) < timeLimit
    % greedy randomzied start
    while maxLoad > Link_cap
        [sol, maxLoad, Loads, Linkenergy] = GreedyRandomizedEne(nNodes, Links, T, sP, nSP, L, Link_cap);
    end
    % ---

    [sol, maxLoad, Loads, Linkenergy] = HillClimbingEne(nNodes, Links, T, sP, nSP, sol, Loads, Linkenergy, L, Link_cap); %% nNodes, Links, T, sP, nSP, sol, Loads, linkEnergy, L

    if maxLoad<bestLoad
        bestSol= sol;
        bestLoad= maxLoad;
        bestLoadTime = toc(t);
    end
    contador= contador + 1;
    somador= somador + maxLoad;
end

nodeTraf = zeros(1, nNodes);
for f=1:nFlows
    if sol(f) ~= 0
       nodes = sP{f}{sol(f)};
       for k = nodes
          nodeTraf(k) = nodeTraf(k) + sum(T(f,3:4)); 
       end
    end
end

NodeEnergy = sum(20 + 80 * sqrt(nodeTraf/Node_cap));
TotalEne = NodeEnergy + Linkenergy;

fprintf('Multi start hill climbing with greedy randomized (all possible paths):\n');
fprintf('Worst link load of the solution: %f\n', maxLoad);
fprintf('Total Ene: %f\n', TotalEne);