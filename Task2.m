%% Mini-Project 2: TRAFFIC ENGINEERING OF TELECOMMUNICATION NETWORKS
% Tiago Pedrosa (93389), Lucas Pinto (98500)
clear all, close all, clc
% ------------------------------------------------------------------
% Variables
load('InputDataProject2.mat')
fprintf('******  TASK 2 ******\n');
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
bestLoad= inf; 
bestEne = inf;
contador= 0;
somador= 0;
bestLinkEne = inf;
maxLoad = inf;
LinkEne = 0;
t = tic;

while toc(t) < timeLimit
    % greedy randomzied start
    while maxLoad > Link_cap
        [sol, maxLoad, Loads, Linkenergy] = GreedyRandomizedEne(nNodes, Links, T, sP, nSP, L, Link_cap);
    end
    % ---

    [sol, maxLoad, Loads, Linkenergy] = HillClimbingEne(nNodes, Links, T, sP, nSP, sol, Loads, Linkenergy, L, Link_cap); %% nNodes, Links, T, sP, nSP, sol, Loads, linkEnergy, L

    if Linkenergy < bestEne
        bestSol= sol;
        bestLoad= maxLoad;
        bestEne = Linkenergy;
        bestLoadTime = toc(t);
        LinkEne = LinkEne + Linkenergy;
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

sleepingLinks = '';
NsleepLinks = 0;
for i = 1 : size(Loads, 1)
    if max(Loads(i, 3:4)) == 0
        NsleepLinks = NsleepLinks + 1;
        sleepingLinks = append(sleepingLinks, ' {', num2str(Loads(i,1)), ', ', num2str(Loads(i,2)), '}');
    end
end

NodeEnergy = sum(20 + 80 * sqrt(nodeTraf/Node_cap));
TotalEne = NodeEnergy + LinkEne;

auxSum = 0;
cnt = 0;
for k= 1:nFlows
   if sum(Loads(k,3:4)) ~= 0
       cnt = cnt +2;
       auxSum = auxSum + sum(Loads(k,3:4));
   end
end
AvgLinkLoad = auxSum / cnt;

fprintf('Multi start hill climbing with greedy randomized (all possible paths):\n');
fprintf('Worst link load of the solution: %.2f Gbps\n', bestLoad);
fprintf('Average link load of the solution: %.2f Gbps\n', AvgLinkLoad);
fprintf('Network energy consumption of the solution: %.2f W\n\t\tNode energy: %.2f W\n\t\tLink energy: %.2f W\n', TotalEne, NodeEnergy, LinkEne);
fprintf('Average round-trip propagation delay of each service: xxxxx\n');
fprintf('Links not supporting any traffic flow: %d links -> %s\n', NsleepLinks, sleepingLinks);
fprintf('Number of cycles run by the algorithm: %d\n', contador);
fprintf('Running time at which the algorithm has obtained the best solution: %.4f ms\n', bestLoadTime*1000);
