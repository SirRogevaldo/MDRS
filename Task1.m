%% Mini-Project 2: TRAFFIC ENGINEERING OF TELECOMMUNICATION NETWORKS
% Tiago Pedrosa (93389), Lucas Pinto (98500)
clear all, close all, clc
% ------------------------------------------------------------------
% Variables
load('InputDataProject2.mat')
nFlows_uni_T1 = size(T1, 1);
nFlows_uni_T2 = size(T2, 1);
nNodes = size(Nodes,1);
nLinks = size(Links,1);
v = 2e5;    % speed of light on fibers 
D = L/v;    % propagation delay on each direction of each link
Link_cap = 100;     % Link capacity in Gbps
Node_cap = 1000;    % Router throughput capacity in Gbps
k = 2;

sP_T1 = cell(1, nFlows_uni_T1);
sP_T2 = cell(1, nFlows_uni_T2);
sP = [sP_T1 sP_T2]';
nSP_T1 = zeros(1, nFlows_uni_T1);
nSP_T2 = zeros(1, nFlows_uni_T2);
nSP = [nSP_T1 nSP_T2]';
T = [T1; T2];

for f = 1:(nFlows_uni_T1 + nFlows_uni_T2)
   [shortestPath, totalCost] = kShortestPath(L,T(f,1),T(f,2),k);
   sP{f} = shortestPath;
   nSP(f) = length(totalCost);
end

timeLimit= 60;
bestLoad= inf; % best = inf, worst = 0
contador= 0;
somador= 0;
t = tic;

while toc(t) < timeLimit
    % greedy randomzied start
    [sol, ~, Loads, energy] = GreedyRandomizedEne(nNodes, Links, T, sP, nSP, L);

    % ---

    [sol, maxLoad, Loads, energy] = HillClimbingEne(nNodes, Links, T, sP, nSP, sol, Loads, energy, L); %% nNodes, Links, T, sP, nSP, sol, Loads, linkEnergy, L

    if maxLoad<bestLoad
        bestSol= sol;
        bestLoad= maxLoad;
        bestLoadTime = toc(t);
    end
    contador= contador + 1;
    somador= somador + maxLoad;
end

fprintf('Multi start hill climbing with greedy randomized (all possible paths):\n');
fprintf('\t W = %.2f Gbps, No. sol = %d, Av. W = %.2f, time = %.2f sec\n', bestLoad, contador, somador/contador, bestLoadTime);