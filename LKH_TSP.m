function [TSPsolution] = LKH_TSP(CostMatrix,pars_struct,fname_tsp,LKHdir,TSPLIBdir)
%
%   Syntax:
%   TSPsolution = LKH_TSP(CostMatrix,pars_struct,fname_tsp,LKHdir,TSPLIBdir)
%
%   This functions solves TSP problems using the Lin-Kernighan-Helsgaun
%   solver. It assumes that a compiled executable of the LKH solver as
%   found at: http://www.akira.ruc.dk/~keld/research/LKH/ is available at
%   the LKHdir directory. Furthermore a TSPLIB directory is assumed.
%   For the definition of the TSPLIB and the compilation of the LKH code
%   check the aforementioned url. 
%
%   Inputs:
%   CostMatrix      : the Cost Matrix of the (asymmetric) TSP. [e.g. it can be an NxN matrix of distances]
%   pars_struct     : parameters structure with
%                   : -> CostMatrixMulFactor (value that makes Cost Matrix
%                        almost integer. [eg. pars_struct.CostMatrixMulFactor = 1000; ]
%                     -> user_comment (a user comment for the problem) [optional]
%   fname_tsp       : the filename to save the tsp problem
%   LKHdir          : the directory of the LKH executable
%   TSPLIBdir       : the directory of the TSPLIB files
%   
%   Outputs:
%   TSPsolution     : the TSP solution
%   
%   Authors:
%   Kostas Alexis (kalexis@unr.edu)
%

CostMatrix_tsp = pars_struct.CostMatrixMulFactor*CostMatrix;
CostMatrix_tsp = floor(CostMatrix_tsp);
user_comment = pars_struct.user_comment;

writeProblemFiles(strcat(LKHdir, "/", TSPLIBdir, "/", fname_tsp),CostMatrix_tsp,pars_struct);

%%  Solve the TSP Problem via the LKH Heuristic

start_lkh_time = cputime;
lkh_cmd = strcat(LKHdir, "/LKH", " ", LKHdir, "/", TSPLIBdir, "/", fname_tsp, '.par');
[status,cmdout] = system(lkh_cmd);
end_lkh_time = cputime;

solution_file = strcat(LKHdir, "/", TSPLIBdir, "/", fname_tsp, ".txt");
tsp_sol_cell = importdata(solution_file);

% rm_solution_file_cmd = ['rm ' LKHdir fname_tsp '.txt'];
% [status,cmdout] = system(rm_solution_file_cmd);

tsp_sol = [];
for i = 1:length(tsp_sol_cell.textdata)
    if ~isempty(str2num(tsp_sol_cell.textdata{i}))
        tsp_sol(end+1) = str2num(tsp_sol_cell.textdata{i});
    end
end
tsp_sol(end) = [];

TSPsolution = tsp_sol;

end