function writeProblemFiles(fileName, A, par)
    paramID = fopen(strcat(fileName, ".par"), "w");
    fprintf(paramID, "PROBLEM_FILE = %s\n", strcat(fileName,".tsp"));
    fprintf(paramID, "OUTPUT_TOUR_FILE = %s\n", strcat(fileName,".txt"));
    fprintf(paramID, "DEPOT = %d\n", 1);
    fprintf(paramID, "MTSP_MIN_SIZE = %d\nMTSP_OBJECTIVE = %s\nEOF", 1, "MINMAX"); 
    fclose(paramID);
    
    problemID = fopen(strcat(fileName, ".tsp"), "w");
    name = split(fileName,"\");
    fprintf(problemID, "NAME : %s\n", name(end));
    fprintf(problemID, "COMMENT : %s\n", par.user_comment);
    fprintf(problemID, "TYPE : %s\n", "TSP");
    fprintf(paramID, "DIMENSION : %d\n", length(A));
    fprintf(problemID, "VEHICLES : %d\n", par.vehicles);
    fprintf(problemID, "EDGE_WEIGHT_TYPE : EXPLICIT\nEDGE_WEIGHT_FORMAT : UPPER_ROW\nEDGE_WEIGHT_SECTION :\n");
    for i = 1:length(A)-1
        for j = i+1:length(A)
            if A(i,j) == 0
                fprintf(problemID, "%d ", par.infDist);
            else
                fprintf(problemID, "%d ", A(i,j));
            end
        end
        fprintf(problemID, "\n");
    end
    fprintf(problemID, "EOF");
    fclose(problemID);
end