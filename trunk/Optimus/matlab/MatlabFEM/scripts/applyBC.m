function [matTangentStiff, vecRHS] = applyBC(strcBVP, matTangentStiff, vecRHS)
    for (dof=1:strcBVP.nFixedDOFs)
        matTangentStiff(strcBVP.vecFixedstrcModel.matCoors(dof),strcBVP.vecFixedLMCoors(dof)) = 1;
        matTangentStiff(strcBVP.vecFixedLMCoors(dof),strcBVP.vecFixedstrcModel.matCoors(dof)) = 1;
    end                
    vecRHS(strcBVP.vecFixedstrcModel.matCoors)=vecRHS(strcBVP.vecFixedstrcModel.matCoors)-strcBVP.forcesInFixed(:);
    vecRHS(strcBVP.vecFixedLMCoors) = vecRHS(strcBVP.vecFixedLMCoors)-strcBVP.vertDisp(strcBVP.vecFixedstrcModel.matCoors);

    %Apply boundary conditions for displaced nodes by Lagrange-Multipliers
    for dof=1:strcBVP.nDisplDOFs         
        matTangentStiff(strcBVP.vecDisplstrcModel.matCoors(dof),strcBVP.vecDisplLMCoors(dof))=1;
        matTangentStiff(strcBVP.vecDisplLMCoors(dof),strcBVP.vecDisplstrcModel.matCoors(dof))=1;
    end        
    if (strcBVP.nDisplDOFs > 0)        
        vecRHS(strcBVP.vecDisplstrcModel.matCoors)=vecRHS(strcBVP.vecDisplstrcModel.matCoors)-strcBVP.forcesInDispl(:);
        vecRHS(strcBVP.vecDisplLMCoors) = vecRHS(strcBVP.vecDisplLMCoors) - strcBVP.vertDisp(strcBVP.vecDisplstrcModel.matCoors);
    end
end