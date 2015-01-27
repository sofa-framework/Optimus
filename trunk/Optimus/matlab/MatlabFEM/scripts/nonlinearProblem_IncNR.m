function forwardProblem_NR(experimentConf)

    [strcParams, strcModel, strcFEM, strcIncompress, strcBVP, strcSolution] = setupProblem(experimentConf);
            
    if (strcParams.doProfile) profile on; end

    if (strcModel.givenPrescribed)
        temp=strcModel.prescribedDispls./strcParams.numIncSteps;
        stepPrescribedDispls = reshape(temp',strcBVP.nDisplDOFs,1);    
    end
    vecPseudoLoads=zeros(strcBVP.nSystemSize,1);
            
    vecDirectLoads=zeros(strcBVP.nSystemSize,1);    
    
    strcBVP.vertDisp=zeros(strcFEM.size,1);
    
    strcModel.totalGravity=strcModel.gravity;        
    
    for incStep=1:strcSolution.numIncSteps  %only for gravity        
        fprintf('  ===== Incremental Step %d =====\n', incStep);
        
        if (strcModel.givenPrescribed)
            tempIndices=strcFEM.size+strcIncompress.size+strcBVP.nFixedDOFs+[1:strcBVP.nDisplDOFs];
            vecPseudoLoads(tempIndices)=vecPseudoLoads(tempIndices)+stepPrescribedDispls(1:strcBVP.nDisplDOFs);                   
        end
        
        if (strcModel.inverseProblem)
            strcModel.gravity=( (incStep/strcSolution.numIncSteps) * strcModel.totalGravity);
        else
            strcModel.gravity=(incStep/strcSolution.numIncSteps)*strcModel.totalGravity;
            vecDirectLoads(1:strcModel.nDOFs) = (incStep/strcSolution.numIncSteps)*strcModel.directLoads;    
        end
        relativeResidual = 1;        
        itt=0;    
        
        while (relativeResidual > strcSolution.newtonStopCriterium && itt < strcSolution.newtonMaxIt)
            disp(sprintf('    ----- Newton Iteration %d -----', itt));
            
            itt=itt+1;
            strcSolution.varActualIter=itt;
            %assembleFEM f, K(u) and aK'(u)
            if (strcmp(strcSolution.solutionMethod,'full'))
                if (strcmp(strcSolution.lagrangian,'total') )
                    [vecObjectLoad, vecObjectStiff, matTangentStiff] = assemble_FKA_total(strcModel, strcFEM,strcIncompress, strcBVP,1);
                elseif (strcmp(strcSolution.lagrangian,'updated') )
                    strcBVP.matCoorsUpdated = strcModel.matCoorsRest + reshape(strcBVP.vertDisp,3,strcModel.nNodes)';
                    [vecObjectLoad, vecObjectStiff, matTangentStiff] = assemble_FKA_updated(strcModel, strcFEM,strcIncompress, strcBVP,1);
                end
            elseif (strcmp(strcSolution.solutionMethod,'quasi'))
                if (strcmp(strcSolution.lagrangian,'total') )
                    [vecObjectLoad, vecObjectStiff] = assemble_FK_total(strcModel, strcFEM, strcBVP);
                    matTangentStiff=assemble_aA_total(strcModel,strcFEM,strcBVP,strcSolution,vecObjectStiff);
                elseif (strcmp(strcSolution.lagrangian,'updated') )
                    strcBVP.matCoorsUpdated = strcModel.matCoorsRest + reshape(strcBVP.vertDisp,3,strcModel.nNodes)';
                    [vecObjectLoad, vecObjectStiff] = assemble_FK_updated(strcModel, strcFEM, strcBVP);
                    matTangentStiff=assemble_aA_updated(strcModel,strcFEM,strcBVP,strcSolution,vecObjectStiff);
                end
            end
            %T=toc; fprintf('    Assemble_FK + Assemble_aA in %f [s] \n', T);
            
            %if (itt > 1)
            %    return
            %end
            %compute RHS as   f-A(u)            
                                                                
            vecRHS=vecDirectLoads+vecObjectLoad+vecPseudoLoads-vecObjectStiff;
                                                
            %apply boundary conditions for fixed nodes by Lagrange-Multipliers            
            [matTangentStiff, vecRHS] = applyBC(strcBVP, matTangentStiff, vecRHS);      
            
            %full(matTangentStiff(1:12,1:12))
            
            %Initial Residual computation
            if (itt==1)
                initialResidual = norm(vecRHS);
            end
            absoluteResidual=norm(vecRHS);
            if (initialResidual ~= 0)
                relativeResidual=absoluteResidual/initialResidual;
            else
                fprintf('    Homogeneous problem, nothing to solve!\n');
                relativeResidual = 0;
            end
            fprintf('    Residuals: abs=%f  rel=%f\n', absoluteResidual, relativeResidual);
            
            format short g
            
            %MAT=full(matTangentStiff(1:15,1:15))
            
            %RHS=vecRHS
            
            %solving the linearized system
            tic
            deltaU=matTangentStiff\vecRHS;
            
            fprintf('Norm of solution [only for DOFs]: %f\n', norm(deltaU(1:strcModel.nDOFs)));
            
            T=toc;
            %fprintf('    System solved in %g [s]\n', T);
            if ( strcParams.saveData > 3)
                save(sprintf('%s/data_inc%02d_nr%03d.mat',strcParams.outDir, incStep, itt), 'matTangentStiff','vecRHS','deltaU');
            end
            
            %extract pressure (for the incompressibility) and forces
            strcBVP.vertDisp = strcBVP.vertDisp+deltaU([1:strcModel.nDOFs]);
            
            
            if (strcIncompress.isIncompress)
                strcIncompress.vertPressure = strcIncompress.vertPressure+deltaU([strcModel.nDOFs+1:strcModel.nDOFs+strcIncompress.size]);
            end
            strcBVP.forcesInFixed(1:strcBVP.nFixedDOFs) = strcBVP.forcesInFixed(1:strcBVP.nFixedDOFs) + deltaU(strcModel.nDOFs+strcIncompress.size+[1:strcBVP.nFixedDOFs]);
            strcBVP.forcesInDispl(1:strcBVP.nDisplDOFs) = strcBVP.forcesInDispl(1:strcBVP.nDisplDOFs)+deltaU(strcModel.nDOFs+strcIncompress.size+strcBVP.nFixedDOFs+[1:strcBVP.nDisplDOFs]);
            
        end  %END Newton-Raphson
        
        forcesFixed=reshape(strcBVP.forcesInFixed,3,strcBVP.nFixedNodes)';
        forcesDispl=reshape(strcBVP.forcesInDispl,3,strcBVP.nDisplNodes)';
        vertDisps=reshape(strcBVP.vertDisp,3,strcModel.nNodes)';
        pos=strcModel.matCoors + vertDisps;
        
        if (strcParams.saveData > 2)
            %saveAsText(sprintf('%s/gravity_inc%03d.mat', strcParams.outDir,incStep),strcModel.gravity);
            saveAsText(sprintf('%s/fixed_forces_inc%03d.mat', strcParams.outDir,incStep),forcesFixed);
            %saveAsText(sprintf('%s/displacements_inc%03d.mat', strcParams.outDir,incStep),vertDisps);
            saveAsText(sprintf('%s/positions_inc%03d.mat', strcParams.outDir,incStep),pos);
            save(sprintf('%s/results_inc%03d.mat',strcParams.outDir, incStep),'vertDisps','forcesFixed','forcesDispl');            
        end
        
    end  %incremental loading

    forcesFixed=reshape(strcBVP.forcesInFixed,3,strcBVP.nFixedNodes)';
    forcesDispl=reshape(strcBVP.forcesInDispl,3,strcBVP.nDisplNodes)';
    vertDisps=reshape(strcBVP.vertDisp,3,strcModel.nNodes)';
    
    pos=strcModel.matCoors + vertDisps;
    if (strcParams.saveData > 1)
        saveAsText(sprintf('%s/fixed_forces_final.mat', strcParams.outDir),forcesFixed);
        %saveAsText(sprintf('%s/displacements_final.mat', strcParams.outDir),vertDisps);
        saveAsText(sprintf('%s/positions_final.mat', strcParams.outDir),pos);
        save(sprintf('%s/results_final.mat',strcParams.outDir),'vertDisps','forcesFixed','forcesDispl');                    
    end
    
    
    fprintf('    =====================Results: ===================\n');
    %disp('      Positions: ');    
    %disp(pos);
     
    disp('      Forces in fixed:');        
    disp(forcesFixed);
    
    if (strcModel.givenPrescribed)
        disp('      Forces in displaced:');        
        disp(forcesDispl);    
    end
    
    %save and finalize
    if (strcParams.doProfile) profile off; profsave; end                                    
    
    fprintf('Storing results to %s\n', strcParams.outDir);
    cd(strcParams.srcDir);    
end