classdef PathPlanning
    properties
    end
    
    methods (Static)      
        function optimizedPath = generatePath(startPoint, endPoint, topographicMap, measurement, camerasClass)
            % Setting up constants
            step = 50;
            beta = 100;
            width = measurement.chamberSize(1);
            height = measurement.chamberSize(2);
            solutionFound = false;
            if (~topographicMap.isFree(startPoint) || ~topographicMap.isFree(endPoint))
                msgbox('One position in the path is not free', 'Error', 'error');
            end
            % Generating RRT*-double-optimized path
%             figure;
%             hold on;
%             imagesc([0 640], [0 480], topographicMap.map);
            
            tree(1, :) = [startPoint(1) startPoint(2) startPoint(3) 0 0 1];
            tree(2, :) = [endPoint(1) endPoint(2) endPoint(3) 0 0 0];
%             plot(startPoint(1), startPoint(2), 'o');
%             plot(endPoint(1), endPoint(2), 'o');
            count = 2;
            actualTree = 1;
            k = 1;
            while k <= 5000
                sample = [randi([-width/2 width/2]) randi([-height/2 height/2]) (randi([-314 314])/100)];

                [nearstNode, index] = services.PathPlanning.getNearestNode(sample, tree, actualTree);
                newNode = services.PathPlanning.generateNewNode(nearstNode, sample, step);
                [idParent, distance] = services.PathPlanning.getParent(newNode, tree, actualTree, beta, topographicMap);
                if (idParent ~= 0)
%                     plot(newNode(1), newNode(2), 'o');
                    count = count + 1;
                    tree(count, :) = [newNode(1) newNode(2) newNode(3) idParent distance actualTree];
                    tree = services.PathPlanning.reWrite(count, newNode, tree, actualTree, beta, topographicMap);

                    if (actualTree == 1)
                        [nearestNodeOtherTree, indexNearestNode] = services.PathPlanning.getNearestNode(newNode, tree, 0);
                        actualTree = 0;
                    else
                        [nearestNodeOtherTree, indexNearestNode] = services.PathPlanning.getNearestNode(newNode, tree, 1);
                        actualTree = 1;
                    end

                    if (~services.PathPlanning.verifyCollision(nearestNodeOtherTree, newNode, topographicMap)) 
                        if (actualTree == 1)
                            primitivePath = services.PathPlanning.getPrimitivePath(tree, count, indexNearestNode);
                        else
                            primitivePath = services.PathPlanning.getPrimitivePath(tree, indexNearestNode, count);
                        end

                        optimizedPath = services.PathPlanning.optimizePath(primitivePath, topographicMap, camerasClass);

                        solutionFound = true;
                        disp('Solution Found!');
                        break;
                    end
                end

                k = k + 1
            end
            
            if (solutionFound == true)
%                 figure;
%                 hold on;
%                 imagesc([-1500 1500], [-1500 1500], topographicMap.map);
%                 for i = 1:(length(primitivePath) - 1)
%                     plot([primitivePath(i, 1) primitivePath(i+1, 1)], [primitivePath(i, 2) primitivePath(i+1, 2)], '*-');
%                 end
                
%                 figure;
%                 hold on;
%                 imagesc([-500 500], [-500 500], topographicMap.map);
%                 for i = 1:(length(optimizedPath) - 1)
%                     plot([optimizedPath(i, 1) optimizedPath(i+1, 1)], [optimizedPath(i, 2) optimizedPath(i+1, 2)], '*-');
%                 end
            else
                msgbox('A solution to one position in the path was not found, verify if robot is into the workspace', 'Error', 'error');
            end
        end
        
        function path = optimizePath(primitivePath, topographicMap, camerasClass)
            count = 1;
            index = 0;
            path(1, :) = primitivePath(1, :);
            for i = 1:(length(primitivePath)-1)
                if (i >= index)
                    for j = i+1:length(primitivePath)
                        if (services.PathPlanning.verifyCollision(primitivePath(i, :), primitivePath(j, :), topographicMap))
                            count = count + 1;
                            path(count, :) = primitivePath(j-1, :);
                            index = j - 1; 
                            break;
                        else
                            if (j == length(primitivePath))
                                count = count + 1;
                                path(count, :) = primitivePath(j, :); 
                                index = j;
                                break;
                            end
                        end
                    end
                end
            end
        end
        
        function primitivePath = getPrimitivePath(tree, indexStartTree, indexEndTree)
            count = 0;
            
            finished = false;
            index = indexEndTree;
            while finished == false
                parentIndex = tree(index, 4);
                count = count + 1;
                if (parentIndex ~= 0)
                    path(count, :) = [tree(index, 1) tree(index, 2) tree(index, 3)];
                    index = parentIndex;
                else
                    path(count, :) = [tree(index, 1) tree(index, 2) tree(index, 3)];
                    finished = true;
                end
            end  
            path = flipud(path);
            finished = false;
            index = indexStartTree;
            while finished == false
                parentIndex = tree(index, 4);
                count = count + 1;
                if (parentIndex ~= 0)
                    path(count, :) = [tree(index, 1) tree(index, 2) tree(index, 3)];
                    index = parentIndex;
                else
                    path(count, :) = [tree(index, 1) tree(index, 2) tree(index, 3)];
                    finished = true;
                end
            end
            
            primitivePath = path;
        end
        
        function [idParent, minDistance] = getParent(newNode, tree, actualTree, beta, topographicMap)
            aux = size(tree);
            lastIndexTree = aux(1);
            
            idParent = 0;
            minDistance = 1000000;
            for i = 1: lastIndexTree
                if (tree(i, 6) == actualTree)
                    Spoint = [tree(i,1) tree(i,2) tree(i,3)];
                    distance = ((newNode(1) - tree(i,1))^2 + (newNode(2) - tree(i,2))^2 + (800*(newNode(3) - tree(i,3)))^2)^0.5; 
                    if (distance < beta)    
                        distance = distance + tree(i, 5);
                        if (minDistance == 1000000)
                            if (~services.PathPlanning.verifyCollision(Spoint, newNode, topographicMap))
                                idParent = i;
                                minDistance = distance; 
                            end
                        else
                            if (distance < minDistance)
                                if (~services.PathPlanning.verifyCollision(Spoint, newNode, topographicMap))
                                    idParent = i;
                                    minDistance = distance;
                                end
                            end
                        end
                    end 
                end
            end
        end
        
        function tree = reWrite(newNodeIndex, newNode, tree, actualTree, beta, topographicMap)
            aux = size(tree);
            lastIndexTree = aux(1);

            for i = 1: lastIndexTree
                if (tree(i, 6) == actualTree)
                    if (i ~= newNodeIndex)
                        Spoint = [tree(i,1) tree(i,2) tree(i,3)];
                        distance = ((newNode(1) - tree(i,1))^2 + (newNode(2) - tree(i,2))^2 + (800*(newNode(3) - tree(i,3)))^2)^0.5; 
                        if (distance < beta)    
                            distance = distance + tree(newNodeIndex, 5);
                            if (distance < tree(i, 5))
                                if (~services.PathPlanning.verifyCollision(Spoint, newNode, topographicMap))
                                    tree(i, 4) = newNodeIndex;
                                    tree(i, 5) = distance;
                                end
                            end
                        end
                    end
                end
            end
        end
        
        function newNode = generateNewNode(nearstNode, sample, step)
            vector = sample - nearstNode;
            vector = vector / norm(vector);
            
            newNode = nearstNode + step*[vector(1) vector(2) 0] + 0.1*[0 0 vector(3)];
        end
        
        function [nearest_node, index] = getNearestNode(newNode, tree, actualTree) 
            aux = size(tree);
            lastIndexTree = aux(1);

            minDistance = 1000000;
            for i = 1:lastIndexTree
                if (tree(i, 6) == actualTree)
                    if minDistance == 1000000
                        index = i;
                        nearest_node = [tree(i, 1) tree(i, 2) tree(i, 3)];
                        minDistance = ((newNode(1) - tree(i, 1))^2 + (newNode(2) - tree(i, 2))^2 + (800*(newNode(3) - tree(i, 3)))^2)^0.5;
                    else
                        distance = ((newNode(1) - tree(i,1))^2 + (newNode(2) - tree(i,2))^2 + (800*(newNode(3) - tree(i, 3)))^2)^0.5;
                        if (distance < minDistance)
                            index = i;
                            nearest_node = [tree(i, 1) tree(i, 2) tree(i, 3)];
                            minDistance = distance;
                        end
                    end                
                end
            end
        end
        
        function response = verifyCollision(startPoint, endPoint, topographicMap)
            vector = endPoint - startPoint;
            errorPhi = endPoint(3) - startPoint(3);
            
            vectorMagnitude = norm([vector(1) vector(2) errorPhi]);
            
            n = floor(vectorMagnitude / 1);
            if (n < 10) 
                n = 10;
            end
            
            response = false;
            t = 0;
            while t <= 1
                point = startPoint + t*vector;
                if(point(3) < -pi)
                    msgbox('phi is passing -pi in pathPlanning verifyCollision');
                elseif(point(3) > pi)
                    msgbox('phi is passing +pi in pathPlanning verifyCollision');
                end
                
                if (~topographicMap.isFree(point))
                    response = true;
                    break;
                end
                
                t = t + 1/n;
            end
        end
    end
end
