function isCollision = CollisionDetection(robot, qMatrix, faces, vertex, faceNormals, returnOnceFound)
    % Function to check if a trajectory collides with environment obstacles
    if nargin < 6
        returnOnceFound = true;
    end
    isCollision = false;

    for qIndex = 1:size(qMatrix,1)
        tr = GetLinkPoses(qMatrix(qIndex,:), robot);
        for i = 1 : size(tr,3)-1
            for faceIndex = 1:size(faces,1)
                vertOnPlane = vertex(faces(faceIndex,1)',:);
                [intersectP, check] = LinePlaneIntersection(faceNormals(faceIndex,:), vertOnPlane, tr(1:3,4,i)', tr(1:3,4,i+1)');
                if check == 1 && IsIntersectionPointInsideTriangle(intersectP, vertex(faces(faceIndex,:)', :))
                    plot3(intersectP(1), intersectP(2), intersectP(3), 'g*');
                    disp('Intersection Detected');
                    isCollision = true;
                    if returnOnceFound
                        return
                    end
                end
            end
        end
    end
end
