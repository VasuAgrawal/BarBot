numTests = 1000;
for i = 0:numTests
    testPoint = rand([3,1])*15; % test point between 0-1
   
    rotation = rand([3,1])*6*pi - 2*pi; % rotation between -2pi - 4pi
    
    outputPoint = Project(testPoint, rotation);
    if abs(testPoint - outputPoint) > 0.0001
        fprintf('Test %d failed!', i);
        testPoint
        rotation
        outputPoint
    end
end

