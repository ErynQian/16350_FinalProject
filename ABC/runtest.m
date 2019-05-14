
function[finalrobotx, finalroboty] = runtest(mapfile, robotstart, targetstart)

envmap = load(mapfile);





%current positions of the target and robot
robotpos = robotstart;
targetpos = targetstart;

%now comes the main loop
hr = -1;
ht = -1;
numofmoves = 0;
caught = 0;
for i = 1:2000

    %draw the positions
    if (hr ~= -1)
        delete(hr);
        %delete(ht);
    end;
    
    hr = text(robotpos(1), robotpos(2), 'R');
    %ht = text(targetpos(1), targetpos(2), 'T');

    pause(0.1);
    %pause();
    
    %call robot planner to find what they want to do
    tStart = tic;
    newrobotpos = robotplanner(envmap, robotpos, targetpos); 
    
    %compute movetime for the target
    tElapsed = toc(tStart);
    timeTaken = tElapsed*1000; % in ms
    
    movetime = max(1, ceil(timeTaken/200));
    
    %check that the new commanded position is valid
    if (newrobotpos(1) < 1 | newrobotpos(1) > size(envmap, 1) | ...
            newrobotpos(2) < 1 | newrobotpos(2) > size(envmap, 2))
        fprintf(1, 'ERROR: out-of-map robot position commanded\n');
        return;
    elseif (envmap(newrobotpos(1), newrobotpos(2)) ~= 0)
        fprintf(1, 'ERROR: invalid robot position commanded\n');
        return;
    elseif (abs(newrobotpos(1)-robotpos(1)) > 1 | abs(newrobotpos(2)-robotpos(2)) > 1)
        fprintf(1, 'ERROR: invalid robot move commanded\n');
        return;
    end;     

    text(newrobotpos(1), newrobotpos(2), 'R');   

    %call target planner to see how they move within the robot planning
    %time
    %newtargetpos = targetplanner(envmap, robotpos, targetpos, targetstart, movetime);
    newtargetpos = targetpos;
       
    %make the moves
    robotpos = newrobotpos;
    targetpos = newtargetpos;
    numofmoves = numofmoves + 1;
    
    %check if target is caught
    if (abs(robotpos(1)-targetpos(1)) <= 1 & abs(robotpos(2)-targetpos(2)) <= 1)
        caught = 1;
        finalrobotx = robotpos(1);
        finalroboty = robotpos(2);
        break;
    end;
    
end;

fprintf(1, 'target caught=%d number of moves made=%d\n', caught, numofmoves);