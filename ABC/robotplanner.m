function[newrobotpos] = robotplanner(envmap, robotpos, targetpos);

MEX = 1;

numofdirs = 8;
dX = [-1 -1 -1  0  0  1 1 1];
dY = [-1  0  1 -1  1 -1 0 1];

%failed to find an acceptable move
newrobotpos = robotpos;

if (MEX == 1)
	%if using MEX, here you would call the planner
	action = planner(envmap, robotpos, targetpos);
	newrobotpos = robotpos+cast(action,'like',robotpos);

else
	%otherwise do planning right here 

	%for now greedily move towards the target, 
	%but this is where you can put your planner 
	mindisttotarget = 1000000;
	for dir = 1:numofdirs
    	   newx = robotpos(1) + dX(dir);
     	   newy = robotpos(2) + dY(dir);
    
    	   if (newx >= 1 & newx <= size(envmap, 1) & newy >= 1 & newy <= size(envmap, 2))
              if (envmap(newx, newy) == 0)
                disttotarget = sqrt((newx-targetpos(1))^2 + (newy-targetpos(2))^2);
                if(disttotarget < mindisttotarget)
                  mindisttotarget = disttotarget;
                  newrobotpos(1) = newx;
                  newrobotpos(2) = newy;
                end;
              end;
    	   end;
	end;
end;
