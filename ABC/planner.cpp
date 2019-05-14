/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <functional>
#include <queue>
#include <vector>
#include <iostream>

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ROBOT_IN	prhs[1]
#define	GOAL_IN     prhs[2]


/* Output Arguments */
#define	ACTION_OUT	plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

using namespace std;

typedef struct {
    int x;
    int y;
    double g;
    double h;
    double f;
} state_t;

struct CompareState { 
    bool operator()(state_t const& s1, state_t const& s2)
    { 
        return s1.f > s2.f;
    } 
}; 

int call = 0;

class AstarPlanner {
public:
    AstarPlanner(
            double *map,
            int x_size,
            int y_size,
            int robotposeX,
            int robotposeY,
            int goalposeX,
            int goalposeY
    );
        
    void planner(char *p_actionX, char *p_actionY);

    
private:
    //environment params
    double* map;
    int x_size;
    int y_size;
    int mapsize;
    int robotposeX;
    int robotposeY;
    int goalposeX;
    int goalposeY;

    //8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    //planner params
    vector<double> gvalues;
    //bool cmp = [](state_t &s1, state_t &s2){return (s1.f) < (s2.f);};
    bool goal_expanded;
    //std::priority_queue<state_t, vector<state_t>, decltype(cmp)> open;
    std::priority_queue<state_t, vector<state_t>, CompareState> open;
    vector<int> close; //1 for close; 2 for open; 0 for not expanded

    void update_gvalue(int x, int y, double new_gval);
    double get_gvalue(int x, int y);
    double get_hvalue(int x, int y);
    double get_cost(state_t s1, state_t s2);
    double get_cost(int x1, int y1, int x2, int y2);
    state_t init_state(int x, int y);
    void close_state(int x, int y);
    void open_state(int x, int y);
    vector<state_t> successor_list(int x, int y);
    void least_cost_backtrack(char *p_actionX, char *p_actionY);




};

AstarPlanner::AstarPlanner(
        double *map,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int goalposeX,
        int goalposeY) :
        map(map),
        x_size(x_size),
        y_size(y_size),
        mapsize((x_size+1)*(y_size+1)),
        robotposeX(robotposeX),
        robotposeY(robotposeY),
        goalposeX(goalposeX),
        goalposeY(goalposeY),
        goal_expanded(false),
        gvalues(mapsize, INFINITY),
        close(mapsize, 0)
{
    //initialize start state to g=0
    update_gvalue(robotposeX, robotposeY, 0);

    //initialize open queue
    state_t start = init_state(robotposeX, robotposeY);
    open.push(start);
}

//ComputePath function
//while(sgoal is not expanded and OPEN ≠ 0)
//  remove s with the smallest [f(s) = g(s)+h(s)] from OPEN;
//  insert s into CLOSED;
//  for every successor s’ of s such that s’ not in CLOSED
//      if g(s’) > g(s) + c(s,s’)
//          g(s’) = g(s) + c(s,s’);
//          insert s’ into OPEN;

void AstarPlanner::planner(char *p_actionX, char *p_actionY) {
    //Compute Path
    while (!goal_expanded && !open.empty())
    {
        state_t s = open.top();
        open.pop();
        close_state(s.x, s.y);
        //mexPrintf("insert state (%d,%d) into CLOSE\n", s.x, s.y);
        if (s.x == goalposeX && s.y == goalposeY) goal_expanded = true;
        for (state_t nexts : successor_list(s.x, s.y))
        {
            //double g_nexts = get_gvalue(nexts.x, nexts.y);
            double cost = get_cost(s, nexts);
            s.g = get_gvalue(s.x, s.y);
            if (nexts.g > s.g + cost)
                update_gvalue(nexts.x, nexts.y, s.g+cost);
            int stat = close[GETMAPINDEX(nexts.x, nexts.y, x_size, y_size)];
            //stat = 1 for close; 2 for open; 0 for not expanded
            if (stat == 0) open_state(nexts.x, nexts.y);
            //mexPrintf("insert state (%d,%d) into OPEN\n", nexts.x, nexts.y);
        }
    }
    //Backtracking least-cost path
    least_cost_backtrack(p_actionX, p_actionY);
}

void AstarPlanner::least_cost_backtrack(char *p_actionX, char *p_actionY) {
    bool reached_startpoint = false;
    int trackstate_x = goalposeX; int trackstate_y = goalposeY;
    int dir;
    double min_gstar, gstar;
    int min_x = trackstate_x;
    int min_y = trackstate_y;
    while (!reached_startpoint) {
        min_gstar = INFINITY;
        for (dir = 0; dir < NUMOFDIRS; dir++) {
            int newx = trackstate_x + dX[dir];
            int newy = trackstate_y + dY[dir];
            if (newx == robotposeX && newy == robotposeY) {
                reached_startpoint = true;
                *p_actionX = -dX[dir];
                *p_actionY = -dY[dir];
                break;
            }

            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) {
                gstar = get_gvalue(newx, newy) +
                        get_cost(newx, newy, trackstate_x, trackstate_y);
                if (gstar <= min_gstar) {
                    min_gstar = gstar;
                    min_x = newx;
                    min_y = newy;
                }
            }
        }
        trackstate_x = min_x;
        trackstate_y = min_y;
    }
}

//heuristic: euclidean distance
double AstarPlanner::get_hvalue(int x, int y) {
    double dist = sqrt(((x-goalposeX)*(x-goalposeX) +
            (y-goalposeY)*(y-goalposeY)));
    return dist;
}

double AstarPlanner::get_gvalue(int x, int y) {
    return gvalues[GETMAPINDEX(x,y,x_size,y_size)];
}


void AstarPlanner::update_gvalue(int x, int y, double new_gval) {
    gvalues[GETMAPINDEX(x,y,x_size,y_size)] = new_gval;
}

state_t AstarPlanner::init_state(int x, int y) {
    state_t s;
    s.x = x;
    s.y = y;
    s.g = get_gvalue(x,y);
    s.h = get_hvalue(x,y);
    s.f = s.g + s.h;
    return s;
}

void AstarPlanner::close_state(int x, int y) {
    close[GETMAPINDEX(x,y,x_size,y_size)] = 1;
}

void AstarPlanner::open_state(int x, int y) {
    open.push(init_state(x,y));
    close[GETMAPINDEX(x,y,x_size,y_size)] = 2;
}


vector<state_t> AstarPlanner::successor_list(int x, int y) {
    vector<state_t> SL;
    int newx, newy;
    for(int dir = 0; dir < NUMOFDIRS; dir++)
    {
        newx = x + dX[dir];
        newy = y + dY[dir];
        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) {
            int mapidx = GETMAPINDEX(newx, newy, x_size, y_size);
            if (close[mapidx] == 0 && map[mapidx] == 0) //not expanded & free
                SL.push_back(init_state(newx, newy));
        }
    }
    return SL;
}

double AstarPlanner::get_cost(state_t s1, state_t s2) {
    if (s1.x == s2.x && s1.y == s2.y) return 0;
    else if (s1.x == s2.x || s1.y == s2.y) return 1;
    else return sqrt(2);
}

double AstarPlanner::get_cost(int x1, int y1, int x2, int y2) {
    if (x1 == x2 && y1 == y2) return 0;
    else if (x1 == x2 || y1 == y2) return 1;
    else return sqrt(2);
}




//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector <x,y> for the robot pose
//3rd is a row vector <x,y> for the target pose
//plhs should contain output parameters (1): 
//1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[] )
     
{
    /* Check for proper number of arguments */    
    if (nrhs != 3) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Three input arguments required."); 
    } else if (nlhs != 1) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/     
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");         
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/     
    int goalpose_M = mxGetM(GOAL_IN);
    int goalpose_N = mxGetN(GOAL_IN);
    if(goalpose_M != 1 || goalpose_N != 2){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
                "goalpose vector should be 1 by 2.");         
    }
    double* goalposeV = mxGetPr(GOAL_IN);
    int goalposeX = (int)goalposeV[0];
    int goalposeY = (int)goalposeV[1];
        
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxINT8_CLASS, mxREAL); 
    char* action_ptr = (char*)  mxGetPr(ACTION_OUT);
            
    /* Do the actual planning in a subroutine */
    AstarPlanner myPlanner(map, x_size, y_size, robotposeX, robotposeY, goalposeX, goalposeY);
    myPlanner.planner(&action_ptr[0], &action_ptr[1]);
    //planner(map, x_size, y_size, robotposeX, robotposeY, goalposeX, goalposeY, &action_ptr[0], &action_ptr[1]);
    return;
    
}





