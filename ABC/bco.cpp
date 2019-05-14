#include "bco.hpp"
#include <chrono>
#include <ctime>

void ABC::generate_map()
{
    for (int i = 0; i < D; ++i)
        for (int j = 0; j < D; ++j)
            if (i == j) cities[i][j] = 0; //distance between a city to itself = 0
            else cities[i][j] = 1;

    for (int i = 0; i < D; ++i) {
        for (int j = 0; j < D; ++j) if(cities[i][j] == 1) {
            int seed = 6;
//            cities[i][j] = 1 + rand()%100; //distance between city i and j
            double dist = pow(waypoints[i][0]-waypoints[j][0], 2) + pow(waypoints[i][1]-waypoints[j][1], 2);
            cities[i][j] = (int)sqrt(dist);
            cities[j][i] = cities[i][j];
        }
    }
}

//void ABC::generate_cities(int numofcities, double **waypoints) {
//    for (int i = 0; i < D; ++i)
//        for (int j = 0; j < D; ++j)
//            if (i == j) cities[i][j] = 0; //distance between a city to itself = 0
//            else cities[i][j] = 1;
//
//    for (int i = 0; i < D; ++i) {
//        for (int j = 0; j < D; ++j) if(cities[i][j] == 1) {
//                double dist = sqrt (pow((waypoints[i][0] - waypoints[j][0]),2) +
//                                 pow((waypoints[i][0] - waypoints[j][0]),2));
//                cities[i][j] = (int)dist;
//                cities[j][i] = cities[i][j];
//            }
//    }
//}

/* initialize all food sources*/
void ABC::initialization() {
    for (int i = 0; i < FoodNumber; i++) init(i);
    GlobalMin = fitness1[0];
    for (int i = 0; i < D; i++) GlobalParams[i] = Foods1[0][i];
//    for (int j = 0; j < D; j++) cout << solution[j] << " ";
//    cout << endl;
}

/* Employed Bee Phase */
void ABC::SendEmployedBees()
{
    //Send one employed bee to every food source
    for (int i = 0; i < FoodNumber; i++) {

        //mutate solution i
        generate_mutation(i);

        if (FitnessSol > fitness1[i]) { // when new solution is better, replace the old one
            trial[i] = 0;
            for (int j = 0; j < D; j++) Foods1[i][j] = solution1[j];
//            f[i] = ObjValSol;
            fitness1[i] = FitnessSol;
        }
        else { // old solution is still better, increment trial counter
            trial[i] ++;
        }
    }
}

/* A food source is chosen with the probability which is proportional to its quality
 * prob(i)=fitness(i)/sum(fitness)
 * OR prob(i)=a*fitness(i)/max(fitness)+b (this one is implemented here)
 * probability values are calculated by using fitness values and
 * normalized by dividing maximum fitness value*/
void ABC::CalculateProbabilities()
{
    double maxfit = fitness1[0];
    for (int i = 1; i < FoodNumber; i++)
        if (fitness1[i] < maxfit) maxfit = fitness1[i];
    for (int i = 0; i < FoodNumber; i++)
        prob[i] = (0.9*(fitness1[i]/maxfit)) + 0.1;
}

/* Onlooker Bee Phase */
void ABC::SendOnlookerBees()
{
    int i = 0, t = 0;
    while (t < FoodNumber) {
        if (random_0to1() < prob[i]) {
            t++;
            generate_mutation(i);

            if (FitnessSol > fitness1[i]) { //if new solution is better, replace the old one
                trial[i] = 0;
                for (int j = 0; j < D; j++) Foods1[i][j] = solution1[j];
//                f[i] = ObjValSol;
                fitness1[i] = FitnessSol;
            }
            else { //if old solution is still better, increment trial counter
                trial[i] ++;
            }
        }
        i++;
        if (i == FoodNumber - 1) i = 0;
    }
}

void ABC::MarkBestResource()
{
    for (int i = 0; i < FoodNumber; i++) {
        if (fitness1[i] < GlobalMin) {
            GlobalMin = fitness1[i];
            for (int j = 0; j < D; j++) GlobalParams[j] = Foods1[i][j];
        }
    }
}

/* Scout Bee Phase */
void ABC::SendScoutBees()
{
    //find out the food source that has maximum number of trials
    int maxtrial = 0;
    for (int i = 1; i < FoodNumber; i++) {
        if (trial[i] > trial[maxtrial]) maxtrial = i;
    }
    //Release the employed bee from his post. Make him a scout bee.
    if (trial[maxtrial] >= limit) init(maxtrial);
}

int main() {
    ABC TSP = ABC();
    //TSP.generate_cities((int)*numofcities, waypoints);
    TSP.generate_map();

    auto start = std::chrono::system_clock::now();
    TSP.initialization();
    for (int i = 0; i < maxCycle; i++) {
        TSP.SendEmployedBees();
        TSP.CalculateProbabilities();
        TSP.SendOnlookerBees();
        TSP.MarkBestResource();
        TSP.SendScoutBees();
    }
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;


    TSP.PrintMap();
    TSP.PrintOptimalSolution();
    cout << "computing time: " << elapsed_seconds.count() << "s" << endl;
    return 0;
}



//void TSPplanner(double* map, int xsize, int ysize, double* numofcities, double** waypoints, double** plan) {
//    ABC TSP = ABC(map, xsize, ysize, (int)*numofcities, waypoints);
//    TSP.generate_cities((int)*numofcities, waypoints);
//    TSP.initialization();
//    for (int i = 0; i < maxCycle; i++) {
//        TSP.SendEmployedBees();
//        TSP.CalculateProbabilities();
//        TSP.SendOnlookerBees();
//        TSP.MarkBestResource();
//        TSP.SendScoutBees();
//    }
//
//    TSP.PrintMap();
//    TSP.PrintOptimalSolution();
//    TSP.return_bestplan(plan);
//}
//
//void mexFunction( int nlhs, mxArray *plhs[],
//                  int nrhs, const mxArray*prhs[] ) {
//    /* Check for proper number of arguments */
//    if (nrhs != 3) {
//        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
//                           "Three input arguments required.");
//    } else if (nlhs != 1) {
//        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
//                           "One output argument required.");
//    }
//
//    /* get the dimensions of the map and the map matrix itself*/
//    int x_size = mxGetM(MAP_IN);
//    int y_size = mxGetN(MAP_IN);
//    double* map = mxGetPr(MAP_IN);
//    double* numofCities = mxGetPr(NUMCT_IN);
//    double** cities = (double**)mxGetPr(CITIES_IN);
//
//    int steps = *numofCities + 1;
//    BESTPLAN_OUT = mxCreateNumericMatrix( (mwSize)steps, (mwSize)2, mxINT8_CLASS, mxREAL);
//    double* bestplan = (double*) mxGetPr(BESTPLAN_OUT);
//    TSPplanner(map, x_size, y_size, numofCities, cities, &bestplan);
//    return;
//
//}
