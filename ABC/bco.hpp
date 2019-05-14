#include <vector>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <ctime>

#define LB -100
#define UB 100
#define D 10 //Dimension: number of cities
#define NP D
#define FoodNumber (NP/2)

//Control parameters
#define limit 220
/*A food source which could not be improved through "limit"
 * trials is abandoned by its employed bee*/
#define maxCycle 240000
/*The number of cycles for foraging {a stopping criteria}*/
#define  runtime 30
/*Algorithm can be run many times in order to see its robustness*/

using namespace std;

class ABC {
public:
    ABC() {
        vector<int> city1{15, 15}; waypoints.push_back(city1);
        vector<int> city2{30, 30}; waypoints.push_back(city2);
        vector<int> city3{40, 60}; waypoints.push_back(city3);
        vector<int> city4{70, 10}; waypoints.push_back(city4);
        vector<int> city5{20, 55}; waypoints.push_back(city5);
        vector<int> city6{80, 60}; waypoints.push_back(city6);
        vector<int> city7{45, 5}; waypoints.push_back(city7);
        vector<int> city8{15, 35}; waypoints.push_back(city8);
        vector<int> city9{70, 30}; waypoints.push_back(city9);
        vector<int> city10{62, 45}; waypoints.push_back(city10);
//        vector<int> city11{65, 25}; waypoints.push_back(city11);
//        vector<int> city12{45, 20}; waypoints.push_back(city12);
//        vector<int> city13{55, 30}; waypoints.push_back(city13);
//        vector<int> city14{10, 60}; waypoints.push_back(city14);
//        vector<int> city15{35, 5}; waypoints.push_back(city15);
//        vector<int> city16{20, 50}; waypoints.push_back(city16);
//        vector<int> city17{15, 30}; waypoints.push_back(city17);
//        vector<int> city18{50, 40}; waypoints.push_back(city18);
//        vector<int> city19{60, 10}; waypoints.push_back(city19);
//        vector<int> city20{10, 5}; waypoints.push_back(city20);

    };

    void generate_map();
    //void generate_cities(int numofcities, double** cities);
    void initialization();
    void SendEmployedBees();
    void CalculateProbabilities();
    void SendOnlookerBees();
    void MarkBestResource ();
    void SendScoutBees();


    void PrintOptimalSolution() {
        cout << "Optimal solution:" << endl;
        for (int j = 0; j < D; j++) {
            cout << (int)GlobalParams[j]  << " -> ";
        }
        cout << (int)GlobalParams[0] << endl;

        //prepare to write to cities.m
        ofstream f;
        f.open("cities.m");

        //print out matlab entries
        cout << "citiesList = [" << endl;
        f << "citiesList = [" << endl;
        for (int j = 0; j < D; j++) {
            int c = (int)GlobalParams[j];
            cout << "[" << waypoints[c][0] << " " << waypoints[c][1] << "]" << endl;
            f << "[" << waypoints[c][0] << " " << waypoints[c][1] << "]" << endl;
        }
        int c = (int)GlobalParams[0];
        cout << "[" << waypoints[c][0] << " " << waypoints[c][1] << "]" << endl;
        cout << "];" << endl;
        f << "[" << waypoints[c][0] << " " << waypoints[c][1] << "]" << endl;
        f << "];" << endl;

        cout << "Length of the generated path: " << GlobalMin << endl;
        f.close();
    }

    void PrintMap() {
        cout << "c : ";
        for (int i = 0; i < D; ++i) {
            cout << i << "  ";
            if (to_string(i).size() == 1)
                cout << " ";
        }
        cout << endl;
        for (int i = 0; i < D; ++i) {
            if (to_string(i).size() == 1)
                cout << i << " : ";
            else cout << i << ": ";
            for (int j = 0; j < D; ++j) {
                cout << cities[i][j] << " ";
                if (to_string(cities[i][j]).size() == 1)
                    cout << "  ";
                else if (to_string(cities[i][j]).size() == 2)
                    cout << " ";
            }
            cout << endl;
        }
    }

private:
    vector<vector<int>> waypoints;
    int numofcities;
    int cities[D][D];
    double Foods[FoodNumber][D]; /*Foods is the population of food sources. Each row of Foods matrix is a vector holding D parameters to be optimized. The number of rows of Foods matrix equals to the FoodNumber*/
    double Foods1[FoodNumber][D]; // For TSP
//    double f[FoodNumber];  /*f is a vector holding objective function values associated with food sources */
//    double f1[FoodNumber];
    double fitness[FoodNumber]; /*fitness is a vector holding fitness (quality) values associated with food sources*/
    double fitness1[FoodNumber];
    double trial[FoodNumber]; /*trial is a vector holding trial numbers through which solutions can not be improved*/
    double prob[FoodNumber]; /*prob is a vector holding probabilities of food sources (solutions) to be chosen*/
    double solution [D]; /*New solution (neighbour) produced by v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij}) j is a randomly chosen parameter and k is a randomly chosen solution different from i*/
    double solution1 [D];
//    double ObjValSol = 0; /*Objective function value of new solution*/
    double FitnessSol = 0; /*Fitness value of new solution*/
    int neighbour = 0, param2change = 0; /*param2change corrresponds to j, neighbour corresponds to k in equation v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij})*/
    double GlobalMin = 0; /*Optimum solution obtained by ABC algorithm*/
    double GlobalParams[D]; /*Parameters of the optimum solution*/
    double GlobalMins[runtime]; /*GlobalMins holds the GlobalMin of each run in multiple runs*/
    double r; /*a random number in the range [0,1)*/

/*a function pointer returning double and taking a D-dimensional array as argument */
/*If your function takes additional arguments then change function pointer definition and lines calling "...=function(solution);" in the code*/



    int computeFitness1 (double sol[D]) {
        int sum = 0;
        for (int i = 0; i < D-1; i++) {
            sum += cities[(int)sol[i]][(int)sol[i+1]];
        }
        sum += cities[(int)sol[D-1]][(int)sol[0]];
        return sum;
    }



    void SPV1(double sol[D]) {
        int minindex = 0;
        double min = 0;

        vector<double> copy; copy.reserve(D);
        for (int k = 0; k < D; k++) {
            copy.push_back(sol[k]);
        }

        for (int c = 0; c < D; c++) {
            minindex = 0; min = sol[0];
            for (int i = 0; i < D; i++) {
                if (sol[i] < min) {
                    min = sol[i];
                    minindex = i;
                }
            }
            solution1[minindex] = c;
            sol[minindex] = INFINITY;
        }

        //restore sol values
        for (int k = 0; k < D; k++) {
            sol[k] = copy[k];
        }
    }

    void SPV(int index) {
        int minindex = 0;
        vector<double> copy; copy.reserve(D);
        for (int k = 0; k < D; k++) {
            copy.push_back(Foods[index][k]);
        }
        double min = 0;
        for (int c = 0; c < D; c++) {
            minindex = 0; min = Foods[index][0];
            for (int i = 0; i < D; i++) {
                if (Foods[index][i] < min) {
                    min = Foods[index][i];
                    minindex = i;
                }
            }
            Foods1[index][minindex] = c;
            Foods[index][minindex] = INFINITY;
        }
        //restore Foods values
        for (int k = 0; k < D; k++) {
            Foods[index][k] = copy[k];
        }
    }

    /*
     * Variables are initialized in the range [lb,ub].
     * If each parameter has different range, use arrays lb[j], ub[j] instead of lb and ub
     * Counters of food sources are also initialized in this function
     */
    void init(int index)
    {
        //init a sequence of random continuous values
        for (int j = 0; j < D; j++) {
            r = ((double)rand()) / ((double)(RAND_MAX)+(double)(1)) ;
            Foods[index][j] = r*(UB-LB)+LB;
            solution[j] = Foods[index][j];
//            cout << Foods[index][j] << " ";
        }
//        cout << endl;
        //convert the sequence to discrete values
        SPV(index);
        //put them into TSP context
        for (int j = 0; j < D; j++) solution1[j] = Foods1[index][j];

//        f[index] = function(solution); //solution quality
        fitness1[index] = computeFitness1(solution1); //fitness: TSP path length
//        fitness[index] = computeFitness(f[index]); //fitness: ABC food source nectar amount
        trial[index] = 0; //This food source haven't been optimized
    }

    double random_0to1()
    {
        return ((double)rand() / ((double)(RAND_MAX)+(double)(1)) );
    }

    double generate_mutation_V2(int i) {

        int first = rand() % D;
        int second = rand() % D;
        while (second == first) second = rand() % D;
        for (int j = 0; j < D; j++) {
            solution[j] = Foods[i][j];
        }
        SPV1(solution);
        double temp = solution1[first];
        solution1[first] = solution1[second];
        solution1[second] = temp;
        FitnessSol = computeFitness1(solution1);

    }

    //generate mutation for solution i
    double generate_mutation(int i)
    {
        //pick a random waypoint to change on solution i
        param2change = (int)(random_0to1()*D);

        //pick a random solution to mutate
        neighbour = (int)(random_0to1()*FoodNumber);
        while (neighbour == i) {
            //ensure the selected solution is different from solution i
            neighbour = (int)(random_0to1()*FoodNumber);
        }

        /* v_{ij}=x_{ij}+\phi_{ij}*(x_{ij}-x_{kj})
         * v_i: solution mutation
         * x: Foods
         * j: param2change; k: neighbor
         * phi: r
         */
        for (int j = 0; j < D; j++) {
            solution[j] = Foods[i][j];
//            cout << Foods[i][j] << " ";
        }
//        cout << endl;
        double phi = (random_0to1()-0.5)*2;
        double x_ij = Foods[i][param2change];
        double x_kj = Foods[neighbour][param2change];
        solution[param2change] = x_ij + phi*(x_ij - x_kj); //v_ij
        //boundary check
        if (solution[param2change] < LB) solution[param2change] = LB;
        else if (solution[param2change] > UB) solution[param2change] = UB;

        //Evaluate the new solution
        SPV1(solution);
        FitnessSol = computeFitness1(solution1);
    }
};