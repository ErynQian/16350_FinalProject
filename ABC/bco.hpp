#include <vector>
#include <stdlib.h>
#include <iostream>
#include <math.h>

#define LB -100
#define UB 100
#define D 20 //Dimension: number of cities
#define NP D
#define FoodNumber (NP/2)

//Control parameters
#define limit 100
/*A food source which could not be improved through "limit"
 * trials is abandoned by its employed bee*/
#define maxCycle 2500
/*The number of cycles for foraging {a stopping criteria}*/
#define  runtime 30
/*Algorithm can be run many times in order to see its robustness*/

using namespace std;

//benchmark function
double sphere(double sol[D])
{
    int j;
    double top=0;
    for(j=0;j<D;j++)
    {
        top=top+sol[j]*sol[j];
    }
    return top;
}

class ABC {
public:
    ABC() {}

    void generate_map();
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

    int cities[D][D];
    double Foods[FoodNumber][D]; /*Foods is the population of food sources. Each row of Foods matrix is a vector holding D parameters to be optimized. The number of rows of Foods matrix equals to the FoodNumber*/
    double Foods1[FoodNumber][D]; // For TSP
    double f[FoodNumber];  /*f is a vector holding objective function values associated with food sources */
    double f1[FoodNumber];
    double fitness[FoodNumber]; /*fitness is a vector holding fitness (quality) values associated with food sources*/
    double fitness1[FoodNumber];
    double trial[FoodNumber]; /*trial is a vector holding trial numbers through which solutions can not be improved*/
    double prob[FoodNumber]; /*prob is a vector holding probabilities of food sources (solutions) to be chosen*/
    double solution [D]; /*New solution (neighbour) produced by v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij}) j is a randomly chosen parameter and k is a randomly chosen solution different from i*/
    double solution1 [D];
    double ObjValSol = 0; /*Objective function value of new solution*/
    double FitnessSol = 0; /*Fitness value of new solution*/
    int neighbour = 0, param2change = 0; /*param2change corrresponds to j, neighbour corresponds to k in equation v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij})*/
    double GlobalMin = 0; /*Optimum solution obtained by ABC algorithm*/
    double GlobalParams[D]; /*Parameters of the optimum solution*/
    double GlobalMins[runtime]; /*GlobalMins holds the GlobalMin of each run in multiple runs*/
    double r; /*a random number in the range [0,1)*/

/*a function pointer returning double and taking a D-dimensional array as argument */
/*If your function takes additional arguments then change function pointer definition and lines calling "...=function(solution);" in the code*/
    typedef double (*FunctionCallback)(double sol[D]);
    FunctionCallback function = &sphere;

    double computeFitness (double f)
    {
        if (f >= 0) return 1/(f+1);
        else return 1+fabs(f);
    }

    int computeFitness1 (double sol[D]) {
        int sum = 0;
        for (int i = 0; i < D-1; i++) {
            sum += cities[(int)sol[i]][(int)sol[i+1]];
        }
        sum += cities[(int)sol[D-1]][(int)sol[0]];
        return sum;
    }



    void SPV1(double sol[D]) {
        int c = 0, minindex = 0;
        double min = 0;
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
    }

    void SPV(int index) {
        int c = 0, minindex = 0;
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
        }
        //convert the sequence to discrete values
        SPV(index);
        //put them into TSP context
        for (int j = 0; j < D; j++) solution1[j] = Foods1[index][j];

        f[index] = function(solution); //solution quality
        fitness1[index] = computeFitness1(solution1); //fitness: TSP path length
        fitness[index] = computeFitness(f[index]); //fitness: ABC food source nectar amount
        trial[index] = 0; //This food source haven't been optimized
    }

    double random_0to1()
    {
        return ((double)rand() / ((double)(RAND_MAX)+(double)(1)) );
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
        for (int j = 0; j < D; j++) solution[j] = Foods[i][j];
        double phi = (random_0to1()-0.5)*2;
        double x_ij = Foods[i][param2change];
        double x_kj = Foods[neighbour][param2change];
        solution[param2change] = x_ij + phi*(x_ij - x_kj); //v_ij
        //boundary check
        if (solution[param2change] < LB) solution[param2change] = LB;
        else if (solution[param2change] > UB) solution[param2change] = UB;

        //Evaluate the new solution
        ObjValSol = function(solution);
        SPV1(solution);
        FitnessSol = computeFitness1(solution1);
    }
};