#include "bco.hpp"

void ABC::generate_map()
{
    for (int i = 0; i < D; ++i)
        for (int j = 0; j < D; ++j)
            if (i == j) cities[i][j] = 0;
            else cities[i][j] = 1;

    for (int i = 0; i < D; ++i) {
        for (int j = 0; j < D; ++j) if(cities[i][j] == 1) {
            cities[i][j] = 1 + rand()%100;
            cities[j][i] = cities[i][j];
        }
    }
}

/* initialize all food sources*/
void ABC::initialization() {
    for (int i = 0; i < FoodNumber; i++) init(i);
    GlobalMin = fitness1[0];
    for (int i = 0; i < D; i++) GlobalParams[i] = Foods1[0][i];
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
            f[i] = ObjValSol;
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
                f[i] = ObjValSol;
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
    TSP.generate_map();
    TSP.initialization();
    for (int i = 0; i < maxCycle; i++) {
        TSP.SendEmployedBees();
        TSP.CalculateProbabilities();
        TSP.SendOnlookerBees();
        TSP.MarkBestResource();
        TSP.SendScoutBees();
    }

    TSP.PrintMap();
    TSP.PrintOptimalSolution();
    return 0;
}
