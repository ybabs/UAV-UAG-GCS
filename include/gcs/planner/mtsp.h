#ifndef MTSP_H
#define MTSP_H

#include <QObject>
#include <vector>
#include <memory>
#include "dna.h"

#define MAX_POP 100

class MTSP: public QObject
{
    Q_OBJECT
private:
    std::vector<DNA> population;
    std::vector<gcs::Waypoint> waypoints;
    std::size_t number_of_drones;
    int generations;
    DNA best_gene;
    double best_fitness;
    int pop_size;
    float mut_rate;
public:
    MTSP();
    void initialiseGA(std::vector<gcs::Waypoint>& wp, int active_drones);
    DNA pickRandomParent(std::vector<DNA> &population);
    void solve(int num_iter);
    void evolve();
    void generatePopulation();
    void selection();
    void computeFitness();
    void normalizeFitness();
    std::vector<std::vector<size_t>> getBestOrder();

};

#endif // POPULATION_H
