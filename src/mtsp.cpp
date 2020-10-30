#include "gcs/planner/mtsp.h"

MTSP::MTSP()
{
  
}

void MTSP::initialiseGA(std::vector<gcs::Waypoint>& wp, int active_drones)
{
    waypoints = wp;
    number_of_drones = active_drones;
     mut_rate = 0.01;
    generations = 0;
    best_fitness = 0;
    generatePopulation();
    computeFitness();

    solve(200);

}

void MTSP::generatePopulation()
{
    for(std::size_t i = 0; i < MAX_POP; i++)
    {
        population.push_back(DNA(waypoints, number_of_drones));
    }
}

void MTSP::solve(int iter)
{
    for (std::size_t i = 0; i < iter; i++)
    {
        computeFitness();
        normalizeFitness();
        evolve();
    }
}

void MTSP::evolve()
{
    std::vector<DNA> new_pop;

    for(std::size_t i = 0; i < population.size(); i++)
    {
        new_pop.push_back(DNA(waypoints, number_of_drones));
    }

    for(std::size_t i= 0; i < population.size(); i++)
    {
        DNA parentA = pickRandomParent(population);
        DNA parentB = pickRandomParent(population);
        DNA child = parentA.crossover(parentB);
        child.mutate(mut_rate);
        new_pop.at(i) = child;
    }
    population = new_pop;
    generations++;
}

DNA MTSP::pickRandomParent(std::vector<DNA> &pop)
{
    int index = 0;
    double r = getRandomDouble<double>(0, 1);
    while(r>0)
    {
        r = r - pop.at(index).getFitness();
        index++;
    }
    index--;
    return population.at(index);

}

void MTSP::normalizeFitness()
{
    float sum = 0;
    for(std::size_t i = 0; i < population.size(); i++)
    {
        sum+=population.at(i).getFitness();
    }

    for(std::size_t i = 0; i < population.size(); i++)
    {
        double fitness = population.at(i).getFitness();
        double norm_fit = fitness / sum;
        population.at(i).setFitness(norm_fit);
    }
}

void MTSP::computeFitness()
{
    for(std::size_t i = 0; i < population.size(); i++)
    {
        population.at(i).computeFitness();
        double fit = population.at(i).getFitness();
        if(fit > best_fitness)
        {
            best_fitness = fit;
            best_gene = population.at(i);
        }
    }
}

std::vector<std::vector<size_t>> MTSP::getBestOrder()
{
    return best_gene.getSolution();
}