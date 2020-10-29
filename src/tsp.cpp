#include "gcs/planner/tsp.h"

TSP::TSP()
{
    
}

/// swap positions of coordinates in an array based on the element position
/// @param gps_list reference to GPS coordinate vector containing elements
/// @param i First position to be swapped
/// @param j second position to be swapped
/// @returns nothing
void TSP::swap(std::vector<gcs::Waypoint> &gps_list, int i, int j)
{
    gcs::Waypoint temp = gps_list[i];
    gps_list[i] = gps_list[j];
    gps_list[j] = temp;
}

/// swap positions of route orders in an array based on the element position
/// @param route reference to order of positions containing elements
/// @param i First position to be swapped
/// @param j second position to be swapped
/// @returns nothing
 void TSP::swap(std::vector<int>&route, int i, int j)
 {
     int temp = route[i];
     route[i] = route[j];
     route[j] = temp;
 }

 /// shuffles the position of elements within the vector
/// @param route vector to be shuffled
/// @returns nothing
 void TSP::shuffle(std::vector<int> &route)
 {
     // shuffle only the elements that contain the route and
     // skip the starting points
     for (std::size_t i = route.size()-2; i >1; --i)
     {
         int indexA = rand() % (route.size()-2) + 1;
         int indexB = rand() % (route.size()-2) + 1;
         swap(route, indexA, indexB);
     }
 }

  /// normalises the fitness value
 /// @returns nothing
 void TSP::normalizeFitness()
 {
     double sum = 0;
     for(std::size_t i = 0; i < fitness.size(); i++)
     {
         sum+= fitness.at(i);
     }
     for(std::size_t i = 0; i< fitness.size(); i++)
     {
         fitness.at(i) = fitness.at(i)/sum;
     }
 }

 /// computes a fitness value based on the distance of each population
 /// @returns nothing
 void TSP::computeFitness()
 {
     double currentRecord = 100000;
     for(std::size_t i =0; i < population.size(); i++)
     {
        double distance = routeModel.GetRouteDistance(waypoints, population[i]);
        if(distance < recordDistance)
        {
            recordDistance = distance;
            bestOrder = population[i];
            // for(int i = 0; i < bestOrder.size(); i++)
            // {
            //     ROS_INFO("Order --- %d", bestOrder[i]);
            // }
        }
        if(distance < currentRecord)
        {
            currentRecord = distance;
            currentBestOrder = population[i];
            //ROS_INFO("Current Best Distance %f ", currentRecord);
        }
        fitness.at(i) = 1/(pow(distance, 8) + 1);
     }

 }

 /// initialises and populates the fitness, order and ppopulation
 /// @param wp vector containing GPS waypoints
 /// @param MAX_POP maximum number of members of the population
 /// @returns nothing
 void TSP::initialiseGA(std::vector<gcs::Waypoint>& wp, int MAX_POP )
 {
     // delete previous instances of the run
     resetVariables();
     waypoints = wp;
     recordDistance = routeModel.GetRouteDistance(waypoints);

     
     // create a first set of routes based on initial waypoints
     for(std::size_t i = 0; i < waypoints.size(); i++)
     {
         order.push_back(i);
         bestOrder.push_back(i);
        //  ROS_INFO("Lat: %f", waypoints.at(i).latitude);
     }

     // initialise fitness and best route
     // to zero
     for(int i = 0; i < MAX_POP; i++)
     {
         fitness.push_back(0);
     }

     // add list of random routes
     // and shuffle population
     for(int i = 0; i < MAX_POP; i++ )
     {
         population.push_back(order);
         shuffle(population.at(i));
     }
 }

/// Compute the next generation of population.
///
/// Uses a randomised method to pick a parents
/// based on fitness to create a child which is
/// inserted into the next generation of population
 void TSP::nextGeneration()
 {
     // create a new array for the next generation;
     std::vector<std::vector<int>> new_pop;
     // initialise new population to and set previous orders
     for(std::size_t i = 0; i < population.size(); i++)
     {
         new_pop.push_back(order);
     }

     // pick random parents well, bias towards parents with a higher
     // fitness. Then perform crossover and mutation. Mutation rate
     // set to 1% for the moment.
     for(std::size_t i = 0; i < population.size(); i++)
     {
         std::vector<int> parentA = pickRandomParent(population, fitness);
         std::vector<int> parentB = pickRandomParent(population, fitness);
         std::vector<int> child = crossover(parentA, parentB);
         mutate(child, 0.01);

         // reinsert the starting points into the first and last elements
         // of the vector and make circular to fulfill TSP requirements
         child.insert(child.begin(), 0);
         child.insert(child.end(), parentA.size()-1);
         new_pop[i] = child;
     }
     population = new_pop;
 }

 /// pick random parent based on fitness
 /// @param population Population of list of route orders
 /// @param fitness vector containing corresponding fitness values of route orders
 /// @returns a Parent which is made up of an order.
 std::vector<int> TSP::pickRandomParent(std::vector<std::vector<int> > &population, std::vector<float>& fitness)
 {
     int index = 0;
     // returns a random number between 0 and 1;
     double r = ((double) rand() / (RAND_MAX));
     while (r > 0)
     {
         r = r - fitness.at(index);
         index++;
     }
     index--;
     return population.at(index);
 }

  /// performs mutation on the child using a given mutation rate
 /// @param order Child to be mutated
 /// @param mutationRate mutation rate
 /// @returns nothing
 void TSP::mutate(std::vector<int> &order, float mutationRate)
 {
     for(std::size_t i = 0; i < waypoints.size(); i++)
     {
        double r = ((double) rand() / (RAND_MAX));
        if(r < mutationRate)
        {
            // omit the first and last indexes to preserve the
            // circular requirement of home and finishing
            // positions
            int indexA = rand() % (order.size()-2) + 1;
            int indexB = (indexA + 1) % (waypoints.size()-2);
            swap(order, indexA, indexB);
        }
     }
 }

 /// creates a new child based on both parents
 /// @param parentA the first parent
 /// @param parentB the second parent
 /// @returns a child with a route missing two elements.
 /// @see nextGeneration()
 std::vector<int> TSP::crossover(std::vector<int> &parentA, std::vector<int> &parentB)
 {
     // pick a random number from 1 and omit the last element
     // in the array
     int start = rand () % (parentA.size()-2) + 1 ;
     // pick a random number from "start" and omit the last
     // element in the array
     int end = start + 1 + rand() % ((parentA.size()-2) - start +1);

     //create a new vector based from the parent with index (start, end)
     std::vector<int>::const_iterator first = parentA.begin() + start;
     std::vector<int>::const_iterator second = parentA.begin() + end;
     std::vector<int> newOrder(first, second);

     // Add remaining chromosomes from parentB. Check to see if they
     // are missing from ParentA first
     for(std::size_t i = 1; i <= parentB.size()-2; i++)
     {
         int city = parentB.at(i);
         if(std::find(newOrder.begin(), newOrder.end(), city) == newOrder.end())
         {
             //std::cout<< "Move on" <<std::endl;
             newOrder.push_back(city);
         }
     }
     return newOrder;
 }

/// Accessor method to return best order after running 
/// TSP Algorithm
/// @returns vector containing best route. 
std::vector<int> TSP::getBestOrder()
{
    // for(int i = 0; i < bestOrder.size(); i++)
    // {
    //     ROS_INFO("Order --- %d", bestOrder[i]);
    // }
     return bestOrder;
     
}

double TSP::getBestDistance()
{
    return recordDistance;
}

/// Clears all variables from previous run
/// @returns nothing
void TSP::resetVariables()
{
    bestOrder.clear();
    route.clear();
    waypoints.clear();
    fitness.clear();  
    currentBestOrder.clear();
    order.clear();
    population.clear();

}