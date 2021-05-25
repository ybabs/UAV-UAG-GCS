#include "gcs/planner/dna.h"
#include "gcs/utils/utils.h"

DNA::DNA(std::vector<gcs::Waypoint> &wp, int num_drones)
     :bestOrder(num_drones, std::vector<std::size_t>(0))
{
    number_of_cities = wp.size()-2;
    number_of_drones = num_drones;
    waypoints = wp;

    //initialise route
    for(std::size_t i = 1; i <= number_of_cities; i++)
    {
        order.push_back(i);
    }

    std::random_shuffle(order.begin(), order.end());
}

DNA::DNA()
{

}

DNA::DNA(std::vector<std::size_t> &ord, std::vector<gcs::Waypoint>& wp, int num_drones)
      :bestOrder(num_drones, std::vector<std::size_t>(0))
{
    number_of_cities = wp.size();
    number_of_drones = num_drones;
    waypoints = wp;
    record_distance = 100000;
    order = ord;
}

void DNA::computeFitness()
{

    //split the waypoints
    splitWaypoints(number_of_drones);
    for(std::size_t i = 0; i < number_of_drones; i++)
    {
        solution.at(i).insert(solution.at(i).begin(), 0);
        solution.at(i).insert(solution.at(i).end(), waypoints.size()-1);
    }
     double total_distance = 0;

    for(std::size_t i = 0; i < number_of_drones; ++i)
    {

        std::vector<std::size_t> curr_order = solution[i];
        std::vector<gcs::Waypoint> route_waypoint;
        for(std::size_t j = 0; j < curr_order.size(); j++ )
        {
            int index = curr_order[j];
            route_waypoint.push_back(waypoints.at(index));

        }
        double distance = routeModel.GetRouteDistance(route_waypoint);
        total_distance+=distance;
    }

    record_distance = total_distance;
    //fitness = record_distance;
    // ROS_INFO("Best distance is %f", record_distance);
     fitness = 1/pow(total_distance, 2);
    //fitness = total_distance;
}

void DNA::mutate(float mutationRate)
{
    double r = ((double) rand() / (RAND_MAX));
    if(r < mutationRate)
    {
        auto indexA = rand() % (this->getOrder().size());
       int indexB = (indexA + 1) % (this->getOrder().size());
        swap(order, indexA, indexB);
    }


}

void DNA::setFitness(double fitness_)
{
    fitness = fitness_;
}

void DNA::swap(std::vector<std::size_t>&route, int i, int j)
 {
     int temp = route[i];
     route[i] = route[j];
     route[j] = temp;
 }


DNA DNA::crossover( DNA &parentB)
{

//    int start = getRandomInt<std::size_t>(1, this->getOrder().size()-2);
//    int end =  getRandomInt<std::size_t>(start +1 , this->getOrder().size()-2);
    int start = rand () % (this->getOrder().size()-1);
    int end =  getRandomInt<std::size_t>(start +1 , this->getOrder().size()-1);
         // pick a random number from "start" and omit the last
         // element in the array
     //int end = start + 1 + rand() % ((this->getOrder().size()) - start +1);

    std::vector<std::size_t>::const_iterator first = this->getOrder().begin() + start;
    std::vector<std::size_t>::const_iterator second = this->getOrder().begin() + end;

     std::vector<std::size_t> newOrder(first, second);

     for(std::size_t i = 0; i < parentB.getOrder().size(); i++)
      {
          int city = parentB.getOrder().at(i);
          if(std::find(newOrder.begin(), newOrder.end(), city) == newOrder.end())
          {
              //std::cout<< "Move on" <<std::endl;
              newOrder.push_back(city);
          }
      }


    return DNA(newOrder, waypoints, number_of_drones);


}

void DNA::setWaypoints(std::vector<gcs::Waypoint>& wp)
{
    number_of_cities = wp.size();
    waypoints = wp;
}
void DNA::setNumDrones(int nd)
{
     number_of_drones = nd;
}


const std::vector<std::size_t>& DNA::getOrder() const
{
    return order;
}

std::vector<std::vector<std::size_t>>& DNA::getSolution()
{
    return solution;
}

double DNA::getFitness()
{
    return fitness;
}

void DNA::splitWaypoints(size_t n)
 {
    size_t len_wp = order.size() ;
     size_t length = len_wp /n;
     size_t remain =  len_wp % n;
     size_t begin = 0;
     size_t end = 0;

    for(std::size_t i = 0; i < std::min(n, len_wp); i++)
    {
        end += (remain > 0) ? (length + !!(remain--)) : length;

        solution.push_back(std::vector<std::size_t>(order.begin() + begin,
                                                    order.begin() + end));

        begin = end;
    }
 }
