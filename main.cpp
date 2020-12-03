#include "network.h"
#include<cmath>
#include <vector>
#include<algorithm>
#include <typeinfo>
#include <map>

double to_rad(double degree) {
	double pi_ = std::acos(0.0);
    return degree/180 * pi_;
}

double calculate_distance(const row &station1, const row &station2)
{
    double dist{};
    dist = std::sin(to_rad(station1.lat)) * std::sin(to_rad(station2.lat)) + 
    		std::cos(to_rad(station1.lat)) * std::cos(to_rad(station2.lat)) * 
    		std::cos(to_rad(station1.lon - station2.lon));
    //got dist in radian, no need to change back to degree and convert to rad again.
    return 6356.752 * std::acos(dist);
}

//prints a vector
template <typename T>
void display(std::vector<T> &vec){
	std::cout<<"[";
	for(const auto &v:vec){
		std::cout<<v<<" ";
	}
	std::cout<<"]"<<std::endl;
}

class refuel{
public:
	std::string charger_name;
	int charger_idx;
	double charge_time_hrs;
	// Constructor
	//refuel() = default;
	refuel(std::string name, int idx, double t);
	~refuel(){};

};

refuel::refuel(std::string name, int idx, double t):charger_name{name}, charger_idx{idx}, charge_time_hrs{t}{
}

std::ostream &operator<<(std::ostream &os, const refuel &rhs){
	os << rhs.charger_name << " : " << rhs.charge_time_hrs;
	return os;
}

class Planner {
	public:
		const int source;
		const int destination;
		std::vector<refuel> path;
		std::map<int, std::vector<refuel>> path_dict;
		Planner(int s, int d):source{s}, destination{d}{};
		void compute_path(int local_goal);
};

void Planner::compute_path(int local_goal){
	static std::vector<int> visited_idx{};
	visited_idx.push_back(local_goal);
	// if in hash map, return
	if(path_dict.find(local_goal)!= path_dict.end()){
		return;// path_dict[local_goal];
	}

	std::vector<int> reach_idx{}; 
	for(size_t i{0}; i<network.size(); i++){
		// Find nodes which are reachable
		if (calculate_distance(network[local_goal], network[i]) < 320){
			// Discard nodes already visited
			if(std::find(visited_idx.begin(), visited_idx.end(), i) == visited_idx.end()){
				reach_idx.push_back(i);
			}
		}
	}

	auto it = std::find(reach_idx.begin(), reach_idx.end(), source);
	if(it !=reach_idx.end()){
		// source in reach
		double k= 0.0;
		path_dict[local_goal].emplace_back(network[source].name, source, k);
		path_dict[local_goal].emplace_back(network[local_goal].name, local_goal, k);
		//display(path);
		//return path_dict[local_goal];
	}

	//Compute path for all reachable nodes
	for(const int &i:reach_idx){
		compute_path(i);
	}

	// Find min time path from a given path
	int min_time_idx{};
	double min_path_time{1000000};
	double charging_time = 0.0;
	double min_charging_time = 0.0;
	for(int i:reach_idx){
		double time = 0;
		for(size_t j{0}; j<path_dict[i].size(); j++){
			std::cout<<"local_goal"<<local_goal<<std::endl;
			if (j==path_dict[i].size()-1){
				double dist_ = calculate_distance(network[path_dict[i][j].charger_idx], network[local_goal]);
				time += dist_ / 105.0; // Distance time 
				time += dist_ / network[path_dict[i][j].charger_idx].rate; // Charging time;
				charging_time = dist_ / network[path_dict[i][j].charger_idx].rate;
			}
			else{
			time += calculate_distance(network[path_dict[i][j].charger_idx], network[path_dict[i][j+1].charger_idx]) / 105.0; // Distance time 
			std::cout<<"before" <<time<<std::endl;
			time += path_dict[i][j].charge_time_hrs; // Charging time;
			std::cout<<"after" <<time<<std::endl;
			}

		}
		// add reach to goal time.
		if (time < min_path_time){
			min_path_time = time;
			min_charging_time = charging_time;
			min_time_idx = i;
		}
	}

	path_dict[local_goal] = path_dict[min_time_idx];
	path_dict[local_goal].emplace_back(network[local_goal].name, local_goal, min_charging_time);
	//for_each(reach_idx.begin(), reach_idx.end(), [](const int &x){std::cout<<network[x].name<<std::endl;});
	return;
}



//,  row find_station_by_name(const std::sting &station_name, const std::array<row,303> &all_stations){
// 	for(auto &s:all_stations){
// 		if 
// 	}
// }





void test_calculate_distance(){
	row s1 {"blah", 53.3205, -1.7297, 1.0};
    row s2 {"blah1", 53.3186,-1.6997,1.0};
    std::cout<< "Distance Test: " << calculate_distance(s1, s2)<<std::endl;
}

void test_refuel_class(){
	refuel r{"Albany_NY", 0, 1.5};
	std::cout << r << std::endl;
}


void tests(){
	test_refuel_class();
    test_calculate_distance();

}

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Error: requires initial and final supercharger names" << std::endl;        
        return -1;
    }
    
    std::string initial_charger_name = argv[1];
    std::string goal_charger_name = argv[2];
    
    int idx_source;
    int idx_destination;
 	
 	for(size_t i{0}; i<network.size();i++){
    	if (network[i].name == initial_charger_name) {idx_source=i;}
    	if (network[i].name == goal_charger_name) {idx_destination=i;} //TODO: Implement if source or destination not found
    }

    std::cout<< "after source: " << network[idx_source].name << " destination: " << network[idx_destination].name <<std::endl;
    Planner myplanner {idx_source, idx_destination};
    myplanner.compute_path(idx_destination);
   	display(myplanner.path_dict[idx_destination]);
    tests();
    return 0;
}