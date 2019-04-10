#include "building_data.h"
#include "timetable_data.h"

#include <fstream>
#include <iostream>
#include <random>


namespace alg
{
	bool data_building_exist = false;
	std::string instance_name_building;
	int nb_arcs;
	int nb_paths;
	std::vector<double> room_room_path;
	std::vector<bool> path_arc;
	std::vector<int> arc_length;
	std::vector<int> arc_area;
	std::vector<bool> arc_stairs;

	// exit is at index 'nb_locations'
	double get_roomroompath(int room1, int room2, int path) { return room_room_path[room1*(nb_locations + 1)*nb_paths + room2 * nb_paths + path]; } // a_pcd
	bool get_patharc(int path, int arc) { return path_arc[path*nb_arcs + arc]; } // b_ijp


	void read_building_data(const std::string& file_name)
	{
		std::ifstream myfile;
		myfile.open(file_name);
		if (!myfile.is_open())
			std::cerr << "\n\nError: couldn't open input file for building data\n\n";
		else
		{
			// clear old data
			clear_building_data();

			// name
			myfile >> instance_name_building;

			// first read in basic data
			myfile >> nb_arcs;
			myfile >> nb_paths;

			// paths (a_pcd, b_ijp)
			// a_pcd
			int k;
			double a_pcd;
			room_room_path.reserve(nb_locations*nb_locations*nb_paths);
			for (int i = 0; i < nb_locations + 1; ++i)
			{
				for (int j = 0; j < nb_locations + 1; ++j)
				{
					for (int l = 0; l < nb_paths; ++l)
					{
						myfile >> a_pcd;
						room_room_path.push_back(a_pcd);
					}
				}
			}


			// b_ijp
			path_arc.reserve(nb_arcs*nb_paths);
			for (int i = 0; i < nb_paths; ++i)
			{
				for (int j = 0; j < nb_arcs; ++j)
				{
					myfile >> k;
					path_arc.push_back(k);
				}
			}

			// length_ij
			arc_length.reserve(nb_arcs);
			for (int i = 0; i < nb_arcs; ++i)
			{
				myfile >> k;
				arc_length.push_back(k);
			}

			// area_ij
			arc_area.reserve(nb_arcs);
			for (int i = 0; i < nb_arcs; ++i)
			{
				myfile >> k;
				arc_area.push_back(k);
			}

			// stairs_ij
			arc_stairs.reserve(nb_arcs);
			for (int i = 0; i < nb_arcs; ++i)
			{
				myfile >> k;
				arc_stairs.push_back(k);
			}


			// End of file input
			data_building_exist = true;
		}
	}

	void clear_building_data()
	{
		data_building_exist = false;

		instance_name_building = "";
		room_room_path.clear();
		path_arc.clear();
		arc_length.clear();
		arc_area.clear();
		arc_stairs.clear();
	}
}