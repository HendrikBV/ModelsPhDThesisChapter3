/*!
*	@file	building_data.h
*	@author		Hendrik Vermuyten
*	@brief	A set of global variables related to the building information.
*/

#ifndef PROBLEM_DATA_H
#define PROBLEM_DATA_H

#include <vector>
#include <string>

/*!
*	@namespace	alg
*	@brief	The main namespace containing all algorithm elements.
*/
namespace alg
{
	/*!
	*	@brief	Indicates whether the building data have been provided.
	*/
	extern bool data_building_exist;

	/*!
	*	@brief		The name of the building instance.
	*/
	extern std::string instance_name_building;

	/*!
	*	@brief	The number of arcs in the building layout.
	*/
	extern int nb_arcs;

	/*!
	*	@brief	The number of paths in the building layout.
	*/
	extern int nb_paths;

	/*!
	*	@brief	The percentage of students who use path p to go from room c to room d.
	*/
	extern std::vector<double> room_room_path;

	/*!
	*	@brief	Denotes whether path p contains arc (i,j).
	*/
	extern std::vector<bool> path_arc;

	/*!
	*	@brief	The length of arc (i,j).
	*/
	extern std::vector<int> arc_length;

	/*!
	*	@brief	The area of arc (i,j).
	*/
	extern std::vector<int> arc_area;

	/*!
	*	@brief	Indicates whether arc (i,j) denotes stairs.
	*/
	extern std::vector<bool> arc_stairs;



	/*!
	*	@brief	Returns the percentage of people who use the given path to go from room1 to room2.
	*	@param	room1	The origin room.
	*	@param	room2	The destination room.
	*	@param	path	The given path
	*	@returns	The percentage of people who use the given path to go from room1 to room2.
	*/
	extern double get_roomroompath(int room1, int room2, int path);

	/*!
	*	@brief	Returns whether the given path contains the given arc.
	*	@param	path	The given path.
	*	@param	arc		The given arc.
	*	@returns	True if the given path contains the given arc, false otherwise.
	*/
	extern bool get_patharc(int path, int arc);



	/*!
	*	@brief	Read in the building data.
	*	@param	file_name	The name of the input file with the building data.
	*/
	extern void read_building_data(const std::string& file_name);

	/*!
	*	@brief	Delete the building data.
	*/
	extern void clear_building_data();



	/*!
	*	@brief	The maximum walking speed (horizontal surface).
	*/
	constexpr double v_max{ 1.25 };

	/*!
	*	@brief	The maximum density (when walking speed reaches 0).
	*/
	constexpr double rho_max{ 7.60 };

	/*!
	*	@brief	The speed correction for stairs (multiply speed by this number).
	*/
	constexpr double speed_correction_stairs{ 1.2 };

	/*!
	*	@brief	The scaling parameter for the walking speed function.
	*/
	constexpr double walking_alpha{ 1.0 };

	/*!
	*	@brief	The maximum allowed flow per arc per timeslot.
	*/
	constexpr int F_max{ 10000 };

} // namespace alg

#endif // !PROBLEM_DATA_H
