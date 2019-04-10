/*!
*	@file	initial_solution.h
*	@author		Hendrik Vermuyten
*	@brief	A set of global variables related to the initial solution from which the algorithm starts.
*/

#ifndef INITIAL_SOLUTION_H
#define INITIAL_SOLUTION_H

#include <vector>
#include <string>

/*!
*	@namespace	alg
*	@brief	The main namespace containing all algorithm elements.
*/
namespace alg
{
	/*!
	*	@brief	Indicates whether the solution data have been provided.
	*/
	extern bool data_solution_exist;

	/*!
	*	@brief	Indicates the timeslot to which each lecture is assigned.
	*/
	extern std::vector<int> initial_solution_lecture_timing;

	/*!
	*	@brief	Indicates the room to which each lecture is assigned in the initial solution.
	*/
	extern std::vector<int> initial_solution_lecture_room;



	/*!
	*	@brief	Read the solution data from a file.
	*	@param	filename	The name of the input file for the solution data.
	*/
	extern void read_solution_data(const std::string& filename);

	/*!
	*	@brief	Clear the solution data.
	*/
	extern void clear_solution_data();
}

#endif // !INITIAL_SOLUTION_H