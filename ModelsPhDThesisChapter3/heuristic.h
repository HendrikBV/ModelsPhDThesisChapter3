/*!
*	@file	heuristic.h
*	@author		Hendrik Vermuyten
*	@brief	A heuristic for the monolithic problem.
*/

#ifndef HEURISTIC_H
#define HEURISTIC_H

#include <vector>
#include "heuristic_utilities.h"

/*!
*	@namespace	alg
*	@brief	The main namespace containing all algorithm elements.
*/
namespace alg
{
	/*!
	*	@brief	A heuristic for the monolithic problem.
	*/
	class heuristic
	{
		/*!
		*	@brief	Name of the algorithm.
		*/
		static constexpr const char * algorithm_name = "LAHC Heuristic";

		/*!
		*	@brief	The objective value of the current solution.
		*/
		double current_objective = 1e20;

		/*!
		*	@brief	The objective value of the best found solution.
		*/
		double best_objective = 1e20;

		/*!
		*	@brief	The current timetable solution.
		*	Matrix in which the rows indicate the timeslot and the columns the room.
		*	The value in a cell indicates the lecture that is assigned to that timeslot and room.
		*/
		matrix2D<int> current_solution;

		/*!
		*	@brief	The best timetable solution found during the search process.
		*	Matrix in which the rows indicate the timeslot and the columns the room.
		*	The value in a cell indicates the lecture that is assigned to that timeslot and room.
		*/
		matrix2D<int> best_solution;

		/*!
		*	@brief	Count and remember the number of constraint violations for the scheduling conflicts.
		*/
		size_t constraint_violations_scheduling_conflicts;

		/*!
		*	@brief	Count and remember the number of constraint violations for the scheduling conflicts when recalculating the objective value for a candidate solution.
		*/
		size_t new_constraint_violations_scheduling_conflicts;

		/*!
		*	@brief	Count and remember the number of constraint violations for the room compatibilities.
		*/
		size_t constraint_violations_correct_room;

		/*!
		*	@brief	Count and remember the number of constraint violations for the room compatibilities when recalculating the objective value for a candidate solution.
		*/
		size_t new_constraint_violations_correct_room;

		/*!
		*	@brief	Count and remember the number of teacher working time violations.
		*/
		size_t constraint_violations_teacher_working_time;

		/*!
		*	@brief	Count and remember the number of teacher working time violations when recalculating the objective value for a candidate solution.
		*/
		size_t new_constraint_violations_teacher_working_time;

		/*!
		*	@brief	Count and remember the number of constraint violations for the compactness constraints.
		*/
		size_t constraint_violations_compactness_constraints;

		/*!
		*	@brief	Count and remember the number of constraint violations for the compactness constraints when recalculating the objective value for a candidate solution.
		*/
		size_t new_constraint_violations_compactness_constraints;

		/*!
		*	@brief	Calculate and remember the preference score.
		*/
		size_t preference_score;

		/*!
		*	@brief	Calculate and remember the preference score when recalculating the objective value for a candidate solution.
		*/
		size_t new_preference_score;


		/*!
		*	@brief	3DMatrix in which the rows indicate the timeslot, the columns the lecture, and the depth the path.
		*			The value in a cell indicates the fraction of people from the given lecture that use a given path in the given timeslot to evacuate the building.
		*/
		matrix3D<double> evacuations_timeslot_lecture_uses_path;

		/*!
		*	@brief	Matrix in which the rows indicate the timeslot and the columns the flow on each arc for evacuations.
		*/
		matrix2D<double> evacuations_timeslot_flow_per_arc;

		/*!
		*	@brief	Matrix in which the rows indicate the timeslot and the columns the travel time on each arc for evacuations.
		*/
		matrix2D<double> evacuations_timeslot_time_per_arc;

		/*!
		*	@brief	Matrix in which the rows indicate the timeslot and the columns the evacuation time for the people in a given lecture.
		*/
		matrix2D<double> evacuations_timeslot_lecture_travel_time;

		/*!
		*	@brief	Matrix in which the rows indicate the timeslot and the columns the evacuation time of the slowest group of people (i.e. the maximum evacuation time).
		*/
		std::vector<double> evacuations_timeslot_max_travel_time;

		/*!
		*	@brief	3DMatrix in which the rows indicate the timeslot, the columns the lecture, and the depth the path.
		*			The value in a cell indicates the fraction of people from the given lecture that use a given path in the given timeslot to travel to their next lecture.
		*/
		matrix3D<double> travels_timeslot_series_uses_path;

		/*!
		*	@brief	Matrix in which the rows indicate the timeslot and the columns the flow on each arc for travelling between consecutive lectures.
		*/
		matrix2D<double> travels_timeslot_flow_per_arc;

		/*!
		*	@brief	Matrix in which the rows indicate the timeslot and the columns the travek time on each arc for travelling between consecutive lectures.
		*/
		matrix2D<double> travels_timeslot_time_per_arc;

		/*!
		*	@brief	Matrix in which the rows indicate the timeslot and the columns the travel time for each series of students for travelling between consecutive lectures.
		*/
		matrix2D<double> travels_timeslot_series_travel_time;

		/*!
		*	@brief	Matrix in which the rows indicate the timeslot and the columns the maximum travel time (i.e. slowest series of students) for travelling between consecutive lectures.
		*/
		std::vector<double> travels_timeslot_max_travel_time;

		/*!
		*	@brief	New value for the evacuation time in the timeslot that is changed during a room swap.
		*	Used to updated the values of the vector with the evacuation times for all timeslots.
		*/
		double new_max_evac_time_changed_ts_roomswap = 0.0;

		/*!
		*	@brief	New value for the travel times in the timeslots that are changed during a room swap.
		*	Used to updated the values of the vector with the travel times for all timeslots.
		*/
		std::vector<double> new_max_travel_time_changes_ts_roomswap{ 0.0, 0.0 };

		/*!
		*	@brief	New value for the evacuation times in the timeslots that are changed during a lecture swap or Kempe chain.
		*	Used to updated the values of the vector with the evacuation times for all timeslots.
		*/
		std::vector<double> new_max_evac_time_changed_ts{ 0.0, 0.0 };

		/*!
		*	@brief	New value for the travel times in the timeslots that are changed during a lecture swap or Kempe chain.
		*	Used to updated the values of the vector with the travel times for all timeslots.
		*/
		std::vector<double> new_max_travel_time_changes_ts{ 0.0, 0.0, 0.0, 0.0 };

		/*!
		*	@brief	Timeslots for which the travel times need to be calculated.
		*/
		std::vector<int> _relevant_timeslots;



	public:
		/*!
		*	@brief	Run the heuristic.
		*/
		void run();

		/*!
		*	@brief	Is there a constraint set on the maximum preference score?
		*/
		bool _constraint_preferences = false;

		/*!
		*	@brief	The value of the constraint on the preferences.
		*/
		bool _constraint_preferences_value = 1e10;

		/*!
		*	@brief	The relative value of preferences (lambda) and travel or evacuation times (1 - lambda).
		*/
		double _lambda = 0;

		/*!
		*	@brief	The relative value of evacuation times (alpha) and travel times (1 - alpha).
		*/
		double _alpha = 1;

		/*!
		*	@brief	The allowed computation time for the heuristic (in seconds).
		*/
		double _time_limit = 1000;

		/*!
		*	@brief	The type of objective function.
		*/
		enum class objective_type
		{
			maximum_over_timeslots,	///< Maximum over all timeslots of evacuation or travel times
			sum_over_timeslots,		///< Sum over all timeslots of evacuation or travel times
		} _objective_type = objective_type::sum_over_timeslots;

		/*!
		*	@brief	The length of the LAHC list.
		*/
		size_t _LAHC_list_length = 500;

		/*!
		*	@brief	The weight/score of a constraint violation.
		*/
		size_t _penalty_value_constraint_violation = 10000;

	private:

		/*!
		*	@brief	Initialize the matrices for use during the algorithm.
		*/
		void initialize();

		/*!
		*	@brief	The main LAHC loop.
		*/
		void LAHC();

		/*!
		*	@brief	Evaluate the objective value of a solution.
		*	@returns	The objective value of the given solution.
		*/
		double evaluate(const matrix2D<int>& solution);

		/*!
		*	@brief	Evaluate the objective value of a solution using incremental evaluation for a room swap.
		*	@param	solution	The current solution before changes are implemented.
		*	@param	timeslot	The timeslot in which two lectures are swapped between rooms.
		*	@param	room1		The first room in the swap.
		*	@param	room2		The second room in the swap.
		*	@returns	The objective value of the new solution if the given changes were applied.
		*/
		double evaluate_incremental(matrix2D<int>& solution, int timeslot, int room1, int room2);

		/*!
		*	@brief	Evaluate the objective value of a solution using incremental evaluation for a lecture swap.
		*	@param	solution	The current solution before changes are implemented.
		*	@param	timeslot1	The first timeslot in the swap.
		*	@param	timeslot2	The second timeslot in the swap.
		*	@param	room1		The first room in the swap.
		*	@param	room2		The second room in the swap.
		*	@returns	The objective value of the new solution if the given changes were applied.
		*/
		double evaluate_incremental(matrix2D<int>& solution, int timeslot1, int timeslot2, int room1, int room2);

		/*!
		*	@brief	Evaluate the objective value of a solution using incremental evaluation for a Kempe chain move.
		*	@param	solution	The current solution before changes are implemented.
		*	@param	currrent_obj	The objective value of the current solution before changes are implemented.
		*	@param	kempe_chain	The Kempe chain
		*	@param	timeslot1	The first timeslot in the Kempe chain.
		*	@param	timeslot2	The second timeslot in the Kempe chain.
		*	@returns	The objective value of the new solution if the given changes were applied.
		*/
		double evaluate_incremental(matrix2D<int>& solution, double current_obj, const std::vector<std::pair<int, int>>& kempe_chain, int timeslot1, int timeslot2);

		/*!
		*	@brief	Clear all datastructures.
		*/
		void clear();

		/*!
		*	@brief	Write algorithm output to a file or the screen.
		*/
		void write_output();
	};

} // namespace alg

#endif // !HEURISTIC_H


