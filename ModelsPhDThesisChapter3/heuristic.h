/*!
*	@file	heuristic.h
*	@author		Hendrik Vermuyten
*	@brief	A heuristic for the monolithic problem with multithreaded implementation.
*/

#ifndef HEURISTIC_H
#define HEURISTIC_H

#include <vector>
#include <iostream>
#include "heuristic_utilities.h"

/*!
*	@namespace	alg
*	@brief	The main namespace containing all algorithm elements.
*/
namespace alg
{
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/*!
	*	@brief	A POD to store the different components of the objective value.
	*/
	struct information_objective_value
	{
		/*!
		*	@brief	The overall objective value.
		*/
		double objective_value = 1e20;

		/*!
		*	@brief	The number of constraint violations for the scheduling conflicts.
		*/
		double constraint_violations_scheduling_conflicts;

		/*!
		*	@brief	The number of constraint violations for the room compatibilities.
		*/
		double constraint_violations_correct_room;

		/*!
		*	@brief	The number of teacher working time violations.
		*/
		double constraint_violations_teacher_working_time;

		/*!
		*	@brief	The number of constraint violations for the compactness constraints.
		*/
		double constraint_violations_compactness_constraints;

		/*!
		*	@brief	The preference score.
		*/
		double preference_score;


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
		*	@brief	Assignment operator.
		*	@param	other	Another object to save info from.
		*/
		information_objective_value& operator=(const information_objective_value &other)
		{
			if (this != &other)
			{
				objective_value = other.objective_value;

				constraint_violations_scheduling_conflicts = other.constraint_violations_scheduling_conflicts;
				constraint_violations_correct_room = other.constraint_violations_correct_room;
				constraint_violations_teacher_working_time = other.constraint_violations_teacher_working_time;
				constraint_violations_compactness_constraints = other.constraint_violations_compactness_constraints;
				preference_score = other.preference_score;

				evacuations_timeslot_lecture_uses_path = other.evacuations_timeslot_lecture_uses_path;
				evacuations_timeslot_flow_per_arc = other.evacuations_timeslot_flow_per_arc;
				evacuations_timeslot_time_per_arc = other.evacuations_timeslot_time_per_arc;
				evacuations_timeslot_lecture_travel_time = other.evacuations_timeslot_lecture_travel_time;
				evacuations_timeslot_max_travel_time = other.evacuations_timeslot_max_travel_time;
			}
			return *this;
		}

		/*!
		*	@brief	Print the values to screen.
		*/
		void print()
		{
			std::cout << "\nNumber of scheduling conflicts: " << constraint_violations_scheduling_conflicts;
			std::cout << "\nNumber of room violations: " << constraint_violations_correct_room;
			std::cout << "\nNumber of teacher working time violations: " << constraint_violations_teacher_working_time;
			std::cout << "\nNumber of compactness constraint violations: " << constraint_violations_compactness_constraints;
			std::cout << "\nPreferences: " << preference_score;

			double evactimesum = 0;
			double evactimemax = 0;
			for (auto&& et : evacuations_timeslot_max_travel_time)
			{
				evactimesum += et;
				if (et > evactimemax)
					evactimemax = et;
			}
			std::cout << "\nMaximum evacuation time: " << evactimemax;
			std::cout << "\nSum evacuation times: " << evactimesum;
		}
	};



	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/*!
	*	@brief	Class for the parallel threads.
	*/
	class heuristic_subprocess
	{
		/*!
		*	@brief	The objective value of the current solution.
		*/
		information_objective_value current_objective;

		/*!
		*	@brief	The current timetable solution.
		*	Matrix in which the rows indicate the timeslot and the columns the room.
		*	The value in a cell indicates the lecture that is assigned to that timeslot and room.
		*/
		matrix2D<int> current_solution;

		/*!
		*	@brief	The objective value of the best found solution.
		*/
		information_objective_value best_objective;

		/*!
		*	@brief	The best timetable solution found during the search process.
		*	Matrix in which the rows indicate the timeslot and the columns the room.
		*	The value in a cell indicates the lecture that is assigned to that timeslot and room.
		*/
		matrix2D<int> best_solution;

		/*!
		*	@brief	Count and remember the number of constraint violations for the scheduling conflicts when recalculating the objective value for a candidate solution.
		*/
		double new_constraint_violations_scheduling_conflicts;

		/*!
		*	@brief	Count and remember the number of constraint violations for the room compatibilities when recalculating the objective value for a candidate solution.
		*/
		double new_constraint_violations_correct_room;

		/*!
		*	@brief	Count and remember the number of teacher working time violations when recalculating the objective value for a candidate solution.
		*/
		double new_constraint_violations_teacher_working_time;

		/*!
		*	@brief	Count and remember the number of constraint violations for the compactness constraints when recalculating the objective value for a candidate solution.
		*/
		double new_constraint_violations_compactness_constraints;

		/*!
		*	@brief	Calculate and remember the preference score when recalculating the objective value for a candidate solution.
		*/
		double new_preference_score;

		/*!
		*	@brief	New value for the evacuation time in the timeslot that is changed during a room swap.
		*	Used to updated the values of the vector with the evacuation times for all timeslots.
		*/
		double new_max_evac_time_changed_ts_roomswap = 0.0;

		/*!
		*	@brief	New value for the evacuation times in the timeslots that are changed during a lecture swap or Kempe chain.
		*	Used to updated the values of the vector with the evacuation times for all timeslots.
		*/
		std::vector<double> new_max_evac_time_changed_ts{ 0.0, 0.0 };

		/*!
		*	@brief	The number of iterations.
		*/
		size_t _iterations = 0;



	public:
		/*!
		*	@brief	The objective value of the current solution.
		*/
		size_t _process_id;

		/*!
		*	@brief	Get the number of iterations.
		*	@returns	The number of iterations.
		*/
		size_t get_iterations() const { return _iterations; }

		/*!
		*	@brief	Initialize the matrices for use during the algorithm.
		*/
		void initialize();

		/*!
		*	@brief	Run the process for the given time limit.
		*	@param	time_limit	The time limit for the process before synchronization.
		*	@returns	The objective value of the best found solution.
		*/
		double run(double time_limit);

		/*!
		*	@brief	Get the best found solution.
		*	@returns	The best found solution.
		*/
		const matrix2D<int>& get_best_solution() const { return best_solution; }

		/*!
		*	@brief	Get the objective value of the best found solution.
		*	@returns	The objective value of the best found solution.
		*/
		const information_objective_value& get_best_objective_value() const { return best_objective; }

		/*!
		*	@brief	Saves a solution (current & best are updated).
		*	@param	solution	The solution that is set.
		*	@param	objective_value	The objective value of the solution
		*/
		void set_solution(const matrix2D<int>& solution, const information_objective_value &objective_value)
		{
			current_solution = solution;	current_objective = objective_value;
			best_solution = solution;		best_objective = objective_value;
		}



	private:
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
	};



	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/*!
	*	@brief	The main class for the heuristic.
	*/
	class heuristic_master
	{
		/*!
		*	@brief	Name of the algorithm.
		*/
		static constexpr const char * algorithm_name = "SA Heuristic";

		/*!
		*	@brief	The objective value of the best found solution.
		*/
		size_t total_iterations = 0;

		/*!
		*	@brief	The number of SA reheats.
		*/
		size_t reheats = 0;

		/*!
		*	@brief	The objective value of the best found solution.
		*/
		double best_objective = 1e20;

		/*!
		*	@brief	The best timetable solution found during the search process.
		*	Matrix in which the rows indicate the timeslot and the columns the room.
		*	The value in a cell indicates the lecture that is assigned to that timeslot and room.
		*/
		matrix2D<int> best_solution;

		/*!
		*	@brief	The different search subprocesses.
		*/
		std::vector<heuristic_subprocess> threads;



	public:
		/*!
		*	@brief	The type of objective function.
		*/
		enum class objective_type
		{
			maximum_over_timeslots,	///< Maximum over all timeslots of evacuation or travel times
			sum_over_timeslots,		///< Sum over all timeslots of evacuation or travel times
		};

		/*!
		*	@brief	Run the heuristic.
		*/
		void run();

		/*!
		*	@brief	Is there a constraint set on the maximum preference score?
		*/
		static bool _constraint_preferences;

		/*!
		*	@brief	The value of the constraint on the preferences.
		*/
		static bool _constraint_preferences_value;

		/*!
		*	@brief	The relative value of preferences (lambda) and travel or evacuation times (1 - lambda).
		*/
		static double _lambda;

		/*!
		*	@brief	The allowed computation time for the heuristic (in seconds).
		*/
		static double _time_limit;

		/*!
		*	@brief	The number of synchronizations during the search process.
		*/
		static size_t _nb_synchronizations;

		/*!
		*	@brief	The type of objective function.
		*/
		static objective_type _objective_type;

		/*!
		*	@brief	The weight/score of a constraint violation.
		*/
		static double _penalty_value_constraint_violation;

		/*!
		*	@brief	The number of threads used in parallel implementation.
		*/
		static size_t _nb_threads;

		/*!
		*	@brief	The probabilities of the different moves.
		*/
		static double _probability_move[3];

		/*!
		*	@brief	The current temperature of the simulated annealing.
		*/
		static double _SA_temperature;

		/*!
		*	@brief	The coefficient to update the SA temperature: T' = alpha * T.
		*/
		static double _SA_alpha;

		/*!
		*	@brief	The start temperature of the SA.
		*/
		static double _SA_start_temperature;

		/*!
		*	@brief	The minimum temperature, when the SA is reheated.
		*/
		static double _SA_Tmin;


	private:
		void write_output();
	};

} // namespace alg

#endif // !HEURISTIC_H


