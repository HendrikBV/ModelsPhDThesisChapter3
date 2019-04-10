/*!
*	@file	MIP_only_second_stage.h
*	@author		Hendrik Vermuyten
*	@brief	The MIP model for the second stage, where the assignment of lectures to rooms is optimised.
*/

#ifndef MIP_ONLY_SECOND_STAGE_H
#define MIP_ONLY_SECOND_STAGE_H

#include "ilcplex\cplex.h"
#include <vector>
#include <chrono>

/*!
*	@namespace	alg
*	@brief	The main namespace containing all algorithm elements.
*/
namespace alg
{
	/*!
	*	@brief	The MIP model for the second stage, where the assignment of lectures to rooms is optimised.
	*/
	class MIP_only_second_stage
	{
		/*!
		*	@brief	Name of the algorithm.
		*/
		static constexpr const char * algorithm_name = "Only second stage MIP";


		/*!
		*	@brief	Pointer to the CPLEX environment.
		*/
		CPXENVptr env{ nullptr };

		/*!
		*	@brief	Pointer to the CPLEX problem.
		*/
		CPXLPptr problem{ nullptr };

		/*!
		*	@brief	Store for CPLEX error messages.
		*/
		char error_text[CPXMESSAGEBUFSIZE];

		/*!
		*	@brief	Number of columns (variables) in the problem formulation.
		*/
		int numcols_problem;

		/*!
		*	@brief	Number of rows (constraints) in the problem formulation.
		*/
		int numrows_problem;

		/*!
		*	@brief	Indicates whether a solution exists.
		*/
		bool solution_exists = false;

		/*!
		*	@brief	Information on the CPLEX solution status.
		*/
		std::string solution_info;

		/*!
		*	@brief	Objective value of the CPLEX solution.
		*/
		double objective_value;

		/*!
		*	@brief	Solution of the problem.
		*/
		std::vector<int> solution_lecture_room;

		/*!
		*	@brief	The travel times between each pair of consecutive timeslots.
		*/
		std::vector<double> solution_travel_times;

		/*!
		*	@brief	The evacuation times for each timeslot.
		*/
		std::vector<double> solution_evacuation_times;

		/*!
		*	@brief	The computation time for the algorithm.
		*/
		std::chrono::duration<double, std::ratio<1, 1>> computation_time;

		/*!
		*	@brief      The number of the current solution evaluated by exhaustive search.
		*/
		size_t exhaustive_search_solutions_number = 0;

	public:
		/*!
		*	@brief	Run the algorithm.
		*	@param	constraint_on_evacuation_time	True if a constraint on the maximum evacuation time is included in the model.
		*	@param	value	The right-hand-side value for the constraint on the maximum evacuation time.
		*/
		void run_algorithm(bool constraint_on_evacuation_time, double value);

		/*!
		*	@brief      Run an exhaustive search.
		*/
		void run_exhaustive_search();

		/*!
		*	@brief	Evaluate a solution.
		*	@param	lecture_room		A vector that gives the room assigned to each lecture.
		*/
		void evaluate_solution(const std::vector<int> lecture_room);

		/*!
		*	@brief	Get the computation time for the algorithm.
		*	@returns The computation time for the algorithm.
		*/
		double get_computation_time() const { return computation_time.count(); }

		/*!
		*	@brief	Check whether a solution exists.
		*	@returns	True if a solution exists, false otherwise.
		*/
		bool get_solution_exists() const { return solution_exists; }

		/*!
		*	@brief	Get information on the solution.
		*	@returns	Information on the solution.
		*/
		const std::string& get_solution_info() const { return solution_info; }

		/*!
		*	@brief	Get the room assigned to the given lecture in the solution.
		*	@param	lecture	The lecture for which we want to know the room.
		*	@returns	The room assigned to the given lecture in the solution.
		*/
		int get_solution_lecture_room(int lecture) const { return solution_lecture_room.at(lecture); }

		/*!
		*	@brief	Get the travel time between consecutive lectures in the given timeslot.
		*	@param	timeslot	The given timeslot.
		*	@returns	The travel time in the given timeslot.
		*/
		int get_solution_travel_time(int timeslot) const { return solution_travel_times.at(timeslot); }

		/*!
		*	@brief	Get the evacuation time in the given timeslot.
		*	@param	timeslot	The given timeslot.
		*	@returns	The evacuation time in the given timeslot.
		*/
		int get_solution_evacuation_time(int timeslot) const { return solution_evacuation_times.at(timeslot); }

		/*!
		*	@brief	The relative value of evacuation times (alpha) and travel times (1 - alpha).
		*/
		double _alpha = 1;

		/*!
		*	@brief	The optimality tolerance for the solver.
		*	Any number from 0.0 to 1.0; default: 1e-04.
		*/
		double _optimality_tolerance = 1e-04;

		/*!
		*	@brief	The allowed computation time for the MIP model (in seconds).
		*/
		double _time_limit = 3600;

		/*!
		*	@brief	The type of objective function.
		*/
		enum class objective_type
		{
			maximum_over_timeslots,	///< Maximum over all timeslots of evacuation times
			sum_over_timeslots,		///< Sum over all timeslots of evacuation times
		} _objective_type;

	private:
		/*!
		*	@brief	Initialize the CPLEX environment.
		*/
		void initialize_cplex();

		/*!
		*	@brief	Build the CPLEX problem.
		*/
		void build_problem();

		/*!
		*	@brief	Solve the CPLEX problem.
		*/
		void solve_problem();

		/*!
		*	@brief	Clear CPLEX.
		*/
		void clear_cplex();

		/*!
		*	@brief	Write output.
		*/
		void write_output();

		/*!
		*	@brief      Recursive function that generates all possible solutions (for travels).
		*	@param      current_event   The current event that needs to be scheduled (i.e., the current iteration).
		*/
		void generate_all_possible_solutions(int current_event);

		/*!
		*	@brief      Recursive function that generates all possible solutions for each timeslot independently (for evacuations).
		*	@param      timeslot    The timeslot for which we generate all possible solutions
		*	@param      current_event   The current event that needs to be scheduled (i.e., the current iteration).
		*/
		void generate_all_possible_solutions_independently(int timeslot, int current_event);

		/*!
		*	@brief	Fix the assignments for the solution that is to be evaluated.
		*	@param	lecture_room		A vector that gives the room assigned to each lecture.
		*/
		void fix_solution(const std::vector<int> lecture_room);

		/*!
		*	@brief		Check which event is planned in the given timeslot and location.
		*	@param      timeslot        The timeslot to check.
		*	@param      location        The location to check.
		*	@returns    The number of the event that is planned in the given timeslot and location, -1 if no event is planned in that timeslot in that location.
		*/
		int timeslot_location(int timeslot, int location) const;

		/*!
		*	@brief	Add a constraint for the maximum evacuation time.
		*	@param	value	The right-hand-side value for the constraint on the maximum evacuation time
		*/
		void add_constraint_maximum_evacuation_time(double value);

	};

} // namespace alg

#endif // !MIP_ONLY_SECOND_STAGE_H