/*!
*	@file	MIP_monolithic.h
*	@author		Hendrik Vermuyten
*	@brief	The MIP model for the monolithic model.
*/

#ifndef MIP_MONOLITHIC_H
#define MIP_MONOLITHIC_H

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
	*	@brief		The MIP model for the monolithic model.
	*/
	class MIP_monolithic
	{
		/*!
		*	@brief	Name of the algorithm.
		*/
		static constexpr const char * algorithm_name = "Monolithic MIP";

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
		*	@brief	Solution of the problem w.r.t. timeslot assignments.
		*/
		std::vector<int> solution_lecture_timeslot;

		/*!
		*	@brief	Solution of the problem w.r.t. room assignments.
		*/
		std::vector<int> solution_lecture_room;

		/*!
		*	@brief	The objective value for the preferences in the solution.
		*/
		int solution_preferences;

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

	public:
		/*!
		*	@brief	Run the algorithm.
		*	@param	constraint_on_preferences	True if a constraint on the maximum preference score is included in the model.
		*	@param	constraint_on_T				True if a constraint on the travel times / evacuation times is included in the model.
		*	@param	value_pref					The right-hand-side value for the constraint on the maximum preference score.
		*	@param	value_T						The right-hand-side value for the constraint on the travel times / evacuation times.
		*/
		void run_algorithm(bool constraint_on_preferences, bool constraint_on_T, double value_pref = 1e10, double value_T = 1e10);

		/*!
		*	@brief	Evaluate a solution.
		*	@param	lecture_timeslot	A vector that gives the timeslot assigned to each lecture.
		*	@param	lecture_room		A vector that gives the room assigned to each lecture.
		*/
		void evaluate_solution(const std::vector<int> lecture_timeslot, const std::vector<int> lecture_room);

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
		*	@brief	Return the objective value of the best found solution.
		*	@returns	The objective value of the best found solution.
		*/
		bool get_solution_objective_value() const { return objective_value; }

		/*!
		*	@brief	Get the timeslot assigned to the given lecture in the solution.
		*	@param	lecture	The lecture for which we want to know the room.
		*	@returns	The timeslot assigned to the given lecture in the solution.
		*/
		int get_solution_lecture_timeslot(int lecture) const { return solution_lecture_timeslot.at(lecture); }

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
		*	@brief	The relative value of preferences (lambda) and travel or evacuation times (1 - lambda).
		*/
		double _lambda = 1;

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
		*	@brief	Update the objective function coefficients for the analysis of a solution.
		*/
		void update_objective_coefficients_analysis();

		/*!
		*	@brief	Fix the assignments for the solution that is to be evaluated.
		*	@param	lecture_timeslot	A vector that gives the timeslot assigned to each lecture.
		*	@param	lecture_room		A vector that gives the room assigned to each lecture.
		*/
		void fix_solution(const std::vector<int> lecture_timeslot, const std::vector<int> lecture_room);

		/*!
		*	@brief	Add a constraint for the maximum preference score.
		*	@param	value	The right-hand-side value for the constraint.
		*/
		void add_constraint_preferences(double value);

		/*!
		*	@brief	Add a constraint for the travel / evacuation times.
		*	@param	value	The right-hand-side value for the constraint.
		*/
		void add_constraint_TT(double value);
	};

} // namespace alg

#endif // !MIP_MONOLITHIC_H