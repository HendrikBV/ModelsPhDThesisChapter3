/*!
*	@file	MIP_two_stage.h
*	@author		Hendrik Vermuyten
*	@brief	The two-stage MIP model.
*/

#ifndef MIP_TWO_STAGE_H
#define MIP_TWO_STAGE_H

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
	*	@brief	The two-stage MIP model.
	*/
	class MIP_two_stage
	{
		/*!
		*	@brief	Name of the algorithm.
		*/
		static constexpr const char * algorithm_name = "Two-stage MIP";

		/*!
		*	@brief	Pointer to the CPLEX environment.
		*/
		CPXENVptr env{ nullptr };

		/*!
		*	@brief	Pointer to the CPLEX problem for the first stage.
		*/
		CPXLPptr masterproblem{ nullptr };

		/*!
		*	@brief	Pointer to the CPLEX problem for the second stage.
		*/
		CPXLPptr subproblem{ nullptr };

		/*!
		*	@brief	Store for CPLEX error messages.
		*/
		char error_text[CPXMESSAGEBUFSIZE];

		/*!
		*	@brief	Indicates whether a solution exists for the first stage.
		*/
		bool solution_firststage_exists = false;

		/*!
		*	@brief	Information on the CPLEX solution status for the first stage.
		*/
		std::string solution_info_firststage;

		/*!
		*	@brief	Objective value of the CPLEX solution for the first stage.
		*/
		double objective_value_firststage;

		/*!
		*	@brief	Solution of the problem w.r.t. timeslot assignments.
		*/
		std::vector<int> solution_lecture_timeslot;

		/*!
		*	@brief	Indicates whether a solution exists for the second stage.
		*/
		bool solution_secondstage_exists = false;

		/*!
		*	@brief	Information on the CPLEX solution status for the second stage.
		*/
		std::string solution_info_secondstage;

		/*!
		*	@brief	Objective value of the CPLEX solution for the second stage.
		*/
		double objective_value_secondstage;

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
		*	@brief	The computation time for the first stage.
		*/
		std::chrono::duration<double, std::ratio<1, 1>> computation_time_firststage;

		/*!
		*	@brief	The computation time for the second stage.
		*/
		std::chrono::duration<double, std::ratio<1, 1>> computation_time_secondstage;

		/*!
		*	@brief	The number of variables in the first stage MIP.
		*/
		int nb_variables_firststage;

		/*!
		*	@brief	The number of constraints in the first stage MIP.
		*/
		int nb_constraints_firststage;

		/*!
		*	@brief	The number of variables in the second stage MIP.
		*/
		int nb_variables_secondstage;

		/*!
		*	@brief	The number of constraints in the second stage MIP.
		*/
		int nb_constraints_secondstage;

	public:
		/*!
		*	@brief	Run the algorithm.
		*	@param	constraint_on_evacuation_time	True if a constraint on the maximum evacuation time is included in the model.
		*	@param	value	The right-hand-side value for the constraint on the maximum evacuation time.
		*/
		void run_algorithm(bool constraint_on_evacuation_time, double value);

		/*!
		*	@brief	Run the algorithm decomposed on the days.
		*/
		void run_algorithm_days_decomposition();

		/*!
		*	@brief	Run the algorithm decomposed on every timeslot.
		*/
		void run_algorithm_timeslot_decomposition();

		/*!
		*	@brief	Run only the first stage of the model.
		*/
		void run_only_first_stage();

		/*!
		*	@brief	Run only the first stage of the model and evaluate the solution w.r.t. the second stage objective.
		*/
		void run_only_first_stage_and_evaluate_solution();

		/*!
		*	@brief	Get the computation time for the first stage of the algorithm.
		*	@returns The computation time for the first stage of the algorithm.
		*/
		double get_computation_time_firststage() const { return computation_time_firststage.count(); }

		/*!
		*	@brief	Get the computation time for the second stage of the algorithm.
		*	@returns The computation time for the second stage of the algorithm.
		*/
		double get_computation_time_secondstage() const { return computation_time_secondstage.count(); }

		/*!
		*	@brief	Check whether a solution exists.
		*	@returns	True if a solution exists, false otherwise.
		*/
		bool get_solution_exists() const { return (solution_firststage_exists && solution_secondstage_exists); }

		/*!
		*	@brief	Check whether a solution for the first stage MIP exists.
		*	@returns	True if a solution exists, false otherwise.
		*/
		bool get_solution_firststage_exists() const { return solution_firststage_exists; }

		/*!
		*	@brief	Get information on the solution for the first stage.
		*	@returns	Information on the solution for the first stage.
		*/
		const std::string& get_solution_info_firststage() const { return solution_info_firststage; }

		/*!
		*	@brief	Get information on the solution for the second stage.
		*	@returns	Information on the solution for the second stage.
		*/
		const std::string& get_solution_info_secondstage() const { return solution_info_secondstage; }

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
		*	@brief	Build the CPLEX problem for the first stage.
		*/
		void build_masterproblem();

		/*!
		*	@brief	Solve the CPLEX problem for the first stage.
		*/
		void solve_masterproblem();

		/*!
		*	@brief	Clear the CPLEX problem for the first stage.
		*/
		void clear_masterproblem();

		/*!
		*	@brief	Build the CPLEX problem for the second stage.
		*/
		void build_subproblem();

		/*!
		*	@brief	Solve the CPLEX problem for the second stage.
		*/
		void solve_subproblem();

		/*!
		*	@brief	Build the CPLEX problem for the second stage for a specific half day.
		*	@param	day			The day for which the problem needs to be built.
		*	@param	afternoon	1 if afternoon, 0 if morning.
		*/
		void build_subproblem(int day, int afternoon);

		/*!
		*	@brief	Solve the CPLEX problem for the second stage for a specific half day.
		*	@param	day			The day for which the problem needs to be built.
		*	@param	afternoon	1 if afternoon, 0 if morning.
		*/
		void solve_subproblem(int day, int afternoon);

		/*!
		*	@brief	Build the CPLEX problem for the second stage for a specific timeslot.
		*	@param	timeslot	The timeslot for which the problem needs to be built.
		*/
		void build_subproblem(int timeslot);

		/*!
		*	@brief	Build the CPLEX problem for the second stage for a specific timeslot.
		*	@param	timeslot	The timeslot for which the problem needs to be built.
		*/
		void solve_subproblem(int timeslot);


		/*!
		*	@brief	Clear the CPLEX problem for the second stage.
		*/
		void clear_subproblem();

		/*!
		*	@brief	Clear CPLEX.
		*/
		void clear_cplex();

		/*!
		*	@brief	Write output.
		*/
		void write_output();

		/*!
		*	@brief	Add a constraint for the maximum evacuation time.
		*	@param	value	The right-hand-side value for the constraint on the maximum evacuation time
		*/
		void add_constraint_maximum_evacuation_time(double value);

		/*!
		*	@brief	Fix the assignments for the solution that is to be evaluated.
		*	@param	lecture_room		A vector that gives the room assigned to each lecture.
		*/
		void fix_solution(const std::vector<int> lecture_room);
	};

} // namespace alg

#endif // !MIP_TWO_STAGE_H