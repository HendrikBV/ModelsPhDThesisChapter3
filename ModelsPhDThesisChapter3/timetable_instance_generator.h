/*!
*	@file		timetable_instance_generator.h
*  @author     Hendrik Vermuyten
*	@brief		An instance generator to generate problem instances for the algorithm.
*/

#ifndef TIMETABLE_INSTANCE_GENERATOR_H
#define TIMETABLE_INSTANCE_GENERATOR_H

#include <string>
#include <vector>

/*!
*	@namespace	alg
*	@brief	The main namespace containing all algorithm elements.
*/
namespace alg
{
	/*!
	*	@brief		The class to generate timetable instances.
	*/
	class instance_generator
	{
	public:
		/*!
		*	@brief		Generate a standard instance.
		*  @param      name    The name of the instance.
		*  @param      size    The size of the instance (1 == small, 2 == medium, 3 == large).
		*  @param      locations   The number of locations in the instance.
		*/
		void generate_instance(const std::string &name, int size, int locations);

		/*!
		*	@brief		Generate a standard instance.
		*  @param      name    The name of the instance.
		*  @param      timeslots   The number of timeslots in the instance.
		*  @param      events_per_timeslot   The number of events per timeslot in the instance.
		*  @param      locations   The number of locations in the instance.
		*  @param      min_nb_people_per_event   The minimum number of people in every event in the instance.
		*  @param      max_nb_people_per_event   The maximum number of people in every event in the instance.
		*/
		void generate_instance_custom(const std::string& name,
			int timeslots,
			int events_per_timeslot,
			int locations,
			int min_nb_people_per_event,
			int max_nb_people_per_event);

	private:
		/*!
		*	@brief		The size of the instance.
		*/
		int instance_size;

		/*!
		*	@brief		The name of the instance.
		*/
		std::string instance_name;

		/*!
		*	@brief		The number of events in the instance.
		*/
		int nb_events;

		/*!
		*	@brief		The number of timeslots in the instance.
		*/
		int nb_timeslots;

		/*!
		*	@brief		The number of locations in the instance.
		*/
		int nb_locations;

		/*!
		*	@brief		The number of eventgroups in the instance.
		*/
		int nb_eventgroups;

		/*!
		*	@brief		The number of teachers in the instance.
		*/
		int nb_teachers;

		/*!
		*	@brief		The number of events for each timeslot in the instance.
		*/
		std::vector<int> timeslot_nb_events;

		/*!
		*	@brief		The name of each event in the instance.
		*/
		std::vector<std::string> event_names;

		/*!
		*	@brief		The name of each eventgroup in the instance.
		*/
		std::vector<std::string> eventgroup_names;

		/*!
		*	@brief		The name of each location in the instance.
		*/
		std::vector<std::string> location_names;

		/*!
		*	@brief		Indicates which events belong to which eventgroups.
		*/
		std::vector<int> eventgroup_event;

		/*!
		*	@brief		Indicates which events can be assigned to which locations.
		*/
		std::vector<int> event_location_possible;

		/*!
		*	@brief		The number of people in each event.
		*/
		std::vector<int> event_nb_people;

		/*!
		*	@brief		The number of people in each eventgroup.
		*/
		std::vector<int> eventgroup_nb_people;

		/*!
		*	@brief		Indicates whether a teacher teachers a given event.
		*/
		std::vector<int> teacher_event;

		/*!
		*	@brief		The cost of assigning an event to a given timeslot.
		*/
		std::vector<int> cost_event_timeslot;

		/*!
		*	@brief		The type of education for each eventgroup.
		*/
		std::vector<int> eventgroup_typeofeducation;

		/*!
		*	@brief		The name of the initial solution for the instance.
		*/
		std::string solution_name;

		/*!
		*	@brief		The timeslot that is assigned to each event in the initial solution.
		*/
		std::vector<int> event_timeslot;

		/*!
		*	@brief		The locations that is assigned to each event in the initial solution.
		*/
		std::vector<int> event_location;

		/*!
		*	@brief		Indicates which locations are already assigned when constructing a solution.
		*/
		std::vector<bool> location_used;

		/*!
		*	@brief		The standard number of timeslots for small, medium, and large datasets.
		*/
		static constexpr int nb_timeslots_each_dataset = 5;

		/*!
		*	@brief		The probability with which a room can be assigned to an event.
		*/
		static constexpr double prob_room_possible = 0.7;

		/*!
		*	@brief		The minimum number of people per event for a small dataset.
		*/
		static constexpr int nb_people_per_event_minimum_small_dataset = 5;

		/*!
		*	@brief		The maximum number of people per event for a small dataset.
		*/
		static constexpr int nb_people_per_event_maximum_small_dataset = 15;

		/*!
		*	@brief		The minimum number of people per event for a medium dataset.
		*/
		static constexpr int nb_people_per_event_minimum_medium_dataset = 5;

		/*!
		*	@brief		The maximum number of people per event for a medium dataset.
		*/
		static constexpr int nb_people_per_event_maximum_medium_dataset = 25;

		/*!
		*	@brief		The minimum number of people per event for a large dataset.
		*/
		static constexpr int nb_people_per_event_minimum_large_dataset = 10;

		/*!
		*	@brief		The maximum number of people per event for a large dataset.
		*/
		static constexpr int nb_people_per_event_maximum_large_dataset = 30;



		/*!
		*	@brief		Generate the basic data for the instance.
		*/
		void generate_basic_data();

		/*!
		*	@brief		Generate the names for the events, eventgroups, and locations.
		*/
		void generate_names();

		/*!
		*	@brief		Assign events to eventgroups.
		*/
		void generate_eventgroup_events();

		/*!
		*	@brief		Decide on possible locations for the events.
		*/
		void generate_event_location_possible();

		/*!
		*	@brief		Generate the number of people for each eventgroup.
		*/
		void generate_eventgroup_nb_people();

		/*!
		*	@brief		Assign events to teachers.
		*/
		void generate_teacher_event();

		/*!
		*	@brief		Generate cost matrix for assigning events to timeslots.
		*/
		void generate_cost_event_timeslot();

		/*!
		*	@brief		Assign a type of education to each eventgroup.
		*/
		void generate_eventgroup_typeofeducation();

		/*!
		*	@brief		Generate the number of people for each eventgroup for a custom instance.
		*  @param      min_nb_people   The minimum number of people per eventgroup.
		*  @param      max_nb_people   The maximum number of people per eventgroup.
		*/
		void generate_eventgroup_nb_people_custom(int min_nb_people, int max_nb_people);

		/*!
		*	@brief		Generate the number of people for each event.
		*/
		void generate_event_nb_people();

		/*!
		*	@brief		Write the timetable data to a file.
		*/
		void write_timetable_data_to_file();

		/*!
		*	@brief		Clear all timetable data.
		*/
		void clear_timetable_data();


		/*!
		*	@brief		Initialize vectors and assign timeslots to events.
		*/
		void initialize_solution();

		/*!
		*	@brief		Recursive function to generate an initial solution for the problem instance.
		*  @param      current_event   The current event number in the recursion.
		*  @param      cumulative_events_timeslots     The cumulative number of events up to and including the current timeslot.
		*  @returns    True if a feasible solution has been found, false if no such solution exists.
		*/
		bool generate_solution(int current_event, int cumulative_events_timeslot);

		/*!
		*	@brief		Write the initial solution for the problem instance to a file.
		*/
		void write_solution_data_to_file();

		/*!
		*	@brief		Clear all data of the initial solution.
		*/
		void clear_solution_data();
	};

} // namespace timetable

#endif // TIMETABLE_INSTANCE_GENERATOR_H
