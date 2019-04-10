#include "timetable_instance_generator.h"
#include <fstream>
#include <random>
#include <stdexcept>


namespace
{
	std::random_device randdev;
	std::seed_seq seedseq{ randdev(), randdev(), randdev(), randdev(), randdev(), randdev(), randdev(), randdev() };
	std::mt19937_64 generator;
} // anonymous namespace



namespace alg
{
	////////////////////////////////////////////////////////////////////////////////////////////////

	void instance_generator::generate_instance(const std::string& name, int size, int locations)
	{
		instance_size = size;
		nb_locations = locations;

		instance_name = name;
		solution_name = "Start_Solution_";
		solution_name.append(instance_name);


		bool solution_found = false;
		while (!solution_found)
		{
			clear_timetable_data();
			clear_solution_data();

			generate_basic_data();
			generate_names();
			generate_eventgroup_events();
			generate_event_location_possible();
			generate_eventgroup_nb_people();
			generate_event_nb_people();

			initialize_solution();
			solution_found = true;
			int cumulative_nb_events_timeslot = 0;
			for (int t = 0; t < nb_timeslots; ++t)
			{
				for (int l = 0; l < nb_locations; ++l)
					location_used[l] = false;

				if (!generate_solution(cumulative_nb_events_timeslot, cumulative_nb_events_timeslot + timeslot_nb_events[t]))
				{
					solution_found = false;
					break;
				}
				cumulative_nb_events_timeslot += timeslot_nb_events[t];
			}
		}

		write_timetable_data_to_file();
		write_solution_data_to_file();

	}





	////////////////////////////////////////////////////////////////////////////////////////////////

	void instance_generator::generate_instance_custom(const std::string &name,
		int timeslots,
		int events_per_timeslot,
		int locations,
		int min_nb_people_per_event,
		int max_nb_people_per_event)
	{
		// do check
		if (events_per_timeslot > locations)
			throw std::runtime_error("Problem instance cannot have more events_per_timeslot than there are locations!\n"
				"Events per timeslot = " + std::to_string(events_per_timeslot) +
				"\nLocations = " + std::to_string(locations));
		if (min_nb_people_per_event > max_nb_people_per_event)
			throw std::runtime_error("min_nb_people_per_event cannot be larger than max_nb_people_per_event!\n"
				"min_nb_people_per_event = " + std::to_string(min_nb_people_per_event) +
				"\nmax_nb_people_per_event = " + std::to_string(max_nb_people_per_event));


		// name
		instance_name = name;
		solution_name = "Start_Solution_";
		solution_name.append(instance_name);


		bool solution_found = false;
		while (!solution_found)
		{
			clear_timetable_data();
			clear_solution_data();

			// basic data
			nb_timeslots = timeslots;
			nb_locations = locations;
			timeslot_nb_events.reserve(nb_timeslots);
			nb_events = 0;
			for (int t = 0; t < nb_timeslots; ++t) {
				timeslot_nb_events.push_back(events_per_timeslot);
				nb_events += events_per_timeslot;
			}
			nb_eventgroups = nb_locations;
			nb_teachers = nb_events;

			generate_names();
			generate_eventgroup_events();
			generate_event_location_possible();
			generate_eventgroup_nb_people_custom(min_nb_people_per_event, max_nb_people_per_event);
			generate_event_nb_people();

			generate_teacher_event();
			generate_cost_event_timeslot();
			generate_eventgroup_typeofeducation();

			initialize_solution();
			solution_found = true;
			int cumulative_nb_events_timeslot = 0;
			for (int t = 0; t < nb_timeslots; ++t)
			{
				for (int l = 0; l < nb_locations; ++l)
					location_used[l] = false;

				if (!generate_solution(cumulative_nb_events_timeslot, cumulative_nb_events_timeslot + timeslot_nb_events[t]))
				{
					solution_found = false;
					break;
				}
				cumulative_nb_events_timeslot += timeslot_nb_events[t];
			}
		}

		write_timetable_data_to_file();
		write_solution_data_to_file();
	}





	////////////////////////////////////////////////////////////////////////////////////////////////

	void instance_generator::generate_basic_data()
	{
		// nb_locations is input to program because depends on Building Data


		// set the number of timeslots
		nb_timeslots = nb_timeslots_each_dataset;



		// generate a number of events per timeslot
		std::uniform_int_distribution<int> dist_nb_events_timeslot(3, nb_locations);
		nb_events = 0;
		timeslot_nb_events.reserve(nb_timeslots);
		for (int t = 0; t < nb_timeslots; ++t) {
			int nb_events_ts = dist_nb_events_timeslot(generator);
			timeslot_nb_events.push_back(nb_events_ts);
			nb_events += nb_events_ts;
		}


		// generate a number of eventgroups
		nb_eventgroups = nb_locations;


		// generate a number of teachers
		nb_teachers = nb_events;
	}

	void instance_generator::generate_names()
	{
		// Event names
		event_names.reserve(nb_events);
		for (int i = 0; i < nb_events; ++i)
		{
			std::string name;
			name = "Event_";
			name.append(std::to_string(i + 1));
			event_names.push_back(name);
		}

		// Eventgroup names
		eventgroup_names.reserve(nb_eventgroups);
		for (int i = 0; i < nb_eventgroups; ++i)
		{
			std::string name;
			name = "Eventgroup_";
			name.append(std::to_string(i + 1));
			eventgroup_names.push_back(name);
		}

		// Location names
		location_names.reserve(nb_locations);
		for (int i = 0; i < nb_locations; ++i)
		{
			std::string name;
			name = "Room_";
			name.append(std::to_string(i + 1));
			location_names.push_back(name);
		}
	}

	void instance_generator::generate_eventgroup_events()
	{
		// first initialize with zeros everywhere
		eventgroup_event.reserve(nb_eventgroups*nb_events);
		for (int i = 0; i < nb_eventgroups; ++i) {
			for (int j = 0; j < nb_events; ++j) {
				eventgroup_event.push_back(0);
			}
		}


		// now assign events to eventgroups
		int event_nr = 0;
		std::vector<int> remaining_eventgroups;
		remaining_eventgroups.reserve(nb_eventgroups);

		for (int t = 0; t < nb_timeslots; ++t)
		{
			remaining_eventgroups.clear();
			for (int i = 0; i < nb_eventgroups; ++i)
				remaining_eventgroups.push_back(i);

			for (int j = 0; j < timeslot_nb_events[t]; ++j)
			{
				std::uniform_int_distribution<int> dist_eventgroup_index(0, remaining_eventgroups.size() - 1);
				int eventgroup_index = dist_eventgroup_index(generator);
				int eventgroup = remaining_eventgroups[eventgroup_index];
				eventgroup_event[eventgroup*nb_events + event_nr] = 1;
				remaining_eventgroups.erase(remaining_eventgroups.begin() + eventgroup_index);

				++event_nr;
			}
		}
	}

	void instance_generator::generate_event_location_possible()
	{
		std::bernoulli_distribution dist_possible(prob_room_possible);

		event_location_possible.reserve(nb_events*nb_locations);
		for (int i = 0; i < nb_events; ++i)
		{
			for (int j = 0; j < nb_locations; ++j)
			{
				if (dist_possible(generator))
					event_location_possible.push_back(1);
				else
					event_location_possible.push_back(0);
			}
		}
	}

	void instance_generator::generate_eventgroup_nb_people()
	{
		if (instance_size == 1 /* small */)
		{
			std::uniform_int_distribution<int> dist_nb_people(nb_people_per_event_minimum_small_dataset, nb_people_per_event_maximum_small_dataset);

			eventgroup_nb_people.reserve(nb_eventgroups);
			for (int i = 0; i < nb_eventgroups; ++i)
				eventgroup_nb_people.push_back(dist_nb_people(generator));
		}
		else if (instance_size == 2 /* medium */)
		{
			std::uniform_int_distribution<int> dist_nb_people(nb_people_per_event_minimum_medium_dataset, nb_people_per_event_maximum_medium_dataset);

			eventgroup_nb_people.reserve(nb_eventgroups);
			for (int i = 0; i < nb_eventgroups; ++i)
				eventgroup_nb_people.push_back(dist_nb_people(generator));
		}
		else if (instance_size == 3 /* large */)
		{
			std::uniform_int_distribution<int> dist_nb_people(nb_people_per_event_minimum_large_dataset, nb_people_per_event_maximum_large_dataset);

			eventgroup_nb_people.reserve(nb_eventgroups);
			for (int i = 0; i < nb_eventgroups; ++i)
				eventgroup_nb_people.push_back(dist_nb_people(generator));
		}
	}

	void instance_generator::generate_eventgroup_nb_people_custom(int min_nb_people, int max_nb_people)
	{
		std::uniform_int_distribution<int> dist_nb_people(min_nb_people, max_nb_people);

		eventgroup_nb_people.reserve(nb_eventgroups);
		for (int i = 0; i < nb_eventgroups; ++i)
			eventgroup_nb_people.push_back(dist_nb_people(generator));
	}

	void instance_generator::generate_event_nb_people()
	{
		event_nb_people.reserve(nb_events);
		for (int j = 0; j < nb_events; ++j)
		{
			event_nb_people.push_back(0);
			for (int i = 0; i < nb_eventgroups; ++i)
			{
				if (eventgroup_event[i*nb_events + j] == 1)
					event_nb_people[j] += eventgroup_nb_people[i];
			}
		}
	}

	void instance_generator::generate_teacher_event()
	{
		if (nb_teachers == nb_events)
		{
			teacher_event.reserve(nb_teachers*nb_events);
			for (int i = 0; i < nb_teachers; ++i)
			{
				for (int j = 0; j < nb_events; ++j)
				{
					if (i == j)
						teacher_event.push_back(1);
					else
						teacher_event.push_back(0);
				}
			}
		}
		else
		{
			throw std::runtime_error("Error in function instance_generator::generate_teacher_event().\nNumber of teachers should equal number of events.");
			// to do
		}
	}

	void instance_generator::generate_cost_event_timeslot()
	{
		std::uniform_real_distribution<double> dist_prob(0, 1);

		cost_event_timeslot.reserve(nb_events*nb_timeslots);
		for (int i = 0; i < nb_events; ++i)
		{
			for (int t = 0; t < nb_timeslots; ++t)
			{
				double prob = dist_prob(generator);
				if (prob < 0.4)
					cost_event_timeslot.push_back(0);
				else if (prob < 0.6)
					cost_event_timeslot.push_back(50);
				else if (prob < 0.85)
					cost_event_timeslot.push_back(100);
				else if (prob < 0.9)
					cost_event_timeslot.push_back(150);
				else
					cost_event_timeslot.push_back(200);
			}
		}
	}

	void instance_generator::generate_eventgroup_typeofeducation()
	{
		eventgroup_typeofeducation.reserve(nb_eventgroups);
		for (int i = 0; i < nb_eventgroups; ++i)
			eventgroup_typeofeducation.push_back(0);
	}

	void instance_generator::write_timetable_data_to_file()
	{
		std::ofstream stream(instance_name);
		if (!stream.is_open())
			throw std::runtime_error("Error in instance_generator::write_timetable_data_to_file. \nCouldn't open file.");


		// basic data
		stream << instance_name << "\n\n";
		stream << nb_events << "\n";
		stream << nb_eventgroups << "\n";
		stream << nb_timeslots << "\n";
		stream << nb_locations << "\n";
		stream << nb_teachers << "\n\n";

		// names events, eventgroups, and locations
		for (int i = 0; i < nb_events; ++i)
			stream << event_names[i] << "\n";
		stream << "\n";

		for (int i = 0; i < nb_eventgroups; ++i)
			stream << eventgroup_names[i] << "\n";
		stream << "\n";

		for (int i = 0; i < nb_locations; ++i)
			stream << location_names[i] << "\n";
		stream << "\n";

		// eventgroup event
		for (int i = 0; i < nb_eventgroups; ++i) {
			if (i > 0)
				stream << "\n";
			for (int j = 0; j < nb_events; ++j) {
				if (j > 0)
					stream << "\t";
				stream << eventgroup_event[i*nb_events + j];
			}
		}
		stream << "\n\n";

		// event location possible
		for (int i = 0; i < nb_events; ++i) {
			if (i > 0)
				stream << "\n";
			for (int j = 0; j < nb_locations; ++j) {
				if (j > 0)
					stream << "\t";
				stream << event_location_possible[i*nb_locations + j];
			}
		}
		stream << "\n\n";

		// event nb_people
		for (int i = 0; i < nb_events; ++i) {
			if (i > 0)
				stream << "\n";
			stream << event_nb_people[i];
		}
		stream << "\n\n";

		// eventgroup nb_people
		for (int i = 0; i < nb_eventgroups; ++i) {
			if (i > 0)
				stream << "\n";
			stream << eventgroup_nb_people[i];
		}
		stream << "\n\n";

		// teacher lecture
		for (int i = 0; i < nb_teachers; ++i)
		{
			if (i > 0)
				stream << "\n";
			for (int j = 0; j < nb_events; ++j)
			{
				if (j > 0)
					stream << "\t";
				stream << teacher_event[i*nb_events + j];
			}
		}
		stream << "\n\n";

		// cost lecture ts
		for (int i = 0; i < nb_events; ++i)
		{
			if (i > 0)
				stream << "\n";
			for (int j = 0; j < nb_timeslots; ++j)
			{
				if (j > 0)
					stream << "\t";
				stream << cost_event_timeslot[i*nb_timeslots + j];
			}
		}
		stream << "\n\n";

		// series type of education
		for (int i = 0; i < nb_eventgroups; ++i)
		{
			if (i > 0)
				stream << "\n";
			stream << eventgroup_typeofeducation[i];
		}
	}

	void instance_generator::clear_timetable_data()
	{
		timeslot_nb_events.clear();

		event_names.clear();
		eventgroup_names.clear();
		location_names.clear();

		eventgroup_event.clear();
		event_location_possible.clear();
		event_nb_people.clear();
		eventgroup_nb_people.clear();

		teacher_event.clear();
		cost_event_timeslot.clear();
		eventgroup_typeofeducation.clear();
	}





	////////////////////////////////////////////////////////////////////////////////////////////////

	void instance_generator::initialize_solution()
	{
		// initialize vectors
		event_timeslot.reserve(nb_events);
		event_location.reserve(nb_events);
		for (int i = 0; i < nb_events; ++i)
		{
			event_timeslot.push_back(0);
			event_location.push_back(-1);
		}
		location_used.reserve(nb_locations);
		for (int l = 0; l < nb_locations; ++l)
			location_used.push_back(false);


		// assign timeslots
		{
			int event_nr = 0;
			for (int t = 0; t < nb_timeslots; ++t)
			{
				for (int j = 0; j < timeslot_nb_events[t]; ++j)
				{
					// timeslot
					event_timeslot[event_nr] = t;

					// go to next event
					++event_nr;
				}
			}
		}
	}

	bool instance_generator::generate_solution(int current_event, int cumulative_events_timeslot)
	{
		if (current_event >= cumulative_events_timeslot)
		{
			return true;
		}
		else
		{
			for (int l = 0; l < nb_locations; ++l)
			{
				if (!location_used[l] && event_location_possible[current_event*nb_locations + l])
				{
					event_location[current_event] = l;
					location_used[l] = true;

					if (generate_solution(current_event + 1, cumulative_events_timeslot))
						return true;

					event_location[current_event] = -1;
					location_used[l] = false;
				}
			}
		}

		// if no feasible solution has been found for this timeslot
		return false;
	}

	void instance_generator::write_solution_data_to_file()
	{
		std::ofstream stream(solution_name);
		if (!stream.is_open())
			throw std::runtime_error("Error in instance_generator::write_solution_data_to_file. \nCouldn't open file.");

		for (int i = 0; i < nb_events; ++i)
		{
			if (i > 0)
				stream << "\n";
			stream << event_timeslot[i];
			stream << "\t";
			stream << event_location[i];
		}
	}

	void instance_generator::clear_solution_data()
	{
		event_timeslot.clear();
		event_location.clear();
	}



} // namespace alg
