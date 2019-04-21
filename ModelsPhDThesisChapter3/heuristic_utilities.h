/*!
*	@file	heuristic_utilities.h
*	@author		Hendrik Vermuyten
*	@brief	Utilities for the heuristic.
*/

#ifndef HEURISTIC_UTILITIES_H
#define HEURISTIC_UTILITIES_H

#include <vector>

/*!
*	@namespace	alg
*	@brief	The main namespace containing all algorithm elements.
*/
namespace alg
{
	/*!
	*	@brief	Two-dimensional matrix.
	*/
	template<typename T>
	class matrix2D
	{
		/*!
		*	@brief	Vector that contains the elements.
		*/
		std::vector<T> _elements;

		/*!
		*	@brief	The number of rows of the matrix.
		*/
		size_t _nb_rows = 0;

		/*!
		*	@brief	The number of columns of the matrix.
		*/
		size_t _nb_columns = 0;

	public:
		/*!
		*	@brief	Default constructor.
		*/
		matrix2D() { }

		/*!
		*	@brief	Copy constructor.
		*	@param	other	Another matrix to copy-construct a new matrix from.
		*/
		matrix2D(const matrix2D<T>& other)
		{
			_elements = other._elements;
			_nb_columns = other._nb_columns;
			_nb_rows = other._nb_rows;
		}

		/*!
		*	@brief	Assignment operator.
		*	@param	other	Another matrix to assign-construct a new matrix from.
		*/
		matrix2D<T>& operator=(const matrix2D<T>& other)
		{
			if (this != &other)
			{
				_elements = other._elements;
				_nb_columns = other._nb_columns;
				_nb_rows = other._nb_rows;
			}
			return *this;
		}

		/*!
		*	@brief	Set the size of the matrix.
		*	@param	nb_rows		The number of rows.
		*	@param	nb_columns	The number of columns.
		*/
		void set(size_t nb_rows, size_t nb_columns)
		{
			_nb_rows = nb_rows;
			_nb_columns = nb_columns;
			_elements.reserve(_nb_rows*_nb_columns);
			for (int i = 0; i < _nb_rows*_nb_columns; ++i)
				_elements.push_back(T());
		}

		/*!
		*	@brief	Set the size of the matrix and fill every cell with the given value.
		*	@param	nb_rows		The number of rows.
		*	@param	nb_columns	The number of columns.
		*	@param	value		The value to fill every cell with.
		*/
		void set(size_t nb_rows, size_t nb_columns, T value)
		{
			_nb_rows = nb_rows;
			_nb_columns = nb_columns;
			_elements.reserve(_nb_rows*_nb_columns);
			for (int i = 0; i < _nb_rows*_nb_columns; ++i)
				_elements.push_back(value);
		}

		/*!
		*	@brief	Fill every cell with the given value.
		*	@param	value		The value to fill every cell with.
		*/
		void fill(T value)
		{
			for (size_t i = 0; i < _nb_rows*_nb_columns; ++i)
				_elements.at(i) = value;
		}

		/*!
		*	@brief	Access the element at the given row and column (read & write).
		*	@param	row		The given row.
		*	@param	column	The given column.
		*/
		T& at(size_t row, size_t column)
		{
			return _elements.at(row*_nb_columns + column);
		}

		/*!
		*	@brief	Access the element at the given row and column (read only).
		*	@param	row		The given row.
		*	@param	column	The given column.
		*/
		const T& at(size_t row, size_t column) const
		{
			return _elements.at(row*_nb_columns + column);
		}

		/*!
		*	@brief	Delete the data of the matrix and reset the size to zero.
		*/
		void clear()
		{
			_elements.clear();
			_nb_rows = 0;
			_nb_columns = 0;
		}

		/*!
		*	@brief	Destructor.
		*/
		~matrix2D() { }

		/*!
		*	@brief	Print the values of the matrix to screen.
		*/
		void print()
		{
			for (size_t i = 0; i < _nb_rows; ++i)
			{
				std::cout << "\n";
				for (size_t j = 0; j < _nb_columns; ++j)
				{
					std::cout << "\t" << _elements.at(i*_nb_columns + j);
				}
			}
		}
	};






	/*!
	*	@brief	Three-dimensional matrix.
	*/
	template<typename T>
	class matrix3D
	{
		/*!
		*	@brief	Vector that contains the elements.
		*/
		std::vector<T> _elements;

		/*!
		*	@brief	The number of rows of the matrix.
		*/
		size_t _nb_rows = 0;

		/*!
		*	@brief	The number of columns of the matrix.
		*/
		size_t _nb_columns = 0;

		/*!
		*	@brief	The depth (i.e. the third dimension) of the matrix.
		*/
		size_t _depth = 0;

	public:
		/*!
		*	@brief	Default constructor.
		*/
		matrix3D() { }

		/*!
		*	@brief	Copy constructor.
		*	@param	other	Another matrix to copy-construct a new matrix from.
		*/
		matrix3D(const matrix3D<T>& other)
		{
			_elements = other._elements;
			_nb_columns = other._nb_columns;
			_nb_rows = other._nb_rows;
			_depth = other._depth;
		}

		/*!
		*	@brief	Assignment operator.
		*	@param	other	Another matrix to assign-construct a new matrix from.
		*/
		matrix3D<T>& operator=(const matrix3D<T>& other)
		{
			if (this != &other)
			{
				_elements = other._elements;
				_nb_columns = other._nb_columns;
				_nb_rows = other._nb_rows;
				_depth = other._depth;
			}
			return *this;
		}

		/*!
		*	@brief	Set the size of the matrix.
		*	@param	nb_rows		The number of rows.
		*	@param	nb_columns	The number of columns.
		*	@param	depth		The depth.
		*/
		void set(size_t nb_rows, size_t nb_columns, size_t depth)
		{
			_nb_rows = nb_rows;
			_nb_columns = nb_columns;
			_depth = depth;

			_elements.reserve(_nb_rows*_nb_columns*_depth);
			for (int i = 0; i < _nb_rows*_nb_columns*_depth; ++i)
				_elements.push_back(T());
		}

		/*!
		*	@brief	Set the size of the matrix and fill every cell with the given value.
		*	@param	nb_rows		The number of rows.
		*	@param	nb_columns	The number of columns.
		*	@param	depth		The depth.
		*	@param	value		The value to fill every cell with.
		*/
		void set(size_t nb_rows, size_t nb_columns, size_t depth, T value)
		{
			_nb_rows = nb_rows;
			_nb_columns = nb_columns;
			_depth = depth;

			_elements.reserve(_nb_rows*_nb_columns*_depth);
			for (int i = 0; i < _nb_rows*_nb_columns*_depth; ++i)
				_elements.push_back(value);
		}

		/*!
		*	@brief	Fill every cell with the given value.
		*	@param	value		The value to fill every cell with.
		*/
		void fill(T value)
		{
			for (size_t i = 0; i < _nb_rows*_nb_columns*_depth; ++i)
				_elements.at(i) = value;
		}

		/*!
		*	@brief	Access the element at the given row, column, and depth (read & write).
		*	@param	row		The given row.
		*	@param	column	The given column.
		*	@param	at_depth	The given depth.
		*/
		T& at(size_t row, size_t column, size_t at_depth)
		{
			return _elements.at(row*_nb_columns*_depth + column * _depth + at_depth);
		}

		/*!
		*	@brief	Access the element at the given row, column, and depth (read only).
		*	@param	row		The given row.
		*	@param	column	The given column.
		*	@param	at_depth	The given depth.
		*/
		const T& at(size_t row, size_t column, size_t at_depth) const
		{
			return _elements.at(row*_nb_columns*_depth + column * _depth + at_depth);
		}

		/*!
		*	@brief	Delete the data of the matrix and reset the size to zero.
		*/
		void clear()
		{
			_elements.clear();
			_nb_rows = 0;
			_nb_columns = 0;
			_depth = 0;
		}

		/*!
		*	@brief	Destructor.
		*/
		~matrix3D() { }
	};

} // namespace alg

#endif // !HEURISTIC_UTILITIES_H