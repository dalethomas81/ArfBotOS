#pragma once

#include <array>
#include <vector>
#include <ostream>
#include <iostream>

template <typename T>
class Matrix
{
private:
    size_t _Colums, _Rows;
    std::vector<T> _data;

public:
    // Constructors
    explicit Matrix() : _Colums(0), _Rows(0), _data() {}

    explicit Matrix(const size_t colums, const size_t rows, T fill = T()) : _Colums(colums),
                                                                            _Rows(rows),
                                                                            _data(colums * rows) {}

    Matrix(const Matrix<T> &m) : _Colums(m._Colums),
                                 _Rows(m._Rows),
                                 _data(m._data) {}

    // Initialization by vector
    Matrix<T> &operator=(const std::vector<T> &m)
    {
        set(m);
        return *this;
    }

    void set(const std::vector<T> m)
    {
        if (m.size() == _Colums * _Rows)
            _data = m;
    }

    // Operators
    Matrix<T> &operator=(const Matrix<T> &m)
    {
        _Colums = m._Colums;
        _Rows = m._Rows;
        _data = m._data;
        return *this;
    }

    std::vector<T> operator[](size_t i)
    {
        size_t f = _Colums * i;
        size_t s = _Colums * i + _Colums;
        return std::vector<T>(_data.begin() + f, _data.begin() + s);
    }

    // Get item by y and x coord. Faster then []
    T &at(size_t y, size_t x)
    {
        return _data.at(y * _Colums + x);
    }

    // Matrix multiplication
    Matrix<T> operator*(const Matrix<T> &m)
    {
        if (this->_Colums != m._Rows)
        {
            return Matrix<T>();
        }
        Matrix<T> r(m._Colums, this->_Rows);
        for (size_t i = 0; i < this->_Rows; i++)
        {
            for (size_t j = 0; j < m._Colums; j++)
            {
                r._data[m._Colums * i + j] = 0;
                for (size_t k = 0; k < this->_Colums; k++)
                    r._data[m._Colums * i + j] = r._data[m._Colums * i + j] + this->_data[this->_Colums * i + k] * m._data[m._Colums * k + j];
            }
        }
        return r;
    }

    Matrix<T> operator+(const Matrix<T> &m)
    {
        if (_Colums != m._Colums || _Rows != m._Rows)
            return Matrix<T>();
        Matrix<T> r(_Colums, _Rows);
        for (size_t y = 0; y < _Rows; y++)
            for (size_t x = 0; x < _Colums; x++)
                r._data[y * _Colums + x] = _data.at(y * _Colums + x) + m._data.at(y * _Colums + x);
        return r;
    }

    Matrix<T> operator-(const Matrix<T> &m)
    {
        if (_Colums != m._Colums || _Rows != m._Rows)
            return Matrix<T>();
        Matrix<T> r(_Colums, _Rows);
        for (size_t y = 0; y < _Rows; y++)
            for (size_t x = 0; x < _Colums; x++)
                r._data[y * _Colums + x] = _data.at(y * _Colums + x) - m._data.at(y * _Colums + x);
        return r;
    }

    Matrix<T> transpose()
    {
        Matrix<T> r(_Rows, _Colums);
        for (size_t i = 0; i < _Rows; i++)
            for (size_t j = 0; j < _Colums; j++)
                r._data[_Rows * j + i] = _data[_Colums * i + j];
        return r;
    }

    template <typename = typename std::enable_if_t<std::is_arithmetic<T>::value>>
    void scale(double k)
    {
        for (size_t i = 0; i < _Rows; i++)
            for (size_t j = 0; j < _Colums; j++)
                _data[_Rows * j + i] *= k;
    }

    // Print matrix;
    void printMatrix(std::ostream &s)
    {
        for (size_t y = 0; y < _Rows; y++)
        {
            for (size_t x = 0; x < _Colums; x++)
            {
                s << _data[y * _Colums + x] << '\t';
            }
            s << '\n';
        }
    }

    // Get data
    std::vector<T> &data() const
    {
        return _data;
    }

    // get matrix parameters
    const size_t rows() const { return _Rows; }
    const size_t colums() const { return _Colums; }
};