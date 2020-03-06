#pragma once
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

template<class T>
inline
boost::python::list toPythonList(const std::vector<T>& v)
{
  typename std::vector<T>::const_iterator iter;
  boost::python::list l;
  for (iter = v.begin(); iter != v.end(); ++iter)
    l.append(*iter);
  return l;
}

template<typename T>
inline
std::vector<T> toStdVector(const boost::python::object& iterable)
{
  return std::vector<T>( boost::python::stl_input_iterator<T>(iterable),
                         boost::python::stl_input_iterator<T>() );
}

