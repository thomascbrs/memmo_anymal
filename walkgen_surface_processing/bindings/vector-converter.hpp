///////////////////////////////////////////////////////////////////////////////
// This code is copied from https://github.com/loco-3d/crocoddyl
// Namespace have been changed.
// The code is release under the following license :
//
// BSD 3-Clause License
//
// Copyright (C) 2019-2022, LAAS-CNRS, University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BINDINGS_PYTHON_WALKGEN_VECTOR_CONVERTER_HPP_
#define BINDINGS_PYTHON_WALKGEN_VECTOR_CONVERTER_HPP_

#include <boost/python/stl_iterator.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/to_python_converter.hpp>
#include <vector>

namespace walkgen {
namespace python {

namespace bp = boost::python;

/**
 * @brief Create a pickle interface for the std::vector
 *
 * @param[in] Container  Vector type to be pickled
 * \sa Pickle
 */
template <typename Container>
struct PickleVector : bp::pickle_suite {
  static bp::tuple getinitargs(const Container &) { return bp::make_tuple(); }
  static bp::tuple getstate(bp::object op) { return bp::make_tuple(bp::list(bp::extract<const Container &>(op)())); }
  static void setstate(bp::object op, bp::tuple tup) {
    Container &o = bp::extract<Container &>(op)();
    bp::stl_input_iterator<typename Container::value_type> begin(tup[0]), end;
    o.insert(o.begin(), begin, end);
  }
};

/** @brief Type that allows for registration of conversions from python iterable
 * types. */
template <typename Container>
struct list_to_vector {
  /** @note Registers converter from a python iterable type to the provided
   * type. */
  static void register_converter() {
    bp::converter::registry::push_back(&list_to_vector::convertible, &list_to_vector::construct,
                                       bp::type_id<Container>());
  }

  /** @brief Check if PyObject is iterable. */
  static void *convertible(PyObject *object) {
    // Check if it is a list
    if (!PyList_Check(object)) return 0;

    // Retrieve the underlying list
    bp::object bp_obj(bp::handle<>(bp::borrowed(object)));
    bp::list bp_list(bp_obj);
    bp::ssize_t list_size = bp::len(bp_list);

    // Check if all the elements contained in the current vector is of type T
    for (bp::ssize_t k = 0; k < list_size; ++k) {
      bp::extract<typename Container::value_type> elt(bp_list[k]);
      if (!elt.check()) return 0;
    }
    return object;
  }

  /** @brief Convert iterable PyObject to C++ container type.
   *
   * Container Concept requirements:
   *    * Container::value_type is CopyConstructable.
   *    * Container can be constructed and populated with two iterators.
   * i.e. Container(begin, end)
   */
  static void construct(PyObject *object, bp::converter::rvalue_from_python_stage1_data *data) {
    // Object is a borrowed reference, so create a handle indicting it is
    // borrowed for proper reference counting.
    bp::handle<> handle(bp::borrowed(object));

    // Obtain a handle to the memory block that the converter has allocated
    // for the C++ type.
    typedef bp::converter::rvalue_from_python_storage<Container> storage_type;
    void *storage = reinterpret_cast<storage_type *>(data)->storage.bytes;

    typedef bp::stl_input_iterator<typename Container::value_type> iterator;

    // Allocate the C++ type into the converter's memory block, and assign
    // its handle to the converter's convertible variable.  The C++
    // container is populated by passing the begin and end iterators of
    // the python object to the container's constructor.
    new (storage) Container(iterator(bp::object(handle)),  // begin
                            iterator());                   // end
    data->convertible = storage;
  }

  static bp::list tolist(Container &self) {
    typedef bp::iterator<Container> iterator;
    bp::list list(iterator()(self));
    return list;
  }

  // Need bool operator == for each of the vector (ContactPhase ..etc)
  // TODO: Add a flag and a visitor for index function
  // static int index(Container &self, PyObject *name)
  // {
  //   typedef typename bp::iterator<Container> iterator;
  //   iterator it = std::find(self.begin(), self.end(), name);
  //   if (it != self.end())
  //   {
  //     return static_cast<int>(std::distance(self.begin(), it));
  //   }
  //   else
  //   {
  //     throw std::runtime_error("ValueError : the value is not present.");
  //   }
  // }
};

template <typename Container>
struct overload_base_get_item_for_std_vector
    : public boost::python::def_visitor<overload_base_get_item_for_std_vector<Container>> {
  typedef typename Container::value_type value_type;
  typedef typename Container::value_type data_type;
  typedef size_t index_type;

  template <class Class>
  void visit(Class &cl) const {
    cl.def("__getitem__", &overload_base_get_item_for_std_vector::base_get_item,
           bp::return_value_policy<bp::return_by_value>())
        .def("__iter__", boost::python::iterator<Container>());
    // .def("__str__", &overload_base_get_item_for_std_vector::base_str)
    // .def("__repr__", &overload_base_get_item_for_std_vector::base_str);
  }

 private:
  void base_str(boost::python::back_reference<Container &> container) { std::cout << "HELLO" << std::endl; }
  static boost::python::object base_get_item(boost::python::back_reference<Container &> container, PyObject *i_) {
    namespace bp = ::boost::python;

    index_type idx = convert_index(container.get(), i_);
    typename Container::iterator i = container.get().begin();
    std::advance(i, idx);
    if (i == container.get().end()) {
      PyErr_SetString(PyExc_KeyError, "Invalid index");
      bp::throw_error_already_set();
    }

    // typename bp::to_python_indirect<data_type &,
    // bp::detail::make_reference_holder> convert; return
    // bp::object(bp::handle<>(convert(*i)));
    return bp::object(*i);
  }

  static index_type convert_index(Container &container, PyObject *i_) {
    namespace bp = boost::python;
    bp::extract<size_t> i(i_);
    if (i.check()) {
      size_t index = i();
      if (index < 0) index += container.size();
      if (index >= size_t(container.size()) || index < 0) {
        PyErr_SetString(PyExc_IndexError, "Index out of range");
        bp::throw_error_already_set();
      }
      return index;
    }

    PyErr_SetString(PyExc_TypeError, "Invalid index type");
    bp::throw_error_already_set();
    return index_type();
  }
};

template <class C>
struct PrintableVisitor : public bp::def_visitor<PrintableVisitor<C>> {
  template <class PyClass>
  void visit(PyClass &cl) const {
    cl.def(bp::self_ns::str(bp::self_ns::self)).def(bp::self_ns::repr(bp::self_ns::self));
  }
};

/**
 * @brief Expose an std::vector from a type given as template argument.
 *
 * @param[in] T          Type to expose as std::vector<T>.
 * @param[in] Allocator  Type for the Allocator in std::vector<T,Allocator>.
 * @param[in] NoProxy    When set to false, the elements will be copied when
 * returned to Python.
 */
template <class T, class Allocator = std::allocator<T>, bool NoProxy = false>
struct StdVectorPythonVisitor : public bp::vector_indexing_suite<typename std::vector<T, Allocator>, NoProxy>,
                                public list_to_vector<std::vector<T, Allocator>> {
  typedef std::vector<T, Allocator> Container;
  typedef list_to_vector<Container> FromPythonListConverter;

  static void expose(const std::string &class_name, const std::string &doc_string = "") {
    namespace bp = bp;

    // Overload __getitem__ in order to return properly the std::shared_ptr.
    // Taken from pinocchio3 Not working, indexing phases[0] works, phases[0][1]
    // does not recognize the type even if registered
    overload_base_get_item_for_std_vector<Container> visitor = overload_base_get_item_for_std_vector<Container>();

    bp::class_<Container>(class_name.c_str(), doc_string.c_str())
        .def(StdVectorPythonVisitor())
        .def("tolist", &FromPythonListConverter::tolist, bp::arg("self"), "Returns the std::vector as a Python list.")
        // .def("index", &FromPythonListConverter::index, bp::arg("element"),
        // "Returns the index of the element in the list")
        .def_pickle(PickleVector<Container>())
        .def(visitor);
    // .def(PrintableVisitor<Container>());
    // Register conversion
    FromPythonListConverter::register_converter();
  }
};

}  // namespace python
}  // namespace walkgen

#endif  // BINDINGS_PYTHON_WALKGEN_VECTOR_CONVERTER_HPP_
