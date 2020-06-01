/*
 * Copyright (C) Insightness AG, Switzerland - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Stefan Isler <stefan@insightness.com>
 * Fri Dec 15 2017
 */

#ifndef INESS_COMMON_UTIL_TYPE_REGISTRY_HPP_
#define INESS_COMMON_UTIL_TYPE_REGISTRY_HPP_

#include <tuple>
#include <type_traits>
#include <string>
#include <cstdint>
#include <vector>

namespace iness
{
namespace util
{


/*! Provides compile time type information (ids) on a fixed, configurable set of types and functionality to instantiate and call templated objects automatically with the registered
 *  types.
 */
template<typename ... T_SUPPORTED_TYPES>
class TypeRegistry
{

public:
  typedef std::size_t Id;
  typedef std::tuple<T_SUPPORTED_TYPES...> type_list;

public:

  //! Returns a unique type id for each registered type (not that if the same type is registered twice, both will get the same id)
  template<typename T_QUERY_TYPE>
  static std::size_t getId();

  /*! Allows automatically calling the call function of the templated callable object
   * with given arguments, the functor being
   * instantiated with all types in T_CALL_TYPE_SET. The callable object T_CALLABLE must
   * feature a static call function. T_CALL_TYPE_SET must have a tuple member named type_list
   * 
   * This means:
   *  - You create a callable object:
   * 
   *  template<typename TYPE>
   *  struct Callable
   *  {
   *    static void call(*any arguments you like*){};
   *  };
   * 
   *  - And if you do:
   *    ExtendedTypeRegistry::ExecuteCallableOnEach<Callable>::call(*your arguments*);
   * 
   *  - The template will instantiate function calls to the call function as follows:
   *    Callable<std::string>::call(*your arguments*);
   *    Callable<bool>::call(*your arguments*);
   *    Callable<float>::call(*your arguments*);
   *    ... and all other types stored within ExtendedTypeRegistry
   * 
   * */
  template<template <typename> class T_CALLABLE, typename T_CALL_TYPE_SET = TypeRegistry<T_SUPPORTED_TYPES...> >
  struct ExecuteCallableOnEach
  {
    template<typename ... T_VAR_ARGS>
    static void call( T_VAR_ARGS ... _func_obj_constructor_args );
  };

private:

  template<template <typename> class T_TEMPLATE_FUNCTION_TYPE, typename T_CALL_TYPE_SET, std::size_t T_POS>
  struct ExecuteCallableOnEachAt
  {
    template<typename ... T_VAR_ARGS>
    static void call( T_VAR_ARGS ... _func_obj_constructor_args );
  };

  template<template <typename> class T_TEMPLATE_FUNCTION_TYPE, typename T_CALL_TYPE_SET >
  struct ExecuteCallableOnEachAt<T_TEMPLATE_FUNCTION_TYPE, T_CALL_TYPE_SET, std::tuple_size<typename T_CALL_TYPE_SET::type_list>::value >
  {
    template<typename ... T_VAR_ARGS>
    static void call( T_VAR_ARGS ... );
  };
};


template<typename ... T_TYPE_LIST>
struct TypeList
{
  typedef std::tuple<T_TYPE_LIST...> type_list;
};

typedef TypeRegistry<std::string, bool, float, double, int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t, std::vector<std::function<void()>> > ExtendedTypeRegistry;
typedef TypeList<std::string> StringType;
typedef TypeList<bool, float, double, int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t> NumericTypes;
typedef TypeList<std::string,bool, float, double, int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t> StdStreamSupportedTypes;

}
}

#include "impl/type_registry.inl"

#endif // INESS_COMMON_UTIL_TYPE_REGISTRY_HPP_