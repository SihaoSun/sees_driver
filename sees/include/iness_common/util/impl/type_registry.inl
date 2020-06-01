/*
 * Copyright (C) Insightness AG, Switzerland - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Stefan Isler <stefan@insightness.com>
 * Tue Feb 20 2018
 */


namespace iness
{
namespace util
{


template<typename T_ONE, typename T_TWO, std::size_t T_POS,  bool T_COMP = std::is_same<T_ONE,T_TWO>::value >
struct count_pos_if_equal
{
  static constexpr std::size_t get()
  {
    return T_COMP?T_POS:0;
  }
};

//! Class to calculate the position of type T_QUERY_TYPE within the list of types {T...}
template<typename T_QUERY_TYPE, std::size_t T_QUERY_POS, class... T>
struct type_pos;

template<typename T_QUERY_TYPE, std::size_t T_QUERY_POS, typename T_FIRST_TYPE_IN_LIST, typename ... T_TYPE_LIST>
struct type_pos<T_QUERY_TYPE,T_QUERY_POS,T_FIRST_TYPE_IN_LIST, T_TYPE_LIST...>
{
  static const std::size_t result = count_pos_if_equal<T_QUERY_TYPE,T_FIRST_TYPE_IN_LIST,T_QUERY_POS>::get() + type_pos<T_QUERY_TYPE,T_QUERY_POS+1,T_TYPE_LIST...>::result;
};

template<typename T_QUERY_TYPE, std::size_t T_QUERY_POS, typename T_FIRST_TYPE_IN_LIST>
struct type_pos< T_QUERY_TYPE , T_QUERY_POS, T_FIRST_TYPE_IN_LIST >
{
  static const std::size_t result = count_pos_if_equal<T_QUERY_TYPE,T_FIRST_TYPE_IN_LIST,T_QUERY_POS>::get();
};


template<typename ... T_SUPPORTED_TYPES>
template<typename T_QUERY_TYPE>
std::size_t TypeRegistry<T_SUPPORTED_TYPES...>::getId()
{
  static_assert(type_pos<T_QUERY_TYPE,1, T_SUPPORTED_TYPES...>::result!=0,"TypeRegistry::getId:: You tried to query the type id of a type that is not supported by the given registry"); // starting with search pos "1" yields that only for a not found type zero is returned.
  return type_pos<T_QUERY_TYPE,0, T_SUPPORTED_TYPES...>::result; // Now that we know the type is contained, we'll start with index zero
}


template<typename ... T_SUPPORTED_TYPES>
template<template <typename> class T_CALLABLE, typename T_CALL_TYPE_SET >
template<typename ... T_VAR_ARGS>
void TypeRegistry<T_SUPPORTED_TYPES...>::ExecuteCallableOnEach<T_CALLABLE,T_CALL_TYPE_SET>::call( T_VAR_ARGS ... _func_obj_constructor_args )
{
  ExecuteCallableOnEachAt<T_CALLABLE, T_CALL_TYPE_SET ,0>::call(_func_obj_constructor_args...);
  return;
}

template<typename ... T_SUPPORTED_TYPES>
template<template <typename> class T_TEMPLATE_FUNCTION_TYPE, typename T_CALL_TYPE_SET, std::size_t T_POS>
template<typename ... T_VAR_ARGS>
void TypeRegistry<T_SUPPORTED_TYPES...>::ExecuteCallableOnEachAt<T_TEMPLATE_FUNCTION_TYPE,T_CALL_TYPE_SET,T_POS>::call( T_VAR_ARGS ... _func_obj_constructor_args )
{
  typedef typename std::tuple_element<T_POS, typename T_CALL_TYPE_SET::type_list >::type CurrentSwitchType;

  T_TEMPLATE_FUNCTION_TYPE<CurrentSwitchType>::call(_func_obj_constructor_args...);
  
  return ExecuteCallableOnEachAt<T_TEMPLATE_FUNCTION_TYPE,T_CALL_TYPE_SET,T_POS+1>::call(_func_obj_constructor_args...);
}

template<typename ... T_SUPPORTED_TYPES>
template<template <typename> class T_TEMPLATE_FUNCTION_TYPE, typename T_CALL_TYPE_SET >
template<typename ... T_VAR_ARGS>
void TypeRegistry<T_SUPPORTED_TYPES...>::ExecuteCallableOnEachAt<T_TEMPLATE_FUNCTION_TYPE, T_CALL_TYPE_SET, std::tuple_size<typename T_CALL_TYPE_SET::type_list>::value >::call( T_VAR_ARGS ... )
{
  return;
}


}
}