#ifndef FN_OBJ_HPP
#define FN_OBJ_HPP

#include "fn.hpp"
#include "mem_fn.hpp"

namespace std {
namespace fn_tr {
template <typename Class> using call_op_tr_ta = mem_fn_tr_s<decltype(&Class::operator())>;
template <typename Class> struct fn_obj_tr_s_ : static_fn_tr_s<typename call_op_tr_ta<Class>::fn_ta> {
  using call_op_ta = call_op_tr_ta<Class>;
};

template <typename Class> struct fn_obj_tr_s : fn_obj_tr_s_<std::remove_cvref_t<Class>> {};
} // namespace fn_tr
} // namespace std

#endif /* FN_OBJ_HPP */
