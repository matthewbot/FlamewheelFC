#ifndef FC_MATH_TYPE_TRAITS_H
#define FC_MATH_TYPE_TRAITS_H

struct false_type { static constexpr bool value = false; };
struct true_type { static constexpr bool value = true; };

template <typename T>
struct is_arithmetic : public false_type { };

template <>
struct is_arithmetic<float> : public true_type { };

template <>
struct is_arithmetic<double> : public true_type { };

template <>
struct is_arithmetic<char> : public true_type { };

template <>
struct is_arithmetic<short> : public true_type { };

template <>
struct is_arithmetic<int> : public true_type { };

template <>
struct is_arithmetic<long> : public true_type { };

template <bool B, typename T=void>
struct enable_if { };

template <typename T>
struct enable_if<true, T> { using type = T; };

#endif
