#pragma once

#include <pathfinder/geometry/vector2.hpp>

#include <stdexcept>
#include <type_traits>

namespace pathfinder {

namespace detail {

template <typename Tag>
struct ParamFactory {
    constexpr ParamFactory() = default;
    struct Value { double v; };
    constexpr Value operator=(double v) const { return Value{v}; }
};

struct ForwardTag {};
struct BackwardTag {};
struct RightTag {};
struct LeftTag {};
struct UpTag {};
struct DownTag {};
struct XTag {};
struct YTag {};
struct ZTag {};

template <typename Tag, typename T>
struct is_param_of : std::false_type {};

template <typename Tag>
struct is_param_of<Tag, typename ParamFactory<Tag>::Value> : std::true_type {};

template <typename Tag, typename... Ts>
constexpr int count_of() {
    return ((is_param_of<Tag, Ts>::value ? 1 : 0) + ... + 0);
}

template <typename Tag>
constexpr double pick_value() {
    return 0.0;
}

template <typename Tag, typename First, typename... Rest>
constexpr double pick_value(First first, Rest... rest) {
    if constexpr (is_param_of<Tag, First>::value) {
        return first.v;
    } else {
        return pick_value<Tag>(rest...);
    }
}

} // namespace detail

inline constexpr detail::ParamFactory<detail::ForwardTag>  forward;
inline constexpr detail::ParamFactory<detail::BackwardTag> backward;
inline constexpr detail::ParamFactory<detail::RightTag>    right;
inline constexpr detail::ParamFactory<detail::LeftTag>     left;
inline constexpr detail::ParamFactory<detail::UpTag>       up;
inline constexpr detail::ParamFactory<detail::DownTag>     down;
// `x_`, `y_`, `z_` (trailing underscore) avoid clashing with member names
// like `Pose2::x` in user code that does `using namespace pathfinder;`.
inline constexpr detail::ParamFactory<detail::XTag>        x_;
inline constexpr detail::ParamFactory<detail::YTag>        y_;
inline constexpr detail::ParamFactory<detail::ZTag>        z_;

template <typename... Params>
Vector2 resolve_point(Params... params) {
    constexpr int n_forward  = detail::count_of<detail::ForwardTag, Params...>();
    constexpr int n_backward = detail::count_of<detail::BackwardTag, Params...>();
    constexpr int n_right    = detail::count_of<detail::RightTag, Params...>();
    constexpr int n_left     = detail::count_of<detail::LeftTag, Params...>();
    constexpr int n_x        = detail::count_of<detail::XTag, Params...>();
    constexpr int n_y        = detail::count_of<detail::YTag, Params...>();

    static_assert(n_forward + n_backward <= 1,
        "resolve_point: cannot specify both forward and backward");
    static_assert(n_right + n_left <= 1,
        "resolve_point: cannot specify both left and right");
    static_assert(n_forward + n_backward + n_x <= 1,
        "resolve_point: cannot mix forward/backward with x_");
    static_assert(n_right + n_left + n_y <= 1,
        "resolve_point: cannot mix left/right with y_");

    constexpr int n_x_axis = n_forward + n_backward + n_x;
    constexpr int n_y_axis = n_right + n_left + n_y;

    if constexpr (n_x_axis == 0 || n_y_axis == 0) {
        throw std::invalid_argument(
            "resolve_point: must supply both an X-axis (forward/backward/x_) "
            "and a Y-axis (left/right/y_) parameter");
    }

    double x = 0.0;
    double y = 0.0;

    if constexpr (n_forward > 0)  x =  detail::pick_value<detail::ForwardTag>(params...);
    if constexpr (n_backward > 0) x = -detail::pick_value<detail::BackwardTag>(params...);
    if constexpr (n_x > 0)        x =  detail::pick_value<detail::XTag>(params...);

    if constexpr (n_right > 0) y =  detail::pick_value<detail::RightTag>(params...);
    if constexpr (n_left > 0)  y = -detail::pick_value<detail::LeftTag>(params...);
    if constexpr (n_y > 0)     y =  detail::pick_value<detail::YTag>(params...);

    return {x, y};
}

} // namespace pathfinder
