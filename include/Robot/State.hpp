#pragma once

#include <array>
#include <type_traits>

#include <boost/json.hpp>
#include <boost/describe.hpp>
#include <boost/mp11.hpp>


// Point structure with lat/lng and x/y coordinates
struct Point {
    std::array<double, 2> latlng;
    std::array<double, 2> xy;
};


// Main pose data structure
struct State {
    std::array<double, 3> R;
    std::array<std::array<double, 3>, 3> R_cov;
    std::array<double, 3> T;
    std::array<std::array<double, 3>, 3> T_cov;
    Point point;
};


// Describe structs
BOOST_DESCRIBE_STRUCT(Point, (), (latlng, xy))
BOOST_DESCRIBE_STRUCT(State, (), (R, R_cov, T, T_cov, point))


namespace boost {
namespace json {


template<class T>
requires boost::describe::has_describe_members<T>::value
T tag_invoke(
    value_to_tag<T>,
    value const& jv)
{
    using namespace boost::describe;

    T result{};

    auto const& obj = jv.as_object();

    boost::mp11::mp_for_each<
        describe_members<T, mod_public>
    >([&](auto D)
    {
        auto it = obj.find(D.name);

        if (it != obj.end())
        {
            result.*D.pointer =
                value_to<
                    std::remove_cvref_t<
                        decltype(result.*D.pointer)
                    >
                >(it->value());
        }
    });

    return result;
}


} // namespace json
} // namespace boost