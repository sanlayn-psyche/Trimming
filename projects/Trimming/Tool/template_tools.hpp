#pragma once
#include <map>
#include <stdexcept>
#include <type_traits>
#include <memory>
#include <tuple>

#define ENUM_END_LABEL __UNDEF
#define CREATE_ENUM(NAME, ...) \
    enum class NAME { __VA_ARGS__, ENUM_END_LABEL};

template<typename T>
concept EnumClass = std::is_enum_v<T> && !std::is_constructible_v<T, std::underlying_type_t<T>>;


template <typename E>
constexpr auto to_underlying(E e) noexcept
{
    return static_cast<std::underlying_type_t<E>>(e);
}

template<typename E>
constexpr std::underlying_type_t<E> __get_enum_size()
{
    return to_underlying<E>(E::ENUM_END_LABEL);
}


template <typename __tool_interface, typename __tool_type, template<__tool_type> class __tool_class>
class __make_tool_map
{
    static_assert(std::is_base_of_v<__tool_interface, __tool_class<__tool_type::ENUM_END_LABEL>>, "__tool_class must inherit from __tool_interface.");
private:
    std::map<__tool_type, std::unique_ptr<__tool_interface>> m_instances;
 
    template<__tool_type T>
    void act_creat()
    {
        m_instances[T] = std::make_unique<__tool_class<T>>();
        act_creat<static_cast<__tool_type>(to_underlying(T) + 1)>();
    };

    template<>
    void act_creat<__tool_type::ENUM_END_LABEL>(){};

    __tool_type m_type{ static_cast<__tool_type>(0) };
public:
    __make_tool_map()
    {
        act_creat<static_cast<__tool_type>(0)>();
    }
    ~__make_tool_map() = default;

    void set_type(__tool_type t) { m_type = t; }

    __tool_interface* get(__tool_type type) const
    {
        auto it = m_instances.find(type);
        if (it != m_instances.end()) {
            return it->second.get();
        }
        throw std::runtime_error("Unknown type");
    };

    __tool_interface* get() const
    {
        return get(m_type);
    };
};


template <typename __tool_interface, typename __tool_typeA, typename __tool_typeB, template<__tool_typeA, __tool_typeB> class __tool_class>
class __make_comptool_map
{
    static_assert(std::is_base_of_v<__tool_interface, __tool_class<__tool_typeA::ENUM_END_LABEL, __tool_typeB::ENUM_END_LABEL>>, "__tool_class must inherit from __tool_interface.");
private:
    std::map<std::tuple<__tool_typeA, __tool_typeB>, std::unique_ptr<__tool_interface>> m_instances;

    template<__tool_typeA T1, __tool_typeB T2>
    void act_creat()
    {
        m_instances.insert(std::make_tuple(T1, T2), std::make_unique<__tool_class<T1, T2>>());
        //m_instances[T] = std::make_unique<__tool_class<T1, T2>>();
        auto nextA = static_cast<__tool_typeA>(to_underlying(T1) + 1);
        if (nextA != __tool_typeA::ENUM_END_LABEL) act_creat<nextA, T2>();

        auto nextB = static_cast<__tool_typeB>(to_underlying(T2) + 1);
        if (nextB != __tool_typeB::ENUM_END_LABEL) act_creat<T1, nextB>();
    };

    template<>
    void act_creat<__tool_typeA::ENUM_END_LABEL, __tool_typeB::ENUM_END_LABEL>() {};

public:
    __make_comptool_map()
    {
        act_creat<static_cast<__tool_typeA>(0), static_cast<__tool_typeB>(0)>();
    }
    ~__make_comptool_map() = default;

 
    __tool_interface* get(__tool_typeA typea, __tool_typeB typeb) const
    {
        auto it = m_instances.find({ typea, typeb });
        if (it != m_instances.end()) {
            return it->second.get();
        }
        throw std::runtime_error("Unknown type");
    };
};


