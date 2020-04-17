/// @file util_std.h
/// Namespaced utilities for standard containers
#pragma once

#include "momap_log.h"

#include <sstream>
#include <string>
#include <vector>

namespace utils {

    template <typename T>
    void std_vec_append(std::vector<T>& a, std::vector<T> b) {
        if (!b.empty()) {
            a.insert(std::end(a), std::begin(b), std::end(b));
        }
    }

    template<typename T>
    bool std_vec_contains( std::vector<T> vec, T item) {
        return std::find(vec.begin(), vec.end(), item) != vec.end();
    }


    /// Given an object M with map-like semantics, such as map<K, V>,
    /// return a raw pointer to the value mapped to the given _key,
    /// or, if no such value is found, then
    /// log a debug message including _map_name (unless _map_name is empty),
    /// and return NULL.
    /// NOTE: If the map-like-object is a temporary or is deleted,
    /// expect the returned pointer to become dangling, invalid, or invalidated.
    /// You may want to copy the pointed-to object before that happens.
    /// TODO @sprax: migrate to utils.  Issue SG#1171
    template <class K, class V, template <typename, typename...> class M, typename... Args>
    const V *FindValueRawPtr( const M<K, V, Args...>& _map
                            , const K& _key
                            , const std::string& _map_name = ""
                            , const std::string& _key_name = ""
                            , const std::string& _val_name = ""
    ) {
        const auto& found = _map.find(_key);
        if (found != _map.end()) {
            return &found->second;
        }
        if (not _map_name.empty()) {
            momap::log()->debug("DRU:FindValueRawPtr: value {} = {}[{}] NOT FOUND; returning NULL"
                               , _val_name
                               , _map_name
                               , _key_name.empty() ? "key" : _key_name
                               , _key
            );
        }
        return nullptr;
    }

    /// Beware of 'template argument deduction/substitution failed' from g++
    template <class K, class V, template <typename, typename...> class M, typename... Args>
    const std::vector<K> MapKeys(const M<K, V, Args...>& _map)
    {
        std::vector<K> keys;
        for (const auto& pr : _map) {
            // cerr << pr.first << endl;
            keys.emplace_back(pr.first);
        }
        return keys;
    }

    /// Given an object M with map- or set-like find semantics,
    /// such as map<K, V> or set<K>,
    /// return true IFF the given key (of type K) is found in M.
    template <class M>
    bool Contains(const M& _map, const typename M::key_type& _key)
    {
        return _map.find(_key) != _map.end();
    }

    /// Given an object M with map- or set-like find semantics,
    /// such as map<K, V> or set<K>,
    /// return the V value mapped to the given key (of type K)
    /// IFF it is found in M;
    /// otherwise return the given default_value.
    /// (Like Python dict.get(k,d))
    template <class M>
    typename M::mapped_type MappedValueOrDefault( const M& _map
                                                , const typename M::key_type& _key
                                                , const typename M::mapped_type& default_value
    ) {
        const auto& found = _map.find(_key);
        if (found != _map.end()) {
            return found->second;
        }
        momap::log()->debug("DRU:MappedValueOrDefault: key:{} NOT FOUND; return {}", 
          _key, default_value);
        return default_value;
    }

    /// Given an unordered_map M<K, V> such as a WSP container_map_t object
    /// return a raw pointer to the value mapped to the given _key,
    /// or, if no such value is found, then
    /// log a debug message including _map_name (unless _map_name is empty),
    /// and return NULL.
    /// NOTE: If the map-like-object is a temporary or is deleted,
    /// expect the returned pointer to become dangling, invalid, or invalidated.
    /// You may want to copy the pointed-to object before that happens.
    /// NOTE: prefer the more general version above?
    template <class K, class V>
    const V *FindUnorderedValueRawPtr( const std::unordered_map<K, V>& u_map
                                     , const K& _key
                                     , const std::string& _map_name = ""
                                     , const std::string& _key_name = ""
                                     , const std::string& _val_name = ""
    ) {
        const auto& found = u_map.find(_key);
        if (found != u_map.end()) {
            return& found->second;
        }
        if (not _map_name.empty()) {
            momap::log()->debug("DRU:FindValuePtr: value {} = {}[{}] NOT FOUND; returning NULL"
                               , _val_name
                               , _map_name
                               , _key_name.empty() ? "key" : _key_name
                               , _key
            );
        }
        return nullptr;
    }

    /// NOTE: prefer the more general version above,
    /// or utils post migration.
    template <class K, class V>
    const std::vector<K> MapUnorderedKeys(const std::unordered_map<K, V>& _map)
    {
        std::vector<K> keys;
        for (const auto& pr : _map) {
            // cerr << pr.first << endl;
            keys.emplace_back(pr.first);
        }
        return keys;
    }

}   //  namespace utils
