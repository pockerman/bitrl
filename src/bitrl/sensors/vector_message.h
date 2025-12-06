//
// Created by alex on 11/30/25.
//

#ifndef VECTOR_MESSAGE_H
#define VECTOR_MESSAGE_H

#include "bitrl/bitrl_types.h"

#include <iosfwd>
#include <locale>
#include <optional>
#include <string>
#include <vector>
#include <iostream>


namespace bitrl
{
    namespace sensors
    {
        template <typename T>
        struct EigenVectorMessage
        {
            DynVec<T> message;
            static std::optional<EigenVectorMessage<T>> parse(const std::string& msg);
        };

        template <typename T>
        std::optional<EigenVectorMessage<T>>
         EigenVectorMessage<T>::parse(const std::string& msg)
        {
            // --- Trim whitespace ---
            auto trim = [](const std::string& s) {
                size_t start = 0;
                while (start < s.size() && std::isspace(s[start])) start++;
                size_t end = s.size();
                while (end > start && std::isspace(s[end - 1])) end--;
                return s.substr(start, end - start);
            };

            std::string s = trim(msg);

            if (s.empty() || s.front() != '[' || s.back() != ']')
                return std::nullopt;

            // Remove '[' and ']'
            s = s.substr(1, s.size() - 2);

            std::vector<T> values;
            std::stringstream ss(s);
            std::string item;

            while (std::getline(ss, item, ',')) {
                item = trim(item);
                if (item.empty())
                    return std::nullopt;

                std::stringstream item_ss(item);
                T value;

                if (!(item_ss >> value))  // Parsing failed
                    return std::nullopt;

                values.push_back(value);
            }



            EigenVectorMessage<T> result;
            result.message = DynVec<T>::Zero(values.size());
            for (size_t i = 0; i < values.size(); ++i)
                result.message[i] = values[i];
            return result;
        }


    }
}

#endif //VECTOR_MESSAGE_H
