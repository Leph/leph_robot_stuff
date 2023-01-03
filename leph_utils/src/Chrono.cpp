#include <iomanip>
#include <deque>
#include <stdexcept>
#include <algorithm>
#include <leph_utils/Chrono.hpp>

namespace leph {
        
Chrono::Chrono() :
    _durations()
{
}
        
void Chrono::clear()
{
    _durations.clear();
}
        
void Chrono::start(const std::string& name)
{
    if (_durations.count(name) == 0) {
        _durations[name] = SequenceContainer();
        _durations.at(name).reserve(10000);
    }

    _durations[name].push_back(PairTimePoint(
        std::chrono::system_clock::now(),
        std::chrono::system_clock::now()
    ));
}
        
void Chrono::stop(const std::string& name)
{
    if (_durations.count(name) == 0) {
        throw std::logic_error(
            "leph::Chrono::stop: Invalid stop: " 
            + name);
    }

    _durations[name].back().second = 
        std::chrono::system_clock::now();
}
        
void Chrono::print(std::ostream& os) const
{
    //Starting and stopping sequences of durations
    //are analysed to check for correctness and
    //built the hierarchical tree

    //Mapping from duration names to their 
    //children in the hierarchical tree
    std::map<std::string, std::vector<std::string>> children;
    children["leph::Chrono::ROOT"] = std::vector<std::string>();
    //Stack of started duration 
    //(check for well parenthesized)
    std::deque<std::string> openedNames;

    //For each duration name, the index in
    //duration sequence of next to be checked
    //point is stored.
    std::map<std::string, size_t> indexes;
    for (const auto& it : _durations) {
        indexes[it.first] = 0;
    }
    //Current increasing time initialized to zero
    TimePoint time;
    //All start() and stop() call times
    //are iterated in growing order
    while (true) {
        //Look for next time just 
        //following current time.
        //Iterate over all times and look
        //for minimal time above current time.
        std::string minName = "";
        TimePoint minTime;
        bool isStart = true;
        for (const auto& d : _durations) {
            size_t index = indexes.at(d.first);
            if (
                index < d.second.size() && 
                d.second[index].first > time &&
                (minName == "" || minTime > d.second[index].first)
            ) {
                minName = d.first;
                minTime = d.second[index].first;
                isStart = true;
            } else if (
                index < d.second.size() && 
                d.second[index].second > time &&
                (minName == "" || minTime > d.second[index].second)
            ) {
                minName = d.first;
                minTime = d.second[index].second;
                isStart = false;
            }
        }
        //Stop the loop when all durations
        //have been iterated
        if (minName == "") {
            break;
        }
        //If the next time point is a start()
        if (isStart) {
            //Check that the found name is not
            //already opened
            for (const auto& n : openedNames) {
                if (minName == n) {
                    throw std::logic_error(
                        "leph::Chrono:print: Started twice: " 
                        + minName);
                }
            }
            openedNames.push_front(minName);
            //Initialize the hierarchy tree 
            //on the first time
            if (children.count(minName) == 0) {
                children[minName] = std::vector<std::string>();
            }
            //Build children mapping
            std::string parentName;
            if (openedNames.size() > 1) {
                parentName = openedNames[1];
            } else {
                //Append top level node to
                //a virtual single root
                parentName = "leph::Chrono::ROOT";
            }
            //Check if the found name is already 
            //stored as its parent's child
            bool isPresent = false;
            for (const auto& n : children.at(parentName)) {
                if (minName == n) {
                    isPresent = true;
                }
            }
            //Append if it is not
            if (!isPresent) {
                children.at(parentName).push_back(minName);
            }
        //If the next time point is a stop()
        } else {
            //Check for well parenthesized structure
            //using the stack
            if (openedNames.front() != minName) {
                throw std::logic_error(
                    "leph::Chrono:print: Not well parenthesized: " 
                    + minName);
            }
            openedNames.pop_front();
            //Increase iteration index for
            //the found name
            indexes.at(minName) += 1;
        }
        //Update current growing time
        time = minTime;
    }

    //Display the hierarchy recursively
    //from top to leaves using children mapping
    printHierarchy(os, "leph::Chrono::ROOT", 0, children, "");
}

double Chrono::mean(const std::string& name) const
{
    if (_durations.count(name) == 0) {
        throw std::logic_error(
            "leph::Chrono::mean: Unknown name: " 
            + name);
    }
    
    double tmpSum = sum(name);
    return tmpSum/(double)(_durations.at(name).size());
}
double Chrono::max(const std::string& name) const
{
    if (_durations.count(name) == 0) {
        throw std::logic_error(
            "leph::Chrono::max: Unknown name: " 
            + name);
    }

    double tmpMax = 0.0;
    for (size_t i=0;i<_durations.at(name).size();i++) {
        Duration d = _durations.at(name)[i].second 
            - _durations.at(name)[i].first; 
        double time = std::chrono::duration_cast<
            std::chrono::duration<double, std::ratio<1, 1000>>>
            (d).count();
        if (time > tmpMax) {
            tmpMax = time;
        }
    }

    return tmpMax;
}
double Chrono::sum(const std::string& name) const
{
    if (_durations.count(name) == 0) {
        throw std::logic_error(
            "leph::Chrono::sum: Unknown name: " 
            + name);
    }

    double tmpSum = 0.0;
    for (size_t i=0;i<_durations.at(name).size();i++) {
        Duration d = _durations.at(name)[i].second 
            - _durations.at(name)[i].first; 
        tmpSum += std::chrono::duration_cast<
            std::chrono::duration<double, std::ratio<1, 1000>>>
            (d).count();
    }
    if (tmpSum < 0.0) {
        tmpSum = 0.0;
    }

    return tmpSum;
}
        
double Chrono::last(const std::string& name) const
{
    if (
        _durations.count(name) == 0 ||
        _durations.at(name).size() == 0
    ) {
        throw std::logic_error(
            "leph::Chrono::last: Unknown name: " 
            + name);
    }

    Duration d = _durations.at(name).back().second 
            - _durations.at(name).back().first; 
    return std::chrono::duration_cast<
            std::chrono::duration<double, std::ratio<1, 1000>>>
            (d).count();
}
        
void Chrono::printDuration(std::ostream& os, 
    const std::string& name, unsigned int level,
    const std::string& father) const
{
    for (size_t i=0;i<level;i++) {
        os << "    ";
    }
    os << "[" << std::setfill('.') << std::setw(20) 
        << name << "]" 
        << std::setfill(' ');
    os << " *** last=" 
        << std::fixed << std::setw(6) << std::setprecision(3)
        << last(name) << "ms";
    os << " *** mean=" 
        << std::fixed << std::setw(6) << std::setprecision(3)
        << mean(name) << "ms";
    os << " *** count=" 
        << std::setw(6) << _durations.at(name).size();
    os << " *** sum=" 
        << std::fixed << std::setw(8) << std::setprecision(3)
        << sum(name) << "ms";
    os << " *** max=" 
        << std::fixed << std::setw(6) << std::setprecision(3)
        << max(name) << "ms";
    if (father != "") {
        os << " *** ratio=" 
            << std::fixed << std::setw(6) << std::setprecision(2)
            << 100.0*sum(name)/sum(father) << "%";
    }
    os << std::endl;
}
        
void Chrono::printHierarchy(std::ostream& os, 
    const std::string& name, unsigned int level,
    const std::map<std::string, std::vector<std::string>>& children,
    const std::string& father) const
{
    //Display one named duration statistics
    std::string nextParent;
    unsigned int nextLevel;
    if (name != "leph::Chrono::ROOT") {
        printDuration(os, name, level, father);
        nextParent = name;
        nextLevel = level + 1;
    } else {
        nextParent = "";
        nextLevel = 0;
    }
    //Display its children
    for (size_t i=0;i<children.at(name).size();i++) {
        printHierarchy(
            os, children.at(name)[i], 
            nextLevel, children, nextParent);
    }
}

}

