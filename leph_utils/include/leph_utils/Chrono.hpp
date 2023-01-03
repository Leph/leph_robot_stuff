#ifndef LEPH_UTILS_CHRONO_HPP
#define LEPH_UTILS_CHRONO_HPP

#include <iostream>
#include <chrono>
#include <string>
#include <map>
#include <vector>

namespace leph {

/**
 * Chrono
 *
 * C++11 tree chrono wrapper 
 * for test and benchmarking.
 * Call to start() and stop() are assumed to be
 * well parenthesized and the tree structure is 
 * also assumed to be fixed.
 */
class Chrono 
{
    public:

        /**
         * Initialization empty
         */
        Chrono();
        
        /**
         * Reset chrono instance to empty
         */
        void clear();

        /**
         * Create and start a named duration.
         * @param name Duration string name.
         */
        void start(const std::string& name);

        /**
         * Stop and save a named duration.
         * @param name Duration string name.
         */
        void stop(const std::string& name);

        /**
         * Write on given stream an overview of stored times.
         * Note: the time needed to analyse the start()/stop()
         * tree in order to properly print it might be significant.
         * (Linear in the number of stored durations).
         *
         * @param os Optional output stream. 
         * Standard output by default.
         */
        void print(std::ostream& os = std::cout) const;

        /**
         * Compute the mean, max and sum 
         * duration of given duration name.
         *
         * @param name Valid duration string name.
         * @return time duration statistics in milliseconds.
         */
        double mean(const std::string& name) const;
        double max(const std::string& name) const;
        double sum(const std::string& name) const;

        /**
         * Last measured duration.
         *
         * @param name Valid duration string name.
         * @return the last stored duration in milliseconds.
         */
        double last(const std::string& name) const;

    private:

        /**
         * Typedef shortcuts
         */
        typedef std::chrono::duration<double> Duration;
        typedef std::chrono::time_point<std::chrono::system_clock> TimePoint;
        typedef std::pair<TimePoint, TimePoint> PairTimePoint;
        typedef std::vector<PairTimePoint> SequenceContainer;
        typedef std::map<std::string, SequenceContainer> DurationContainer;

        /**
         * Sequences of starting and stopping 
         * time point indexed by their name
         */
        DurationContainer _durations;

        /**
         * Print the given duration name statistics 
         * on given stream with an indent level.
         *
         * @param name Valid duration string name.
         * @param level Left space indentation number.
         * @param father Optional (not empty) name of
         * duration parent node in hierarchy 
         * (for relative time statistics).
         */
        void printDuration(std::ostream& os, 
            const std::string& name, unsigned int level,
            const std::string& father) const;

        /**
         * Print the hierarchy with given level 
         * using given name as root.
         *
         * @param name Valid duration string name.
         * @param level Left space indentation number.
         * @param children Mapping from parent name
         * to list of their children name.
         * @param father If not empty name of 
         * duration parent node in hierarchy 
         */
        void printHierarchy(std::ostream& os, 
            const std::string& name, unsigned int level,
            const std::map<std::string, std::vector<std::string>>& children,
            const std::string& father) const;
};

}

#endif

