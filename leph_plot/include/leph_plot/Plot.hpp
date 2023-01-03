#ifndef LEPH_PLOT_PLOT_HPP
#define LEPH_PLOT_PLOT_HPP

#include <vector>
#include <deque>
#include <string>
#include <map>
#include <set>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <mutex>
#include <cmath>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>

namespace leph {

/**
 * Plot
 *
 * Programming interface to
 * the Linux plotting utility 
 * Gnuplot for 2d and 3d plots.
 * Thread safe using mutex.
 */
class Plot
{
    public:

        /**
         * Plot style enumeration
         */
        enum Style {
            Points,
            Lines,
            LinesPoints,
            ErrorsPoints,
            ErrorsLines,
            Vectors,
            None,
        };

        /**
         * Typedef for labeled data set
         */
        typedef std::map<std::string, double> DataPoint;

        /**
         * Initialization
         */
        Plot() :
            _database(),
            _nextDataPointToAdd(),
            _plots2D(),
            _plots3D(),
            _multiplotLayoutRow(1),
            _multiplotLayoutCol(1),
            _multiplotIndexes(),
            _rangeMinX(1.0),
            _rangeMaxX(0.0),
            _rangeMinY(1.0),
            _rangeMaxY(0.0),
            _rangeMinZ(1.0),
            _rangeMaxZ(0.0),
            _rangeUniform(false),
            _title(""),
            _winWidth(1600),
            _winHeight(900),
            _pipeFd(-1),
            _mutex()
        {
        }

        /**
         * Add a labeled set of values into the
         * internal data container.
         *
         * @param point Data to be copied 
         * into points database.
         */
        void add(const DataPoint& point)
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            _database.push_back(point);
        }
        
        /**
         * Add a labeled set of values into the
         * internal data container.
         * Use variadic template function for nice API:
         * add(label1, val1, label2, val2, ...)
         *
         * @param label Data name.
         * @param label Data value.
         */
        void add(
            const std::string& label, 
            double value)
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            //Termination case
            //Append value to temporary
            _nextDataPointToAdd[label] = value;
            //Append the set to the database
            _database.push_back(_nextDataPointToAdd);
            //Clear the temporary set
            _nextDataPointToAdd.clear();
        }
        template <typename ... LabeledValues>
        void add(
            const std::string& label, 
            double value, 
            LabeledValues... labeledValues)
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            //Append value to temporary
            _nextDataPointToAdd[label] = value;
            //Forward recursive call
            add(labeledValues...);
        }

        /**
         * Remove all the previously added data point (only) 
         * at the beginning of the internal database
         * where the given label exists and has a lower value
         * than the provided threshold.
         *
         * @param label The name on which 
         * filtering the data.
         * @param value The threshold compared
         * against data points.
         */
        void filterLowerThan(const std::string& label, double value)
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            while (
                _database.size() > 0 &&
                _database.front().count(label) > 0 && 
                _database.front().at(label) < value
            ) {
                _database.pop_front();
            }
        }

        /**
         * Merge the given plot data into
         * this. 
         *
         * @param plot Another leph::Plot 
         * whose data are merged into this.
         */
        void merge(const Plot& plot)
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            for (size_t i=0;i<plot._database.size();i++) {
                _database.push_back(plot._database[i]);
            }
        }

        /**
         * Request a 2D plot with X and Y axis.
         *
         * @param xAxis Name of label data 
         * to be plot on X axis. Could be "index"
         * to use the data insertion count.
         * @param yAxis Name of label data 
         * to be plot on Y axis. Could be "all"
         * to use all available data other than 
         * used on X axis.
         * Could be a prefix with ending wildcard "prefix*".
         * @param style Optional Gnuplot style.
         * @param palette If not empty, show the given
         * label data name using color gradient 
         * as a third dimension.
         *
         * @return *this for chainable requests.
         */
        Plot& plot(
            const std::string& xAxis, 
            const std::string& yAxis, 
            const Style style = LinesPoints,
            const std::string& palette = "")
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            for (size_t i=0;i<_plots2D.size();i++) {
                if (
                    _plots2D[i].xAxis == xAxis && 
                    _plots2D[i].yAxis == yAxis
                ) {
                    _plots2D[i].xAxis = xAxis;
                    _plots2D[i].yAxis = yAxis;
                    _plots2D[i].style = style;
                    _plots2D[i].palette = palette;
                    return *this;
                }
            }

            if (yAxis == "all") {
                for (size_t i=0;i<_database.size();i++) {
                    for (const auto& it : _database[i]) {
                        if (it.first != xAxis) {
                            plot(xAxis, it.first, style, palette);
                        }
                    }
                }
                return *this;
            } else if (yAxis.length() >= 1 && yAxis.back() == '*') {
                std::string prefix = yAxis.substr(0, yAxis.length()-1);
                for (size_t i=0;i<_database.size();i++) {
                    for (const auto& it : _database[i]) {
                        if (it.first != xAxis && it.first.find(prefix) == 0) {
                            plot(xAxis, it.first, style, palette);
                        }
                    }
                }
                return *this;
            } else {
                if (
                    (style == ErrorsLines || 
                    style == ErrorsPoints) &&
                    palette == ""
                ) {
                    throw std::logic_error(
                        "leph::Plot::plot: Style errors needs a third column.");
                }
                if (style == Vectors) {
                    throw std::logic_error(
                        "leph::Plot::plot: Style vector needs 4 columns.");
                }
                Plot2D request;
                request.xAxis = xAxis;
                request.yAxis = yAxis;
                request.style = style;
                request.palette = palette;
                _plots2D.push_back(request);
                return *this;
            }
        }
        
        /**
         * Request a 3D plot with X, Y and Z axis.
         *
         * @param xAxis Name of label data 
         * to be plot on X axis. Could be "index"
         * to use the data insertion count.
         * @param yAxis Name of label data 
         * to be plot on Y axis. Could be "index"
         * to use the data insertion count.
         * @param zAxis Name of label data 
         * to be plot on Z axis. Could be "all"
         * to use all available data other than 
         * used on X and Y axis.
         * Could be a prefix with ending wildcard "prefix*".
         * Could also be "ZERO" to use zero value.
         * @param style Optional Gnuplot style.
         * @param palette If not empty, show the given
         * label data name using color gradient 
         * as a fourth dimension.
         *
         * @return *this for chainable requests.
         */
        Plot& plot(
            const std::string& xAxis, 
            const std::string& yAxis, 
            const std::string& zAxis,
            const Style style = Points,
            const std::string& palette = "")
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            for (size_t i=0;i<_plots3D.size();i++) {
                if (_plots3D[i].xAxis == xAxis && 
                    _plots3D[i].yAxis == yAxis &&
                    _plots3D[i].zAxis == zAxis
                ) {
                    _plots3D[i].xAxis = xAxis;
                    _plots3D[i].yAxis = yAxis;
                    _plots3D[i].zAxis = zAxis;
                    _plots3D[i].style = style;
                    _plots3D[i].palette = palette;
                    return *this;
                }
            }

            if (zAxis == "all") {
                for (size_t i=0;i<_database.size();i++) {
                    for (const auto& it : _database[i]) {
                        if (
                            it.first != xAxis && 
                            it.first != yAxis
                        ) {
                            plot(xAxis, yAxis, it.first, style, palette);
                        }
                    }
                }
                return *this;
            } else if (zAxis.length() >= 1 && zAxis.back() == '*') {
                std::string prefix = zAxis.substr(0, zAxis.length()-1);
                for (size_t i=0;i<_database.size();i++) {
                    for (const auto& it : _database[i]) {
                        if (
                            it.first != xAxis && 
                            it.first != yAxis && 
                            it.first.find(prefix) == 0
                        ) {
                            plot(xAxis, yAxis, it.first, style, palette);
                        }
                    }
                }
                return *this;
            } else {
                if (
                    style == ErrorsLines || 
                    style == ErrorsPoints
                ) {
                    throw std::logic_error(
                        "leph::Plot::plot: Style errors not implemented for 3d.");
                }
                if (
                    style == Vectors && 
                    palette == ""
                ) {
                    throw std::logic_error(
                        "leph::Plot::plot: Style vectors needs 4 columns.");
                }
                Plot3D request;
                request.xAxis = xAxis;
                request.yAxis = yAxis;
                request.zAxis = zAxis;
                request.style = style;
                request.palette = palette;
                _plots3D.push_back(request);
                return *this;
            }
        }

        /**
         * Set X,Y,Z axis plotting range.
         * 
         * @param min Lower bound value.
         * @param max Upper bound value.
         * If max > min, auto scaling is used.
         *
         * @return *this for chainable requests.
         */
        Plot& rangeX(double min, double max)
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            _rangeMinX = min;
            _rangeMaxX = max;
            _rangeUniform = false;

            return *this;
        }
        Plot& rangeY(double min, double max)
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            _rangeMinY = min;
            _rangeMaxY = max;
            _rangeUniform = false;

            return *this;
        }
        Plot& rangeZ(double min, double max)
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            _rangeMinZ = min;
            _rangeMaxZ = max;
            _rangeUniform = false;

            return *this;
        }

        /**
         * Enable uniform scaling and reset scaling.
         * @param value If true, uniform scaling is
         * enable on all axis.
         * 
         * @return *this for chainable requests.
         */
        Plot& rangeUniform(bool value = true)
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            _rangeMinX = 1.0;
            _rangeMaxX = 0.0;
            _rangeMinY = 1.0;
            _rangeMaxY = 0.0;
            _rangeMinZ = 1.0;
            _rangeMaxZ = 0.0;
            _rangeUniform = value;

            return *this;
        }

        /**
         * Set the optional string title 
         * for next plot
         */
        Plot& title(const std::string& title)
        {
            _title = title;

            return *this;
        }
        
        /**
         * Set the optional Gnuplot 
         * terminal window size 
         */
        Plot& winSize(unsigned int width, unsigned int height)
        {
            _winWidth = width;
            _winHeight = height;

            return *this;
        }

        /**
         * Set and enable multiplot layout.
         * @param row Number of subplot in row.
         * @param col Number of subplot in column.
         *
         * @return *this for chainable requests.
         */
        Plot& multiplot(unsigned int row, unsigned int col)
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            if (_plots3D.size() != 0) {
                throw std::logic_error(
                    "leph::Plot::multiplot: 3d plots invalid with multiplot.");
            }
            _multiplotLayoutRow = row;
            _multiplotLayoutCol = col;

            return *this;
        }

        /**
         * Following plot calls will be plot 
         * in next multiplot layout.
         *
         * @return *this for chainable requests.
         */
        Plot& nextPlot()
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            if (_plots3D.size() != 0) {
                throw std::logic_error(
                    "leph::Plot::nextPlot: 3d plots invalid with multiplot.");
            }
            if (_plots2D.size() > 0) {
                _multiplotIndexes.push_back(_plots2D.size());
            }

            size_t countAvailabled = 
                _multiplotLayoutRow * _multiplotLayoutCol;
            size_t countNeeded = _multiplotIndexes.size() + 1;
            if (countAvailabled < countNeeded) {
                if (_multiplotLayoutRow == _multiplotLayoutCol) {
                    _multiplotLayoutCol++;
                } else {
                    _multiplotLayoutRow++;
                }
            }

            return *this;
        }

        /**
         * Render and display the requested plots.
         * Do nothing in case of empty plot.
         * Plots are then cleared.
         *
         * @param waitExit If true, the process
         * waits until that the Gnuplot window is closed.
         * @param exportFile If not empty, the generated
         * Gnuplot script is written on disk to given path
         * instead of being displayed.
         * Else if "AUTO" is set, the plot is displayed 
         * and the Gnuplot script is still dumped in /tmp folder.
         */
        void show(
            bool waitExit = true, 
            const std::string& exportFile = "AUTO")
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            if (
                _plots2D.size() > 0 && 
                _plots3D.size() > 0 && 
                _plots3D.front().style != Vectors
            ) {
                throw std::logic_error(
                    "leph::Plot::show: 2d and 3d requests are mixed.");
            }
            if (_plots2D.size() == 0 && _plots3D.size() == 0) {
                return;
            }

            std::string commands = generatePlotting(false);
            
            //Dump Gnuplot script to file
            if (exportFile != "") {
                std::ofstream file;
                if (exportFile == "AUTO") {
                    file.open("/tmp/plot-" + currentDate() + ".plot");
                } else {
                    file.open(exportFile);
                }
                file << commands;
                file.close();
            }

            //Send commands to Gnuplot instance
            if (exportFile == "" || exportFile == "AUTO") {
                if (_pipeFd <= 0) {
                    createGnuplotInstance();
                }
                if (_pipeFd <= 0) {
                    throw std::logic_error(
                        "leph:Plot::show: Closed pipe.");
                }
                ssize_t sizeWriten = write(_pipeFd, commands.c_str(), commands.length());
                if (sizeWriten != (ssize_t)commands.length()) {
                    throw std::logic_error(
                        "leph:Plot::show: Write error.");
                }
                if (waitExit) {
                    waitCloseGnuplotInstance();
                }
            }
            
            //Reset plot requests
            clearPlots();
        }
        
        /**
         * Render and display the requested plots.
         * Do nothing in case of empty plot.
         * Plots are then cleared.
         * The Gnuplot window remains open
         * to allow interactive plotting.
         */
        void stream()
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            if (
                _plots2D.size() > 0 && 
                _plots3D.size() > 0 && 
                _plots3D.front().style != Vectors
            ) {
                throw std::logic_error(
                    "leph::Plot::stream: 2d and 3d requests are mixed.");
            }
            if (_plots2D.size() == 0 && _plots3D.size() == 0) {
                return;
            }

            std::string commands = generatePlotting(true);
            
            //Send commands to Gnuplot instance
            if (_pipeFd <= 0) {
                createGnuplotInstance();
            }
            if (_pipeFd <= 0) {
                throw std::logic_error(
                    "leph:Plot::stream: Closed pipe.");
            }
            ssize_t sizeWriten = write(_pipeFd, commands.c_str(), commands.length());
            if (sizeWriten != (ssize_t)commands.length()) {
                throw std::logic_error(
                    "leph:Plot::stream: Write error.");
            }
            
            //Reset plot requests
            clearPlots();
        }

        /**
         * Write the internal datapoint as a CSV file.
         *
         * @param exportFile If not empty, the generated
         * CSV is written on disk to given path
         * instead of being displayed.
         * Else if "AUTO" is set, the file is dumped in /tmp folder.
         */
        void writeData(const std::string& exportFile = "AUTO")
        {
            std::ostringstream oss;
            std::set<std::string> lastLabels;
            for (size_t i=0;i<_database.size();i++) {
                std::set<std::string> currentLabels;
                for (const auto& it : _database[i]) {
                    currentLabels.insert(it.first);
                }
                if (lastLabels != currentLabels) {
                    oss << "# ";
                    for (const auto& it : _database[i]) {
                        oss << "'" << it.first << "' ";
                    }
                    oss << std::endl;
                    lastLabels = currentLabels;
                } 
                for (const auto& it : _database[i]) {
                    oss << std::setprecision(15) << it.second << " ";
                }
                oss << std::endl;
            }
            //Dump Gnuplot script to file
            if (exportFile != "") {
                std::ofstream file;
                if (exportFile == "AUTO") {
                    file.open("/tmp/data-" + currentDate() + ".csv");
                } else {
                    file.open(exportFile);
                }
                file << oss.str();
                file.close();
            }
        }

        /**
         * Force Gnuplot window to close
         * and terminate the process.
         * Useful with stream() display.
         */
        void closeWindow()
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            if (_pipeFd <= 0) {
                return;
            }
            std::string commands = "quit; quit;";
            ssize_t sizeWriten = write(_pipeFd, commands.c_str(), commands.length());
            if (sizeWriten != (ssize_t)commands.length()) {
                throw std::logic_error(
                    "leph:Plot::closeWindow: Write error.");
            }
            if (_pipeFd != -1) {
                close(_pipeFd);
                _pipeFd = -1;
            }
        }

        /**
         * Reset either points database, plot
         * requests or both
         */
        void clearPoints()
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            _database.clear();
        }
        void clearPlots()
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            _plots2D.clear();
            _plots3D.clear();
            _multiplotLayoutRow = 1;
            _multiplotLayoutCol = 1;
            _multiplotIndexes.clear();
            _rangeMinX = 1.0;
            _rangeMaxX = 0.0;
            _rangeMinY = 1.0;
            _rangeMaxY = 0.0;
            _rangeMinZ = 1.0;
            _rangeMaxZ = 0.0;
            _rangeUniform = false;
            _title = "";
        }
        void clear()
        {
            std::lock_guard<std::recursive_mutex> lock(_mutex);
            clearPoints();
            clearPlots();
        }

    private:

        /**
         * Plot request 2D and 3D
         * structures
         */
        struct Plot2D {
            std::string xAxis;
            std::string yAxis;
            Style style;
            std::string palette;
        };
        struct Plot3D {
            std::string xAxis;
            std::string yAxis;
            std::string zAxis;
            Style style;
            std::string palette;
        };

        /**
         * Container for labeled data point set
         */
        std::deque<DataPoint> _database;

        /**
         * Temporary labeled values set used in
         * recursive (variadic) add() function
         * to be the next DataPoint to append.
         */
        DataPoint _nextDataPointToAdd;

        /**
         * Plot request container
         */
        std::vector<Plot2D> _plots2D;
        std::vector<Plot3D> _plots3D;

        /**
         * Multiplot row and col layout 
         * and _multiplotIndexes hold the split 
         * index in plot2D
         */
        unsigned int _multiplotLayoutRow;
        unsigned int _multiplotLayoutCol;
        std::vector<size_t> _multiplotIndexes;

        /**
         * Plot x,y,z min,max range
         * If max < min, auto scaling is used
         */
        double _rangeMinX;
        double _rangeMaxX;
        double _rangeMinY;
        double _rangeMaxY;
        double _rangeMinZ;
        double _rangeMaxZ;

        /**
         * If true, uniform scaling is enabled
         */
        bool _rangeUniform;

        /**
         * Plot optional title
         */
        std::string _title;

        /**
         * Gnuplot terminal window size
         */
        unsigned int _winWidth;
        unsigned int _winHeight;

        /**
         * Pipe file descriptor
         * to Gnuplot process
         */
        int _pipeFd;

        /**
         * Thread concurent protection
         */
        std::recursive_mutex _mutex;
        
        /**
         * Fork current process to 
         * create a new GnuPlot window
         */
        void createGnuplotInstance()
        {
            //Creating communication pipe
            int pipefd[2];
            if (pipe(pipefd) == -1) {
                throw std::runtime_error(
                    "leph::Plot::createGnuplotInstance: Failed to create pipe.");
            }
            
            //Forking current process
            pid_t pid = fork();
            if (pid > 0) {
                //Closing reading pipe end
                close(pipefd[0]);
                //Saving pipe fd
                _pipeFd = pipefd[1];
            } else if (pid == 0) {
                //Closing writing pipe end
                close(pipefd[1]);
                //Redirecting reading pipe end to standard input
                if (dup2(pipefd[0], STDIN_FILENO) == -1) {
                    throw std::runtime_error(
                        "leph::Plot::createGnuplotInstance: Failed to dup2.");
                }
                //Closing output and err
                int null = open("/dev/null", O_WRONLY);
                if (dup2(null, STDOUT_FILENO) == -1) {
                    throw std::runtime_error(
                        "leph::Plot::createGnuplotInstance: Failed to dup2.");
                }
                if (dup2(null, STDERR_FILENO) == -1) {
                    throw std::runtime_error(
                        "leph::Plot::createGnuplotInstance: Failed to dup2.");
                }
                //Calling Gnuplot
                execlp("gnuplot", "gnuplot", "-", NULL);
            } else {
                throw std::runtime_error(
                    "leph::Plot::createGnuplotInstance: Failed to fork.");
            }
        }

        /**
         * Wait for end of Gnuplot session and
         * close the opened pipe to GnuPlot window instance
         */
        void waitCloseGnuplotInstance()
        {
            waitpid(-1, NULL, 0);
            if (_pipeFd != -1) {
                close(_pipeFd);
                _pipeFd = -1;
            }
        }

        /**
         * Generate and return Gnuplot commands and data
         *
         * @param isStream If true, the window is not closed
         * at the end of command. Use for interactive plotting.
         */
        std::string generatePlotting(bool isStream)
        {
            std::string commandsSum;
            if (isStream) {
                commandsSum += "set terminal qt noraise size ";
            } else {
                commandsSum += "set terminal qt size "; 
            }
            commandsSum += 
                std::to_string(_winWidth) + "," + 
                std::to_string(_winHeight) + ";\n"; 
            commandsSum += generateStyle();

            bool isMultiplot = false;
            if (
                _multiplotLayoutRow != 1 || 
                _multiplotLayoutCol != 1
            ) {
                isMultiplot = true;
            }

            if (isMultiplot) {
                commandsSum += "set multiplot layout ";
                commandsSum += std::to_string(_multiplotLayoutRow);
                commandsSum += ", ";
                commandsSum += std::to_string(_multiplotLayoutCol);
                commandsSum += ";\n";
            }
            int indexSplitMultiplot = -1;
            while (true) {
                std::string commands;
                std::string data;
                if (indexSplitMultiplot >= (int)_multiplotIndexes.size()) {
                    break;
                }
                size_t startIndex2D = 0;
                size_t endIndex2D = _plots2D.size()-1;
                if (isMultiplot) {
                    if (indexSplitMultiplot >= 0) {
                        startIndex2D = _multiplotIndexes[indexSplitMultiplot];
                    } else {
                        startIndex2D = 0;
                    }
                    if (indexSplitMultiplot < (int)_multiplotIndexes.size()-1) {
                        endIndex2D = _multiplotIndexes[indexSplitMultiplot+1]-1;
                    } else {
                        endIndex2D = _plots2D.size() - 1;
                    }
                    indexSplitMultiplot++;
                }
                commands += "set grid;\n";
                commands += "set xlabel 'X';\n";
                commands += "set ylabel 'Y';\n";
                commands += "set zlabel 'Z';\n";
                if (_rangeUniform) {
                    commands += "set size ratio -1;\n";
                }
                if (_rangeMinX < _rangeMaxX) {
                    std::ostringstream oss;
                    oss << "set xrange[" << std::setprecision(15) << _rangeMinX;
                    oss << ":" << std::setprecision(15) << _rangeMaxX << "];\n";
                    commands += oss.str();
                }
                if (_rangeMinY < _rangeMaxY) {
                    std::ostringstream oss;
                    oss << "set yrange[" << std::setprecision(15) << _rangeMinY;
                    oss << ":" << std::setprecision(15) << _rangeMaxY << "];\n";
                    commands += oss.str();
                }
                if (_rangeMinZ < _rangeMaxZ) {
                    std::ostringstream oss;
                    oss << "set zrange[" << std::setprecision(15) << _rangeMinZ;
                    oss << ":" << std::setprecision(15) << _rangeMaxZ << "];\n";
                    commands += oss.str();
                }
                if (_title.length() > 0) {
                    commands += "set title '" + _title + "' noenhanced;\n";
                }

                //Create commands and data to send to gnuplot
                if (
                    _plots2D.size() != 0 || 
                    (_plots3D.size() > 0 && _plots3D.front().style == Vectors)
                ) {
                    commands += "plot ";
                } else {
                    commands += "splot ";
                }
                
                bool isFirst = true;
                for (
                    size_t i=startIndex2D;
                    _plots2D.size() > 0 && i<=endIndex2D;
                    i++
                ) {
                    if (_plots2D[i].style == None) {
                        continue;
                    }
                    bool isPalette = _plots2D[i].palette != "";
                    if (!isFirst) {
                        commands += ", ";
                    }
                    isFirst = false;
                    if (isPalette) {
                        if (
                            _plots2D[i].style == ErrorsLines || 
                            _plots2D[i].style == ErrorsPoints
                        ) {
                            commands += "'-' using 1:2:3 with ";
                        } else {
                            commands += "'-' using 1:2:3 palette with ";
                        }
                    } else {
                        commands += "'-' using 1:2 with ";
                    }
                    if (_plots2D[i].style == Points) {
                        commands += "points";
                    }
                    if (_plots2D[i].style == Lines) {
                        commands += "lines";
                    }
                    if (_plots2D[i].style == LinesPoints) {
                        commands += "linespoints";
                    }
                    if (_plots2D[i].style == ErrorsPoints) {
                        commands += "yerrorpoints";
                    }
                    if (_plots2D[i].style == ErrorsLines) {
                        commands += "yerrorlines";
                    }
                    if (isPalette) {
                        commands += " title '" + _plots2D[i].xAxis 
                            + " --> " + _plots2D[i].yAxis 
                            + " // " + _plots2D[i].palette + "' noenhanced ";
                    } else {
                        commands += " title '" + _plots2D[i].xAxis 
                            + " --> " + _plots2D[i].yAxis + "' noenhanced ";
                    }
                    for (size_t j=0;j<_database.size();j++) {
                        if (
                            (_plots2D[i].xAxis != "index" &&
                            _database[j].count(_plots2D[i].xAxis) == 0) || 
                            _database[j].count(_plots2D[i].yAxis) == 0 ||
                            (isPalette && 
                            _plots2D[i].palette != "index" &&
                            _database[j].count(_plots2D[i].palette) == 0)
                        ) {
                            continue;
                        }
                        std::ostringstream oss;
                        if (_plots2D[i].xAxis == "index") {
                            oss << j << " " << std::setprecision(15) 
                                << _database[j].at(_plots2D[i].yAxis);
                        } else {
                            oss << std::setprecision(15) 
                                << _database[j].at(_plots2D[i].xAxis) << " " 
                                << std::setprecision(15) 
                                << _database[j].at(_plots2D[i].yAxis);
                        }
                        if (isPalette) {
                            if (_plots2D[i].palette == "index") {
                                oss << " " << j;
                            } else {
                                oss << " " 
                                    << std::setprecision(15) 
                                    << _database[j].at(_plots2D[i].palette);
                            }
                        }
                        data += oss.str() + "\n";
                    }
                    data += "end\n";
                }
                for (size_t i=0;i<_plots3D.size();i++) {
                    if (_plots3D[i].style == None) {
                        continue;
                    }
                    bool isPalette = _plots3D[i].palette != "";
                    if (!isFirst) {
                        commands += ", ";
                    }
                    isFirst = false;
                    if (_plots3D[i].style == Vectors) {
                        commands += "'-' using 1:2:($4*cos($3)):($4*sin($3)) with ";
                    } else if (isPalette) {
                        commands += "'-' using 1:2:3:4 palette with ";
                    } else {
                        commands += "'-' using 1:2:3 with ";
                    }
                    if (_plots3D[i].style == Points) {
                        commands += "points";
                    }
                    if (_plots3D[i].style == Lines) {
                        commands += "lines";
                    }
                    if (_plots3D[i].style == LinesPoints) {
                        commands += "linespoints";
                    }
                    if (_plots3D[i].style == Vectors) {
                        commands += "vectors head filled";
                    }
                    if (isPalette) {
                        commands += " title '" + _plots3D[i].xAxis 
                            + "," + _plots3D[i].yAxis 
                            + " --> " + _plots3D[i].zAxis 
                            + " // " + _plots3D[i].palette + "' noenhanced ";
                    } else {
                        commands += " title '" + _plots3D[i].xAxis 
                            + "," + _plots3D[i].yAxis 
                            + " --> " + _plots3D[i].zAxis + "' noenhanced ";
                    }
                    for (size_t j=0;j<_database.size();j++) {
                        if (
                            (_plots3D[i].xAxis != "index" &&
                            _database[j].count(_plots3D[i].xAxis) == 0) || 
                            (_plots3D[i].yAxis != "index" &&
                            _database[j].count(_plots3D[i].yAxis) == 0) || 
                            (_plots3D[i].zAxis != "ZERO" &&
                            _database[j].count(_plots3D[i].zAxis) == 0) ||
                            (isPalette && 
                            _plots3D[i].palette != "index" &&
                            _database[j].count(_plots3D[i].palette) == 0)
                        ) {
                            continue;
                        }
                        std::ostringstream oss;
                        if (_plots3D[i].xAxis == "index") {
                            oss << j << " ";
                        } else {
                            oss << std::setprecision(15) 
                                << _database[j].at(_plots3D[i].xAxis) << " ";
                        }
                        if (_plots3D[i].yAxis == "index") {
                            oss << j << " ";
                        } else {
                            oss << std::setprecision(15) 
                                << _database[j].at(_plots3D[i].yAxis) << " ";
                        }
                        if (_plots3D[i].zAxis == "ZERO") {
                            oss << "0.0";
                        } else {
                            oss << std::setprecision(15) 
                                << _database[j].at(_plots3D[i].zAxis);
                        }
                        if (isPalette) {
                            if (_plots3D[i].palette == "index") {
                                oss << " " << j;
                            } else {
                                oss << " " << std::setprecision(15) 
                                    << _database[j].at(_plots3D[i].palette);
                            }
                        }
                        data += oss.str() + "\n";
                    }
                    data += "end\n";
                }
                commands += ";\n";
                commandsSum += commands + data;
                if (!isMultiplot) {
                    break;
                }
            }
            if (isMultiplot) {
                commandsSum += "unset multiplot;\n";
            }
            if (!isStream) {
                commandsSum += "pause mouse close;\nquit;\nquit;\n";
            }

            return commandsSum;
        }

        /**
         * @return a color-blind palette
         * for Gnuplot style
         */
        std::string generateStyle() const
        {
            std::string cmd =
            "set linetype 1 lc rgb '#000000' lw 1;\n"
            "set linetype 2 lc rgb '#E69F00' lw 1;\n"
            "set linetype 3 lc rgb '#56B4E9' lw 1;\n"
            "set linetype 4 lc rgb '#009E73' lw 1;\n"
            "set linetype 5 lc rgb '#CC79A7' lw 1;\n"
            "set linetype 6 lc rgb '#0072B2' lw 1;\n"
            "set linetype 7 lc rgb '#D55E00' lw 1;\n"
            "set linetype 8 lc rgb '#F0E442' lw 1;\n"
            "set linetype cycle  8;\n"
            "set palette rgbformulae 22,13,-31;\n"
            "set xlabel font ',16';\n"
            "set ylabel font ',16';\n"
            "set zlabel font ',16';\n"
            "set xtics font ',10';\n"
            "set ytics font ',10';\n"
            "set ztics font ',10';\n"
            "set title font ',16';\n"
            "set key font ',16';\n";

            return cmd;
        }
        
        /**
         * @return the current date as a string
         */
        static std::string currentDate()
        {
            std::ostringstream oss;
            time_t t = time(0);
            struct tm* now = localtime(&t);
            oss << now->tm_year + 1900 << "-";
            oss << std::setfill('0') << std::setw(2);
            oss << now->tm_mon + 1 << "-";
            oss << std::setfill('0') << std::setw(2);
            oss << now->tm_mday << "-";
            oss << std::setfill('0') << std::setw(2);
            oss << now->tm_hour << "-";
            oss << std::setfill('0') << std::setw(2);
            oss << now->tm_min << "-";
            oss << std::setfill('0') << std::setw(2);
            oss << now->tm_sec;

            return oss.str();
        }
};

}

#endif

