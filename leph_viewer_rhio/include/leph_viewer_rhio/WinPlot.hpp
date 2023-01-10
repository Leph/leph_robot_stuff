#ifndef LEPH_VIEWER_RHIO_WINPLOT_HPP
#define LEPH_VIEWER_RHIO_WINPLOT_HPP

#include <vector>
#include <string>
#include <leph_plot/Plot.hpp>
#include <leph_viewer_rhio/WinBase.hpp>
#include <leph_viewer_rhio/TreeValue.hpp>

namespace leph {

/**
 * WinPlot
 *
 * NCurses window for plotting RhIO 
 * values using GNUPlot.
 * Several plot instance are hold.
 * The last plot instance is always empty
 * to be filled.
 */
class WinPlot : public WinBase
{
    public:

        /**
         * Initialization with window
         * size and position on screen.
         *
         * @param sizeRow Window height size.
         * @param sizeCol Window width size.
         * @param posRow Window Y position on global screen.
         * @param posCol Window X position on global screen.
         * @param isInteractive If true, the interactive 
         * plot properties will be display
         */
        WinPlot(
            int sizeRow, int sizeCol,
            int posRow, int posCol,
            bool isInteractive);

        /**
         * Memory deallocation
         */
        ~WinPlot();

        /**
         * Update window content.
         * (see @inherited).
         */
        void draw();

        /**
         * @return the internal number 
         * of plot instances
         */
        size_t getCount() const;

        /**
         * Reset to empty the plot
         * given by its index
         */
        void clear(size_t index);

        /**
         * Set the X axis value to a 
         * plot given by its index
         */
        void setAxisX(
            size_t index, 
            TreeValue& value);

        /**
         * Append a Y axis value to a 
         * plot given by its index
         */
        void addAxisY(
            size_t index, 
            TreeValue& value);

        /**
         * Remove a Y data value by its index
         * from a plot instance given by its index
         */
        void removeAxisY(
            size_t indexPlot, 
            size_t indexData);

        /**
         * @return read only access 
         * to X and Y axis values for a plot
         * given by its index
         */
        const TreeValue* axisX(
            size_t index) const;
        const std::vector<TreeValue*>& axisY(
            size_t index) const;

        /**
         * @return direct access to internal
         * plotting instance for a plot given
         * by its index
         */
        const Plot& plot(size_t index) const;
        Plot& plot(size_t index);

        /**
         * Get or set the history length
         * for the plot instance given by its index
         */
        double getHistory(size_t index) const;
        void setHistory(size_t index, double length);
        
        /**
         * Get or set the paused flag
         * for the plot instance given by its index
         */
        bool getIsPaused(size_t index) const;
        void setIsPaused(size_t index, bool isPaused);

        /**
         * @return selected plot instance and
         * Y axis data indexes
         */
        size_t getSelectedPlot() const;
        size_t getSelectedData() const;

    private:

        /**
         * Structure for a 
         * plot instance
         */
        struct PlotInstance_t {
            //Pointer to the X axis data value.
            //If null, time is used.
            TreeValue* dataX;
            //Container of pointer to 
            //the Y axis data values
            std::vector<TreeValue*> dataY;
            //Internal Gnuplot interface
            Plot plot;
            //Plot history length in seconds
            double historyLength;
            //If true, the interacting plot 
            //is currently paused
            bool isPaused;
        };
        
        /**
         * If true, paused and history length
         * plot properties are display
         */
        bool _isInteractive;

        /**
         * Container for all dynamically 
         * allocated plot instances
         */
        std::vector<PlotInstance_t*> _plots;

        /**
         * Index in _plots and in PlotInstance_t::dataY
         * of selected row. If _selectedData is -1,
         * the whole plot instance is selected.
         */
        size_t _selectedPlot;
        size_t _selectedData;

        /**
         * Remove all empty plot instances
         * that are not at the end of plots container
         * and create if needed an empty plot 
         * instance at the end.
         */
        void cleanPlots();
};

}

#endif

