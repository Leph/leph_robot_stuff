#ifndef LEPH_VIEWER_RHIO_WINBASE_HPP
#define LEPH_VIEWER_RHIO_WINBASE_HPP

#include <vector>
#include <string>
#include <ncurses.h>

namespace leph {

/**
 * WinBase
 *
 * Wrapper around NCurses windows
 * to handle the scrolling and borders.
 */
class WinBase
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
         */
        WinBase(
            int sizeRow, int sizeCol,
            int posRow, int posCol);

        /**
         * Deallocation
         */
        ~WinBase();

        /**
         * @return window size and position
         */
        int getSizeRow() const;
        int getSizeCol() const;
        int getPosRow() const;
        int getPosCol() const;

        /**
         * Clear, redraw the windows 
         * and clear the internal parts 
         * container.
         * The window is actually updated only
         * if the flag _needDrawUpdate is set.
         */
        void draw();

        /**
         * @return direct access to focus flag
         */
        bool getFocus() const;

        /**
         * Set the window focus flag
         */
        void setFocus(bool isFocus);

        /**
         * Append a part to the window content.
         *
         * @param str Textual content. 
         * @param attr NCurses display attribute.
         * @param col Window column offset.
         * @return true is the printed line
         * is currently selected.
         */
        bool print(
            const std::string& str, 
            int attr = 0, 
            int col = 0);

        /**
         * Print given text with automatic
         * line break to fit window columns limit.
         *
         * @param str Textual content. 
         * @param attr NCurses display attribute.
         */
        void printMultilines(
            const std::string& str, 
            int attr = 0);

        /**
         * @return current scrolling offset
         */
        int getScrollOffset() const;
        
        /**
         * @return current selected display row
         */
        int getSelectedRow() const;

        /**
         * Scroll the window content up or down.
         *
         * @param delta Positive or negative
         * row to scroll. the scrolling is
         * bounded to content and window row limits.
         */
        void scrolling(int delta);

        /**
         * Move the currently selected row.
         * Moving the selection at the edge of 
         * the window automatically scroll it.
         *
         * @param delta Positive or negative
         * row to move the selection. 
         * the selection is bounded to 
         * display window row limits.
         */
        void selecting(int delta);

        /**
         * @return direct access to the
         * _needDrawUpdate flag
         */
        const bool& needDrawUpdate() const;
        bool& needDrawUpdate();

        /**
         * @return direct access to 
         * internal NCurses window
         */
        const WINDOW* win() const;
        WINDOW* win();
    
    private:

        /**
         * Curses windows height and 
         * width size and global position
         */
        int _sizeRow;
        int _sizeCol;
        int _posRow;
        int _posCol;
        
        /**
         * Curses allocated window instance
         */
        WINDOW* _win;

        /**
         * If true, the window is focused
         */
        bool _isFocus;

        /**
         * Offset for window scrolling
         */
        int _scrollOffset;

        /**
         * Current content row
         * given from user 
         * at last draw iteration
         */
        int _rowContent;

        /**
         * Current display row
         * actualy printed on the window.
         * If -1, no print has been called
         * since last drawing update.
         */
        int _rowDisplay;

        /**
         * Display line number 
         * currently selected
         */
        int _selectedRow;

        /**
         * If true, the window need to be redrawn
         */
        bool _needDrawUpdate;
};

}

#endif

