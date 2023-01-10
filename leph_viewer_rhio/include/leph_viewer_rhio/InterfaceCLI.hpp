#ifndef LEPH_VIEWER_RHIO_INTERFACECLI_HPP
#define LEPH_VIEWER_RHIO_INTERFACECLI_HPP

#include <vector>
#include <string>
#include <leph_viewer_rhio/WinBase.hpp>

namespace leph {

/**
 * InterfaceCLI
 *
 * Manager for NCurses window 
 * interface, initialization and 
 * user input.
 */
class InterfaceCLI
{
    public:

        /**
         * Initialization of NCurses
         * library displaying
         */
        InterfaceCLI();

        /**
         * Close NCurses display
         */
        ~InterfaceCLI();

        /**
         * @return screen row and col size
         */
        int getScreenSizeRow() const;
        int getScreenSizeCol() const;

        /**
         * Add a window to be managed 
         * for user inputs
         */
        void addWindow(WinBase& win);

        /**
         * Update window state from
         * given user input.
         *
         * @param input User input.
         * @return true if the input has
         * actually been recognized and used.
         */
        bool manageWindowControl(int input);

        /**
         * @return the pointer of 
         * the window currently focused
         */
        WinBase* getWinFocused() const;

    private:

        /**
         * Container of managed windows
         */
        std::vector<WinBase*> _windows;

        /**
         * Index in container of currently
         * focused window
         */
        size_t _indexFocus;

        /**
         * Global NCurses screen size
         */
        int _screenSizeRow;
        int _screenSizeCol;
};

}

#endif

