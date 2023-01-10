#include <stdexcept>
#include <ncurses.h>
#include <leph_viewer_rhio/InterfaceCLI.hpp>

namespace leph {

InterfaceCLI::InterfaceCLI() :
    _windows(),
    _indexFocus(0),
    _screenSizeRow(0),
    _screenSizeCol(0)
{
    //NCurses initialization
    initscr();
    //Initialize color palette
    if(has_colors() == FALSE) {
        endwin();
        throw std::runtime_error(
            "leph::InterfaceCLI: "
            "The terminal does not support color.");
    }
    start_color();
    init_pair(1, COLOR_YELLOW, COLOR_BLACK);
    init_pair(2, COLOR_BLUE, COLOR_BLACK);
    //Enable special input keys
    keypad(stdscr, TRUE);
    //Disable input echo
    noecho();
    //Disable input buffering
    //(and catches Ctrl-Z and Ctrl-C)
    raw();
    //Enable non-blocking input reading
    nodelay(stdscr, TRUE);
    //Hide cursor
    curs_set(0);
    //Enable mouse events
    mousemask(
        BUTTON1_CLICKED | BUTTON1_DOUBLE_CLICKED,
        nullptr);
    mouseinterval(200);
    //Draw first main window
    refresh();
    
    //Retrieve screen window size
    getmaxyx(stdscr, _screenSizeRow, _screenSizeCol);
}

InterfaceCLI::~InterfaceCLI()
{
    //Close NCurses display
    endwin();
}

int InterfaceCLI::getScreenSizeRow() const
{
    return _screenSizeRow;
}
int InterfaceCLI::getScreenSizeCol() const
{
    return _screenSizeCol;
}
        
void InterfaceCLI::addWindow(WinBase& win)
{
    _windows.push_back(&win);
}
        
bool InterfaceCLI::manageWindowControl(int input)
{
    if (_windows.size() == 0) {
        return false;
    }
    if (input == KEY_DOWN) {
        getWinFocused()->selecting(1);
        return true;
    }
    if (input == KEY_UP) {
        getWinFocused()->selecting(-1);
        return true;
    }
    if (
        input == KEY_NPAGE ||
        input == KEY_SF
    ) {
        getWinFocused()->selecting(
            getWinFocused()->getSizeRow()/4);
        return true;
    }
    if (
        input == KEY_PPAGE ||
        input == KEY_SR
    ) {
        getWinFocused()->selecting(
            -getWinFocused()->getSizeRow()/4);
        return true;
    }
    if (input == '\t') {
        getWinFocused()->setFocus(false);
        _indexFocus = (_indexFocus + 1) % _windows.size();
        getWinFocused()->setFocus(true);
        return true;
    }
    if(input == KEY_MOUSE) {
        MEVENT event;
        if(getmouse(&event) == OK) {
            int mouseRow = event.y;
            int mouseCol = event.x;
            for (size_t i=0;i<_windows.size();i++) {
                bool isSuccess = wmouse_trafo(
                    _windows[i]->win(), 
                    &mouseRow, &mouseCol, FALSE);
                if (
                    event.bstate == BUTTON1_CLICKED && 
                    isSuccess
                ) {
                    getWinFocused()->setFocus(false);
                    _indexFocus = i;
                    _windows[i]->selecting(
                        mouseRow-_windows[i]->getSelectedRow()-1);
                    _windows[i]->setFocus(true);
                    break;
                }
            }
        }
    }
    return false;
}
        
WinBase* InterfaceCLI::getWinFocused() const
{
    if (_windows.size() > 0) {
        return _windows.at(_indexFocus);
    } else {
        return nullptr;
    }
}

}

