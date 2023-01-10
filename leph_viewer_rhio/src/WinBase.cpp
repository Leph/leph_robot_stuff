#include <stdexcept>
#include <leph_viewer_rhio/WinBase.hpp>

namespace leph {

WinBase::WinBase(
    int sizeRow, int sizeCol,
    int posRow, int posCol) :
    _sizeRow(sizeRow),
    _sizeCol(sizeCol),
    _posRow(posRow),
    _posCol(posCol),
    _win(nullptr),
    _isFocus(false),
    _scrollOffset(0),
    _rowContent(0),
    _rowDisplay(-1),
    _selectedRow(0),
    _needDrawUpdate(true)
{
    _win = newwin(
        sizeRow, sizeCol, posRow, posCol);
}

WinBase::~WinBase()
{
    delwin(_win);
}

int WinBase::getSizeRow() const
{
    return _sizeRow;
}
int WinBase::getSizeCol() const
{
    return _sizeCol;
}
int WinBase::getPosRow() const
{
    return _posRow;
}
int WinBase::getPosCol() const
{
    return _posCol;
}
        
void WinBase::draw()
{
    if (!_needDrawUpdate) {
        return;
    } else {
        _needDrawUpdate = false;
    }

    //Draw the border
    if (_isFocus) {
        wattron(_win, A_STANDOUT);
        wborder(_win, 
            ACS_VLINE, ACS_VLINE, ACS_HLINE, ACS_HLINE, 
            ACS_ULCORNER, ACS_URCORNER, ACS_LLCORNER, ACS_LRCORNER);
        wattroff(_win, A_STANDOUT);
    } else {
        wattron(_win, A_NORMAL);
        wborder(_win, 
            ACS_VLINE, ACS_VLINE, ACS_HLINE, ACS_HLINE, 
            ACS_ULCORNER, ACS_URCORNER, ACS_LLCORNER, ACS_LRCORNER);
        wattroff(_win, A_NORMAL);
    }

    //Flush the window display
    wnoutrefresh(_win);

    //Reset row states
    _rowDisplay = -1;
}

bool WinBase::getFocus() const
{
    return _isFocus;
}

void WinBase::setFocus(bool isFocus)
{
    if (_isFocus != isFocus) {
        _needDrawUpdate = true;
        _isFocus = isFocus;
    }
}

bool WinBase::print(
    const std::string& str, 
    int attr, 
    int col)
{
    //Clear the window at 
    //first print call
    if (_rowDisplay == -1) {
        wclear(_win);
        _rowContent = 0;
        _rowDisplay = 1;
    }

    size_t index = str.find_first_of("\n");
    if (index != std::string::npos && index != str.size()-1) {
        throw std::logic_error(
            "leph::WinBase::append: "
            "Invalid line break: " + str);
    }
    bool isLineBreak = index != std::string::npos;

    //Enable highlight on selected row
    bool isSelected = false;
    if (_isFocus && _rowDisplay == _selectedRow+1) {
        attr = attr | A_STANDOUT;
        isSelected = true;
    }

    //Print on the window if visible with scrolling
    if (_rowContent >= _scrollOffset && _rowDisplay <= _sizeRow-2) {
        if (attr != 0) {
            wattron(_win, attr);
        }
        mvwprintw(
            _win, _rowDisplay, col+1, 
            str.substr(0, _sizeCol-col-2).c_str());
        if (attr != 0) {
            wattroff(_win, attr);
        }
        if (isLineBreak) {
            _rowDisplay++;
        }
    }
    if (isLineBreak) {
        _rowContent++;
    }

    return isSelected;
}

void WinBase::printMultilines(
    const std::string& str, 
    int attr)
{
    int indexBegin = 0;
    while (indexBegin < (int)str.length()) {
        int indexEnd = indexBegin + std::min(
            (int)str.length() - indexBegin, 
            _sizeCol-2);
        print(
            str.substr(indexBegin, indexEnd-indexBegin) + "\n", 
            attr, 0);
        indexBegin = indexEnd;
    }
}
        
int WinBase::getScrollOffset() const
{
    return _scrollOffset;
}
        
int WinBase::getSelectedRow() const
{
    return _selectedRow;
}
 
void WinBase::scrolling(int delta)
{
    _scrollOffset += delta;
    if (_scrollOffset < 0) {
        _scrollOffset = 0;
    }
    if (_rowContent > _sizeRow-2) {
        if (_scrollOffset > _rowContent-_sizeRow+2) {
            _scrollOffset = _rowContent-_sizeRow+2;
        }
    } else {
        _scrollOffset = 0;
    }
    _needDrawUpdate = true;
}
        
void WinBase::selecting(int delta)
{
    _selectedRow += delta;
    if (_selectedRow < 0) {
        scrolling(_selectedRow);
        _selectedRow = 0;
    }
    if (_rowContent < _sizeRow-2 && _selectedRow > _rowContent-1) {
        _selectedRow = _rowContent-1;
    }
    if (_selectedRow > _sizeRow-3) {
        scrolling(_selectedRow-_sizeRow+3);
        _selectedRow = _sizeRow-3;
    }
    _needDrawUpdate = true;
}

const bool& WinBase::needDrawUpdate() const
{
    return _needDrawUpdate;
}
bool& WinBase::needDrawUpdate()
{
    return _needDrawUpdate;
}

const WINDOW* WinBase::win() const
{
    return _win;
}
WINDOW* WinBase::win()
{
    return _win;
}

}

