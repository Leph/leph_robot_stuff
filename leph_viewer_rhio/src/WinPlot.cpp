#include <stdexcept>
#include <leph_viewer_rhio/WinPlot.hpp>

namespace leph {

WinPlot::WinPlot(
    int sizeRow, int sizeCol,
    int posRow, int posCol,
    bool isInteractive) :
    WinBase(sizeRow, sizeCol, posRow, posCol),
    _isInteractive(isInteractive),
    _plots(),
    _selectedPlot((size_t)-1),
    _selectedData((size_t)-1)
{
    cleanPlots();
}
        
WinPlot::~WinPlot()
{
    for (size_t i=0;i<_plots.size();i++) {
        delete _plots[i];
    }
}
        
void WinPlot::draw()
{
    if (!WinBase::needDrawUpdate()) {
        return;
    }

    for (size_t i=0;i<_plots.size();i++) {
        std::string strHistory = "";
        if (_isInteractive) {
            if (_plots[i]->isPaused) {
                strHistory = " (paused)";
            } else {
                strHistory = " (history ";
                if (_plots[i]->historyLength < 0.0) {
                    strHistory += "unlimited";
                } else {
                    strHistory += 
                        std::to_string(_plots[i]->historyLength);
                    strHistory += "s";
                }
                strHistory += ")";
            }
        }
        WinBase::print(
            "[" + std::to_string(i) + "]",
            A_BOLD);
        if (_plots[i]->dataY.size() == 0) {
            bool isSelected = WinBase::print(
                "Plot: empty" + strHistory + "\n", 
                A_BOLD, 4);
            if (isSelected) {
                _selectedPlot = (size_t)-1;
                _selectedData = (size_t)-1;
            }
        } else {
            bool isSelected = false;
            if (_plots[i]->dataX == nullptr) {
                isSelected = WinBase::print(
                    "Plot: time" + strHistory + "\n", 
                    A_BOLD, 4);
            } else {
                isSelected = WinBase::print(
                    "Plot: " + _plots[i]->dataX->path() 
                    + strHistory + "\n", 
                    A_BOLD, 4);
            }
            if (isSelected) {
                _selectedPlot = i;
                _selectedData = (size_t)-1;
            }
        }
        for (size_t j=0;j<_plots[i]->dataY.size();j++) {
            bool isSelected = WinBase::print(
                _plots[i]->dataY[j]->path() + "\n",
                0, 4);
            if (isSelected) {
                _selectedPlot = i;
                _selectedData = j;
            }
        }
    }

    WinBase::draw();
}
        
size_t WinPlot::getCount() const
{
    return _plots.size();
}
        
void WinPlot::clear(size_t index)
{
    if (index >= getCount()) {
        throw std::logic_error(
            "leph::WinPlot::clear: Invalid index.");
    }

    _plots[index]->plot.closeWindow();
    _plots[index]->dataX = nullptr;
    _plots[index]->dataY.clear();
    _plots[index]->plot.clear();
    _plots[index]->historyLength = 20.0;
    _plots[index]->isPaused = false;
    cleanPlots();
    WinBase::needDrawUpdate() = true;
}
        
void WinPlot::setAxisX(
    size_t index, 
    TreeValue& value)
{
    if (index >= getCount()) {
        throw std::logic_error(
            "leph::WinPlot::setAxisX: Invalid index.");
    }

    _plots[index]->dataX = &value;
    cleanPlots();
    WinBase::needDrawUpdate() = true;
}
        
void WinPlot::addAxisY(
    size_t index,
    TreeValue& value)
{
    if (index >= getCount()) {
        throw std::logic_error(
            "leph::WinPlot::addAxisY: Invalid index.");
    }
    
    bool isFound = false;
    for (size_t i=0;i<_plots[index]->dataY.size();i++) {
        if (_plots[index]->dataY[i]->path() == value.path()) {
            isFound = true;
        }
    }
    if (!isFound) {
        _plots[index]->dataY.push_back(&value);
        cleanPlots();
        WinBase::needDrawUpdate() = true;
    }
}

void WinPlot::removeAxisY(
    size_t indexPlot,
    size_t indexData)
{
    if (indexPlot >= getCount()) {
        throw std::logic_error(
            "leph::WinPlot::removeAxisY: Invalid index.");
    }

    if (indexData < _plots[indexPlot]->dataY.size()) {
        _plots[indexPlot]->dataY.erase(
            _plots[indexPlot]->dataY.begin()+indexData);
        if (_plots[indexPlot]->dataY.size() == 0) {
            _plots[indexPlot]->plot.closeWindow();
        }
        cleanPlots();
        WinBase::needDrawUpdate() = true;
    }
}

const TreeValue* WinPlot::axisX(
    size_t index) const
{
    if (index >= getCount()) {
        throw std::logic_error(
            "leph::WinPlot::axisX: Invalid index.");
    }

    return _plots[index]->dataX;
}
const std::vector<TreeValue*>& WinPlot::axisY(
    size_t index) const
{
    if (index >= getCount()) {
        throw std::logic_error(
            "leph::WinPlot::axisY: Invalid index.");
    }

    return _plots[index]->dataY;
}

const Plot& WinPlot::plot(size_t index) const
{
    if (index >= getCount()) {
        throw std::logic_error(
            "leph::WinPlot::plot: Invalid index.");
    }

    return _plots[index]->plot;
}
Plot& WinPlot::plot(size_t index)
{
    if (index >= getCount()) {
        throw std::logic_error(
            "leph::WinPlot::plot: Invalid index.");
    }

    return _plots[index]->plot;
}

double WinPlot::getHistory(size_t index) const
{
    if (index >= getCount()) {
        throw std::logic_error(
            "leph::WinPlot::getHistory: Invalid index.");
    }

    return _plots[index]->historyLength;
}
void WinPlot::setHistory(size_t index, double length)
{
    if (index >= getCount()) {
        throw std::logic_error(
            "leph::WinPlot::setHistory: Invalid index.");
    }

    _plots[index]->historyLength = length;
    WinBase::needDrawUpdate() = true;
}

bool WinPlot::getIsPaused(size_t index) const
{
    if (index >= getCount()) {
        throw std::logic_error(
            "leph::WinPlot::getIsPaused: Invalid index.");
    }

    return _plots[index]->isPaused;
}
void WinPlot::setIsPaused(size_t index, bool isPaused)
{
    if (index >= getCount()) {
        throw std::logic_error(
            "leph::WinPlot::setIsPaused: Invalid index.");
    }

    _plots[index]->isPaused = isPaused;
    WinBase::needDrawUpdate() = true;
}

size_t WinPlot::getSelectedPlot() const
{
    return _selectedPlot;
}
size_t WinPlot::getSelectedData() const
{
    return _selectedData;
}
        
void WinPlot::cleanPlots()
{
    if (
        _plots.size() == 0 || 
        _plots.back()->dataY.size() != 0
    ) {
        _plots.push_back(
            new PlotInstance_t());
        WinPlot::clear(_plots.size()-1);
    }
    while (true) {
        bool isFound = false;
        for (size_t i=0;i<_plots.size()-1;i++) {
            if (_plots[i]->dataY.size() == 0) {
                isFound = true;
                delete _plots[i];
                for (int j=i;j<(int)_plots.size()-1;j++) {
                    _plots[j] = _plots[j+1];
                }
                _plots.pop_back();
            }
        }
        if (!isFound) {
            break;
        }
    }
}
        
}

