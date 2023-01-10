#include <iostream>
#include <thread>
#include <chrono>
#include <ncurses.h>
#include <rhio_common/Logging.hpp>
#include <leph_plot/Plot.hpp>
#include <leph_viewer_rhio/InterfaceCLI.hpp>
#include <leph_viewer_rhio/TreeNode.hpp>
#include <leph_viewer_rhio/TreeValue.hpp>
#include <leph_viewer_rhio/WinBase.hpp>
#include <leph_viewer_rhio/WinTree.hpp>
#include <leph_viewer_rhio/WinPlot.hpp>

static void drawPlot(
    leph::Plot& plot, 
    double timeBegin,
    const std::vector<leph::TreeValue*>& axisY,
    const std::map<std::string, std::vector<RhIO::LogValBool>>& containerBool,
    const std::map<std::string, std::vector<RhIO::LogValInt>>& containerInt,
    const std::map<std::string, std::vector<RhIO::LogValFloat>>& containerFloat)
{
    plot.clear();
    for (size_t i=0;i<axisY.size();i++) {
        std::string path = axisY[i]->path().substr(1);
        if (containerBool.count(path) > 0) {
            const std::vector<RhIO::LogValBool>& dataBool = 
                containerBool.at(path);
            for (size_t j=0;j<dataBool.size();j++) {
                plot.add(
                    "time", 
                    ((double)dataBool[j].timestamp-timeBegin)/1000000.0,
                    axisY[i]->path(), (double)dataBool[j].value
                );
            }
        }
        if (containerInt.count(path) > 0) {
            const std::vector<RhIO::LogValInt>& dataInt = 
                containerInt.at(path);
            for (size_t j=0;j<dataInt.size();j++) {
                plot.add(
                    "time", 
                    ((double)dataInt[j].timestamp-timeBegin)/1000000.0,
                    axisY[i]->path(), (double)dataInt[j].value
                );
            }
        }
        if (containerFloat.count(path) > 0) {
            const std::vector<RhIO::LogValFloat>& dataFloat = 
                containerFloat.at(path);
            for (size_t j=0;j<dataFloat.size();j++) {
                plot.add(
                    "time", 
                    ((double)dataFloat[j].timestamp-timeBegin)/1000000.0,
                    axisY[i]->path(), (double)dataFloat[j].value
                );
            }
        }
    }
    for (size_t i=0;i<axisY.size();i++) {
        plot.plot("time", axisY[i]->path());
    }
    plot.show(false, "");
    plot.closeWindow();
}

static void drawMultiPlot(
    leph::Plot& plot, 
    double timeBegin,
    const std::vector<const std::vector<leph::TreeValue*>*>& axisY,
    const std::map<std::string, std::vector<RhIO::LogValBool>>& containerBool,
    const std::map<std::string, std::vector<RhIO::LogValInt>>& containerInt,
    const std::map<std::string, std::vector<RhIO::LogValFloat>>& containerFloat)
{
    plot.clear();
    for (size_t k=0;k<axisY.size();k++) {
        for (size_t i=0;i<axisY[k]->size();i++) {
            std::string path = axisY[k]->at(i)->path().substr(1);
            if (containerBool.count(path) > 0) {
                const std::vector<RhIO::LogValBool>& dataBool = 
                    containerBool.at(path);
                for (size_t j=0;j<dataBool.size();j++) {
                    plot.add(
                        "time", 
                        ((double)dataBool[j].timestamp-timeBegin)/1000000.0,
                        axisY[k]->at(i)->path(), (double)dataBool[j].value
                    );
                }
            }
            if (containerInt.count(path) > 0) {
                const std::vector<RhIO::LogValInt>& dataInt = 
                    containerInt.at(path);
                for (size_t j=0;j<dataInt.size();j++) {
                    plot.add(
                        "time", 
                        ((double)dataInt[j].timestamp-timeBegin)/1000000.0,
                        axisY[k]->at(i)->path(), (double)dataInt[j].value
                    );
                }
            }
            if (containerFloat.count(path) > 0) {
                const std::vector<RhIO::LogValFloat>& dataFloat = 
                    containerFloat.at(path);
                for (size_t j=0;j<dataFloat.size();j++) {
                    plot.add(
                        "time", 
                        ((double)dataFloat[j].timestamp-timeBegin)/1000000.0,
                        axisY[k]->at(i)->path(), (double)dataFloat[j].value
                    );
                }
            }
        }
    }

    plot.multiplot(axisY.size(), 1);
    for (size_t k=0;k<axisY.size();k++) {
        for (size_t i=0;i<axisY[k]->size();i++) {
            plot.plot("time", axisY[k]->at(i)->path());
        }
        plot.nextPlot();
    }
    plot.show(false, "");
    plot.closeWindow();
}

int main(int argc, char** argv)
{
    //Parse user inputs
    std::string filepath = "";
    if (argc != 2) {
        std::cout 
            << "USAGE: ./rhioLogPlot "
            << "log_file_path" 
            << std::endl;
        std::cout 
            << "Use NCurses interface to interactively "
            << "plot the content of a RhIO log file." 
            << std::endl;
        return 1;
    }
    //Retrieve user parameters
    filepath = argv[1];
    
    //Load RhIO binary log file
    std::cout << "Loading RhIO log from: " << filepath << std::endl;
    std::ifstream file(filepath);
    std::map<std::string, std::vector<RhIO::LogValBool>> containerBool;
    std::map<std::string, std::vector<RhIO::LogValInt>> containerInt;
    std::map<std::string, std::vector<RhIO::LogValFloat>> containerFloat;
    std::map<std::string, std::vector<RhIO::LogValStr>> containerStr;
    bool isSuccess = RhIOReadBinaryLog(
        file,
        containerBool, 
        containerInt, 
        containerFloat, 
        containerStr);
    file.close();
    if (!isSuccess) {
        std::cout << "Loading error" << std::endl;
        return 1;
    }

    //Building the RhIO tree 
    //from value names
    leph::TreeNode root("/");
    for (const auto& it : containerBool) {
        root.addValueByPath(it.first, leph::TreeValue::ValueType::BOOL);
    }
    for (const auto& it : containerInt) {
        root.addValueByPath(it.first, leph::TreeValue::ValueType::INT);
    }
    for (const auto& it : containerFloat) {
        root.addValueByPath(it.first, leph::TreeValue::ValueType::FLOAT);
    }

    //Retrieve the earliest timestamp
    //to offset all value timestamps
    double timeBegin = -1.0;
    for (const auto& it : containerBool) {
        if (
            it.second.size() > 0 && 
            (timeBegin < 0.0 || it.second.front().timestamp < timeBegin)
        ) {
            timeBegin = it.second.front().timestamp;
        }
    }
    for (const auto& it : containerInt) {
        if (
            it.second.size() > 0 && 
            (timeBegin < 0.0 || it.second.front().timestamp < timeBegin)
        ) {
            timeBegin = it.second.front().timestamp;
        }
    }
    for (const auto& it : containerFloat) {
        if (
            it.second.size() > 0 && 
            (timeBegin < 0.0 || it.second.front().timestamp < timeBegin)
        ) {
            timeBegin = it.second.front().timestamp;
        }
    }

    //NCurses initialization
    leph::InterfaceCLI* interface = new leph::InterfaceCLI();
    int screenRows = interface->getScreenSizeRow();
    int screenCols = interface->getScreenSizeCol();

    //Create NCurses windows
    leph::WinBase* winTitle = new leph::WinBase(
        3, (screenCols/2)*2, 
        0, 0);
    winTitle->print("RhIO Curses Log: " + filepath);
    leph::WinTree* winTree = new leph::WinTree(
        screenRows-6, screenCols/2,
        3, 0, 
        root);
    leph::WinPlot* winPlot = new leph::WinPlot(
        screenRows-7, screenCols/2,
        3, screenCols/2, false);
    leph::WinBase* winInfo = new leph::WinBase(
        3, screenCols/2,
        screenRows-3, 0);
    leph::WinBase* winHelp = new leph::WinBase(
        4, screenCols/2,
        screenRows-4, screenCols/2);
    winTree->setFocus(true);
    //Set windows to be managed by 
    //the interface for user controls
    interface->addWindow(*winTree);
    interface->addWindow(*winPlot);

    //Main loop
    bool isFirstLoop = true;
    int userDigit = -1;
    while (true) {
        //Retrieve user input
        int input = getch();
        //Quit state
        if (input == 'q') {
            break;
        }
        //Handle windows selection and scrolling
        bool isUserWinControls = 
            interface->manageWindowControl(input);
        //Collapse selected node in tree
        if (
            input == '\n' && 
            winTree->getFocus() &&
            winTree->selectedNode() != nullptr
        ) {
            winTree->selectedNode()->isCollapsed() = 
                !winTree->selectedNode()->isCollapsed();
            winTree->needDrawUpdate() = true;
        }
        //Plot select value in tree
        if (
            input == 'p' && 
            winTree->getFocus() &&
            winTree->selectedValue() != nullptr
        ) {
            size_t count = winPlot->getCount();
            if (userDigit == -1) {
                if (count <= 1) {
                    winPlot->addAxisY(
                        0, *(winTree->selectedValue()));
                } else {
                    winPlot->addAxisY(
                        count-2, *(winTree->selectedValue()));
                }
            } else if (userDigit < (int)count) {
                winPlot->addAxisY(
                    userDigit, *(winTree->selectedValue()));
            }
        }
        //Draw selected plot instance in plot window
        if (
            input == 'p' && 
            winPlot->getFocus() &&
            winPlot->getSelectedPlot() != (size_t)-1 &&
            winPlot->axisY(winPlot->getSelectedPlot()).size() > 0
        ) {
            size_t index = winPlot->getSelectedPlot();
            drawPlot(
                winPlot->plot(index),
                timeBegin,
                winPlot->axisY(index),
                containerBool, 
                containerInt,
                containerFloat);
        }
        //Draw all plot instance as a multiplot
        if (
            input == 'm' && 
            winPlot->getFocus()
        ) {
            std::vector<const std::vector<leph::TreeValue*>*> tmpAxis;
            for (size_t i=0;i<winPlot->getCount();i++) {
                if (winPlot->axisY(i).size() > 0) {
                    tmpAxis.push_back(&(winPlot->axisY(i)));
                }
            }
            drawMultiPlot(
                winPlot->plot(0),
                timeBegin,
                tmpAxis,
                containerBool, 
                containerInt,
                containerFloat);
        }
        //Remove selected value in plot
        if (
            input == 'd' && 
            winPlot->getFocus() &&
            winPlot->getSelectedPlot() != (size_t)-1 &&
            winPlot->getSelectedData() != (size_t)-1
        ) {
            winPlot->removeAxisY(
                winPlot->getSelectedPlot(),
                winPlot->getSelectedData());
        }
        //Remove a selected complete plot
        if (
            input == 'd' && 
            winPlot->getFocus() &&
            winPlot->getSelectedPlot() != (size_t)-1 &&
            winPlot->getSelectedData() == (size_t)-1
        ) {
            winPlot->clear(
                winPlot->getSelectedPlot());
        }
        //Main windows display
        winTree->draw();
        winPlot->draw();
        //Update help and info window
        //for win Tree
        if (
            (isFirstLoop || isUserWinControls) && 
            winTree->getFocus()
        ) {
            //Help
            winHelp->printMultilines(
                "[q] Quit, "
                "[TAB] Switch window, "
                "[ENTER] Collapse nodes, "
                "[[0-9]p] Add a value into a plot");
            winHelp->needDrawUpdate() = true;
            //Info
            if (winTree->selectedNode() != nullptr) {
                winInfo->print(winTree->selectedNode()->path()+"/");
            }
            if (winTree->selectedValue() != nullptr) {
                winInfo->print(winTree->selectedValue()->path());
            }
            winInfo->needDrawUpdate() = true;
        }
        //For win Plot
        if (
            (isFirstLoop || isUserWinControls) && 
            winPlot->getFocus()
        ) {
            //Help
            winHelp->printMultilines(
                "[q] Quit, "
                "[TAB] Switch window, "
                "[p] Draw selected plot, "
                "[m] Draw all plots as multiplot, "
                "[d]: Remove a value/plot");
            winHelp->needDrawUpdate() = true;
            //Info
            if (
                winPlot->getSelectedPlot() != (size_t)-1 &&
                winPlot->getSelectedData() == (size_t)-1
            ) {
                winInfo->print(
                    "Plot " + 
                    std::to_string(winPlot->getSelectedPlot()));
            }
            if (
                winPlot->getSelectedPlot() != (size_t)-1 &&
                winPlot->getSelectedData() != (size_t)-1
            ) {
                winInfo->print(
                    winPlot->axisY(winPlot->getSelectedPlot())
                    .at(winPlot->getSelectedData())->path());
            }
            if (
                winPlot->getSelectedPlot() == (size_t)-1 &&
                winPlot->getSelectedData() == (size_t)-1
            ) {
                winInfo->print("Plot empty");
            }
            winInfo->needDrawUpdate() = true;
        }
        //Other windows display
        winTitle->draw();
        winHelp->draw();
        winInfo->draw();
        //Retrieve user digit inputs
        if (std::isdigit(input)) {
            userDigit = input - '0';
        } else if (input != ERR) {
            userDigit = -1;
        }
        //NCurses screen update
        doupdate();
        //Waiting
        std::this_thread::sleep_for(
            std::chrono::milliseconds(20));
        isFirstLoop = false;
    }
    
    //Destroy NCurses windows
    //and close NCurses environment
    delete winTitle;
    delete winTree;
    delete winPlot;
    delete winInfo;
    delete winHelp;
    delete interface;

    return 0;
}

