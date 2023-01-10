#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <ncurses.h>
#include <leph_plot/Plot.hpp>
#include <RhIOClient.hpp>
#include <rhio_common/LockFreeDoubleQueue.hpp>
#include <leph_viewer_rhio/InterfaceCLI.hpp>
#include <leph_viewer_rhio/TreeNode.hpp>
#include <leph_viewer_rhio/TreeValue.hpp>
#include <leph_viewer_rhio/WinBase.hpp>
#include <leph_viewer_rhio/WinTree.hpp>
#include <leph_viewer_rhio/WinPlot.hpp>

static void syncTree(
    leph::TreeNode& root, 
    RhIO::ClientReq& clientReq,
    const std::string& path)
{
    std::vector<std::string> listBool = 
        clientReq.listValuesBool(path);
    std::vector<std::string> listInt = 
        clientReq.listValuesInt(path);
    std::vector<std::string> listFloat = 
        clientReq.listValuesFloat(path);
    for (size_t i=0;i<listBool.size();i++) {
        root.addValueByPath(path+"/"+listBool.at(i), leph::TreeValue::ValueType::BOOL);
    }
    for (size_t i=0;i<listInt.size();i++) {
        root.addValueByPath(path+"/"+listInt.at(i), leph::TreeValue::ValueType::INT);
    }
    for (size_t i=0;i<listFloat.size();i++) {
        root.addValueByPath(path+"/"+listFloat.at(i), leph::TreeValue::ValueType::FLOAT);
    }
    
    std::vector<std::string> listChildren = 
        clientReq.listChildren(path);
    for (size_t i=0;i<listChildren.size();i++) {
        syncTree(root, clientReq, path+"/"+listChildren.at(i));
    }
}

static void addValueToStream(
    std::map<std::string, unsigned int>& streamedValues,
    leph::TreeValue& value,
    RhIO::ClientReq& clientReq,
    std::mutex& mutexData)
{
    std::string path = value.path().substr(1);
    mutexData.lock();
    if (streamedValues.count(path) == 0) {
        streamedValues.insert(std::make_pair(path, 0));
    } 
    if (streamedValues.at(path) == 0) {
        clientReq.enableStreamingValue(path);
    }
    streamedValues.at(path)++;
    mutexData.unlock();
}
static void delValueToStream(
    std::map<std::string, unsigned int>& streamedValues,
    leph::TreeValue& value,
    RhIO::ClientReq& clientReq,
    std::mutex& mutexData)
{
    std::string path = value.path().substr(1);
    mutexData.lock();
    if (streamedValues.count(path) == 0) {
        streamedValues.insert(std::make_pair(path, 0));
    } 
    if (streamedValues.at(path) == 1) {
        clientReq.disableStreamingValue(path);
    }
    if (streamedValues.at(path) > 0) {
        streamedValues.at(path)--;
    }
    mutexData.unlock();
}
static void checkValueToStream(
    std::map<std::string, unsigned int>& streamedValues,
    RhIO::ClientReq& clientReq,
    std::mutex& mutexData)
{
    mutexData.lock();
    for (const auto& it : streamedValues) {
        if (it.second > 0) {
            clientReq.checkStreamingValue(it.first);
        }
    }
    mutexData.unlock();
}

static void threadPlotting(
    bool& isQuitAsked,
    RhIO::LockFreeDoubleQueue<
        std::tuple<std::string, double, double>>& queueValues,
    std::mutex& mutexWin,
    std::atomic<bool>& needUpdateWin,
    leph::WinPlot* winPlot,
    double& timeStart)
{
    double lastAddedToPlotTime = -1.0;
    while (!isQuitAsked) {
        //Swap data buffer
        queueValues.swapBufferFromReader();
        //Plot data in queue
        size_t queueSize = queueValues.getSizeFromReader();
        const auto& queueRef = queueValues.getBufferFromReader();
        while (needUpdateWin.load()) {
            std::this_thread::yield();
        }
        mutexWin.lock();
        for (size_t i=0;i<winPlot->getCount();i++) {
            if (winPlot->axisY(i).size() > 0) {
                if (winPlot->getIsPaused(i)) {
                    continue;
                }
                for (size_t j=0;j<queueSize;j++) {
                    bool isFound = false;
                    for (size_t k=0;k<winPlot->axisY(i).size();k++) {
                        if (
                            winPlot->axisY(i).at(k)->path().substr(1) == 
                            std::get<0>(queueRef.at(j))
                        ) {
                            isFound = true;
                        }
                    }
                    if (isFound) {
                        double time = std::get<1>(queueRef.at(j));
                        winPlot->plot(i).add(
                            "time", time-timeStart,
                            std::get<0>(queueRef.at(j)), 
                            std::get<2>(queueRef.at(j))
                        );
                        if (
                            lastAddedToPlotTime < 0.0 || 
                            lastAddedToPlotTime < time-timeStart
                        ) {
                            lastAddedToPlotTime = time-timeStart;
                        }
                    }
                }
                if (winPlot->getHistory(i) > 0.0 && !winPlot->getIsPaused(i)) {
                    winPlot->plot(i).filterLowerThan(
                        "time", lastAddedToPlotTime-winPlot->getHistory(i));
                }
                for (size_t k=0;k<winPlot->axisY(i).size();k++) {
                    winPlot->plot(i).plot(
                        "time", 
                        winPlot->axisY(i).at(k)->path().substr(1));
                }
                winPlot->plot(i).winSize(960, 540);
                winPlot->plot(i).stream();
            }
        }
        mutexWin.unlock();
    }
}

int main(int argc, char** argv)
{
    //Parse user inputs
    if (argc > 2) {
        std::cout 
            << "USAGE: ./rhioCursesServer [remote_host]"
            << std::endl;
        return 1;
    }
    //Retrieve remote host
    std::string remoteHost = "localhost";
    if (argc == 2) {
        remoteHost = argv[1];
    }

    //RhIO clients
    std::string endpoint = 
        std::string("tcp://") 
        + remoteHost
        + std::string(":")
        + std::to_string(RhIO::PortServerRep);
    std::cout << "Connecting to: " << endpoint << std::endl;
    RhIO::ClientReq clientReq(endpoint);
    RhIO::ClientSub clientSub(
        std::string("udp://") 
        + RhIO::AddressMulticast
        + std::string(":")
        + std::to_string(RhIO::PortServerPub));

    //Retrieve controller start time
    bool timeStartIsInit = false;
    double timeStart = 0.0;
    
    //Building the RhIO tree 
    //from RhIO server
    leph::TreeNode root("/");
    syncTree(root, clientReq, "");
    
    //NCurses initialization
    leph::InterfaceCLI* interface = new leph::InterfaceCLI();
    int screenRows = interface->getScreenSizeRow();
    int screenCols = interface->getScreenSizeCol();

    //Create NCurses windows
    leph::WinBase* winTitle = new leph::WinBase(
        3, (screenCols/2)*2, 
        0, 0);
    winTitle->print("RhIO Curses Server: " + endpoint);
    leph::WinTree* winTree = new leph::WinTree(
        screenRows-6, screenCols/2,
        3, 0, root);
    leph::WinPlot* winPlot = new leph::WinPlot(
        screenRows-7, screenCols/2,
        3, screenCols/2, true);
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

    //Value streaming state initialization
    std::mutex mutexData;
    std::mutex mutexWin;
    std::atomic<bool> needUpdateWin(false);
    std::map<std::string, unsigned int> streamedValues;
    RhIO::LockFreeDoubleQueue<
        std::tuple<std::string, double, double>> 
        queueValues(100000);
    
    //Setup RhIO receiving callbacks
    clientSub.setHandlerBool(
        [&mutexData, &queueValues, &streamedValues, &timeStart, &timeStartIsInit]
        (const std::string name, int64_t timestamp, bool val) 
    {
        mutexData.lock();
        bool isFound = (streamedValues.count(name) != 0);
        mutexData.unlock();
        if (isFound) {
            queueValues.appendFromWriter(std::make_tuple(
                name, (double)timestamp/1000000.0, (double)val));
        }
        if (!timeStartIsInit) {
            timeStart = (double)timestamp/1000000.0;
            timeStartIsInit = true;
        }
    });
    clientSub.setHandlerInt(
        [&mutexData, &queueValues, &streamedValues, &timeStart, &timeStartIsInit]
        (const std::string name, int64_t timestamp, int64_t val) 
    {
        mutexData.lock();
        bool isFound = (streamedValues.count(name) != 0);
        mutexData.unlock();
        if (isFound) {
            queueValues.appendFromWriter(std::make_tuple(
                name, (double)timestamp/1000000.0, (double)val));
        }
        if (!timeStartIsInit) {
            timeStart = (double)timestamp/1000000.0;
            timeStartIsInit = true;
        }
    });
    clientSub.setHandlerFloat(
        [&mutexData, &queueValues, &streamedValues, &timeStart, &timeStartIsInit]
        (const std::string name, int64_t timestamp, double val) 
    {
        mutexData.lock();
        bool isFound = (streamedValues.count(name) != 0);
        mutexData.unlock();
        if (isFound) {
            queueValues.appendFromWriter(std::make_tuple(
                name, (double)timestamp/1000000.0, (double)val));
        }
        if (!timeStartIsInit) {
            timeStart = (double)timestamp/1000000.0;
            timeStartIsInit = true;
        }
    });

    //Start plotting thread
    bool isQuitAsked = false;
    std::thread thread(
        [&isQuitAsked, &queueValues, &mutexWin, 
        &needUpdateWin, winPlot, &timeStart]()
    {
        threadPlotting(
            isQuitAsked, queueValues, mutexWin, 
            needUpdateWin, winPlot, timeStart);
    });

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
        //Synchronisation with remote server
        if (
            input == 's' &&
            winTree->getFocus()
        ) {
            mutexData.lock();
            syncTree(root, clientReq, "");
            checkValueToStream(
                streamedValues,
                clientReq,
                mutexData);
            timeStartIsInit = false;
            mutexData.unlock();
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
                    needUpdateWin.store(true);
                    mutexWin.lock();
                    winPlot->addAxisY(
                        0, *(winTree->selectedValue()));
                    needUpdateWin.store(false);
                    mutexWin.unlock();
                    addValueToStream(
                        streamedValues,
                        *(winTree->selectedValue()), 
                        clientReq,
                        mutexData);
                } else {
                    needUpdateWin.store(true);
                    mutexWin.lock();
                    winPlot->addAxisY(
                        count-2, *(winTree->selectedValue()));
                    needUpdateWin.store(false);
                    mutexWin.unlock();
                    addValueToStream(
                        streamedValues,
                        *(winTree->selectedValue()), 
                        clientReq,
                        mutexData);
                }
            } else if (userDigit < (int)count) {
                needUpdateWin.store(true);
                mutexWin.lock();
                winPlot->addAxisY(
                    userDigit, *(winTree->selectedValue()));
                needUpdateWin.store(false);
                mutexWin.unlock();
                addValueToStream(
                    streamedValues,
                    *(winTree->selectedValue()), 
                    clientReq,
                    mutexData);
            }
        }
        //Remove selected value in plot
        if (
            input == 'd' && 
            winPlot->getFocus() &&
            winPlot->getSelectedPlot() != (size_t)-1 &&
            winPlot->getSelectedData() != (size_t)-1
        ) {
            size_t tmpIndexPlot = winPlot->getSelectedPlot();
            size_t tmpIndexData = winPlot->getSelectedData();
            const std::vector<leph::TreeValue*>& tmpAxis = 
                winPlot->axisY(tmpIndexPlot);
            delValueToStream(
                streamedValues,
                *(tmpAxis.at(tmpIndexData)),
                clientReq,
                mutexData);
            needUpdateWin.store(true);
            mutexWin.lock();
            winPlot->removeAxisY(
                tmpIndexPlot, tmpIndexData);
            needUpdateWin.store(false);
            mutexWin.unlock();
        }
        //Remove a selected complete plot
        if (
            input == 'd' && 
            winPlot->getFocus() &&
            winPlot->getSelectedPlot() != (size_t)-1 &&
            winPlot->getSelectedData() == (size_t)-1
        ) {
            size_t tmpIndexPlot = winPlot->getSelectedPlot();
            const std::vector<leph::TreeValue*>& tmpAxis = 
                winPlot->axisY(tmpIndexPlot);
            for (size_t i=0;i<tmpAxis.size();i++) {
                delValueToStream(
                    streamedValues,
                    *(tmpAxis.at(i)),
                    clientReq,
                    mutexData);
            }
            needUpdateWin.store(true);
            mutexWin.lock();
            winPlot->clear(tmpIndexPlot);
            needUpdateWin.store(false);
            mutexWin.unlock();
        }
        //Change history
        if (
            input == 'h' && 
            winPlot->getFocus() &&
            winPlot->getSelectedPlot() != (size_t)-1 &&
            !winPlot->getIsPaused(winPlot->getSelectedPlot())
        ) {
            needUpdateWin.store(true);
            mutexWin.lock();
            size_t tmpIndexPlot = winPlot->getSelectedPlot();
            double length = winPlot->getHistory(tmpIndexPlot);
            if (length < 0.0) {
                winPlot->setHistory(tmpIndexPlot, 5.0);
            } else if (length == 5.0) {
                winPlot->setHistory(tmpIndexPlot, 20.0);
            } else if (length == 20.0) {
                winPlot->setHistory(tmpIndexPlot, 60.0);
            } else if (length == 60.0) {
                winPlot->setHistory(tmpIndexPlot, -1.0);
            }
            needUpdateWin.store(false);
            mutexWin.unlock();
        }
        //Toggle pause
        if (
            input == 'p' && 
            winPlot->getFocus() &&
            winPlot->getSelectedPlot() != (size_t)-1
        ) {
            needUpdateWin.store(true);
            mutexWin.lock();
            size_t tmpIndexPlot = winPlot->getSelectedPlot();
            winPlot->setIsPaused(
                tmpIndexPlot, 
                !winPlot->getIsPaused(tmpIndexPlot));
            needUpdateWin.store(false);
            mutexWin.unlock();
        }
        //Clear a plot
        if (
            input == 'c' && 
            winPlot->getFocus() &&
            winPlot->getSelectedPlot() != (size_t)-1
        ) {
            needUpdateWin.store(true);
            mutexWin.lock();
            size_t tmpIndexPlot = winPlot->getSelectedPlot();
            winPlot->plot(tmpIndexPlot).closeWindow();
            winPlot->plot(tmpIndexPlot).clear();
            needUpdateWin.store(false);
            mutexWin.unlock();
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
                "[s] Sync with server, "
                "[[0-9]p] Add a value into a plot");
            winHelp->needDrawUpdate() = true;
            //Info
            if (winTree->selectedNode() != nullptr) {
                winInfo->print(winTree->selectedNode()->path()+"/");
            }
            if (winTree->selectedValue() != nullptr) {
                winInfo->print(
                    winTree->selectedValue()->path() +
                    " [" +
                    winTree->selectedValue()->typeAsStr() +
                    "] ");
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
                "[h] Cycle history, "
                "[p] Pause plot, "
                "[c] Clear plot, "
                "[d]: Remove a value or plot");
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

    isQuitAsked = true;
    thread.join();

    //Stop all remaining streaming
    for (const auto& it : streamedValues) {
        if (it.second > 0) {
            clientReq.disableStreamingValue(it.first);
        }
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

