#ifndef LEPH_UTILS_SPACEMOUSE_HPP
#define LEPH_UTILS_SPACEMOUSE_HPP

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <thread>
#include <atomic>

namespace leph {

/**
 * SpaceMouse
 *
 * Simple driver to read 6DOFs space mouse device
 */
class SpaceMouse
{
    public:

        /**
         * Empty initialization
         */
        SpaceMouse();

        /**
         * Open the device
         *
         * @param devicePath Path to system input device
         * (/dev/input/eventX)
         */
        void openDevice(const std::string& devicePath);

        /**
         * Open first listed available device
         */
        void openDefault();

        /**
         * Close the device if needed
         */
        ~SpaceMouse();

        /**
         * Return last read linear or angular input
         * normalized between -1.0 and 1.0
         */
        Eigen::Vector3d getLin() const;
        Eigen::Vector3d getAng() const;

        /**
         * Return true is button left or right is pushed
         */
        bool isButtonLeft() const;
        bool isButtonRight() const;

        /**
         * List and return path to available SpaceMouse devices
         */
        static std::vector<std::string> listDevices();

    private:

        /**
         * Evdev data structure
         */
        struct Event_t {
            struct timeval time;
            unsigned short type;
            unsigned short code;
            int value;
        };

        /**
         * Time in seconds at reader thread start 
         * used to compute time offset
         */
        time_t _timeStart;

        /**
         * Device descriptor
         * and device path path
         */
        int _fd;
        std::string _devicePath;

        /**
         * Reader thread
         */
        std::atomic<bool> _isContinue;
        std::thread _thread;

        /**
         * Last received relative linear and angular input
         */
        std::atomic<double> _inputLinX;
        std::atomic<double> _inputLinY;
        std::atomic<double> _inputLinZ;
        std::atomic<double> _inputAngX;
        std::atomic<double> _inputAngY;
        std::atomic<double> _inputAngZ;

        /**
         * Last received data timestamp
         */
        std::atomic<double> _timeLinX;
        std::atomic<double> _timeLinY;
        std::atomic<double> _timeLinZ;
        std::atomic<double> _timeAngX;
        std::atomic<double> _timeAngY;
        std::atomic<double> _timeAngZ;

        /**
         * True if button left or right is pushed
         */
        std::atomic<bool> _isButtonLeft;
        std::atomic<bool> _isButtonRight;
        
        /**
         * Main thread of reader in blocking mode 
         * to update internal state
         */
        void mainReader();

        /**
         * @return time as double either now or 
         * from given timeval structure
         */
        double getTimeNow() const;
        double getTimeFrom(const struct timeval tv) const;

        /**
         * Deadband implementation
         */ 
        double deadband(
            double value, 
            double width, 
            bool isContinuous) const;

};

}

#endif

