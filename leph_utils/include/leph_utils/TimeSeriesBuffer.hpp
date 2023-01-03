#ifndef LEPH_UTILS_TIMESERIESBUFFER_HPP
#define LEPH_UTILS_TIMESERIESBUFFER_HPP

#include <vector>
#include <atomic>
#include <stdexcept>

namespace leph {

/**
 * TimeSeriesBuffer
 *
 * Thread safe rolling buffer 
 * to save timed generic data from 
 * real time thread and then do
 * interpolation.
 * One writer, multiple readers.
 * The writer update the data in place,
 * then push/commit the changes.
 *
 * Template parameters:
 * @param T Type of stored value
 * @param func Function pointer computing the weighted 
 * average between given two values 
 * (must deal with angular values)
 * 
 * The average function takes inputs:
 * @param First weight (in 0:1)
 * @param First value
 * @param Second weight (in 0:1)
 * @param Second value
 */
//TODO XXX Implement lock-free safety 
//TODO XXX using two checks of writer state (abortable ?)
template <typename T, T (*Func)(double, const T&, double, const T&)>
class TimeSeriesBuffer
{
    public:

        /**
         * Stored timed value
         */
        struct Point {
            double time;
            T value;
        };

        /**
         * Initialization and allocation 
         * of the buffer
         *
         * @param bufferLength The total length of pre allocated
         * buffer size (history length)
         * @param initVal Optional buffer initialization value
         */
        TimeSeriesBuffer(size_t bufferLength, const T& initVal = T()) :
            _initVal(initVal),
            _bufferSize(bufferLength),
            _head(0),
            _size(0),
            _data()
        {
            //Data allocation with security margin
            _data.resize(
                _bufferSize + _marginSize, 
                {0.0, _initVal});
        }

        /**
         * @return the constant size of pre allocated buffer
         */
        size_t length() const
        {
            return _bufferSize;
        }

        /**
         * @return the number of currently 
         * stored values (the size can only increase)
         */
        size_t size() const
        {
            return _size.load();
        }

        /**
         * @return the lower and upper contained time.
         * @throw an std::logic_error if the buffer is empty.
         */
        double timeMin() const
        {
            return getPoint(_size.load()-1).time;
        }
        double timeMax() const
        {
            return getPoint(0).time;
        }

        /**
         * Alias to getPoint(0).
         *
         * @return last inserted time and value.
         * Warning: the returned reference should be copied
         * as soon as possible to prevent memory corruption.
         * @throw an std::logic_error if the buffer is empty.
         */
        const Point& lastPoint() const
        {
            return getPoint(0);
        }

        /**
         * Retrieve a stored timed point 
         * directly from index.
         * Index goes in reversed time order:
         * 0: latest inserted point
         * size()-1: earliest inserted point
         *
         * @return Timed point
         * Warning: the returned reference should be copied
         * as soon as possible to prevent memory corruption.
         * @throw an std::logic_error if index is invalid
         */
        const Point& getPoint(size_t index) const
        {
            //Retrieve size and head index
            size_t head = _head.load();
            size_t size = _size.load();
            //Retrieve element
            const Point* ptr = getElement(index, head, size);
            
            //Check index range
            if (ptr == nullptr) {
                throw std::logic_error(
                    "leph::TimeSeriesBuffer:getPoint: invalid index: " 
                    + std::to_string(index));
            } else {
                return *ptr;
            }
        }

        /**
         * Compute and return the interpolation
         * of stored values at given time.
         * Memory copy/allocation.
         * 
         * If the container is empty, default 
         * Point (0.0, T()) is returned.
         * If asked time is out of contained time range,
         * The nearest timed Point is returned.
         */
        Point getInterpolation(double time) const noexcept
        {
            //Retrieve size and head index
            size_t head = _head.load();
            size_t size = _size.load();
            //Empty case
            if (size == 0) {
                return {0.0, _initVal};
            }
            //Retrieve time bounds
            double timeLow = getElement(size-1, head, size)->time;
            double timeUp = getElement(0, head, size)->time;
            //Limit cases
            if (size == 1) {
                return *(getElement(0, head, size));
            } else if (time <= timeLow) {
                return *(getElement(size-1, head, size));
            } else if (time >= timeUp) {
                return *(getElement(0, head, size));
            }
    
            //Bijection search
            size_t indexLow = size - 1;
            size_t indexUp = 0;
            while (indexLow - indexUp > 1) {
                size_t indexMiddle = (indexLow + indexUp)/2;
                if (getElement(indexMiddle, head, size)->time <= time) {
                    indexLow = indexMiddle;
                } else {
                    indexUp = indexMiddle;
                }
            }

            //Compute interpolation weight
            const Point* pointLow = getElement(indexLow, head, size);
            const Point* pointUp = getElement(indexUp, head, size);
            double weightLow = 
                (pointLow->time - time)/(pointLow->time - pointUp->time);
            double weightUp = 
                (time - pointUp->time)/(pointLow->time - pointUp->time);
            //Compute and return interpolation
            return {
                time, 
                Func(
                    weightUp, 
                    pointLow->value,
                    weightLow, 
                    pointUp->value)
                };
        }

        /**
         * RT write access to the next element to be pushed.
         * The writer is expected to update the (reserved) 
         * memory in place and then call push() to commit.
         */
        Point& nextPoint() noexcept
        {
            size_t head = _head.load();
            return _data[head];
        }

        /**
         * RT commit into the buffer the 
         * data point updated by the writer
         * through nextPoint().
         * The nextPoint time must be strictly 
         * increasing within the buffer.
         *
         * @return False if the next time
         * is invalid and the point has not 
         * been committed.
         */
        bool push() noexcept
        {
            //Retrieve internal state
            size_t head = _head.load();
            size_t size = _size.load();
            //Enforce increasing time
            if (size > 0) {
                double lastestTime = getElement(0, head, size)->time;
                if (_data[head].time <= lastestTime) {
                    return false;
                }
            }
            //Update internal state
            _head.store((head + 1) % (_bufferSize + _marginSize));
            if (size < _bufferSize) {
                _size.store(size + 1);
            }

            return true;
        }

    private:

        /**
         * Additional hidden buffer size for
         * concurrent data access correctness
         */
        constexpr static size_t _marginSize = 500;

        /**
         * Default init or zero template value
         */
        const T _initVal;

        /**
         * Pre allocated rolling buffer size
         */
        const size_t _bufferSize;

        /**
         * Current index position of next 
         * element to write on
         */
        std::atomic<size_t> _head;
        
        /**
         * Current number of stored values
         */
        std::atomic<size_t> _size;

        /**
         * Data container of pairs (time, value)
         */
        std::vector<Point> _data;

        /**
         * @return pointer to element at given index 
         * using provided head and size.
         * nullptr is returned on invalid index.
         */
        const Point* getElement(
            size_t index, size_t head, size_t size) const noexcept
        {
            //Check index range
            if (index >= size) {
                return nullptr;
            }

            //Access to data
            size_t indexData = 
                (head - index - 1 + _bufferSize + _marginSize) 
                % (_bufferSize + _marginSize);
            return &(_data[indexData]);
        }
        Point* getElement(
            size_t index, size_t head, size_t size) noexcept
        {
            //Check index range
            if (index >= size) {
                return nullptr;
            }

            //Access to data
            size_t indexData = 
                (head - index - 1 + _bufferSize + _marginSize) 
                % (_bufferSize + _marginSize);
            return &(_data[indexData]);
        }
};

}

#endif

