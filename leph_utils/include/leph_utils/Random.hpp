#ifndef LEPH_UTILS_RANDOM_HPP
#define LEPH_UTILS_RANDOM_HPP

#include <random>

namespace leph {

/**
 * Random
 *
 * Simple wrapper around C++11 random interface.
 * Not thread safe. Not real time safe 
 * (random device initialization is system dependent).
 */
class Random
{
    public:

        /**
         * Random engine initialization with
         * a random seed generated from
         * system dependent random device.
         */
        Random() :
            _device(),
            _engine(),
            _seed(_device()),
            _distUniform(0.0, 1.0),
            _distGaussian(0.0, 1.0)
        {
            _engine.seed(_seed);
        }

        /**
         * Random engine initialization from given seed.
         *
         * @param seed Seed to initialized the random engine.
         */
        Random(unsigned int seed) :
            _device(),
            _engine(),
            _seed(seed),
            _distUniform(0.0, 1.0),
            _distGaussian(0.0, 1.0)
        {
            _engine.seed(_seed);
        }

        /**
         * @return used seed
         */
        unsigned int getSeed() const
        {
            return _seed;
        }

        /**
         * Generate random uniformly distributed value.
         *
         * @param min Minimum bound.
         * @param max Maximum bound.
         */
        double randUniform(double min, double max)
        {
            return min + (max-min)*_distUniform(_engine);
        }
        
        /**
         * Generate random Gaussian distributed value.
         *
         * @param mean Gaussian mean
         * @param stddev Gaussian standard deviation
         */
        double randGaussian(double mean, double stddev)
        {
            return mean + stddev*_distGaussian(_engine);
        }

    private:

        /**
         * System dependent random device.
         * Limited and expensive source of entropy.
         */
        std::random_device _device;

        /**
         * Random engine generator.
         * Initialized with a seed and cheap
         * source of (pseudo) entropy.
         */
        std::default_random_engine _engine;
        
        /**
         * Seed used to initiliaed the random engine
         */
        unsigned int _seed;
    
        /**
         * Uniform distribution over [0.0:1.0]
         */
        std::uniform_real_distribution<double> _distUniform;

        /**
         * Normalized Gaussian distribution 
         * (0.0 mean, 1.0 stddev)
         */
        std::normal_distribution<double> _distGaussian;
};

}

#endif

