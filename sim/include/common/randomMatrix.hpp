#include <Eigen/Dense>
#include <random>
#include <cstdint>
#include "xoshiro.hpp"

/*
Version 1.0 | George Martin | 20261121

This class is used to fill a pre-allocated matrix with random values, using the Xoshiro random value algorithm. Intended use is as an inherited class of a sensor module.

Values are Gaussian, with a mean of 0 and standard deviation of 1.

Eigen uses std::rand() which is not as random, and not as performant as modern pseudorandom number generators (if we are to trust https://www.learncpp.com/cpp-tutorial/introduction-to-random-number-generation/ which I do)
*/
template <typename Scalar, int Rows, int Cols>
class RandomMatrix
{
public:
    static_assert(Rows > 0 && Cols > 0, "Rows and Cols must be positive compile-time constants.");

    explicit RandomMatrix(Eigen::Matrix<Scalar, Rows, Cols>& target, uint64_t seed)
            : target_(target), rng_(seed) {}

        void update() {
            for (Eigen::Index col = 0; col < target_.cols(); ++col) {
                for (Eigen::Index row = 0; row < target_.rows(); ++row) {
                    target_(row, col) = static_cast<Scalar>(gaussian_(rng_));
                }
            }
        }

private:
    Eigen::Matrix<Scalar, Rows, Cols>& target_;
    XoshiroCpp::Xoshiro256PlusPlus rng_;
    std::normal_distribution<double> gaussian_{0.0,1.0}; // mean = 0.0, standard deviation = 1.0
};