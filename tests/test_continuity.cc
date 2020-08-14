///@file: test_continuity.cc -- part of a googletest suite
#include <gtest/gtest.h>
#include "util_math.h"
typedef drake::trajectories::PiecewisePolynomial<double> PPType;

// Construct the PiecewisePolynomial.
TEST(UtilMath, is_continuous_with_same_plan)
{
    const std::vector<double> breaks = { -1.0, 0.0, 1.0 };
    std::vector<Eigen::MatrixXd> samples(3);
    for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
        samples[i].resize(2, 1);
        samples[i](0, 0) = std::pow(breaks[i], 2);
        samples[i](1, 0) = std::abs(breaks[i]);
    }
    
    std::unique_ptr<PPType> square = std::make_unique<PPType>(PPType::FirstOrderHold(breaks, samples));
    EXPECT_TRUE(utils::is_continuous(square, square, 0, Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2)));
}

TEST(UtilMath, is_pos_continuous)
{
    const std::vector<double> breaks = { -1.0, 0.0, 2.0 };
    std::vector<Eigen::MatrixXd> samples1(3);
    for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
        samples1[i].resize(1, 1);
        samples1[i](0, 0) = std::pow(breaks[i], 2);
    }
    std::unique_ptr<PPType> square = std::make_unique<PPType>(PPType::FirstOrderHold(breaks, samples1));
    
    std::vector<Eigen::MatrixXd> samples2(3);
    for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
        samples2[i].resize(1, 1);
        samples2[i](0, 0) = std::abs(breaks[i]);
    }
    std::unique_ptr<PPType> abs = std::make_unique<PPType>(PPType::FirstOrderHold(breaks, samples2));

    EXPECT_TRUE(utils::is_continuous(square, abs, 1, Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)));
    EXPECT_FALSE(!utils::is_continuous(square, abs, 0.25, Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)));
}

TEST(UtilMath, is_vel_continuous)
{
    const std::vector<double> breaks = { -1.0, 0.0, 1.0 };
    std::vector<Eigen::MatrixXd> samples(3);
    for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
        samples[i].resize(1, 1);
        samples[i](0, 0) = std::pow(breaks[i], 2);
    }
    std::unique_ptr<PPType> square = std::make_unique<PPType>(PPType::FirstOrderHold(breaks, samples));
    
    std::vector<Eigen::MatrixXd> samples_h(3);
    for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
        samples_h[i].resize(1, 1);
        samples_h[i](0, 0) = std::pow(breaks[i], 2) + 100;
    }
    std::unique_ptr<PPType> square_shifted_h = std::make_unique<PPType>(PPType::FirstOrderHold(breaks, samples_h));

    EXPECT_TRUE(!utils::is_continuous(square, square_shifted_h, 0.1, Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)));
    EXPECT_TRUE(!utils::is_continuous(square, square_shifted_h, 0.67, Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)));
}

TEST(UtilMath, is_acc_continuous)
{
    const std::vector<double> breaks = { -1.0, 0.0, 1.0 };
    std::vector<Eigen::MatrixXd> samples(3);
    for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
        samples[i].resize(1, 1);
        samples[i](0, 0) = std::pow(breaks[i], 2);
    }
    std::unique_ptr<PPType> square = std::make_unique<PPType>(PPType::FirstOrderHold(breaks, samples));
    
    std::vector<Eigen::MatrixXd> samples_h(3);
    for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
        samples_h[i].resize(1, 1);
        samples_h[i](0, 0) = breaks[i];
    }
    std::unique_ptr<PPType> line = std::make_unique<PPType>(PPType::FirstOrderHold(breaks, samples_h));

    EXPECT_TRUE(!utils::is_continuous(square, line, -0.1, Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)));
    //EXPECT_TRUE(!utils::is_continuous(square, line, 0.67, Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)));
}

TEST(UtilMath, is_not_continuous)
{
    const std::vector<double> breaks = { -1.0, 0.0, 1.0 };
    std::vector<Eigen::MatrixXd> samples(3);
    for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
        samples[i].resize(1, 1);
        samples[i](0, 0) = std::pow(breaks[i], 2);
    }
    std::unique_ptr<PPType> square = std::make_unique<PPType>(PPType::FirstOrderHold(breaks, samples));
    
    std::vector<Eigen::MatrixXd> samples_h(3);
    for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
        samples_h[i].resize(1, 1);
        samples_h[i](0, 0) = breaks[i];
    }
    std::unique_ptr<PPType> line = std::make_unique<PPType>(PPType::FirstOrderHold(breaks, samples_h));

    EXPECT_TRUE(!utils::is_continuous(square, line, -0.1, Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)));
}
