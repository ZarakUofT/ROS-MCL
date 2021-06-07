#include "../math.h"
#include "gtest/gtest.h"

#define ANSI_TXT_GRN "\033[0;32m"
#define ANSI_TXT_MGT "\033[0;35m" //Magenta
#define ANSI_TXT_DFT "\033[0;0m" //Console default
#define GTEST_BOX "[     cout ] "
#define COUT_GTEST ANSI_TXT_GRN << GTEST_BOX //You could add the Default
#define COUT_GTEST_MGT COUT_GTEST << ANSI_TXT_MGT

using namespace Math;

TEST (MathTests, TestLogitCalc){
    EXPECT_EQ(0.0, logit(0.5));
}   

TEST(MathTests, TestNormalDist) {
    uint numElems = 100;
    double sigma = 0.5;
    double val;

    uint inRange = 0;
    uint outRange = 0;

    for (int i = 0; i < numElems; i++){
        val = sample_normal_dist(0.0, sigma);

        if (val <= sigma && val >= -sigma)
            inRange++;
        else
            outRange++;
    }
    ASSERT_LE(outRange, inRange);
}

TEST(MathTests, TestGaussianDist) {
    uint numElems = 100;
    double sigma = 5.0;
    double val;

    uint inRange = 0;
    uint outRange = 0;

    for (int i = 0; i < numElems; i++){
        val = sample_gaussian_dist(sigma);

        if (val <= sigma && val >= -sigma)
            inRange++;
        else
            outRange++;
    }
    ASSERT_LE(outRange, inRange);
}

TEST(MathTests, TestNormalizeAngle) {
    std::cout << COUT_GTEST_MGT << "normalized value = " << normalize(2* M_PI) << ANSI_TXT_DFT << std::endl;
}

TEST(MathTests, TestAngleDiff) {
    double angleDiff = 0.0;
    angleDiff = angle_diff(M_PI / 4, M_PI/8);
    std::cout << COUT_GTEST_MGT << "Angle diff = " << angleDiff << ANSI_TXT_DFT << std::endl;
    ASSERT_NEAR(M_PI/8, angleDiff, 0.01);
}

TEST(MathTests, TestRandomNumbers) {
    std::vector<uint> nums;
    Math::random_integers_in_range((uint)0, (uint)10000, (uint)10, nums);
    ASSERT_NO_FATAL_FAILURE();
}

int main(int argc, char* argv[]){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}