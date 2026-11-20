#include "cuberl/base/cuberl_types.h"
#include "bitrl/utils/experience_buffer.h"

#include <gtest/gtest.h>
#include <vector>

namespace{

using cuberl::real_t;
using cuberl::uint_t;
using bitrl::utils::ExperienceBuffer;

struct Experience
{
    uint_t item;
};
}


TEST(TestExperienceBuffer, Test_constructor) {

    ExperienceBuffer<Experience> buffer(3);

    ASSERT_TRUE(buffer.empty());
    ASSERT_EQ(buffer.capacity(), static_cast<uint_t>(3));
    ASSERT_EQ(buffer.size(), static_cast<uint_t>(0));
}

TEST(TestExperienceBuffer, Test_append) {

    ExperienceBuffer<Experience> buffer(3);

    ASSERT_TRUE(buffer.empty());
    ASSERT_EQ(buffer.capacity(), static_cast<uint_t>(3));

    Experience exp1{1};
    buffer.append(exp1);

    Experience exp2{2};
    buffer.append(exp2);

    Experience exp3{3};
    buffer.append(exp3);

    ASSERT_EQ(buffer.size(), static_cast<uint_t>(3));
    ASSERT_EQ(buffer[0].item, exp1.item);
    ASSERT_EQ(buffer[1].item, exp2.item);
    ASSERT_EQ(buffer[2].item, exp3.item);
}

TEST(TestExperienceBuffer, Test_replacement) {

    ExperienceBuffer<Experience> buffer(3);

    ASSERT_TRUE(buffer.empty());
    ASSERT_EQ(buffer.capacity(), static_cast<uint_t>(3));

    Experience exp1{1};
    buffer.append(exp1);

    Experience exp2{2};
    buffer.append(exp2);

    Experience exp3{3};
    buffer.append(exp3);

    ASSERT_EQ(buffer.size(), static_cast<uint_t>(3));
    ASSERT_EQ(buffer[0].item, exp1.item);
    ASSERT_EQ(buffer[1].item, exp2.item);
    ASSERT_EQ(buffer[2].item, exp3.item);

    // now add an extra element
    Experience exp4{4};
    buffer.append(exp4);

    ASSERT_EQ(buffer[0].item, exp2.item);
    ASSERT_EQ(buffer[1].item, exp3.item);
    ASSERT_EQ(buffer[2].item, exp4.item);

}


TEST(TestExperienceBuffer, Test_sample) {

    ExperienceBuffer<Experience> buffer(4);

    ASSERT_TRUE(buffer.empty());
    ASSERT_EQ(buffer.capacity(), static_cast<uint_t>(4));

    Experience exp1{1};
    buffer.append(exp1);

    Experience exp2{2};
    buffer.append(exp2);

    Experience exp3{3};
    buffer.append(exp3);
	
	Experience exp4{4};
    buffer.append(exp4);

    std::vector<Experience> batch;
	batch.reserve(2);
	
	// sample from the buffer
	buffer.sample(2, batch, 42);
	
	ASSERT_EQ(batch.size(), 2);

}



