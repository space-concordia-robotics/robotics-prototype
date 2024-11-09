#include "gtest/gtest.h"
#include "serdes/UbxSerializer.hpp"

// Since we are writting in little endian, we will read from the end of the payload to the start
class UbxSerializerTest : public testing::Test
{
protected:
    UbxSerializer serializer_;
};

TEST_F(UbxSerializerTest, givenSerializerObjectIsInstantiated_whenDefaultConstructorIsUsed_thenPayloadIsEmpty)
{
    EXPECT_EQ(serializer_.getPayloadReadOnly().size(), 0);
}

TEST_F(UbxSerializerTest, givenSerializerObjectIsInstantiated_whenSizeIsGiven_thenPayloadIsCorrectSize)
{
    UbxSerializer serializer(10);
    EXPECT_EQ(serializer.getPayloadReadOnly().capacity(), 10);
}

TEST_F(UbxSerializerTest, givenEmptyPayload_whenWriteU1_thenPayloadContainsHexRepresentation)
{
    // Arrange
    UbxSerializer serializer;
    const auto& payload = serializer.getPayloadReadOnly();
    const int indicesUsedPerWrite = 1;

    //Act
    serializer.writeU1(18);
    serializer.writeU1(1);
    serializer.writeU1(-1);

    // Assert
    ASSERT_EQ(payload.size(), indicesUsedPerWrite * 3);
    EXPECT_EQ(payload[2], 0xFF);
    EXPECT_EQ(payload[1], 0x01);
    EXPECT_EQ(payload[0], 0x12);
}

TEST_F(UbxSerializerTest, givenEmptyPayload_whenWriteU2_thenPayloadContainsHexRepresentation)
{
    // Arrange
    UbxSerializer serializer;
    const auto& payload = serializer.getPayloadReadOnly();
    constexpr int indicesUsedPerWrite = 2;

    //Act
    serializer.writeU2(17); // 0x11 = 17
    serializer.writeU2(1);
    serializer.writeU2(-1); // -1 -> 65535

    // Assert
    ASSERT_EQ(payload.size(), indicesUsedPerWrite * 3);
    EXPECT_EQ(payload[5], 0xFF);
    EXPECT_EQ(payload[4], 0xFF);
    EXPECT_EQ(payload[3], 0x00);
    EXPECT_EQ(payload[2], 0x01);
    EXPECT_EQ(payload[1], 0);
    EXPECT_EQ(payload[0], 0x11);
}

TEST_F(UbxSerializerTest, givenEmptyPayload_whenWriteU4_thenPayloadContainsHexRepresentation)
{
    // Arrange
    UbxSerializer serializer;
    const auto& payload = serializer.getPayloadReadOnly();
    constexpr int indicesUsedPerWrite = 4;

    //Act
    serializer.writeU4(36); // 0x24 = 36
    serializer.writeU4(-1); // 0xFFFFFFFF = -1
    serializer.writeU4(2147483647); // 0x7FFFFFF = 2147483647

    // Assert
    ASSERT_EQ(payload.size(), indicesUsedPerWrite * 3);


    EXPECT_EQ(payload[11], 0x7F);
    EXPECT_EQ(payload[10], 0xFF);
    EXPECT_EQ(payload[9], 0xFF);
    EXPECT_EQ(payload[8], 0xFF);

    EXPECT_EQ(payload[7], 0xFF);
    EXPECT_EQ(payload[6], 0xFF);
    EXPECT_EQ(payload[5], 0xFF);
    EXPECT_EQ(payload[4], 0xFF);

    EXPECT_EQ(payload[3], 0);
    EXPECT_EQ(payload[2], 0);
    EXPECT_EQ(payload[1], 0);
    EXPECT_EQ(payload[0], 0x24);
}
