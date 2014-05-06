#include "CppUTest/TestHarness.h"
#include <string>

extern "C" {
#include <json.h>
}

TEST_GROUP(JSONTestGroup) {
    JsonNode *node;

    void teardown()
    {
        // If node is an array or an object, each children will be deleted by a
        // single call to json_delete on root node.
        json_delete(node);
    }

    void encode_and_compare(JsonNode *node, std::string expected)
    {
        char *encoded;
        encoded = json_encode(node);

        STRCMP_EQUAL(expected.c_str(), encoded);
        free(encoded);
    }
};

TEST(JSONTestGroup, ConstructNumberNodes)
{
    node = json_mknumber(42);
    encode_and_compare(node, "42");
}

TEST(JSONTestGroup, CanConstructEmptyArray)
{
    node = json_mkarray();
    encode_and_compare(node, "[]");
}

TEST(JSONTestGroup, CanPopulateArray)
{
    const int len = 3;
    node = json_mkarray();
    for (int i=0;i<len;i++)
        json_append_element(node, json_mknumber(i));
    encode_and_compare(node, "[0,1,2]");
}

TEST(JSONTestGroup, CanCreateEmptyObject)
{
    node = json_mkobject();
    encode_and_compare(node, "{}");
}

TEST(JSONTestGroup, CanCreatePointObject)
{
    node = json_mkobject();
    json_append_member(node, "x", json_mknumber(42));
    json_append_member(node, "y", json_mknumber(42));
    encode_and_compare(node, "{\"x\":42,\"y\":42}");
}

TEST(JSONTestGroup, CanDecodeEmptyObject)
{
    node = json_decode("{}");
    CHECK_EQUAL(JSON_OBJECT, node->tag);
}

TEST(JSONTestGroup, CanDecodeObject)
{
    node = json_decode("[42, 30]");
    CHECK_EQUAL(42, json_find_element(node, 0)->number_);
    CHECK_EQUAL(30, json_find_element(node, 1)->number_);
}
