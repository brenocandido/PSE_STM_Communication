#include <CUnit/CUnit.h>
#include <CUnit/Basic.h>
#include "CUnit/Automated.h"
#include "CUnit/Console.h"

#include <stdio.h>
#include <stdlib.h>
#include "bufHandler.h"

/* Test Suite setup and cleanup functions: */
int init_suite(void) { return 0; }
int clean_suite(void) { return 0; }
void setup_suite(void) { }
void teardown_suite(void) { }

/************* Test case functions ****************/
void test_bufHandler_init_null_0(void)
{
    MsgBuffer_t buf[] = {0};   

    CU_ASSERT_FALSE(bufHandler_init(NULL, buf, 1));
}

void test_bufHandler_init_null_1(void)
{
    BufHandler_t bufHandler;

    CU_ASSERT_FALSE(bufHandler_init(&bufHandler, NULL, 1));
}

void test_bufHandler_init_zero_size(void)
{
    MsgBuffer_t buf[] = {0};   
    BufHandler_t bufHandler;

    CU_ASSERT_FALSE(bufHandler_init(&bufHandler, buf, 0));
}

void test_bufHandler_init_right(void)
{
    size_t bufSize = 1;
    MsgBuffer_t buf[] = {0};   
    BufHandler_t bufHandler;

    CU_ASSERT_TRUE(bufHandler_init(&bufHandler, buf, bufSize));
    CU_ASSERT_EQUAL(bufHandler.BUF_SIZE, bufSize);
}

void test_increase_send_index_null(void)
{
    CU_ASSERT_FALSE(bufHandler_increaseSendIndex(NULL));
}

void test_increase_send_index_increase(void)
{
    size_t bufSize = 2;
    MsgBuffer_t buf[bufSize];
    BufHandler_t bufHandler;

    bufHandler_init(&bufHandler, buf, bufSize);

    CU_ASSERT_TRUE(bufHandler_increaseSendIndex(&bufHandler));
    CU_ASSERT_EQUAL(bufHandler.bufSendIndex, 1);
}

void test_increase_send_index_wrap_around(void)
{
    size_t bufSize = 2;
    MsgBuffer_t buf[bufSize];
    BufHandler_t bufHandler;

    bufHandler_init(&bufHandler, buf, bufSize);

    for (int i = 0; i < bufSize; i++)
    {
        bufHandler_increaseSendIndex(&bufHandler);
    }

    CU_ASSERT_EQUAL(bufHandler.bufSendIndex, 0);
}

void test_increase_rcv_index_null(void)
{
    CU_ASSERT_FALSE(bufHandler_increaseRcvIndex(NULL));   
}

void test_increase_rcv_index_increase(void)
{
    size_t bufSize = 2;
    MsgBuffer_t buf[bufSize];
    BufHandler_t bufHandler;

    bufHandler_init(&bufHandler, buf, bufSize);

    CU_ASSERT_TRUE(bufHandler_increaseRcvIndex(&bufHandler));
    CU_ASSERT_EQUAL(bufHandler.bufRcvIndex, 1);
}

void test_increase_rcv_index_wrap_around(void)
{
    size_t bufSize = 2;
    MsgBuffer_t buf[bufSize];
    BufHandler_t bufHandler;

    bufHandler_init(&bufHandler, buf, bufSize);

    for (int i = 0; i < bufSize; i++)
    {
        bufHandler_increaseRcvIndex(&bufHandler);
    }

    CU_ASSERT_EQUAL(bufHandler.bufRcvIndex, 0);
}

void test_empty_null(void)
{
    CU_ASSERT_FALSE(bufHandler_checkEmpty(NULL));   
}

void test_empty(void)
{
    size_t bufSize = 2;
    MsgBuffer_t buf[bufSize];
    BufHandler_t bufHandler;

    bufHandler_init(&bufHandler, buf, bufSize);

    bufHandler_increaseRcvIndex(&bufHandler);
    bufHandler_increaseSendIndex(&bufHandler);

    CU_ASSERT_TRUE(bufHandler_checkEmpty(&bufHandler));   
}

void test_not_empty(void)
{
    size_t bufSize = 2;
    MsgBuffer_t buf[bufSize];
    BufHandler_t bufHandler;

    bufHandler_init(&bufHandler, buf, bufSize);

    bufHandler_increaseRcvIndex(&bufHandler);

    CU_ASSERT_FALSE(bufHandler_checkEmpty(&bufHandler));   
}

CU_TestInfo test_array_init[] = 
{
  { "test_init_null_0", test_bufHandler_init_null_0 },
  { "test_init_null_1", test_bufHandler_init_null_1 },
  { "test_init_zero_size", test_bufHandler_init_zero_size },
  CU_TEST_INFO_NULL,
};

CU_TestInfo test_array_index[] = 
{
  { "test_send_index_null", test_increase_send_index_null },
  { "test_send_index_increase", test_increase_send_index_increase },
  { "test_send_index_wrap_around", test_increase_send_index_wrap_around },
  { "test_rcv_index_null", test_increase_rcv_index_null },
  { "test_rcv_index_increase", test_increase_rcv_index_increase },
  { "test_rcv_index_wrap_around", test_increase_rcv_index_wrap_around },
  CU_TEST_INFO_NULL,
};

CU_TestInfo test_array_empty[] = 
{
  { "test_empty_null", test_empty_null },
  { "test_empty", test_empty },
  { "test_not_empty", test_not_empty },
  CU_TEST_INFO_NULL,
};

CU_SuiteInfo suites[] = 
{
  { "test_suite_init", init_suite, clean_suite, setup_suite, teardown_suite, test_array_init },
  { "test_suite_index", init_suite, clean_suite, setup_suite, teardown_suite, test_array_index },
  { "test_suite_empty", init_suite, clean_suite, setup_suite, teardown_suite, test_array_empty },
  CU_SUITE_INFO_NULL,
};

/************* Test Runner Code goes here **************/
int main()
{
    /* Initialize CUnit test registry */
    if (CU_initialize_registry() != CUE_SUCCESS)
    {
        return CU_get_error();
    }

    CU_ErrorCode error = CU_register_suites(suites);

    if (error != 0)
    {
        CU_cleanup_registry();
        return CU_get_error();
    }

    /* Run all tests using the basic interface */
    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    printf("\n");
    CU_basic_show_failures(CU_get_failure_list());
    printf("\n\n");

    /* Clean up registry and return */
    CU_cleanup_registry();

    return CU_get_error();
}
