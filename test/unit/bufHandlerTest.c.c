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

/************* Test case functions ****************/
void test_bufHandler(void)
{
    BufHandler_t *pTest = NULL;

    /* Pass */
        // 1. Right Argument
    CU_ASSERT_FALSE(bufHandler_init(pTest));

}

/************* Test Runner Code goes here **************/
int main()
{
    CU_pSuite pSuite = NULL;

    /* Initialize CUnit test registry */
    if (CU_initialize_registry() != CUE_SUCCESS)
    {
        return CU_get_error();
    }

    /* Add a suite to the registry */
    pSuite = CU_add_suite("test_suite_bufHandler", init_suite, clean_suite);
    if (NULL == pSuite)
    {
        CU_cleanup_registry();
        return CU_get_error();
    }

    /* Add the tests to the suite */
    if (NULL == CU_add_test(pSuite, "test_bufHandler", test_bufHandler))
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
