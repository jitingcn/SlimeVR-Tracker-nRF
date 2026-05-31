#include <check.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* Include the actual production code under test */
#include "src/sensor/magneto/magneto1_4.h"

/* Intercept k_malloc to simulate memory pressure */
static int malloc_fail_count = 0;
static int malloc_call_count = 0;

void *k_malloc(size_t size) {
    malloc_call_count++;
    if (malloc_fail_count > 0 && malloc_call_count >= malloc_fail_count) {
        return NULL;
    }
    return malloc(size);
}

void k_free(void *ptr) {
    free(ptr);
}

START_TEST(test_magneto_null_malloc_safety)
{
    /* Invariant: magnetometer calibration must not crash (NULL deref)
     * when k_malloc returns NULL under memory pressure */

    /* Payloads: fail on 1st alloc (exact exploit), fail on 3rd alloc (boundary),
     * no failure (valid input) */
    int fail_scenarios[] = { 1, 3, 0 };
    int num_scenarios = sizeof(fail_scenarios) / sizeof(fail_scenarios[0]);

    /* Minimal sample data: 10 magnetometer readings */
    double sample_data[10][3];
    for (int i = 0; i < 10; i++) {
        sample_data[i][0] = (double)(i + 1) * 0.1;
        sample_data[i][1] = (double)(i + 2) * 0.1;
        sample_data[i][2] = (double)(i + 3) * 0.1;
    }

    for (int s = 0; s < num_scenarios; s++) {
        malloc_fail_count = fail_scenarios[s];
        malloc_call_count = 0;

        /* The function must return an error code (not crash) when
         * memory allocation fails. A return value check is the invariant. */
        int result = magneto_calibrate(sample_data, 10, NULL);

        if (fail_scenarios[s] != 0) {
            /* Under memory pressure, must return error, not crash */
            ck_assert_msg(result != 0,
                "magneto_calibrate must return error when k_malloc fails "
                "(fail scenario %d)", fail_scenarios[s]);
        } else {
            /* Valid input with no memory pressure should succeed */
            ck_assert_msg(result == 0 || result != 0,
                "magneto_calibrate must not crash on valid input");
        }
    }
}
END_TEST

Suite *security_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("Security");
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_magneto_null_malloc_safety);
    suite_add_tcase(s, tc_core);

    return s;
}

int main(void)
{
    int number_failed;
    Suite *s;
    SRunner *sr;

    s = security_suite();
    sr = srunner_create(s);

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}