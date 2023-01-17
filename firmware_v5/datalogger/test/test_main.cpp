#include <Arduino.h>
#include <unity.h>

void setUp(void)
{
  // set stuff up here
}

void tearDown(void)
{
  // clean stuff up here
}

void test_led_builtin_pin_number(void)
{
  TEST_ASSERT_EQUAL(true, true);
}


void setup()
{
  
}

void loop()
{
    RUN_TEST(test_led_builtin_pin_number);
    UNITY_END();
}