#include "driver/gpio.h"
#include "unity.h"
#include "unity_test_utils.h"
#include "relay_chn.h"


const gpio_num_t gpio_map[] = {GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_18, GPIO_NUM_19};
const uint8_t gpio_count = sizeof(gpio_map) / sizeof(gpio_map[0]);
const uint8_t relay_chn_count = gpio_count / 2;

TEST_CASE("relay chn inits correctly", "[relay_chn]")
{
    TEST_ESP_OK(relay_chn_create(gpio_map, gpio_count));
}

TEST_CASE("Relay channels run forward and update state", "[relay_chn][forward]")
{
    TEST_ESP_OK(relay_chn_create(gpio_map, gpio_count));
    // Test forward run on all channels
    for (uint8_t i = 0; i < relay_chn_count; i++) {
        TEST_ASSERT_EQUAL(RELAY_CHN_STATE_STOPPED, relay_chn_get_state(i));
        relay_chn_run_forward(i); // Run the channel forward
        TEST_ASSERT_EQUAL(RELAY_CHN_STATE_FORWARD, relay_chn_get_state(i));
        relay_chn_stop(i); // Stop the channel
        TEST_ASSERT_EQUAL(RELAY_CHN_STATE_STOPPED, relay_chn_get_state(i));
        
        relay_chn_flip_direction(i); // Flip the direction
        TEST_ASSERT_EQUAL(RELAY_CHN_DIRECTION_FLIPPED, relay_chn_get_direction(i));
        relay_chn_run_forward(i); // Run the channel forward
        TEST_ASSERT_EQUAL(RELAY_CHN_STATE_FORWARD, relay_chn_get_state(i));
        relay_chn_stop(i); // Stop the channel
        TEST_ASSERT_EQUAL(RELAY_CHN_STATE_STOPPED, relay_chn_get_state(i));
    }
}

TEST_CASE("Relay channels run reverse and update state", "[relay_chn][reverse]")
{
    TEST_ESP_OK(relay_chn_create(gpio_map, gpio_count));
    // Test reverse run on all channels
    for (uint8_t i = 0; i < relay_chn_count; i++) {
        TEST_ASSERT_EQUAL(RELAY_CHN_STATE_STOPPED, relay_chn_get_state(i));
        relay_chn_run_reverse(i); // Run the channel reverse
        TEST_ASSERT_EQUAL(RELAY_CHN_STATE_REVERSE, relay_chn_get_state(i));
        relay_chn_stop(i); // Stop the channel
        TEST_ASSERT_EQUAL(RELAY_CHN_STATE_STOPPED, relay_chn_get_state(i));
        
        relay_chn_flip_direction(i); // Flip the direction
        TEST_ASSERT_EQUAL(RELAY_CHN_DIRECTION_FLIPPED, relay_chn_get_direction(i));
        relay_chn_run_reverse(i); // Run the channel forward
        TEST_ASSERT_EQUAL(RELAY_CHN_STATE_REVERSE, relay_chn_get_state(i));
        relay_chn_stop(i); // Stop the channel
        TEST_ASSERT_EQUAL(RELAY_CHN_STATE_STOPPED, relay_chn_get_state(i));
    }
}

static void check_channels_state_unchanged(void)
{
    for (uint8_t i = 0; i < relay_chn_count; i++) {
        TEST_ASSERT_EQUAL(RELAY_CHN_STATE_STOPPED, relay_chn_get_state(i));
        TEST_ASSERT_EQUAL(RELAY_CHN_DIRECTION_DEFAULT, relay_chn_get_direction(i));
    }
}

TEST_CASE("Relay channels do not change state for invalid channel", "[relay_chn][invalid]")
{
    TEST_ESP_OK(relay_chn_create(gpio_map, gpio_count));
    // Test invalid channel run
    relay_chn_run_forward(relay_chn_count + 1); // Run the channel forward
    check_channels_state_unchanged();
    relay_chn_run_reverse(relay_chn_count + 1); // Run the channel reverse
    check_channels_state_unchanged();
    relay_chn_stop(relay_chn_count + 1); // Stop the channel
    TEST_ASSERT_EQUAL(RELAY_CHN_STATE_STOPPED, relay_chn_get_state(relay_chn_count + 1));
    check_channels_state_unchanged();
    relay_chn_flip_direction(relay_chn_count + 1); // Flip the direction
    check_channels_state_unchanged();
}