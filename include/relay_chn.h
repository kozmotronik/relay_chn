#ifndef RELAY_CHN_H
#define RELAY_CHN_H
/**
 * @file relay_chn.h
 * 
 * @author
 * Ismail Sahillioglu <ismailsahillioglu@gmail.com>
 *
 * @date 2025.02.08
 * 
 * @defgroup relay_chn Relay Channel Controller
 * @ingroup components
 * @{
 * One relay channel consists of 2 output relays, hence 2 GPIO pins are required for each relay channel.
 * This module provides an API to control the relay channels, specifically to drive bipolar motors.
 * It also provides APIs to control the direction of the relay channel, bipolar motors in mind.
 * The module also automatically manages the direction change inertia to prevent short-circuiting the motor.
 * The STOP command overrides any other command and clears the pending command if any.
 * 
 * The module internally uses a custom esp event loop to handle relay commands serially to ensure
 * reliability and prevent conflict operations. Also, the esp timer is used to manage the direction change inertia.
 */

#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enumeration for relay channel direction.
 */
enum relay_chn_direction_enum {
    RELAY_CHN_DIRECTION_DEFAULT, ///< Default direction of the relay channel.
    RELAY_CHN_DIRECTION_FLIPPED  ///< Flipped direction of the relay channel.
};

/**
 * @brief Alias for the enum type relay_chn_direction_enum.
 */
typedef enum relay_chn_direction_enum relay_chn_direction_t;

/**
 * @brief Enums that represent the state of a relay channel.
 */
enum relay_chn_state_enum {
    RELAY_CHN_STATE_FREE,       ///< The relay channel is free to run or execute commands.
    RELAY_CHN_STATE_STOPPED,    ///< The relay channel is stopped and not running.
    RELAY_CHN_STATE_FORWARD,    ///< The relay channel is running in the forward direction.
    RELAY_CHN_STATE_REVERSE,    ///< The relay channel is running in the reverse direction.
    RELAY_CHN_STATE_FORWARD_PENDING,    ///< The relay channel is pending to run in the forward direction.
    RELAY_CHN_STATE_REVERSE_PENDING,    ///< The relay channel is pending to run in the reverse direction.
};

/**
 * @brief Alias for the enum type relay_chn_state_enum.
 */
typedef enum relay_chn_state_enum relay_chn_state_t;


/**
 * @brief Create and initialize relay channels.
 *
 * This function initializes the relay channels based on the provided GPIO map.
 *
 * @param gpio_map Pointer to an array of GPIO numbers that correspond to the relay channels.
 * @param gpio_count The number of GPIOs in the gpio_map array.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: General failure
 */
esp_err_t relay_chn_create(const gpio_num_t* gpio_map, uint8_t gpio_count);

/**
 * @brief Get the state of the specified relay channel.
 *
 * This function retrieves the current state of the relay channel identified by the given channel ID.
 *
 * @param chn_id The ID of the relay channel whose state is to be retrieved.
 * @return The current state of the specified relay channel.
 */
relay_chn_state_t relay_chn_get_state(uint8_t chn_id);

/**
 * @brief Get the state string of the specified relay channel.
 *
 * This function returns a string representation of the state of the relay
 * channel identified by the given channel ID.
 *
 * @param chn_id The ID of the relay channel whose state is to be retrieved.
 *               The valid range of channel IDs depends on the specific hardware
 *               and implementation.
 *
 * @return A pointer to a string representing the state of the specified relay
 *         channel. The returned string is managed internally and should not be
 *         modified or freed by the caller.
 */
char *relay_chn_get_state_str(uint8_t chn_id);

/**
 * @brief Runs the relay channel in the forward direction.
 *
 * This function activates the specified relay channel to run in the forward direction.
 *
 * @param chn_id The ID of the relay channel to be activated.
 */
void relay_chn_run_forward(uint8_t chn_id);

/**
 * @brief Runs the relay channel in reverse.
 *
 * This function activates the specified relay channel to run in reverse.
 *
 * @param chn_id The ID of the relay channel to be reversed.
 */
void relay_chn_run_reverse(uint8_t chn_id);

/**
 * @brief Stops the relay channel specified by the channel ID.
 *
 * This function stops the operation of the relay channel identified by the 
 * provided channel ID. It is typically used to turn off or disable the relay 
 * channel.
 *
 * @param chn_id The ID of the relay channel to stop.
 */
void relay_chn_stop(uint8_t chn_id);

/**
 * @brief Flips the direction of the specified relay channel.
 *
 * This function toggles the direction of the relay channel identified by the
 * given channel ID. It is typically used to change the state of the relay
 * from its current direction to the opposite direction.
 *
 * @param chn_id The ID of the relay channel to flip. This should be a valid
 *               channel ID within the range of available relay channels.
 */
void relay_chn_flip_direction(uint8_t chn_id);

/**
 * @brief Get the direction of the specified relay channel.
 *
 * This function retrieves the direction configuration of a relay channel
 * identified by the given channel ID.
 *
 * @param chn_id The ID of the relay channel to query.
 * @return The direction of the specified relay channel as a value of type
 *         relay_chn_direction_t.
 */
relay_chn_direction_t relay_chn_get_direction(uint8_t chn_id);

#ifdef __cplusplus
}
#endif

/// @}

#endif // RELAY_CHN_H