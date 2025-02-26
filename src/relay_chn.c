/**
 * @file relay_chn.c
 * 
 * @author
 * Ismail Sahillioglu <ismailsahillioglu@gmail.com>
 *
 * @date 2025.02.08
 * 
 * @ingroup relay_chn
 * 
 * @brief This file contains the implementation of the relay channel component.
 * @{
 */

#include <stdio.h>
#include <stdlib.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_event_base.h"
#include "esp_event.h"
#include "relay_chn.h"
#include "sdkconfig.h"


#define RELAY_CHN_OPPOSITE_INERTIA_MS CONFIG_RELAY_CHN_OPPOSITE_INERTIA_MS
#define RELAY_CHN_COUNT CONFIG_RELAY_CHN_COUNT
#define RELAY_CHN_ENABLE_TILTING CONFIG_RELAY_CHN_ENABLE_TILTING

static const char *TAG = "relay_chn";

ESP_EVENT_DEFINE_BASE(RELAY_CHN_CMD_EVENT);


/**
 * @brief Enumeration for relay channel commands.
 */
enum relay_chn_cmd_enum {
    RELAY_CHN_CMD_NONE,     ///< No command.
    RELAY_CHN_CMD_STOP,     ///< Stop the relay channel.
    RELAY_CHN_CMD_FORWARD,  ///< Run the relay channel in the forward direction.
    RELAY_CHN_CMD_REVERSE,  ///< Run the relay channel in the reverse direction.
    RELAY_CHN_CMD_FLIP,     ///< Flip the direction of the relay channel.
    RELAY_CHN_CMD_FREE      ///< Free the relay channel.
};

/// @brief Alias for the enum type relay_chn_cmd_enum.
typedef enum relay_chn_cmd_enum relay_chn_cmd_t;

/**
 * @brief Structure to hold runtime information for a relay channel.
 */
typedef struct relay_chn_run_info_type {
    relay_chn_cmd_t last_run_cmd;   ///< The last run command issued on the relay channel; forward or reverse.
    uint32_t last_run_cmd_time_ms;  ///< The time in milliseconds when the last run command was issued.
} relay_chn_run_info_t;

/**
 * @brief Structure to hold the output configuration of a relay channel.
 */
typedef struct relay_chn_output_type {
    gpio_num_t forward_pin; ///< GPIO pin number for the forward direction.
    gpio_num_t reverse_pin; ///< GPIO pin number for the reverse direction.
    relay_chn_direction_t direction; ///< The current direction of the relay channel.
} relay_chn_output_t;

typedef struct relay_chn_type relay_chn_t; // Forward declaration

/**
 * @brief Function pointer type for relay channel command execution functions.
 * @param relay_chn Pointer to the relay channel to execute the command on.
 */
typedef void(*relay_chn_cmd_fn_t)(relay_chn_t*);

#if RELAY_CHN_ENABLE_TILTING == 0

/**
 * @brief Structure to hold the state and configuration of a relay channel.
 */
typedef struct relay_chn_type {
    uint8_t id; ///< The ID of the relay channel.
    relay_chn_state_t state;        ///< The current state of the relay channel.
    relay_chn_run_info_t run_info;  ///< Runtime information of the relay channel.
    relay_chn_output_t output;      ///< Output configuration of the relay channel.
    relay_chn_cmd_t pending_cmd;    ///< The command that is pending to be issued
    esp_timer_handle_t inertia_timer;       ///< Timer to handle the opposite direction inertia time.
} relay_chn_t;

#else

/**
 * @name Tilt Pattern Timing Definitions
 * @{
 * The min and max timing definitions as well as the default timing definitions.
 * These definitions are used to define and adjust the tilt sensitivity.
 */

#define RELAY_CHN_TILT_RUN_MIN_MS 50
#define RELAY_CHN_TILT_RUN_MAX_MS 10
#define RELAY_CHN_TILT_PAUSE_MIN_MS 450
#define RELAY_CHN_TILT_PAUSE_MAX_MS 90

#define RELAY_CHN_TILT_DEFAULT_RUN_MS 15
#define RELAY_CHN_TILT_DEFAULT_PAUSE_MS 150

#define RELAY_CHN_TILT_DEFAULT_SENSITIVITY \
    ( (RELAY_CHN_TILT_DEFAULT_RUN_MS - RELAY_CHN_TILT_RUN_MIN_MS) \
    * 100 / (RELAY_CHN_TILT_RUN_MAX_MS - RELAY_CHN_TILT_RUN_MIN_MS) )
/// @}


ESP_EVENT_DEFINE_BASE(RELAY_CHN_TILT_CMD_EVENT_BASE);

/// @brief Tilt commands.
enum relay_chn_tilt_cmd_enum {
    RELAY_CHN_TILT_CMD_NONE,    ///< No command.
    RELAY_CHN_TILT_CMD_STOP,    ///< Tilt command stop.
    RELAY_CHN_TILT_CMD_FORWARD, ///< Tilt command for forward.
    RELAY_CHN_TILT_CMD_REVERSE  ///< Tilt command for reverse.
};

/// @brief Alias for the enum type relay_chn_tilt_cmd_enum.
typedef enum relay_chn_tilt_cmd_enum relay_chn_tilt_cmd_t;

/// @brief Tilt steps.
enum relay_chn_tilt_step_enum {
    RELAY_CHN_TILT_STEP_NONE,       ///< No step.
    RELAY_CHN_TILT_STEP_PENDING,    ///< Pending step.
    RELAY_CHN_TILT_STEP_MOVE,       ///< Move step. Tilt is driving either for forward or reverse.
    RELAY_CHN_TILT_STEP_PAUSE       ///< Pause step. Tilt is paused.
};

/// @brief Alias for the enum relay_chn_tilt_step_enum.
typedef enum relay_chn_tilt_step_enum relay_chn_tilt_step_t;

/// @brief Tilt timing structure to manage tilt pattern timing.
typedef struct relay_chn_tilt_timing_struct {
    uint8_t sensitivity;    ///< Tilt sensitivity in percentage (%).
    uint32_t move_time_ms;  ///< Move time in milliseconds.
    uint32_t pause_time_ms; ///< Pause time in milliseconds.
} relay_chn_tilt_timing_t;

/// @brief Tilt control structure to manage tilt operations.
typedef struct relay_chn_tilt_control_struct {
    relay_chn_tilt_cmd_t cmd;       ///< The tilt command in process.
    relay_chn_tilt_step_t step;     ///< Current tilt step.
    relay_chn_tilt_timing_t tilt_timing;    ///< Tilt timing structure.
    esp_timer_handle_t tilt_timer;          ///< Tilt timer handle.
} relay_chn_tilt_control_t;

/**
 * @brief Structure to hold the state and configuration of a relay channel.
 */
typedef struct relay_chn_type {
    uint8_t id; ///< The ID of the relay channel.
    relay_chn_state_t state;        ///< The current state of the relay channel.
    relay_chn_run_info_t run_info;  ///< Runtime information of the relay channel.
    relay_chn_output_t output;      ///< Output configuration of the relay channel.
    relay_chn_cmd_t pending_cmd;    ///< The command that is pending to be issued
    esp_timer_handle_t inertia_timer;       ///< Timer to handle the opposite direction inertia time.
    relay_chn_tilt_control_t tilt_control;  ///< Tilt control block.
} relay_chn_t;

static esp_err_t relay_chn_init_tilt_control(relay_chn_t *relay_chn);
static esp_err_t relay_chn_tilt_init(void);

#endif // RELAY_CHN_ENABLE_TILTING


/**
 * @brief Structure to manage the state change listeners. 
 */
struct relay_chn_state_listener_manager_type {
    uint8_t listener_count;                 ///< The number of registered listeners.
    relay_chn_state_listener_t *listeners;  ///< The list that holds references to the registered listeners.
} relay_chn_state_listener_manager;


static relay_chn_t relay_channels[RELAY_CHN_COUNT];
static esp_event_loop_handle_t relay_chn_event_loop;


// Private function declarations
// Event handler for the relay channel command event
static void relay_chn_event_handler(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

/**
 * @brief Check if the provided channel ID is valid.
 * 
 * @param chn_id Channel ID to check.
 * @return true Channel ID is valid.
 * @return false Channel ID is invalid.
 */
static bool relay_chn_is_channel_id_valid(uint8_t chn_id);

/**
 * @brief Dispatches a relay channel command to the event loop.
 * 
 * @param relay_chn The relay channel.
 * @param cmd The command to dispatch.
 */
static void relay_chn_dispatch_cmd(relay_chn_t *relay_chn, relay_chn_cmd_t cmd);

/**
 * @brief Returns the string representation of a relay channel command.
 * 
 * @param cmd The relay channel command.
 * @return char* The string representation of the command.
 */
static char *relay_chn_cmd_str(relay_chn_cmd_t cmd);

/**
 * @brief Timer callback function for relay channel direction change inertia.
 * 
 * This function is called when the opposite direction inertia timer expires. It checks if the channel
 * has a pending command and dispatches it if there is one.
 * 
 * @param arg The channel ID of the relay channel.
 */
static void relay_chn_timer_cb(void* arg)
{
    uint8_t chn_id = *(uint8_t*) arg;
    if (!relay_chn_is_channel_id_valid(chn_id)) {
        ESP_LOGE(TAG, "relay_chn_timer_cb: Invalid relay channel ID!");
        return;
    }
    relay_chn_t* relay_chn = &relay_channels[chn_id];
    // Does channel have a pending command?
    if (relay_chn->pending_cmd != RELAY_CHN_CMD_NONE) {
        relay_chn_dispatch_cmd(relay_chn, relay_chn->pending_cmd);
        relay_chn->pending_cmd = RELAY_CHN_CMD_NONE;
    }
    else {
        ESP_LOGE(TAG, "relay_chn_timer_cb: No pending cmd for relay channel %d!", chn_id);
    }
}

static esp_err_t relay_chn_init_timer(relay_chn_t *relay_chn)
{
    char timer_name[32];
    snprintf(timer_name, sizeof(timer_name), "relay_chn_%d_timer", relay_chn->id);
    esp_timer_create_args_t timer_args = {
        .callback = relay_chn_timer_cb,
        .arg = &relay_chn->id,
        .name = timer_name
    };
    return esp_timer_create(&timer_args, &relay_chn->inertia_timer);
}

/**
 * @brief Check if the provided GPIO pin number is valid for the current device.
 * 
 * @param gpio The GPIO pin number to check.
 * @return true GPIO pin number is valid.
 * @return false GPIO pin number is invalid.
 */
static bool relay_chn_is_gpio_valid(gpio_num_t gpio)
{
    return gpio >= 0 && gpio < GPIO_PIN_COUNT;
}

static esp_err_t relay_chn_create_event_loop()
{
    esp_event_loop_args_t loop_args = {
        .queue_size = RELAY_CHN_COUNT * 8,
        .task_name = "relay_chn_event_loop",
        .task_priority = ESP_TASKD_EVENT_PRIO - 1,
        .task_stack_size = 2048,
        .task_core_id = tskNO_AFFINITY
    };
    esp_err_t ret = esp_event_loop_create(&loop_args, &relay_chn_event_loop);
    ret |= esp_event_handler_register_with(relay_chn_event_loop,
                                            RELAY_CHN_CMD_EVENT,
                                            ESP_EVENT_ANY_ID,
                                            relay_chn_event_handler, NULL);
    return ret;
}

esp_err_t relay_chn_create(const gpio_num_t* gpio_map, uint8_t gpio_count)
{
    // Check if the device's GPIOs are enough for the number of channels
    if (RELAY_CHN_COUNT > (GPIO_PIN_COUNT / 2)) {
        ESP_LOGE(TAG, "Not enough GPIOs for the number of channels!");
        ESP_LOGE(TAG, "Max available num of channels: %d, requested channels: %d", GPIO_PIN_COUNT / 2, RELAY_CHN_COUNT);
        return ESP_ERR_INVALID_ARG;
    }

    // Check if the provided GPIOs correspond to the number of channels
    if (gpio_count != RELAY_CHN_COUNT * 2) {
        ESP_LOGE(TAG, "Invalid number of GPIOs provided: %d", gpio_count);
        ESP_LOGE(TAG, "Expected number of GPIOs: %d", RELAY_CHN_COUNT * 2);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    for (int i = 0; i < RELAY_CHN_COUNT; i++) {
        int gpio_index = i << 1; // gpio_index = i * 2
        gpio_num_t forward_pin = gpio_map[gpio_index];
        gpio_num_t reverse_pin = gpio_map[gpio_index + 1];
        // Check if the GPIOs are valid
        if (!relay_chn_is_gpio_valid(forward_pin)) {
            ESP_LOGE(TAG, "Invalid GPIO pin number: %d", forward_pin);
            return ESP_ERR_INVALID_ARG;
        }
        if (!relay_chn_is_gpio_valid(reverse_pin)) {
            ESP_LOGE(TAG, "Invalid GPIO pin number: %d", reverse_pin);
            return ESP_ERR_INVALID_ARG;
        }
        // Check if the GPIOs are valid

        // Initialize the GPIOs
        ret = gpio_reset_pin(forward_pin);
        ret |= gpio_set_direction(forward_pin, GPIO_MODE_OUTPUT);
        
        ret |= gpio_reset_pin(reverse_pin);
        ret |= gpio_set_direction(reverse_pin, GPIO_MODE_OUTPUT);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize GPIOs relay channel %d!", i);
            return ret;
        }
        // Initialize the GPIOs

        // Initialize the relay channel
        relay_chn_t* relay_chn = &relay_channels[i];
        relay_chn->id = i;
        relay_chn->output.forward_pin = forward_pin;
        relay_chn->output.reverse_pin = reverse_pin;
        relay_chn->output.direction = RELAY_CHN_DIRECTION_DEFAULT;
        relay_chn->state = RELAY_CHN_STATE_FREE;
        relay_chn->pending_cmd = RELAY_CHN_CMD_NONE;
        relay_chn->run_info.last_run_cmd = RELAY_CHN_CMD_NONE;
        ret |= relay_chn_init_timer(relay_chn); // Create direction change inertia timer
#if RELAY_CHN_ENABLE_TILTING == 1
        ret |= relay_chn_init_tilt_control(relay_chn);
#endif
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize relay channel %d!", i);
            return ret;
        }
    }

    // Create relay channel command event loop
    ret |= relay_chn_create_event_loop();

#if RELAY_CHN_ENABLE_TILTING == 1
    // Must call after the event loop is initialized
    ret |= relay_chn_tilt_init(); // Initialize tilt feature
#endif

    // Init the state listener manager
    relay_chn_state_listener_manager.listeners = malloc(sizeof(relay_chn_state_listener_t*));
    if (relay_chn_state_listener_manager.listeners == NULL) {
        ESP_LOGE(TAG, "Failed to initialize memory for the listeners!");
        ret = ESP_ERR_NO_MEM;
    }

    return ret;
}

static int relay_chn_listener_index(relay_chn_state_listener_t listener)
{
    for (int i = 0; i < relay_chn_state_listener_manager.listener_count; i++) {
        if (relay_chn_state_listener_manager.listeners[i] == listener) {
            // This is the listener to unregister. Check if it is in the middle
            ESP_LOGD(TAG, "relay_chn_listener_index: Listener %p; found at index %d.", listener, i);
            return i;
        }
    }
    return -1;
}

esp_err_t relay_chn_register_listener(relay_chn_state_listener_t listener)
{
    if (listener == NULL) {
        ESP_LOGE(TAG, "relay_chn_register_listener: A NULL listener given.");
        return ESP_ERR_INVALID_ARG;
    }

    if (relay_chn_listener_index(listener) > -1) {
        ESP_LOGD(TAG, "relay_chn_register_listener: The listener %p is already registered.", listener);
        return ESP_OK;
    }

    ESP_LOGD(TAG, "relay_chn_register_listener: Register listener: %p", listener);
    relay_chn_state_listener_manager.listeners[relay_chn_state_listener_manager.listener_count] = listener;
    // Update listener count
    relay_chn_state_listener_manager.listener_count++;

    return ESP_OK;
}

void relay_chn_unregister_listener(relay_chn_state_listener_t listener)
{
    if (listener == NULL) {
        ESP_LOGD(TAG, "relay_chn_unregister_listener: A NULL listener given, nothing to do.");
        return;
    }
    // Search the listener in the listeners list and get its index if exists
    int i = relay_chn_listener_index(listener);
    if (i == -1) {
        ESP_LOGD(TAG, "relay_chn_unregister_listener: %p is not registered already.", listener);
        return;
    }

    uint8_t max_index = relay_chn_state_listener_manager.listener_count - 1;
    // Check whether the listener's index is in the middle
    if (i == max_index) {
        // free(&relay_chn_state_listener_manager.listeners[i]);
        relay_chn_state_listener_manager.listeners[i] = NULL;
    }
    else {
        // It is in the middle, so align the next elements in the list and then free the last empty pointer
        // Align the next elements
        uint8_t num_of_elements = max_index - i;
        relay_chn_state_listener_t *pnext = NULL;
        // (i + j): current index; (i + j + 1): next index
        for (uint8_t j = 0; j < num_of_elements; j++) {
            uint8_t current_index = i + j;
            uint8_t next_index = current_index + 1;
            pnext = &relay_chn_state_listener_manager.listeners[next_index];
            relay_chn_state_listener_manager.listeners[current_index] = *pnext;
        }
        // free(&relay_chn_state_listener_manager.listeners[max_index]); // Free the last element
        relay_chn_state_listener_manager.listeners[max_index] = NULL; // Free the last element
    }
    // Decrease listener count
    relay_chn_state_listener_manager.listener_count--;
}

/**
 * @brief Check channel ID validity
 * 
 * @param chn_id Channel ID to check
 * @return true If channel is valid
 * @return false If channel is invalid
 */
static bool relay_chn_is_channel_id_valid(uint8_t chn_id)
{
    bool valid = (chn_id < RELAY_CHN_COUNT) || chn_id == RELAY_CHN_ID_ALL;
    if (!valid) {
        ESP_LOGE(TAG, "Invalid channel ID: %d", chn_id);
    }
    return valid;
}


// Dispatch relay channel command to its event loop
static void relay_chn_dispatch_cmd(relay_chn_t *relay_chn, relay_chn_cmd_t cmd) {
    if (cmd == RELAY_CHN_CMD_NONE) {
        return;
    }
    esp_event_post_to(relay_chn_event_loop,
                        RELAY_CHN_CMD_EVENT,
                        cmd,
                        &relay_chn->id,
                        sizeof(relay_chn->id), portMAX_DELAY);
}

static esp_err_t relay_chn_start_esp_timer_once(esp_timer_handle_t esp_timer, uint32_t time_ms)
{
    esp_err_t ret = esp_timer_start_once(esp_timer, time_ms * 1000);
    if (ret == ESP_ERR_INVALID_STATE) {
        // This timer is already running, stop the timer first
        ret = esp_timer_stop(esp_timer);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            return ret;
        }
        ret = esp_timer_start_once(esp_timer, time_ms * 1000);
    }
    return ret;
}

static void relay_chn_update_state(relay_chn_t *relay_chn, relay_chn_state_t new_state)
{
    relay_chn_state_t old = relay_chn->state;
    relay_chn->state = new_state;

    for (uint8_t i = 0; i < relay_chn_state_listener_manager.listener_count; i++) {
        relay_chn_state_listener_t listener = relay_chn_state_listener_manager.listeners[i];
        if (listener == NULL) {
            relay_chn_state_listener_manager.listener_count -= 1;
            ESP_LOGD(TAG, "relay_chn_update_state: A listener is NULL at index: %u", i);
        }
        // Emit the state change to the listeners
        listener(relay_chn->id, old, new_state);
    }
}

/**
 * @brief The command issuer function.
 * 
 * This function is the deciding logic for issuing a command to a relay channel. It evaluates 
 * the current state of the channel before issuing the command. Then it decides whether to run
 * the command immediately or wait for the opposite inertia time.
 * 
 * The STOP command is an exception, it is always run immediately since it is safe in any case.
 * 
 * Another special consideration is the FLIP command. If the channel is running, the FLIP command
 * is issued after the channel is stopped. If the channel is stopped, the FLIP command is issued
 * immediately.
 * 
 * @param relay_chn The relay channel to issue the command to.
 * @param cmd The command to issue.
 */
static void relay_chn_issue_cmd(relay_chn_t* relay_chn, relay_chn_cmd_t cmd)
{
    if (cmd == RELAY_CHN_CMD_NONE) {
        return;
    }
    
    if (cmd == RELAY_CHN_CMD_STOP) {
        if (relay_chn->state == RELAY_CHN_STATE_STOPPED) {
            return; // Do nothing if already stopped
        }
        // If the command is STOP, issue it immediately
        relay_chn_dispatch_cmd(relay_chn, cmd);
        return;
    }

    // Evaluate the channel's next move depending on its status
    switch (relay_chn->state)
    {
        case RELAY_CHN_STATE_FREE:
            // If the channel is free, run the command immediately
            relay_chn_dispatch_cmd(relay_chn, cmd);
            break;
        
        case RELAY_CHN_STATE_FORWARD_PENDING:
        case RELAY_CHN_STATE_REVERSE_PENDING:
            // The channel is already waiting for the opposite inertia time,
            // so do nothing unless the command is STOP
            if (cmd == RELAY_CHN_CMD_STOP) {
                relay_chn_dispatch_cmd(relay_chn, cmd);
            }
            break;
        
        case RELAY_CHN_STATE_STOPPED:
            if (relay_chn->run_info.last_run_cmd == cmd || relay_chn->run_info.last_run_cmd == RELAY_CHN_CMD_NONE) {
                // Since the state is STOPPED, the inertia timer should be running and must be invalidated
                // with the pending FREE command
                esp_timer_stop(relay_chn->inertia_timer);
                relay_chn->pending_cmd = RELAY_CHN_CMD_NONE;

                // If this is the first run or the last run command is the same as the current command,
                // run the command immediately
                relay_chn_dispatch_cmd(relay_chn, cmd);
            }
            else {
                // If the last run command is different from the current command, calculate the time passed 
                // since the last run command stopped and decide whether to run the command immediately or wait
                uint32_t inertia_time_passed_ms = (uint32_t) (esp_timer_get_time() / 1000) - relay_chn->run_info.last_run_cmd_time_ms;
                uint32_t inertia_time_ms = RELAY_CHN_OPPOSITE_INERTIA_MS - inertia_time_passed_ms;
                if (inertia_time_ms > 0) {
                    relay_chn->pending_cmd = cmd;
                    relay_chn_state_t new_state = cmd == RELAY_CHN_CMD_FORWARD 
                            ? RELAY_CHN_STATE_FORWARD_PENDING : RELAY_CHN_STATE_REVERSE_PENDING;
                    relay_chn_update_state(relay_chn, new_state);
                    // If the time passed is less than the opposite inertia time, wait for the remaining time
                    relay_chn_start_esp_timer_once(relay_chn->inertia_timer, inertia_time_ms);
                }
                else {
                    // If the time passed is more than the opposite inertia time, run the command immediately
                    relay_chn_dispatch_cmd(relay_chn, cmd);
                }
            }
            break;
        
        case RELAY_CHN_STATE_FORWARD:
        case RELAY_CHN_STATE_REVERSE:
            if (cmd == RELAY_CHN_CMD_FLIP) {
                // If the command is FLIP, stop the running channel first, then issue the FLIP command
                relay_chn_dispatch_cmd(relay_chn, RELAY_CHN_CMD_STOP);
                relay_chn_dispatch_cmd(relay_chn, cmd);
                return;
            }
            
            if (relay_chn->run_info.last_run_cmd == cmd) {
                // If the last run command is the same as the current command, do nothing
                return;
            }
            
            // Stop the channel first before the schedule
            relay_chn_dispatch_cmd(relay_chn, RELAY_CHN_CMD_STOP);

            // If the last run command is different from the current command, wait for the opposite inertia time
            relay_chn->pending_cmd = cmd;
            relay_chn_state_t new_state = cmd == RELAY_CHN_CMD_FORWARD 
                    ? RELAY_CHN_STATE_FORWARD_PENDING : RELAY_CHN_STATE_REVERSE_PENDING;
            relay_chn_update_state(relay_chn, new_state);
            relay_chn_start_esp_timer_once(relay_chn->inertia_timer, RELAY_CHN_OPPOSITE_INERTIA_MS);
            break;

        default: ESP_LOGD(TAG, "relay_chn_evaluate: Unknown relay channel state!");
    }
}

/* relay_chn APIs */
relay_chn_state_t relay_chn_get_state(uint8_t chn_id)
{
    if (!relay_chn_is_channel_id_valid(chn_id)) {
        return RELAY_CHN_STATE_STOPPED;
    }
    return relay_channels[chn_id].state;
}

char *relay_chn_get_state_str(uint8_t chn_id)
{
    if (!relay_chn_is_channel_id_valid(chn_id)) {
        return "INVALID";
    }
    return relay_chn_state_str(relay_channels[chn_id].state);
}

static void relay_chn_issue_cmd_on_all_channels(relay_chn_cmd_t cmd)
{
    for (int i = 0; i < RELAY_CHN_COUNT; i++) {
        relay_chn_issue_cmd(&relay_channels[i], cmd);
    }
}

void relay_chn_run_forward(uint8_t chn_id)
{
    if (!relay_chn_is_channel_id_valid(chn_id)) return;

    if (chn_id == RELAY_CHN_ID_ALL) {
        relay_chn_issue_cmd_on_all_channels(RELAY_CHN_CMD_FORWARD);
        return;
    }

    relay_chn_t* relay_chn = &relay_channels[chn_id];
    relay_chn_issue_cmd(relay_chn, RELAY_CHN_CMD_FORWARD);
}

void relay_chn_run_reverse(uint8_t chn_id)
{
    if (!relay_chn_is_channel_id_valid(chn_id)) return;

    if (chn_id == RELAY_CHN_ID_ALL) {
        relay_chn_issue_cmd_on_all_channels(RELAY_CHN_CMD_REVERSE);
        return;
    }

    relay_chn_t* relay_chn = &relay_channels[chn_id];
    relay_chn_issue_cmd(relay_chn, RELAY_CHN_CMD_REVERSE);
}

void relay_chn_stop(uint8_t chn_id)
{
    if (!relay_chn_is_channel_id_valid(chn_id)) return;

    if (chn_id == RELAY_CHN_ID_ALL) {
        relay_chn_issue_cmd_on_all_channels(RELAY_CHN_CMD_STOP);
        return;
    }

    relay_chn_t* relay_chn = &relay_channels[chn_id];
    relay_chn_issue_cmd(relay_chn, RELAY_CHN_CMD_STOP);
}

void relay_chn_flip_direction(uint8_t chn_id)
{
    if (!relay_chn_is_channel_id_valid(chn_id)) return;

    if (chn_id == RELAY_CHN_ID_ALL) {
        relay_chn_issue_cmd_on_all_channels(RELAY_CHN_CMD_FLIP);
        return;
    }

    relay_chn_t* relay_chn = &relay_channels[chn_id];
    relay_chn_issue_cmd(relay_chn, RELAY_CHN_CMD_FLIP);    
}

relay_chn_direction_t relay_chn_get_direction(uint8_t chn_id)
{
    if (!relay_chn_is_channel_id_valid(chn_id)) {
        return RELAY_CHN_DIRECTION_DEFAULT;
    }
    return relay_channels[chn_id].output.direction;
}
/* relay_chn APIs */

static esp_err_t relay_chn_output_stop(relay_chn_t *relay_chn)
{
    esp_err_t ret;
    ret = gpio_set_level(relay_chn->output.forward_pin, 0);
    ret |= gpio_set_level(relay_chn->output.reverse_pin, 0);
    return ret;
}

static esp_err_t relay_chn_output_forward(relay_chn_t *relay_chn)
{
    esp_err_t ret;
    ret = gpio_set_level(relay_chn->output.forward_pin, 1);
    ret |= gpio_set_level(relay_chn->output.reverse_pin, 0);
    return ret;
}

static esp_err_t relay_chn_output_reverse(relay_chn_t *relay_chn)
{
    esp_err_t ret;
    ret = gpio_set_level(relay_chn->output.forward_pin, 0);
    ret |= gpio_set_level(relay_chn->output.reverse_pin, 1);
    return ret;
}

static void relay_chn_execute_stop(relay_chn_t *relay_chn)
{
    if (relay_chn_output_stop(relay_chn) != ESP_OK) {
        ESP_LOGE(TAG, "relay_chn_execute_stop: Failed to output stop for relay channel #%d!", relay_chn->id);
    }
    relay_chn_state_t previous_state = relay_chn->state;
    relay_chn_update_state(relay_chn, RELAY_CHN_STATE_STOPPED);

    // If there is any pending command, cancel it since the STOP command is issued right after it
    relay_chn->pending_cmd = RELAY_CHN_CMD_NONE;
    // Invalidate the channel's timer if it is active
    esp_timer_stop(relay_chn->inertia_timer);

    // Save the last run time only if the previous state was either STATE FORWARD
    // or STATE_REVERSE. Then schedule a free command.
    if (previous_state == RELAY_CHN_STATE_FORWARD || previous_state == RELAY_CHN_STATE_REVERSE) {
        // Record the command's last run time
        relay_chn->run_info.last_run_cmd_time_ms = esp_timer_get_time() / 1000;
        // Schedule a free command for the channel
        relay_chn->pending_cmd = RELAY_CHN_CMD_FREE;
        relay_chn_start_esp_timer_once(relay_chn->inertia_timer, RELAY_CHN_OPPOSITE_INERTIA_MS);
    } else {
        // If the channel was not running one of the run or fwd, issue a free command immediately
        relay_chn_dispatch_cmd(relay_chn, RELAY_CHN_CMD_FREE);
    }
}

static void relay_chn_execute_forward(relay_chn_t *relay_chn)
{
    if (relay_chn_output_forward(relay_chn) != ESP_OK) {
        ESP_LOGE(TAG, "relay_chn_execute_forward: Failed to output forward for relay channel #%d!", relay_chn->id);
        return;
    }
    relay_chn->run_info.last_run_cmd = RELAY_CHN_CMD_FORWARD;
    relay_chn_update_state(relay_chn, RELAY_CHN_STATE_FORWARD);
}

static void relay_chn_execute_reverse(relay_chn_t *relay_chn)
{
    if (relay_chn_output_reverse(relay_chn) != ESP_OK) {
        ESP_LOGE(TAG, "relay_chn_execute_reverse: Failed to output reverse for relay channel #%d!", relay_chn->id);
        return;
    }
    relay_chn->run_info.last_run_cmd = RELAY_CHN_CMD_REVERSE;
    relay_chn_update_state(relay_chn, RELAY_CHN_STATE_REVERSE);
}

static void relay_chn_execute_flip(relay_chn_t *relay_chn)
{
    // Flip the output GPIO pins
    gpio_num_t temp = relay_chn->output.forward_pin;
    relay_chn->output.forward_pin = relay_chn->output.reverse_pin;
    relay_chn->output.reverse_pin = temp;
    // Flip the direction
    relay_chn->output.direction = (relay_chn->output.direction == RELAY_CHN_DIRECTION_DEFAULT) 
                                    ? RELAY_CHN_DIRECTION_FLIPPED 
                                    : RELAY_CHN_DIRECTION_DEFAULT;
    // Set an inertia on the channel to prevent any immediate movement
    relay_chn->pending_cmd = RELAY_CHN_CMD_FREE;
    relay_chn_start_esp_timer_once(relay_chn->inertia_timer, RELAY_CHN_OPPOSITE_INERTIA_MS);
}

void relay_chn_execute_free(relay_chn_t *relay_chn)
{
    relay_chn->pending_cmd = RELAY_CHN_CMD_NONE;
    // Invalidate the channel's timer if it is active
    esp_timer_stop(relay_chn->inertia_timer);
    relay_chn_update_state(relay_chn, RELAY_CHN_STATE_FREE);
}

static void relay_chn_event_handler(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    uint8_t chn_id = *(uint8_t*) event_data;
    if (!relay_chn_is_channel_id_valid(chn_id)) {
        return;
    }
    relay_chn_t* relay_chn = &relay_channels[chn_id];
    ESP_LOGD(TAG, "relay_chn_event_handler: Channel %d, Command: %s", relay_chn->id, relay_chn_cmd_str(event_id));
    switch (event_id) {
        case RELAY_CHN_CMD_STOP:
            relay_chn_execute_stop(relay_chn);
            break;
        case RELAY_CHN_CMD_FORWARD:
            relay_chn_execute_forward(relay_chn);
            break;
        case RELAY_CHN_CMD_REVERSE:
            relay_chn_execute_reverse(relay_chn);
            break;
        case RELAY_CHN_CMD_FLIP:
            relay_chn_execute_flip(relay_chn);
            break;
        case RELAY_CHN_CMD_FREE:
            relay_chn_execute_free(relay_chn);
            break;
        default:
            ESP_LOGD(TAG, "Unknown relay channel command!");
    }
}

static char *relay_chn_cmd_str(relay_chn_cmd_t cmd)
{
    switch (cmd) {
        case RELAY_CHN_CMD_STOP:
            return "STOP";
        case RELAY_CHN_CMD_FORWARD:
            return "FORWARD";
        case RELAY_CHN_CMD_REVERSE:
            return "REVERSE";
        case RELAY_CHN_CMD_FLIP:
            return "FLIP";
        case RELAY_CHN_CMD_FREE:
            return "FREE";
        default:
            return "UNKNOWN";
    }
}

char *relay_chn_state_str(relay_chn_state_t state)
{
    switch (state) {
        case RELAY_CHN_STATE_FREE:
            return "FREE";
        case RELAY_CHN_STATE_STOPPED:
            return "STOPPED";
        case RELAY_CHN_STATE_FORWARD:
            return "FORWARD";
        case RELAY_CHN_STATE_REVERSE:
            return "REVERSE";
        case RELAY_CHN_STATE_FORWARD_PENDING:
            return "FORWARD_PENDING";
        case RELAY_CHN_STATE_REVERSE_PENDING:
            return "REVERSE_PENDING";
#if RELAY_CHN_ENABLE_TILTING == 1
        case RELAY_CHN_STATE_TILT_FORWARD:
            return "TILT_FORWARD";
        case RELAY_CHN_STATE_TILT_REVERSE:
            return "TILT_REVERSE";
#endif
        default:
            return "UNKNOWN";
    }
}

#if RELAY_CHN_ENABLE_TILTING == 1

/**
 * @brief Dispatch a tilt command to the relay channel event loop.
 *
 * @param relay_chn The relay channel to send the command to.
 * @param cmd The tilt command.
 * @return
 *    - ESP_OK on success.
 *    - ESP_ERR_INVALID_ARG if the command is none.
 *    - Other error codes on failure.
 */
static esp_err_t relay_chn_dispatch_tilt_cmd(relay_chn_t *relay_chn, relay_chn_tilt_cmd_t cmd)
{
    if (cmd == RELAY_CHN_TILT_CMD_NONE) return ESP_ERR_INVALID_ARG;
    return esp_event_post_to(relay_chn_event_loop,
                            RELAY_CHN_TILT_CMD_EVENT_BASE,
                            cmd,
                            &relay_chn->id,
                            sizeof(relay_chn->id), portMAX_DELAY);
}

/**
 * @brief Get the required timing before tilting depending on the last run.
 *
 * @param relay_chn the relay channel.
 * @param cmd The tilt command.
 * @return The time that is required in ms.
 */
static uint32_t relay_chn_get_required_timing_before_tilting(relay_chn_t *relay_chn, relay_chn_tilt_cmd_t cmd)
{
    if (cmd == RELAY_CHN_TILT_CMD_FORWARD && relay_chn->run_info.last_run_cmd == RELAY_CHN_CMD_REVERSE)
        return 0;
    else if (cmd == RELAY_CHN_TILT_CMD_REVERSE && relay_chn->run_info.last_run_cmd == RELAY_CHN_CMD_FORWARD)
        return 0;
    
    uint32_t inertia_time_passed_ms = (uint32_t) (esp_timer_get_time() / 1000) - relay_chn->run_info.last_run_cmd_time_ms;
    return RELAY_CHN_OPPOSITE_INERTIA_MS - inertia_time_passed_ms;
}

/**
 * @brief Issue a tilt command to a specific relay channel.
 *
 * @param chn_id The channel ID.
 * @param cmd The tilt command.
 */
static void relay_chn_issue_tilt_cmd(uint8_t chn_id, relay_chn_tilt_cmd_t cmd)
{
    relay_chn_t* relay_chn = &relay_channels[chn_id];

    if (relay_chn->run_info.last_run_cmd == RELAY_CHN_CMD_NONE) {
        // Do not tilt if the channel hasn't been run before
        ESP_LOGD(TAG, "relay_chn_issue_tilt_cmd: Tilt will not be executed since the channel hasn't been run yet");
        return;
    }
    
    if (relay_chn->tilt_control.cmd == cmd) {
        ESP_LOGD(TAG, "relay_chn_issue_tilt_cmd: There is already a tilt command in progress!");
        return;
    }

    // Set the command that will be processed
    relay_chn->tilt_control.cmd = cmd;
    switch (relay_chn->state) {
        case RELAY_CHN_STATE_FREE:
            // Relay channel is free, tilt can be issued immediately
            relay_chn_dispatch_tilt_cmd(relay_chn, cmd);
            break;
        
        case RELAY_CHN_STATE_FORWARD_PENDING:
        case RELAY_CHN_STATE_REVERSE_PENDING:
            // Issue a stop command first so that the timer and pending cmd get cleared
            relay_chn_dispatch_cmd(relay_chn, RELAY_CHN_CMD_STOP);
            // break not put intentionally
        case RELAY_CHN_STATE_STOPPED: {
            // Check if channel needs timing before tilting
            uint32_t req_timing_ms = relay_chn_get_required_timing_before_tilting(relay_chn, cmd);
            if (req_timing_ms == 0) {
                relay_chn_dispatch_tilt_cmd(relay_chn, cmd);
            } else {
                // Channel needs timing before running tilting action, schedule it
                relay_chn->tilt_control.step = RELAY_CHN_TILT_STEP_PENDING;
                relay_chn_start_esp_timer_once(relay_chn->tilt_control.tilt_timer, req_timing_ms);
            }
            break;
        }

        case RELAY_CHN_STATE_FORWARD:
            if (cmd == RELAY_CHN_TILT_CMD_FORWARD) {
                // Stop the running channel first
                relay_chn_dispatch_cmd(relay_chn, RELAY_CHN_CMD_STOP);
                // Schedule for tilting
                relay_chn->tilt_control.step = RELAY_CHN_TILT_STEP_PENDING;
                relay_chn_start_esp_timer_once(relay_chn->tilt_control.tilt_timer, RELAY_CHN_OPPOSITE_INERTIA_MS);
            } else if (cmd == RELAY_CHN_TILT_CMD_REVERSE) {
                // Stop the running channel first
                relay_chn_dispatch_cmd(relay_chn, RELAY_CHN_CMD_STOP);
                // If the tilt cmd is TILT_REVERSE then dispatch it immediately
                relay_chn_dispatch_tilt_cmd(relay_chn, cmd);
            }
            break;

        case RELAY_CHN_STATE_REVERSE:
            if (cmd == RELAY_CHN_TILT_CMD_REVERSE) {
                // Stop the running channel first
                relay_chn_dispatch_cmd(relay_chn, RELAY_CHN_CMD_STOP);
                // Schedule for tilting
                relay_chn->tilt_control.step = RELAY_CHN_TILT_STEP_PENDING;
                relay_chn_start_esp_timer_once(relay_chn->tilt_control.tilt_timer, RELAY_CHN_OPPOSITE_INERTIA_MS);
            } else if (cmd == RELAY_CHN_TILT_CMD_FORWARD) {
                // Stop the running channel first
                relay_chn_dispatch_cmd(relay_chn, RELAY_CHN_CMD_STOP);
                // If the tilt cmd is TILT_FORWARD then dispatch it immediately
                relay_chn_dispatch_tilt_cmd(relay_chn, cmd);
            }
            break;

        default:
            ESP_LOGD(TAG, "relay_chn_issue_tilt_cmd: Unexpected relay channel state: %s!", relay_chn_state_str(relay_chn->state));
    }
}

static void relay_chn_issue_tilt_cmd_on_all_channels(relay_chn_tilt_cmd_t cmd)
{
    for (int i = 0; i < RELAY_CHN_COUNT; i++) {
        relay_chn_issue_tilt_cmd(i, cmd);
    }
}

static void relay_chn_issue_tilt_auto(uint8_t chn_id)
{
    relay_chn_t* relay_chn = &relay_channels[chn_id];
    if (relay_chn->run_info.last_run_cmd == RELAY_CHN_CMD_FORWARD || relay_chn->state == RELAY_CHN_STATE_FORWARD) {
        relay_chn_issue_tilt_cmd(chn_id, RELAY_CHN_TILT_CMD_FORWARD);
    }
    else if (relay_chn->run_info.last_run_cmd == RELAY_CHN_CMD_REVERSE || relay_chn->state == RELAY_CHN_STATE_REVERSE) {
        relay_chn_issue_tilt_cmd(chn_id, RELAY_CHN_TILT_CMD_REVERSE);
    }
}

void relay_chn_tilt_auto(uint8_t chn_id)
{
    if (!relay_chn_is_channel_id_valid(chn_id)) {
        return;
    }

    // Execute for all channels
    if (chn_id == RELAY_CHN_ID_ALL) {
        for (int i = 0; i < RELAY_CHN_COUNT; i++) {
            relay_chn_issue_tilt_auto(i);
        }
        return;
    }
    // Execute for a single channel
    else relay_chn_issue_tilt_auto(chn_id);
}

void relay_chn_tilt_forward(uint8_t chn_id)
{
    if (!relay_chn_is_channel_id_valid(chn_id)) {
        return;
    }
    
    if (chn_id == RELAY_CHN_ID_ALL) relay_chn_issue_tilt_cmd_on_all_channels(RELAY_CHN_TILT_CMD_FORWARD);
    else relay_chn_issue_tilt_cmd(chn_id, RELAY_CHN_TILT_CMD_FORWARD);
}

void relay_chn_tilt_reverse(uint8_t chn_id)
{
    if (!relay_chn_is_channel_id_valid(chn_id)) {
        return;
    }
    
    if (chn_id == RELAY_CHN_ID_ALL) relay_chn_issue_tilt_cmd_on_all_channels(RELAY_CHN_TILT_CMD_REVERSE);
    else relay_chn_issue_tilt_cmd(chn_id, RELAY_CHN_TILT_CMD_REVERSE);
}

static void _relay_chn_tilt_stop(uint8_t chn_id)
{
    relay_chn_t* relay_chn = &relay_channels[chn_id];
    if (relay_chn->tilt_control.cmd != RELAY_CHN_TILT_CMD_NONE) {
        esp_event_post_to(relay_chn_event_loop, 
                          RELAY_CHN_TILT_CMD_EVENT_BASE,
                            RELAY_CHN_TILT_CMD_STOP,
                          &relay_chn->id,
                     sizeof(relay_chn->id), portMAX_DELAY);
    }
}

void relay_chn_tilt_stop(uint8_t chn_id)
{
    if (!relay_chn_is_channel_id_valid(chn_id)) {
        return;
    }
    
    if (chn_id == RELAY_CHN_ID_ALL) {
        for (int i = 0; i < RELAY_CHN_COUNT; i++) {
            _relay_chn_tilt_stop(i);
        }
    }
    else {
        _relay_chn_tilt_stop(chn_id);
    }
}

static void relay_chn_set_tilt_timing_values(relay_chn_tilt_timing_t *tilt_timing,
                                             uint8_t sensitivity,
                                             uint32_t run_time_ms,
                                             uint32_t pause_time_ms)
{
    tilt_timing->sensitivity = sensitivity;
    tilt_timing->move_time_ms = run_time_ms;
    tilt_timing->pause_time_ms = pause_time_ms;
}

static void _relay_chn_tilt_sensitivity_set(relay_chn_t *relay_chn, uint8_t sensitivity)
{
    if (sensitivity >= 100) {
        relay_chn_set_tilt_timing_values(&relay_chn->tilt_control.tilt_timing,
                                              100,
                                              RELAY_CHN_TILT_RUN_MAX_MS,
                                              RELAY_CHN_TILT_PAUSE_MAX_MS);
    }
    else if (sensitivity == 0) {
        relay_chn_set_tilt_timing_values(&relay_chn->tilt_control.tilt_timing,
                                              0,
                                              RELAY_CHN_TILT_RUN_MIN_MS,
                                              RELAY_CHN_TILT_PAUSE_MIN_MS);
    }
    else {
        // Compute the new timing values from the sensitivity percent value by using linear interpolation
        uint32_t tilt_run_time_ms = 0, tilt_pause_time_ms = 0;
        tilt_run_time_ms = RELAY_CHN_TILT_RUN_MIN_MS + (sensitivity * (RELAY_CHN_TILT_RUN_MAX_MS - RELAY_CHN_TILT_RUN_MIN_MS) / 100);
        tilt_pause_time_ms = RELAY_CHN_TILT_PAUSE_MIN_MS + (sensitivity * (RELAY_CHN_TILT_PAUSE_MAX_MS - RELAY_CHN_TILT_PAUSE_MIN_MS) / 100);
        relay_chn_set_tilt_timing_values(&relay_chn->tilt_control.tilt_timing,
                                          sensitivity,
                                          tilt_run_time_ms,
                                          tilt_pause_time_ms);
    }
}

void relay_chn_tilt_sensitivity_set(uint8_t chn_id, uint8_t sensitivity)
{
    if (!relay_chn_is_channel_id_valid(chn_id)) {
        return;
    }
    
    if (chn_id == RELAY_CHN_ID_ALL) {
        for (int i = 0; i < RELAY_CHN_COUNT; i++) {
            _relay_chn_tilt_sensitivity_set(&relay_channels[i], sensitivity);
        }
    }
    else {
        _relay_chn_tilt_sensitivity_set(&relay_channels[chn_id], sensitivity);
    }
}

esp_err_t relay_chn_tilt_sensitivity_get(uint8_t chn_id, uint8_t *sensitivity, size_t length)
{
    if (!relay_chn_is_channel_id_valid(chn_id)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (sensitivity == NULL) {
        ESP_LOGD(TAG, "relay_chn_tilt_sensitivity_get: sensitivity is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (chn_id == RELAY_CHN_ID_ALL) {
        if (length < RELAY_CHN_COUNT) {
            ESP_LOGD(TAG, "relay_chn_tilt_sensitivity_get: length is too short to store all sensitivity values");
            return ESP_ERR_INVALID_ARG;
        }

        for (int i = 0; i < RELAY_CHN_COUNT; i++) {
            sensitivity[i] = relay_channels[i].tilt_control.tilt_timing.sensitivity;
        }
        return ESP_OK;
    }
    *sensitivity = relay_channels[chn_id].tilt_control.tilt_timing.sensitivity;
    return ESP_OK;
}

static void relay_chn_tilt_execute_tilt_stop(relay_chn_t *relay_chn)
{
    // Stop the channel's timer if active
    esp_timer_stop(relay_chn->tilt_control.tilt_timer);
    // Invalidate tilt cmd and step
    relay_chn->tilt_control.cmd = RELAY_CHN_TILT_CMD_NONE;
    relay_chn->tilt_control.step = RELAY_CHN_TILT_STEP_NONE;
    // Stop the channel
    if (relay_chn_output_stop(relay_chn) != ESP_OK) {
        ESP_LOGE(TAG, "relay_chn_tilt_execute_tilt_stop: Failed to output stop for relay channel #%d!", relay_chn->id);
    }
    relay_chn_dispatch_cmd(relay_chn, RELAY_CHN_CMD_STOP);
}

static void relay_chn_tilt_execute_tilt_forward(relay_chn_t *relay_chn)
{
    if (relay_chn_output_reverse(relay_chn) != ESP_OK) {
        ESP_LOGE(TAG, "relay_chn_tilt_execute_tilt_forward: Failed to output reverse for relay channel #%d!", relay_chn->id);
        // Stop tilting because of the error
        relay_chn_dispatch_tilt_cmd(relay_chn, RELAY_CHN_TILT_CMD_STOP);
        return;
    }
    // Set the move time timer
    relay_chn_start_esp_timer_once(relay_chn->tilt_control.tilt_timer,
                            relay_chn->tilt_control.tilt_timing.move_time_ms);
    // Set to pause step
    relay_chn->tilt_control.step = RELAY_CHN_TILT_STEP_PAUSE;
}

static void relay_chn_tilt_execute_tilt_reverse(relay_chn_t *relay_chn)
{
    if (relay_chn_output_forward(relay_chn) != ESP_OK) {
        ESP_LOGE(TAG, "relay_chn_tilt_execute_tilt_reverse: Failed to output forward for relay channel #%d!", relay_chn->id);
        // Stop tilting because of the error
        relay_chn_dispatch_tilt_cmd(relay_chn, RELAY_CHN_TILT_CMD_STOP);
        return;
    }
    // Set the move time timer
    relay_chn_start_esp_timer_once(relay_chn->tilt_control.tilt_timer,
                            relay_chn->tilt_control.tilt_timing.move_time_ms);
    // Set to pause step
    relay_chn->tilt_control.step = RELAY_CHN_TILT_STEP_PAUSE;
}

static void relay_chn_tilt_execute_tilt_pause(relay_chn_t *relay_chn)
{
    // Pause the channel
    if (relay_chn_output_stop(relay_chn) != ESP_OK) {
        ESP_LOGE(TAG, "relay_chn_tilt_execute_tilt_stop: Failed to output stop for relay channel #%d!", relay_chn->id);
        // Stop tilting because of the error
        relay_chn_dispatch_tilt_cmd(relay_chn, RELAY_CHN_TILT_CMD_STOP);
        return;
    }
    // Set the pause time timer
    relay_chn_start_esp_timer_once(relay_chn->tilt_control.tilt_timer,
                            relay_chn->tilt_control.tilt_timing.pause_time_ms);
    // Set to move step
    relay_chn->tilt_control.step = RELAY_CHN_TILT_STEP_MOVE;
}

static void relay_chn_tilt_event_handler(void *handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    uint8_t chn_id = *(uint8_t*) event_data;
    if (!relay_chn_is_channel_id_valid(chn_id)) {
        return;
    }
    relay_chn_t* relay_chn = &relay_channels[chn_id];
    ESP_LOGD(TAG, "relay_chn_event_handler: Channel %d, Command: %s", relay_chn->id, relay_chn_cmd_str(event_id));
    switch(event_id) {
        case RELAY_CHN_TILT_CMD_STOP:
            relay_chn_tilt_execute_tilt_stop(relay_chn);
            break;
        case RELAY_CHN_TILT_CMD_FORWARD:
            relay_chn_tilt_execute_tilt_forward(relay_chn);
            // Update channel state
            relay_chn_update_state(relay_chn, RELAY_CHN_STATE_TILT_FORWARD);
            break;
        case RELAY_CHN_TILT_CMD_REVERSE:
            relay_chn_tilt_execute_tilt_reverse(relay_chn);
            // Update channel state
            relay_chn_update_state(relay_chn, RELAY_CHN_STATE_TILT_REVERSE);
            break;
        default:
            ESP_LOGW(TAG, "Unexpected relay channel tilt command: %ld!", event_id);
    }
}

// Timer callback for the relay_chn_tilt_control_t::tilt_timer
static void relay_chn_tilt_timer_cb(void *arg)
{
    uint8_t chn_id = *(uint8_t*) arg;
    if (!relay_chn_is_channel_id_valid(chn_id)) {
        ESP_LOGE(TAG, "relay_chn_tilt_timer_cb: Invalid relay channel ID!");
        return;
    }
    relay_chn_t* relay_chn = &relay_channels[chn_id];
    
    switch (relay_chn->tilt_control.step)
    {
        case RELAY_CHN_TILT_STEP_MOVE:
            if (relay_chn->tilt_control.cmd == RELAY_CHN_TILT_CMD_FORWARD) {
                relay_chn_tilt_execute_tilt_forward(relay_chn);
            }
            else if (relay_chn->tilt_control.cmd == RELAY_CHN_TILT_CMD_REVERSE) {
                relay_chn_tilt_execute_tilt_reverse(relay_chn);
            }
            break;

        case RELAY_CHN_TILT_STEP_PAUSE:
            relay_chn_tilt_execute_tilt_pause(relay_chn);
            break;

        case RELAY_CHN_TILT_STEP_PENDING:
            // Just dispatch the pending tilt command
            relay_chn_dispatch_tilt_cmd(relay_chn, relay_chn->tilt_control.cmd);
            break;
        
        default:
            break;
    }
}

static esp_err_t relay_chn_init_tilt_control(relay_chn_t *relay_chn)
{
    relay_chn_tilt_control_t *tilt_control = &relay_chn->tilt_control;
    tilt_control->cmd = RELAY_CHN_TILT_CMD_NONE;
    tilt_control->step = RELAY_CHN_TILT_STEP_NONE;
    tilt_control->tilt_timing.sensitivity = RELAY_CHN_TILT_DEFAULT_SENSITIVITY;
    tilt_control->tilt_timing.move_time_ms = RELAY_CHN_TILT_DEFAULT_RUN_MS;
    tilt_control->tilt_timing.pause_time_ms = RELAY_CHN_TILT_DEFAULT_PAUSE_MS;
    
    // Create tilt timer for the channel
    char timer_name[32];
    snprintf(timer_name, sizeof(timer_name), "relay_chn_%2d_tilt_timer", relay_chn->id);
    esp_timer_create_args_t timer_args = {
        .callback = relay_chn_tilt_timer_cb,
        .arg = &relay_chn->id,
        .name = timer_name
    };
    return esp_timer_create(&timer_args, &relay_chn->tilt_control.tilt_timer);
}

// Should call once from relay_chn_init
static esp_err_t relay_chn_tilt_init(void)
{
    esp_err_t ret;
    ret = esp_event_handler_register_with(relay_chn_event_loop, 
                                          RELAY_CHN_TILT_CMD_EVENT_BASE,
                                            ESP_EVENT_ANY_ID,
                                       relay_chn_tilt_event_handler, NULL);
    return ret;
}

#endif // RELAY_CHN_ENABLE_TILTING

/// @}