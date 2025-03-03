# relay_chn - Relay Channel Controller

An ESP-IDF component for controlling relay channels, specifically designed for driving bipolar motors through relay pairs.

## Features

- Controls multiple relay channel pairs (up to 8 channels)
- Built-in direction change inertia protection
- Automatic command sequencing and timing
- Event-driven architecture for reliable operation
- Forward/Reverse direction control
- Direction flipping capability
- State monitoring and reporting
- Optional sensitivty adjustable tilting feature

## Description

Each relay channel consists of 2 output relays controlled by 2 GPIO pins. The component provides APIs to control these relay pairs while ensuring safe operation, particularly for driving bipolar motors. It prevents short-circuits by automatically managing direction changes with configurable inertia timing.

It also provides an optional tilting interface per channel base. Tilting makes a channel move with a specific pattern moving with small steps at a time. Tilting is specifically designed for controlling some types of curtains that need to be adjusted to let enter specific amount of day light.  
Since it operates on relays, the switching frequency is limited to 10Hz which complies with the most of the general purpose relays' requirements. The minimum frequency is 2Hz and the duty cycle is about 10% in all ranges.  
The module also handles all the required timing between the movement transitions automatically to ensure reliable operation.

## Configuration

Configure the component through menuconfig under "Relay Channel Driver Configuration":

- `CONFIG_RELAY_CHN_OPPOSITE_INERTIA_MS`: Time to wait before changing direction (200-1500ms, default: 800ms)
- `CONFIG_RELAY_CHN_COUNT`: Number of relay channels (1-8, default: 1)
- `CONFIG_RELAY_CHN_ENABLE_TILTING`: Enable tilting interface on all channels. (default: n)

## Installation

1. Copy the component to your project's components directory
2. Add dependency to your project's `idf_component.yml`:

```yaml
dependencies:
  relay_chn:
    version: "^0.1.0"
```

## Usage

### 1. Initialize relay channels

```c
// Define GPIO pins for relay channels
const gpio_num_t gpio_map[] = {GPIO_NUM_4, GPIO_NUM_5};  // One channel example
const uint8_t gpio_count = sizeof(gpio_map) / sizeof(gpio_map[0]);

// Create and initialize relay channels
esp_err_t ret = relay_chn_create(gpio_map, gpio_count);
if (ret != ESP_OK) {
    // Handle error
}
```

### 2. Control relay channels

```c
// Run channel 0 forward
relay_chn_run_forward(0);

// Run channel 0 reverse
relay_chn_run_reverse(0);

// Stop channel 0
relay_chn_stop(0);

// Flip direction of channel 0
relay_chn_flip_direction(0);
```

### 3. Monitor channel state

```c
// Get channel state
relay_chn_state_t state = relay_chn_get_state(0);
char *state_str = relay_chn_get_state_str(0);

// Get channel direction
relay_chn_direction_t direction = relay_chn_get_direction(0);
```

### 4. Tilting Interface (if enabled)

```c
// Assuming CONFIG_RELAY_CHN_ENABLE_TILTING is enabled

// Start tilting automatically (channel 0)
relay_chn_tilt_auto(0);

// Tilt forward (channel 0)
relay_chn_tilt_forward(0);

// Tilt reverse (channel 0)
relay_chn_tilt_reverse(0);

// Stop tilting (channel 0)
relay_chn_tilt_stop(0);

// Set tilting sensitivity (channel 0, sensitivity as percentage)
relay_chn_tilt_sensitivity_set(0, 90);

// Get tilting sensitivity (channel 0, sensitivty as percentage)
uint8_t sensitivity = relay_chn_tilt_sensitivity_get(0);
```

## License

[MIT License](LICENSE) - Copyright (c) 2025 kozmotronik.
