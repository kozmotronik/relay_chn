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

## Description

Each relay channel consists of 2 output relays controlled by 2 GPIO pins. The component provides APIs to control these relay pairs while ensuring safe operation, particularly for driving bipolar motors. It prevents short-circuits by automatically managing direction changes with configurable inertia timing.

## Configuration

Configure the component through menuconfig under "Relay Channel Driver Configuration":

- `CONFIG_RELAY_CHN_OPPOSITE_INERTIA_MS`: Time to wait before changing direction (200-1500ms, default: 800ms)
- `CONFIG_RELAY_CHN_COUNT`: Number of relay channels (1-8, default: 1)

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

## License

[MIT License](LICENSE) - Copyright (c) 2025 kozmotronik.
