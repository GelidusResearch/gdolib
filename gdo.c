/* GdoLib - A library for controlling garage door openers.
 * Copyright (C) 2024  Konnected Inc.
 * Copyright (C) 2025  Gelidus Research Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define __STDC_FORMAT_MACROS 1

#include "gdo_priv.h"
#include "secplus.h"
#include <string.h>

#define __STDC_FORMAT_MACROS 1
#include <inttypes.h>

static const char *TAG = "gdolib";

/***************************** LOCAL FUNCTION DECLARATIONS ****************************/
static void obst_isr_handler(void *arg);
static void gdo_main_task(void *arg);
static void gdo_sync_task(void *arg);
static void gdo_contact_task(void *arg);
static void v1_status_timer_cb(void *arg);
static void motion_detect_timer_cb(void *arg);
static void door_position_sync_timer_cb(void *arg);
static void scheduled_cmd_timer_cb(void *arg);
static void scheduled_event_timer_cb(void *arg);
static void obst_timer_cb(void *arg);
static void obst_test_pulse_timer_cb(void *arg);
static void tof_timer_cb(void *arg);
static void get_paired_devices(gdo_paired_device_type_t type);
static void update_light_state(gdo_light_state_t light_state);
static void update_lock_state(gdo_lock_state_t lock_state);
static void update_learn_state(gdo_learn_state_t learn_state);
static void handle_light_action(gdo_light_action_t light_action);
static void update_obstruction_state(gdo_obstruction_state_t obs_state);
static void update_motion_state(gdo_motion_state_t motion_state);
static void update_motor_state(gdo_motor_state_t motor_state);
static void update_button_state(gdo_button_state_t button_state);
static void update_openings(uint8_t nibble, uint16_t openings);
static void update_ttc(uint16_t ttc_seconds);
static void update_paired_devices(gdo_paired_device_type_t type, uint8_t count);
static void update_battery_state(gdo_battery_state_t battery_state);
static void update_door_state(const gdo_door_state_t door_state);
static void decode_packet(uint8_t *packet);
static void decode_dry_contact(gdo_contact_type_t contact, uint32_t level);
static esp_err_t get_status();
static esp_err_t get_openings();
static esp_err_t send_door_action(gdo_door_action_t action);
static esp_err_t transmit_packet(uint8_t *packet);
static esp_err_t queue_command(gdo_command_t command, uint8_t nibble,
                               uint8_t byte1, uint8_t byte2);
static esp_err_t queue_v1_command(gdo_v1_command_t command);
static esp_err_t schedule_command(gdo_sched_cmd_args_t *cmd_args, uint32_t time_us);
static esp_err_t schedule_event(gdo_event_type_t event, uint32_t time_us);
static esp_err_t gdo_v1_toggle_cmd(gdo_v1_command_t cmd);
static esp_err_t queue_event(gdo_event_t event);
static esp_err_t send_event(gdo_event_type_t event);

/****************************** GLOBAL VARIABLES **********************************/

static gdo_event_callback_t g_event_callback;

static gdo_status_t g_status = {
    .protocol = GDO_PROTOCOL_UNKNOWN,
    .door = GDO_DOOR_STATE_UNKNOWN,
    .light = GDO_LIGHT_STATE_MAX,
    .lock = GDO_LOCK_STATE_MAX,
    .motion = GDO_MOTION_STATE_MAX,
    .motor = GDO_MOTOR_STATE_MAX,
    .button = GDO_BUTTON_STATE_MAX,
    .battery = GDO_BATT_STATE_UNKNOWN,
    .learn = GDO_LEARN_STATE_MAX,
    .obstruction = GDO_OBSTRUCTION_STATE_MAX,
    .paired_devices = {GDO_PAIRED_DEVICE_COUNT_UNKNOWN,
                       GDO_PAIRED_DEVICE_COUNT_UNKNOWN,
                       GDO_PAIRED_DEVICE_COUNT_UNKNOWN,
                       GDO_PAIRED_DEVICE_COUNT_UNKNOWN,
                       GDO_PAIRED_DEVICE_COUNT_UNKNOWN},
    .synced = false,
    .ttc_enabled = false,
    .openings = 0,
    .ttc_seconds = 0,
    .open_ms = 0,
    .close_ms = 0,
    .door_position = -1,
    .door_target = -1,
    .client_id = 0x5AFE,
    .toggle_only = false,
    .last_move_direction = GDO_DOOR_STATE_UNKNOWN,
    .tof_timer_active = false,
    .tof_timer_usecs = 250000,
    .obst_test_pulse_timer_active = false,
    .obst_test_pulse_timer_usecs = 50000,
    .vehicle_parked_threshold = 100,
    .vehicle_parked_threshold_variance = 5,
};

static bool g_protocol_forced;
static gdo_config_t g_config;
static uint32_t g_door_start_moving_ms;
static TaskHandle_t gdo_main_task_handle;
static TaskHandle_t gdo_sync_task_handle;
static TaskHandle_t gdo_contact_task_handle[GDO_CONTACT_MAX - 1];
static QueueHandle_t gdo_tx_queue;
static QueueHandle_t gdo_event_queue;
static esp_timer_handle_t motion_detect_timer;
static esp_timer_handle_t door_position_sync_timer;
static esp_timer_handle_t obst_timer;
static esp_timer_handle_t tof_timer;
static esp_timer_handle_t obst_test_pulse_timer;
static esp_timer_handle_t v1_status_timer;
static void *g_user_cb_arg;
static uint32_t g_tx_delay_ms = 50;
static uint32_t g_ttc_delay_s = 0;
static portMUX_TYPE gdo_spinlock = portMUX_INITIALIZER_UNLOCKED;

const static uint32_t OBST_CHECK_PERIOD = 100; // Milliseconds between checks for obstruction
const static uint32_t OBST_LOWER_LIMIT = 1;    // Number of pulses to consider clear state

/******************************* PUBLIC API FUNCTIONS **********************************/

/**
 * @brief Initializes the GDO driver.
 * @param config The configuration for the GDO driver.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the config is invalid,
 * ESP_ERR_NO_MEM if memory allocation fails, ESP_ERR_INVALID_STATE if the
 * driver is already initialized.
 */
esp_err_t gdo_init(const gdo_config_t *config)
{
  esp_err_t err = ESP_OK;

  if (!config || config->uart_num >= UART_NUM_MAX ||
      config->uart_tx_pin >= GPIO_NUM_MAX ||
      config->uart_rx_pin >= GPIO_NUM_MAX ||
      config->obst_in_pin >= GPIO_NUM_MAX ||
      config->obst_tp_pin >= GPIO_NUM_MAX ||
      config->dc_open_pin >= GPIO_NUM_MAX ||
      config->dc_close_pin >= GPIO_NUM_MAX ||
      config->dc_discrete_open_pin >= GPIO_NUM_MAX ||
      config->dc_discrete_close_pin >= GPIO_NUM_MAX)
  {
    return ESP_ERR_INVALID_ARG;
  }

  if (gdo_tx_queue)
  { // using this as a proxy for the driver being initialized
    return ESP_ERR_INVALID_STATE;
  }

  g_config = *config;

  // Initialize a default dry contact debounce if not set.
  if (g_config.dc_debounce_ms == 0)
    g_config.dc_debounce_ms = GDO_DRY_CONTACT_DEBOUNCE_MS;

  esp_timer_create_args_t timer_args = {.callback = motion_detect_timer_cb,
                                        .arg = NULL,
                                        .dispatch_method = ESP_TIMER_TASK,
                                        .name = "motion_detect_timer"};
  err = esp_timer_create(&timer_args, &motion_detect_timer);
  if (err != ESP_OK)
  {
    return err;
  }

  timer_args.callback = door_position_sync_timer_cb;
  timer_args.arg = NULL;
  timer_args.dispatch_method = ESP_TIMER_TASK;
  timer_args.name = "door_position_sync_timer";
  err = esp_timer_create(&timer_args, &door_position_sync_timer);
  if (err != ESP_OK)
  {
    return err;
  }

  if ((g_config.obst_in_pin > 0) && !g_config.obst_from_status)
  {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << g_config.obst_in_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    static gdo_obstruction_stats_t obst_stats;
    portMUX_INITIALIZE(&obst_stats.mux);
    obst_stats.sleep_micros = 0;

    err = gpio_config(&io_conf);
    if (err != ESP_OK)
    {
      return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
      return err;
    }

    err = gpio_isr_handler_add(g_config.obst_in_pin, obst_isr_handler, (void *)&obst_stats);
    if (err != ESP_OK)
    {
      return err;
    }

    timer_args.callback = obst_timer_cb;
    timer_args.arg = (void *)&obst_stats;
    timer_args.dispatch_method = ESP_TIMER_TASK;
    timer_args.name = "obst_timer";
    err = esp_timer_create(&timer_args, &obst_timer);

    // Check the obstruction pulse counts every 100ms
    err = esp_timer_start_periodic(obst_timer, OBST_CHECK_PERIOD * 1000);

    if (err != ESP_OK)
    {
      return err;
    }
  }
  else if (g_status.protocol == GDO_PROTOCOL_DRY_CONTACT)
  {
    // dry contact protocol requires obstruction sensor input pin
    ESP_LOGE(TAG, "Failed to initialize GDOLIB... dry contact protocol requires obstruction sensor GPIO pin");
    return ESP_FAIL;
  }
  else if (g_config.dc_close_pin > 0 || g_config.dc_open_pin > 0)
  {
    // we need to install gpio isr service for later use
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
      return err;
    }
  }

  if (g_status.tof_timer_active)
  {
    timer_args.callback = tof_timer_cb;
    timer_args.arg = (void *)&g_status;
    timer_args.dispatch_method = ESP_TIMER_TASK;
    timer_args.name = "tof_timer";
    err = esp_timer_create(&timer_args, &tof_timer);
    err = esp_timer_start_periodic(tof_timer, g_status.tof_timer_usecs);
    if (err != ESP_OK)
    {
      return err;
    }
  }

  if (g_config.obst_tp_pin > 0)
  {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << g_config.obst_tp_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_conf); // Configure the GPIO pin
    if (err != ESP_OK)
    {
      return err;
    }
  }

  if (g_status.obst_test_pulse_timer_active)
  {
    timer_args.callback = obst_test_pulse_timer_cb;
    timer_args.arg = (void *)&g_status;
    timer_args.dispatch_method = ESP_TIMER_TASK;
    timer_args.name = "obst_test_pulse_timer";
    err = esp_timer_create(&timer_args, &obst_test_pulse_timer);
    err = esp_timer_start_periodic(obst_test_pulse_timer, g_status.obst_test_pulse_timer_usecs);
    if (err != ESP_OK)
    {
      return err;
    }
  }

  if (g_status.protocol == GDO_PROTOCOL_DRY_CONTACT)
  {
    // Need to setup the pins we will use for dry-contact output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << g_config.uart_tx_pin) |
                        ((g_config.dc_discrete_open_pin) ? (1ULL << g_config.dc_discrete_open_pin) : 0) |
                        ((g_config.dc_discrete_close_pin) ? (1ULL << g_config.dc_discrete_close_pin) : 0),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    err = gpio_config(&io_conf);
    if (err != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to configure dry-contact GPIO output pins");
      return err;
    }
  }
  else
  {
    // dry contact does not require serial comms
    // Begin in secplus protocol v1 as its the easiest to detect.
    uart_config_t uart_config = {
        .baud_rate = 1200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        uart_config.source_clk = UART_SCLK_DEFAULT,
#endif
    };

    err = uart_param_config(g_config.uart_num, &uart_config);
    if (err != ESP_OK)
    {
      return err;
    }

    if (g_config.invert_uart)
    {
      err = uart_set_line_inverse(g_config.uart_num,
                                  UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);
      if (err != ESP_OK)
      {
        return err;
      }
    }

    err = uart_set_pin(g_config.uart_num, g_config.uart_tx_pin,
                       g_config.uart_rx_pin, UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
      return err;
    }
  }

  gdo_tx_queue = xQueueCreate(16, sizeof(gdo_tx_message_t));
  if (!gdo_tx_queue)
  {
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

/**
 * @brief Stops and deletes the GDO driver and resets all state values to
 * defaults.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the driver is not
 * initialized.
 */
esp_err_t gdo_deinit(void)
{
  esp_err_t err = ESP_OK;
  if (!gdo_tx_queue)
  { // using this as a proxy for the driver being initialized
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG, "Shutdown GDOLIB tasks");
  if (v1_status_timer)
  {
    esp_timer_delete(v1_status_timer);
    v1_status_timer = NULL;
  }

  if (gdo_main_task_handle)
  {
    vTaskDelete(gdo_main_task_handle);
    gdo_main_task_handle = NULL;
  }

  if (gdo_sync_task_handle)
  {
    vTaskDelete(gdo_sync_task_handle);
    gdo_sync_task_handle = NULL;
  }

  for (int i = 0; i < GDO_CONTACT_MAX - 1; i++)
  {
    if (gdo_contact_task_handle[i])
    {
      vTaskDelete(gdo_contact_task_handle[i]);
      gdo_contact_task_handle[i] = NULL;
    }
  }

  if (gdo_tx_queue)
  {
    vQueueDelete(gdo_tx_queue);
    gdo_tx_queue = NULL;
  }

  if (gdo_event_queue)
  {
    vQueueDelete(gdo_event_queue);
    gdo_event_queue = NULL;
  }

  if (motion_detect_timer)
  {
    esp_timer_delete(motion_detect_timer);
    motion_detect_timer = NULL;
  }

  if (door_position_sync_timer)
  {
    esp_timer_delete(door_position_sync_timer);
    door_position_sync_timer = NULL;
  }

  if (obst_timer)
  {
    esp_timer_delete(obst_timer);
    obst_timer = NULL;
  }

  if (tof_timer)
  {
    esp_timer_delete(tof_timer);
    tof_timer = NULL;
  }

  if (obst_test_pulse_timer)
  {
    esp_timer_delete(obst_test_pulse_timer);
    obst_test_pulse_timer = NULL;
  }

  g_protocol_forced = false;
  g_status.synced = false;
  g_status.protocol = GDO_PROTOCOL_UNKNOWN;
  g_status.door = GDO_DOOR_STATE_UNKNOWN;
  g_status.light = GDO_LIGHT_STATE_MAX;
  g_status.lock = GDO_LOCK_STATE_MAX;
  g_status.motion = GDO_MOTION_STATE_MAX;
  g_status.motor = GDO_MOTOR_STATE_MAX;
  g_status.button = GDO_BUTTON_STATE_MAX;
  g_status.battery = GDO_BATT_STATE_UNKNOWN;
  g_status.learn = GDO_LEARN_STATE_MAX;
  g_status.obstruction = GDO_OBSTRUCTION_STATE_CLEAR;
  g_status.paired_devices.total_remotes = GDO_PAIRED_DEVICE_COUNT_UNKNOWN;
  g_status.paired_devices.total_keypads = GDO_PAIRED_DEVICE_COUNT_UNKNOWN;
  g_status.paired_devices.total_wall_controls = GDO_PAIRED_DEVICE_COUNT_UNKNOWN;
  g_status.paired_devices.total_accessories = GDO_PAIRED_DEVICE_COUNT_UNKNOWN;
  g_status.paired_devices.total_all = GDO_PAIRED_DEVICE_COUNT_UNKNOWN;
  g_status.openings = 0;
  g_status.ttc_seconds = 0;
  g_status.open_ms = 0;
  g_status.close_ms = 0;
  g_status.door_position = -1;
  g_status.door_target = -1;

  err = gpio_reset_pin(g_config.uart_tx_pin);
  if (err != ESP_OK)
  {
    goto done;
  }

  err = gpio_reset_pin(g_config.uart_rx_pin);
  if (err != ESP_OK)
  {
    goto done;
  }

  err = uart_driver_delete(g_config.uart_num);

done:
  return err;
}

/**
 * @brief Starts the GDO driver and the UART.
 * @param event_callback The callback function to be called when an event
 * occurs.
 * @param user_arg optional user argument to be passed to the callback.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if task creation fails,
 * ESP_ERR_INVALID_STATE if the driver is not initialized.
 */
esp_err_t gdo_start(gdo_event_callback_t event_callback, void *user_arg)
{
  esp_err_t err = ESP_OK;
  if (!gdo_tx_queue)
  { // using this as a proxy for the driver being initialized
    return ESP_ERR_INVALID_STATE;
  }
  g_user_cb_arg = user_arg;

  if (g_status.protocol != GDO_PROTOCOL_DRY_CONTACT)
  {
    // dry contact does not require serial comms
    err = uart_driver_install(g_config.uart_num, RX_BUFFER_SIZE, 0, 32,
                              &gdo_event_queue, 0);
    if (err != ESP_OK)
    {
      return err;
    }

    uart_flush(g_config.uart_num);
  }
  else
  {
    gdo_event_queue = xQueueCreate(16, sizeof(gdo_event_t));
    if (!gdo_event_queue)
    {
      return ESP_ERR_NO_MEM;
    }
  }

  if (g_config.dc_open_pin > 0)
  {
    // Test for > 0 even though zero is valid GPIO pin, but for GDO use case, we require pin to be > 0
    static gdo_contact_t info;
    info.contact = GDO_CONTACT_DOOR_OPEN;
    info.pin = g_config.dc_open_pin;
    // Medium high priority as it needs to handle in real time, pin to CPU 1 so does not share with HomeKit/WiFi/mdns/etc.
#ifdef CONFIG_FREERTOS_UNICORE
    if (xTaskCreate(gdo_contact_task, "gdo_open_ISR", 4096, &info, 16, &gdo_contact_task_handle[GDO_CONTACT_DOOR_OPEN - 1]) != pdPASS)
#else
    if (xTaskCreatePinnedToCore(gdo_contact_task, "gdo_open_ISR", 4096, &info, 15, &gdo_contact_task_handle[GDO_CONTACT_DOOR_OPEN - 1], 1) != pdPASS)
#endif
    {
      return ESP_ERR_NO_MEM;
    }
  }

  if (g_config.dc_close_pin > 0)
  {
    // Test for > 0 even though zero is valid GPIO pin, but for GDO use case, we require pin to be > 0
    static gdo_contact_t info;
    info.contact = GDO_CONTACT_DOOR_CLOSE;
    info.pin = g_config.dc_close_pin;
    // Medium high priority as it needs to handle in real time, pin to CPU 1 so does not share with HomeKit/WiFi/mdns/etc.
#ifdef CONFIG_FREERTOS_UNICORE
    if (xTaskCreate(gdo_contact_task, "gdo_close_ISR", 4096, &info, 16, &gdo_contact_task_handle[GDO_CONTACT_DOOR_CLOSE - 1]) != pdPASS)
#else
    if (xTaskCreatePinnedToCore(gdo_contact_task, "gdo_close_ISR", 4096, &info, 15, &gdo_contact_task_handle[GDO_CONTACT_DOOR_CLOSE - 1], 1) != pdPASS)
#endif
    {
      return ESP_ERR_NO_MEM;
    }
  }

  // Medium high priority as it needs to handle in real time, pin to CPU 1 so does not share with HomeKit/WiFi/mdns/etc.
#ifdef CONFIG_FREERTOS_UNICORE
  if (xTaskCreate(gdo_main_task, "gdo_main_task", 4096, NULL, 15, &gdo_main_task_handle) != pdPASS)
#else
  if (xTaskCreatePinnedToCore(gdo_main_task, "gdo_main_task", 4096, NULL, 15, &gdo_main_task_handle, 1) != pdPASS)
#endif
  {
    return ESP_ERR_NO_MEM;
  }

  err = gdo_sync();
  if (err != ESP_OK)
  {
    return err;
  }

  g_event_callback = event_callback;
  ESP_LOGI(TAG, "GDO Started");
  return err;
}

/**
 * @brief Gets the current status of the GDO.
 * @param status a pointer to the status structure to be filled.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if status is NULL.
 * @note This function is performed in a critical section and should be called
 * with caution.
 */
esp_err_t gdo_get_status(gdo_status_t *status)
{
  if (!status)
  {
    return ESP_ERR_INVALID_ARG;
  }
  portENTER_CRITICAL(&gdo_spinlock);
  *status = g_status;
  portEXIT_CRITICAL(&gdo_spinlock);
  return ESP_OK;
}

/**
 * @brief Starts the task that syncs the state of the GDO with the controller.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if task creation fails,
 * ESP_ERR_NOT_FINISHED if the task is already running, ESP_ERR_INVALID_STATE if
 * the driver is not started or already synced.
 */
esp_err_t gdo_sync(void)
{
  if (g_status.protocol == GDO_PROTOCOL_DRY_CONTACT)
  {
    // dry contact cannot do a sync, door status remains unknown until some activity
    g_status.synced = true;
    return ESP_OK;
  }

  if (!gdo_main_task_handle ||
      g_status.synced)
  { // using this as a proxy for the driver being started
    return ESP_ERR_INVALID_STATE;
  }

  if (!gdo_sync_task_handle)
  {
    // Medium high priority as it needs to handle in real time, pin to CPU 1 so does not share with HomeKit/WiFi/mdns/etc.
#ifdef CONFIG_FREERTOS_UNICORE
    if (xTaskCreate(gdo_sync_task, "gdo_task", 4096, NULL, 15, &gdo_sync_task_handle) != pdPASS)
#else
    if (xTaskCreatePinnedToCore(gdo_sync_task, "gdo_task", 4096, NULL, 15, &gdo_sync_task_handle, 1) != pdPASS)
#endif
    {
      return ESP_ERR_NO_MEM;
    }
  }
  else
  {
    return ESP_ERR_NOT_FINISHED;
  }
  return ESP_OK;
}

/**
 * @brief Opens the door.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if
 * the encoding fails.
 */
esp_err_t gdo_door_open(void)
{

  g_status.door_target = 0;
  if (g_status.door == GDO_DOOR_STATE_OPENING ||
      g_status.door == GDO_DOOR_STATE_OPEN)
  {
    if (g_status.ttc_enabled == true)
      gdo_set_time_to_close(g_ttc_delay_s);
    ESP_LOGI(TAG, "Door already opening or open, ignore request to open door");
    return ESP_OK;
  }

  return send_door_action(GDO_DOOR_ACTION_OPEN);
}

/**
 * @brief Closes the door.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if
 * the encoding fails.
 */
esp_err_t gdo_door_close(void)
{
  g_status.door_target = 10000;
  ESP_LOGI(TAG, "Send door close");
  if (g_status.door == GDO_DOOR_STATE_CLOSING ||
      g_status.door == GDO_DOOR_STATE_CLOSED)
  {
    ESP_LOGI(TAG, "Door already closed or closing, ignore request to close door");
    return ESP_OK;
  }

  return send_door_action(GDO_DOOR_ACTION_CLOSE);
}

/**
 * @brief Stops the door.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if
 * the encoding fails.
 */
esp_err_t gdo_door_stop(void)
{
  if (g_status.door == GDO_DOOR_STATE_OPENING ||
      g_status.door == GDO_DOOR_STATE_CLOSING)
  {
    return send_door_action(GDO_DOOR_ACTION_STOP);
  }
  ESP_LOGI(TAG, "Door is not opening or closing, ignore request to stop door");
  return ESP_OK;
}

/**
 * @brief Toggles the door.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if
 * the encoding fails.
 */
esp_err_t gdo_door_toggle(void)
{
  if (g_status.ttc_enabled == true)
    gdo_set_time_to_close(g_ttc_delay_s);
  return send_door_action(GDO_DOOR_ACTION_TOGGLE);
}

/**
 * @brief Moves the door to a specific target position.
 * @param target The target position to move the door to, 0-10000.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the target is out of range,
 * ESP_ERR_NO_MEM if the queue is full, ESP_ERR_INVALID_STATE if the door
 * position or durations are unknown or the door is moving.
 */
esp_err_t gdo_door_move_to_target(uint32_t target)
{
  esp_err_t err = ESP_OK;

  if (target > 10000)
  {
    return ESP_ERR_INVALID_ARG;
  }

  if (g_status.door_position < 0 || g_status.close_ms == 0 ||
      g_status.open_ms == 0 || g_status.door == GDO_DOOR_STATE_OPENING ||
      g_status.door == GDO_DOOR_STATE_CLOSING)
  {
    ESP_LOGW(TAG, "Unable to move to target, door position or durations are "
                  "unknown or door is moving");
    return ESP_ERR_INVALID_STATE;
  }

  g_status.door_target = target;

  if (g_status.door_target == 0)
  {
    return gdo_door_open();
  }
  else if (g_status.door_target == 10000)
  {
    return gdo_door_close();
  }

  gdo_door_action_t action = GDO_DOOR_ACTION_MAX;
  int delta = g_status.door_position - g_status.door_target;
  float duration_ms = 0.0f;
  if (delta < 0)
  {
    action = GDO_DOOR_ACTION_CLOSE;
    duration_ms = (g_status.close_ms / 10000.f) * -delta;
  }
  else if (delta > 0)
  {
    action = GDO_DOOR_ACTION_OPEN;
    duration_ms = (g_status.open_ms / 10000.f) * delta;
  }
  else
  {
    ESP_LOGD(TAG, "Door is already at target %.2f",
             g_status.door_position / 100.0f);
    return ESP_OK;
  }

  if (duration_ms < 500)
  {
    ESP_LOGW(TAG, "Duration is too short, ignoring move to target");
    return ESP_ERR_INVALID_ARG;
  }

  gdo_sched_cmd_args_t args = {
      .cmd = (uint32_t)GDO_DOOR_ACTION_STOP,
      .door_cmd = true,
  };
  err = schedule_command(&args, duration_ms * 1000);
  if (err != ESP_OK)
  {
    return err;
  }

  err = send_door_action(action);
  return err;
}

/**
 * @brief Turns the light on.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if
 * the encoding fails.
 */
esp_err_t gdo_light_on_check(bool check)
{
  esp_err_t err = ESP_OK;

  if (g_status.protocol & GDO_PROTOCOL_SEC_PLUS_V1)
  {
    // Sec+1.0 is a toggle, so we need to know current light state (if we care)
    if (!check || g_status.light != GDO_LIGHT_STATE_ON)
    {
      err = gdo_v1_toggle_cmd(V1_CMD_TOGGLE_LIGHT_PRESS);
      // No need to explicitly update status, wall panel (or emulation) is doing that constantly
    }
  }
  else
  {
    // Sec+2.0 has discrete on/off commands, no need to check current state
    err = queue_command(GDO_CMD_LIGHT, GDO_LIGHT_ACTION_ON, 0, 0);
    if (check && err == ESP_OK)
    {
      // retrieve door (light) status if requested. This is optional because it is slow
      get_status();
    }
  }

  return err;
}

esp_err_t gdo_light_on()
{
  // Default light_on function will check for current state
  // or retrieve door status after turning on.
  return gdo_light_on_check(true);
}

/**
 * @brief Turns the light off.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if
 * the encoding fails.
 */
esp_err_t gdo_light_off_check(bool check)
{
  esp_err_t err = ESP_OK;

  if (g_status.protocol & GDO_PROTOCOL_SEC_PLUS_V1)
  {
    // Sec+1.0 is a toggle, so we need to know current light state (if we care)
    if (!check || g_status.light != GDO_LIGHT_STATE_OFF)
    {
      err = gdo_v1_toggle_cmd(V1_CMD_TOGGLE_LIGHT_PRESS);
      // No need to explicitly update status, wall panel (or emulation) is doing that constantly
    }
  }
  else
  {
    // Sec+2.0 has discrete on/off commands, no need to check current state
    err = queue_command(GDO_CMD_LIGHT, GDO_LIGHT_ACTION_OFF, 0, 0);
    if (check && err == ESP_OK)
    {
      // retrieve door (light) status if requested. This is optional because it is slow
      get_status();
    }
  }

  return err;
}

esp_err_t gdo_light_off()
{
  // Default light_off function will check for current state
  // or retrieve door status after turning off.
  return gdo_light_off_check(true);
}

/**
 * @brief Toggles the light.
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if current state is unknown,
 * ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if the encoding fails.
 */
esp_err_t gdo_light_toggle(void)
{
  if (g_status.light == GDO_LIGHT_STATE_ON)
  {
    return gdo_light_off();
  }
  else if (g_status.light == GDO_LIGHT_STATE_OFF)
  {
    return gdo_light_on();
  }
  else
  {
    return ESP_ERR_NOT_FOUND;
  }
}

/**
 * @brief Locks the GDO.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if
 * the encoding fails.
 */
esp_err_t gdo_lock(void)
{
  if (g_status.lock == GDO_LOCK_STATE_LOCKED)
  {
    return ESP_OK;
  }

  if (g_status.protocol & GDO_PROTOCOL_SEC_PLUS_V1)
  {
    return gdo_v1_toggle_cmd(V1_CMD_TOGGLE_LOCK_PRESS);
  }
  else
  {
    return queue_command(GDO_CMD_LOCK, GDO_LOCK_ACTION_LOCK, 0, 0);
  }
}

/**
 * @brief Unlocks the GDO.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if
 * the encoding fails.
 */
esp_err_t gdo_unlock(void)
{
  if (g_status.lock == GDO_LOCK_STATE_UNLOCKED)
  {
    return ESP_OK;
  }

  if (g_status.protocol & GDO_PROTOCOL_SEC_PLUS_V1)
  {
    return gdo_v1_toggle_cmd(V1_CMD_TOGGLE_LOCK_PRESS);
  }
  else
  {
    return queue_command(GDO_CMD_LOCK, GDO_LOCK_ACTION_UNLOCK, 0, 0);
  }
}

/**
 * @brief Toggles the lock state of the GDO.
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if current state is unknown,
 * ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if the encoding fails.
 */
esp_err_t gdo_lock_toggle(void)
{
  if (g_status.lock == GDO_LOCK_STATE_LOCKED)
  {
    return gdo_unlock();
  }
  else if (g_status.lock == GDO_LOCK_STATE_UNLOCKED)
  {
    return gdo_lock();
  }
  else
  {
    return ESP_ERR_NOT_FOUND;
  }
}

/**
 * @brief Activates the learn mode on the GDO.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if
 * the encoding fails, ESP_ERR_NOT_SUPPORTED if the protocol is secplus v1.
 */
esp_err_t gdo_activate_learn(void)
{
  if (g_status.protocol != GDO_PROTOCOL_SEC_PLUS_V2)
  {
    return ESP_ERR_NOT_SUPPORTED;
  }

  esp_err_t err = queue_command(GDO_CMD_LEARN, GDO_LEARN_ACTION_ACTIVATE, 0, 0);
  if (err != ESP_OK)
  {
    return err;
  }
  return get_status();
}

/**
 * @brief Deactivates the learn mode on the GDO.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if
 * the encoding fails, ESP_ERR_NOT_SUPPORTED if the protocol is secplus v1.
 */
esp_err_t gdo_deactivate_learn(void)
{
  if (g_status.protocol != GDO_PROTOCOL_SEC_PLUS_V2)
  {
    return ESP_ERR_NOT_SUPPORTED;
  }

  esp_err_t err =
      queue_command(GDO_CMD_LEARN, GDO_LEARN_ACTION_DEACTIVATE, 0, 0);
  if (err != ESP_OK)
  {
    return err;
  }
  return get_status();
}

/**
 * @brief Clears the paired devices from the GDO.
 * @param type The type of paired devices to clear.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the type is invalid,
 * ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if the encoding fails,
 * ESP_ERR_NOT_SUPPORTED if the protocol is secplus v1.
 */
esp_err_t gdo_clear_paired_devices(gdo_paired_device_type_t type)
{
  esp_err_t err = ESP_OK;

  if (g_status.protocol != GDO_PROTOCOL_SEC_PLUS_V2)
  {
    return ESP_ERR_NOT_SUPPORTED;
  }

  if (type >= GDO_PAIRED_DEVICE_TYPE_MAX)
  {
    return ESP_ERR_INVALID_ARG;
  }

  if (type == GDO_PAIRED_DEVICE_TYPE_ALL)
  {
    err = queue_command(GDO_CMD_CLEAR_PAIRED_DEVICES,
                        (GDO_PAIRED_DEVICE_TYPE_REMOTE - 1), 0, 0);
    if (err != ESP_OK)
    {
      return err;
    }

    err = queue_command(GDO_CMD_CLEAR_PAIRED_DEVICES,
                        (GDO_PAIRED_DEVICE_TYPE_KEYPAD - 1), 0, 0);
    if (err != ESP_OK)
    {
      return err;
    }

    err = queue_command(GDO_CMD_CLEAR_PAIRED_DEVICES,
                        (GDO_PAIRED_DEVICE_TYPE_WALL_CONTROL - 1), 0, 0);
    if (err != ESP_OK)
    {
      return err;
    }

    err = queue_command(GDO_CMD_CLEAR_PAIRED_DEVICES,
                        (GDO_PAIRED_DEVICE_TYPE_ACCESSORY - 1), 0, 0);
    if (err != ESP_OK)
    {
      return err;
    }
  }
  else
  {
    err = queue_command(GDO_CMD_CLEAR_PAIRED_DEVICES, (type - 1), 0, 0);
    if (err != ESP_OK)
    {
      return err;
    }
  }

  // return codes from theses do not indicate command success so skip the rc
  // check
  get_status();
  get_paired_devices(type);
  return err;
}

/**
 * @brief Sets the Security+ V2 rolling code.
 * @param rolling_code The rolling code to set.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the GDO is already
 * synced.
 */
esp_err_t gdo_set_rolling_code(uint32_t rolling_code)
{
  if (g_status.synced)
  {
    return ESP_ERR_INVALID_STATE;
  }

  g_status.rolling_code = rolling_code;
  return ESP_OK;
}

/**
 * @brief Sets the Security+ V2 client id.
 * @param client_id The client id to set.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the GDO is already
 * synced.
 */
esp_err_t gdo_set_client_id(uint32_t client_id)
{
  if (g_status.synced)
  {
    return ESP_ERR_INVALID_STATE;
  }

  g_status.client_id = client_id;
  return ESP_OK;
}

/**
 * @brief Sets the protocol to use to communicate with the GDO.
 * @param protocol The protocol to use.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the protocol is invalid,
 * ESP_ERR_INVALID_STATE if the protocol is already set.
 */
esp_err_t gdo_set_protocol(gdo_protocol_type_t protocol)
{
  if (g_status.protocol > 0 && g_status.protocol < GDO_PROTOCOL_MAX)
  {
    return ESP_ERR_INVALID_STATE;
  }

  if (protocol < GDO_PROTOCOL_MAX)
  {
    g_status.protocol = protocol;
    g_protocol_forced = protocol > GDO_PROTOCOL_UNKNOWN;
    return ESP_OK;
  }
  return ESP_ERR_INVALID_ARG;
}

/**
 * @brief Sets the time the door takes to open from fully closed in
 * milliseconds.
 * @param ms The time the door takes to open from fully closed in milliseconds.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the ms is invalid.
 */
esp_err_t gdo_set_open_duration(uint16_t ms)
{
  if (ms < 1000 || ms > 65000)
  {
    return ESP_ERR_INVALID_ARG;
  }

  g_status.open_ms = ms;
  return ESP_OK;
}

/**
 * @brief Sets the time the door takes to close from fully open in milliseconds.
 * @param ms The time the door takes to close from fully open in milliseconds.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the ms is invalid.
 */
esp_err_t gdo_set_close_duration(uint16_t ms)
{
  if (ms < 1000 || ms > 65000)
  {
    return ESP_ERR_INVALID_ARG;
  }

  g_status.close_ms = ms;
  return ESP_OK;
}

/**
 * @brief Sets the minimum time in milliseconds to wait between sending consecutive commands.
 * @param ms The minimum time in milliseconds.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the time is invalid.
 */
esp_err_t gdo_set_min_command_interval(uint32_t ms)
{
  if (ms < 300)
  {
    ESP_LOGE(TAG, "Invalid minimum command interval: %" PRIu32, ms);
    return ESP_ERR_INVALID_ARG;
  }

  g_tx_delay_ms = ms;
  return ESP_OK;
}

/************************************ LOCAL FUNCTIONS ************************************/

/**
 * @brief This task stated by `gdo_sync()` to sync the state of the GDO with the
 * controller.
 * @details This task will query the GDO for the current state of the door, and
 * device information in a loop until timeout or all information is received.
 * Once complete an event of GDO_SYNC_COMPLETE is queued and The task will then
 * delete itself. The status of the sync is stored in the g_status.synced.
 */
static void gdo_sync_task(void *arg)
{
  bool synced = true;

  if (g_status.protocol != GDO_PROTOCOL_SEC_PLUS_V2)
  {
    uart_set_baudrate(g_config.uart_num, 1200);
    uart_set_parity(g_config.uart_num, UART_PARITY_EVEN);
    uart_flush(g_config.uart_num);
    xQueueReset(gdo_event_queue);

    // Delay forever if there is a smart panel connected to allow it to come online and sync before we do anything.
    ulTaskNotifyTake(pdTRUE, g_status.protocol == GDO_PROTOCOL_SEC_PLUS_V1_WITH_SMART_PANEL ? portMAX_DELAY : pdMS_TO_TICKS(2500));

    if (g_status.door == GDO_DOOR_STATE_UNKNOWN)
    {
      ESP_LOGW(TAG, "V1 panel not found, trying emulation");
      esp_timer_create_args_t timer_args = {.callback = v1_status_timer_cb,
                                            .arg = NULL,
                                            .dispatch_method = ESP_TIMER_TASK,
                                            .name = "v1_status_timer"};

      if (esp_timer_create(&timer_args, &v1_status_timer) != ESP_OK)
      {
        ESP_LOGE(TAG, "Failed to create V1 status timer");
        synced = false;
        goto done;
      }
      else
      {
        uart_flush(g_config.uart_num);
        xQueueReset(gdo_event_queue);
        esp_timer_start_periodic(v1_status_timer, 250 * 1000);
      }

      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000));

      if (g_status.door == GDO_DOOR_STATE_UNKNOWN && !g_protocol_forced)
      {
        ESP_LOGW(TAG, "secplus V1 panel emulation failed, trying secplus V2 panel emulation");
        esp_timer_stop(v1_status_timer);
        esp_timer_delete(v1_status_timer);
      }
      else
      {
        goto done;
      }
    }
    else
    {
      ESP_LOGI(TAG, "V1 panel found");
      g_status.protocol = GDO_PROTOCOL_SEC_PLUS_V1_WITH_SMART_PANEL;
      goto done;
    }
  }

  uint32_t timeout = esp_timer_get_time() / 1000 + 5000;
  uint8_t sync_stage = 0;
  g_status.protocol = GDO_PROTOCOL_SEC_PLUS_V2;
  uart_set_baudrate(g_config.uart_num, 9600);
  uart_set_parity(g_config.uart_num, UART_PARITY_DISABLE);
  uart_flush(g_config.uart_num);
  xQueueReset(gdo_event_queue);

  // We send a get openings because if we have a new client ID then the
  // first command may be ignored, and sometime doors will throw away
  // duplicate commands in a row.  So we want the get_status in following
  // loop to actually work.
  get_openings();

  for (;;)
  {
    if ((esp_timer_get_time() / 1000) > timeout)
    {
      synced = false;
      break;
    }

    vTaskDelay(pdMS_TO_TICKS((g_tx_delay_ms * 2 > 500) ? g_tx_delay_ms * 2 : 500));

    if (g_status.door == GDO_DOOR_STATE_UNKNOWN)
    {
      ESP_LOGI(TAG, "SYNC TASK: Getting status");
      get_status();
      continue;
    }
    else if (sync_stage < 1)
    {
      sync_stage = 1;
      timeout += 1000;
    }

    if (g_status.openings == 0)
    {
      ESP_LOGI(TAG, "SYNC TASK: Getting openings");
      get_openings();
      continue;
    }
    else if (sync_stage < 2)
    {
      sync_stage = 2;
      timeout += 1000;
    }

    if (g_status.paired_devices.total_all == GDO_PAIRED_DEVICE_COUNT_UNKNOWN)
    {
      ESP_LOGI(TAG, "SYNC TASK: Getting all paired devices");
      get_paired_devices(GDO_PAIRED_DEVICE_TYPE_ALL);
      continue;
    }
    else if (sync_stage < 3)
    {
      sync_stage = 3;
      timeout += 1000;
    }

    if (g_status.paired_devices.total_remotes == GDO_PAIRED_DEVICE_COUNT_UNKNOWN)
    {
      ESP_LOGI(TAG, "SYNC TASK: Getting remotes");
      get_paired_devices(GDO_PAIRED_DEVICE_TYPE_REMOTE);
      continue;
    }
    else if (sync_stage < 4)
    {
      sync_stage = 4;
      timeout += 1000;
    }

    if (g_status.paired_devices.total_keypads == GDO_PAIRED_DEVICE_COUNT_UNKNOWN)
    {
      ESP_LOGI(TAG, "SYNC TASK: Getting keypads");
      get_paired_devices(GDO_PAIRED_DEVICE_TYPE_KEYPAD);
      continue;
    }
    else if (sync_stage < 5)
    {
      sync_stage = 5;
      timeout += 1000;
    }

    if (g_status.paired_devices.total_wall_controls == GDO_PAIRED_DEVICE_COUNT_UNKNOWN)
    {
      ESP_LOGI(TAG, "SYNC TASK: Getting wall controls");
      get_paired_devices(GDO_PAIRED_DEVICE_TYPE_WALL_CONTROL);
      continue;
    }
    else if (sync_stage < 6)
    {
      sync_stage = 6;
      timeout += 1000;
    }

    if (g_status.paired_devices.total_accessories == GDO_PAIRED_DEVICE_COUNT_UNKNOWN)
    {
      ESP_LOGI(TAG, "SYNC TASK: Getting accessories");
      get_paired_devices(GDO_PAIRED_DEVICE_TYPE_ACCESSORY);
      continue;
    }

    queue_event((gdo_event_t){GDO_EVENT_PAIRED_DEVICES_UPDATE});
    break;
  }

done:
  g_status.synced = synced;
  if (synced)
  {
    if (g_status.protocol & GDO_PROTOCOL_SEC_PLUS_V1)
    {
      g_status.toggle_only = true;
    }
  }
  else if (!g_protocol_forced)
  {
    g_status.protocol = GDO_PROTOCOL_UNKNOWN;
  }
  // We use queue event rather than send event to ensure that the callback
  // function is called from the main thread (same thread as all other callbacks).
  queue_event((gdo_event_t){GDO_EVENT_SYNC_COMPLETE});
  gdo_sync_task_handle = NULL;
  vTaskDelete(NULL);
}

/**
 * @brief Handles the obstruction interrupt and increments the count in the
 * stats struct.
 */
static void IRAM_ATTR obst_isr_handler(void *arg)
{
  gdo_obstruction_stats_t *stats = (gdo_obstruction_stats_t *)arg;
  portENTER_CRITICAL_ISR(&stats->mux);
  ++stats->count;
  portEXIT_CRITICAL_ISR(&stats->mux);
}

/****************************** TIMER CALLBACKS ************************************/

/**
 * @brief Runs every 100ms and checks the count of obstruction interrupts.
 * @details 1 or more interrupts in 100ms is considered clear, 0 with the pin low
 * is asleep, and 0 with the pin high is obstructed. When the obstruction state
 * changes an event of GDO_EVENT_OBST is queued.
 */
static void obst_timer_cb(void *arg)
{
  gdo_obstruction_stats_t *stats = (gdo_obstruction_stats_t *)arg;
  int64_t micros_now = esp_timer_get_time();
  gdo_obstruction_state_t obs_state = g_status.obstruction;

  portENTER_CRITICAL(&stats->mux);
  if (stats->count >= OBST_LOWER_LIMIT)
  {
    // Pulses being received, sensor is working and clear
    obs_state = GDO_OBSTRUCTION_STATE_CLEAR;
    stats->count = 0;
    portEXIT_CRITICAL(&stats->mux);
  }
  else if (stats->count == 0)
  {
    // If there have been no pulses then the pin is steady high or low.
    if (gpio_get_level(g_config.obst_in_pin))
    {
      // sensor is asleep
      stats->sleep_micros = micros_now;
    }
    else
    {
      // if last asleep more than 700ms ago, then there is an obstruction present
      if (micros_now - stats->sleep_micros > 700 * 1000)
      {
        obs_state = GDO_OBSTRUCTION_STATE_OBSTRUCTED;
      }
    }
    portEXIT_CRITICAL(&stats->mux);
  }
  else
  {
    // count is between one and OBST_LOWER_LIMIT, leave state unchanged
    portEXIT_CRITICAL(&stats->mux);
  }

  if (obs_state != g_status.obstruction)
  {
    update_obstruction_state(obs_state);
  }
}

/**
 * @brief obst test pulse timer callback.
 * @details Optional, tiggers on a 50ms interval timer on the preconfigured output pin.
 * The callback is tasked with generating a 50ms square wave pulse on the obst test pin.
 * The pulse is used to test the obstruction sensor
 */
inline static void obst_test_pulse_timer_cb(void *arg)
{
  static bool last_set_value = false;
  last_set_value = !last_set_value;
  gpio_set_level(g_config.obst_tp_pin, last_set_value);
}

/**
 * @brief If we received a motion detection from the GDO it started at timer
 * that will call this after 3 seconds unless reset. This will clear the motion
 * detected state if not reset.
 */
static void motion_detect_timer_cb(void *arg)
{
  update_motion_state(GDO_MOTION_STATE_CLEAR);
}

/**
 * @brief This timer is started when the door starts moving and will queue an
 * event of `GDO_EVENT_DOOR_POSITION_UPDATE` every 500ms until the door stops.
 */
static void door_position_sync_timer_cb(void *arg)
{
  int32_t duration = (esp_timer_get_time() / 1000) - g_door_start_moving_ms;
  float direction_ms = g_status.door == GDO_DOOR_STATE_OPENING
                           ? -g_status.open_ms
                           : g_status.close_ms;
  float delta = (duration / direction_ms) * 10000.0f;

  g_status.door_position += delta;
  if (g_status.door_position < 0)
  {
    g_status.door_position = 0;
  }
  else if (g_status.door_position > 10000)
  {
    g_status.door_position = 10000;
  }

  g_door_start_moving_ms += duration;

  if (g_status.door_position == 0 || g_status.door_position == 10000)
  {
    esp_timer_stop(door_position_sync_timer);
  }
  // We use queue event rather than send event to ensure that the callback
  // function is called from the main thread (same thread as all other callbacks).
  queue_event((gdo_event_t){GDO_EVENT_DOOR_POSITION_UPDATE});
}

/**
 * @brief This timer is started when a command is scheduled to be sent at a
 * specific time. When the timer expires it will send the command to the GDO.
 */
static void scheduled_cmd_timer_cb(void *arg)
{
  gdo_sched_cmd_args_t *args = (gdo_sched_cmd_args_t *)arg;

  if (args->door_cmd)
  {
    send_door_action((gdo_door_action_t)args->cmd);
  }
  else
  {
    queue_command((gdo_command_t)args->cmd, args->nibble, args->byte1,
                  args->byte2);
  }

  // Command timers are one-shot, delete the timer and free the args.
  esp_timer_delete(args->timer);
  free(args);
}

/**
 * @brief This timer is started when the GDO is in secplus v1 protocol and will
 * send a status request every 250ms if no wall panel is detected.
 */
static void v1_status_timer_cb(void *arg)
{
  const char secplus1_cmds[] = {0x35, 0x35, 0x35, 0x35, 0x33, 0x33, 0x53,
                                0x53, 0x38, 0x3A, 0x3A, 0x3A, 0x39, 0x38,
                                0x3A, 0x38, 0x3A, 0x39, 0x3A};
  static uint8_t index = 0;
  queue_v1_command((gdo_v1_command_t)secplus1_cmds[index++]);
  if (index == 18)
  {
    index = 15;
  }
}

/**
 * @brief This timer is started when an event is scheduled to be sent at a specific time.
 * When the timer expires it will send the event to the main event queue.
 */
static void scheduled_event_timer_cb(void *arg)
{
  gdo_sched_evt_args_t *args = (gdo_sched_evt_args_t *)arg;
  if (queue_event((gdo_event_t){args->event}) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to queue scheduled event");
  }

  // Event timers are one-shot, delete the timer and free the args.
  esp_timer_delete(args->timer);
  free(arg);
}

/******************************* COMMAND FUNCTIONS ************************************/

/**
 * @brief Creates a timer to send a command at a specific time.
 * @param cmd_args A structure containing the command and arguments to send to
 * the GDO.
 * @param time_us The time in microseconds to send the command, must be more
 * than 50 microseconds.
 */
static esp_err_t schedule_command(gdo_sched_cmd_args_t *cmd_args,
                                  uint32_t time_us)
{
  esp_err_t err = ESP_OK;
  if (!cmd_args || time_us < 50)
  {
    return ESP_ERR_INVALID_ARG;
  }

  /* Allocate the memory for the args and copy the data into it.
   * This is freed in the scheduled_cmd_timer_cb function.
   */
  gdo_sched_cmd_args_t *args =
      (gdo_sched_cmd_args_t *)malloc(sizeof(gdo_sched_cmd_args_t));
  if (!args)
  {
    return ESP_ERR_NO_MEM;
  }

  *args = *cmd_args;
  esp_timer_create_args_t timer_args = {.callback = scheduled_cmd_timer_cb,
                                        .arg = args,
                                        .dispatch_method = ESP_TIMER_TASK,
                                        .name = "scheduled_cmd_timer"};

  err = esp_timer_create(&timer_args, &args->timer);
  if (err != ESP_OK)
  {
    free(args);
    return err;
  }

  err = esp_timer_start_once(args->timer, time_us);
  if (err != ESP_OK)
  {
    free(args);
  }

  return err;
}

/**
 * @brief Creates a timer to send an event at a specific time.
 * @param event The event to send to the main event queue.
 * @param time_us The time in microseconds to send the event, must be more than 50 microseconds.
 */
static esp_err_t schedule_event(gdo_event_type_t event, uint32_t time_us)
{
  esp_err_t err = ESP_OK;
  if (time_us < 50)
  {
    return ESP_ERR_INVALID_ARG;
  }

  gdo_sched_evt_args_t *event_arg = (gdo_sched_evt_args_t *)malloc(sizeof(gdo_sched_evt_args_t));
  if (!event_arg)
  {
    return ESP_ERR_NO_MEM;
  }

  event_arg->event = event;
  esp_timer_create_args_t timer_args = {
      .callback = scheduled_event_timer_cb,
      .arg = event_arg,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "scheduled_event_timer"};

  err = esp_timer_create(&timer_args, &event_arg->timer);
  if (err != ESP_OK)
  {
    free(event_arg);
    return err;
  }

  err = esp_timer_start_once(event_arg->timer, time_us);
  if (err != ESP_OK)
  {
    free(event_arg);
  }

  return err;
}

/**
 * @brief Secplus V1 only has toggle commands, this function will send a press and schedule a release command
 * @param cmd The command to send to the GDO.
 * @param time_us The time in microseconds to send the command, must be more
 * than 50 microseconds.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full.
 */
static esp_err_t gdo_v1_toggle_cmd(gdo_v1_command_t cmd)
{
  esp_err_t err = queue_v1_command(cmd);
  if (err == ESP_OK)
  {
    gdo_sched_cmd_args_t args = {
        .cmd = (uint32_t)cmd + 1, // release is always 1 higher than press
        .door_cmd = false,
    };
    return schedule_command(&args, g_tx_delay_ms * 1000);
  }

  return err;
}

/**
 * @brief dry-contact toggle a pin. This function will set a pin to level 1 and schedule a reset to zero
 * @param pin The GPIO pin number
 * @return ESP_OK on success, or return code from queue_command()
 */
static esp_err_t gdo_dc_toggle_pin(gpio_num_t pin)
{
  esp_err_t err = queue_command(pin, 1, 0, 0);
  if (err == ESP_OK)
  {
    gdo_sched_cmd_args_t args = {
        .cmd = (uint32_t)pin,
        .door_cmd = false,
        .nibble = 0,
    };
    return schedule_command(&args, 500 * 1000);
  }
  return err;
}

/**
 * @brief Wrapper for sending secplus v1 commands
 * @param command The command to send to the GDO.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full.
 */
static esp_err_t queue_v1_command(gdo_v1_command_t command)
{
  return queue_command((gdo_command_t)command, 0, 0, 0);
}

/**
 * @brief Encodes and queues a command to be sent to the GDO.
 * @param command The command to send to the GDO.
 * @param nibble The nibble of the command.
 * @param byte1 The first byte of the command.
 * @param byte2 The second byte of the command.
 * @details The command is encoded into a packet and queued to be sent to the
 * GDO.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the driver is not
 * initialized, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if the encoding
 * fails.
 */
static esp_err_t queue_command(gdo_command_t command, uint8_t nibble,
                               uint8_t byte1, uint8_t byte2)
{
  if (!gdo_tx_queue)
  {
    return ESP_ERR_INVALID_STATE;
  }

  gdo_tx_message_t message;
  message.cmd = command;
  message.packet = (uint8_t *)malloc(19); // will be freed in the gdo_main_task
  message.sent_ms = esp_timer_get_time() / 1000;

  // if we are here without a protocol defined then V1 testing failed, proceed
  // with v2
  if (g_status.protocol == GDO_PROTOCOL_SEC_PLUS_V2)
  {
    uint64_t cmd = command;
    uint64_t fixed = ((cmd & ~0xff) << 24) | g_status.client_id;
    uint32_t data =
        (byte2 << 24) | (byte1 << 16) | (nibble << 8) | (cmd & 0xff);

    if (encode_wireline(g_status.rolling_code, fixed, data, message.packet) !=
        0)
    {
      free(message.packet);
      return ESP_FAIL;
    }

    g_status.rolling_code = (g_status.rolling_code + 1) & 0xfffffff;
  }
  else if (g_status.protocol == GDO_PROTOCOL_DRY_CONTACT)
  {
    message.packet[0] = command;
    message.packet[1] = nibble; // what to set pin too
  }
  else
  {
    *message.packet = command;
  }

  print_buffer(g_status.protocol, message.packet, true);
  if (xQueueSendToBack(gdo_tx_queue, &message, 0) == pdFALSE)
  {
    return ESP_ERR_NO_MEM;
  }

  return queue_event((gdo_event_t){GDO_EVENT_TX_PENDING});
}

/**
 * @brief Transmits a packet to the GDO from the UART.
 * @param packet The packet to send to the GDO.
 * @return ESP_OK on success, other non-zero errors from the UART driver.
 */
static esp_err_t transmit_packet(uint8_t *packet)
{
  esp_err_t err = ESP_OK;

  if (g_status.protocol == GDO_PROTOCOL_SEC_PLUS_V2)
  {
    // The packet transmission needs to start with a break, which is a low
    // signal for approximately 13 bit times. Since the UART driver does not
    // support a break signal, we set the baud rate to 6900  and write a single
    // byte of 0x00 to simulate the break. 8 data bits + 1 start bit = ~1305us
    // of low time, then 144us time high for the stop. This is close enough to
    // the 13 bit times required for a break.
    err = uart_set_baudrate(g_config.uart_num, 6900);
    if (err != ESP_OK)
    {
      return err;
    }

    uint8_t start = 0x0;
    if (uart_write_bytes(g_config.uart_num, &start, 1) < 0)
    {
      return ESP_FAIL;
    }

    // wait until sent then revert to 9600 baud to send the packet
    err = uart_wait_tx_done(g_config.uart_num, portMAX_DELAY);
    if (err != ESP_OK)
    {
      return err;
    }

    err = uart_set_baudrate(g_config.uart_num, 9600);
    if (err != ESP_OK)
    {
      return err;
    }

    if (uart_write_bytes(g_config.uart_num, packet, GDO_PACKET_SIZE) < 0)
    {
      return ESP_FAIL;
    }

    // make sure to finish this packet before returning incase there is another
    // in the queue so we don't change the baud rate in the middle of this
    // packet
    err = uart_wait_tx_done(g_config.uart_num, portMAX_DELAY);
    if (err != ESP_OK)
    {
      return err;
    }

    // flush the rx buffer since it will now have the data we just sent.
    err = uart_flush_input(g_config.uart_num);
  }
  else if (g_status.protocol == GDO_PROTOCOL_DRY_CONTACT)
  {
    // packet[0] is the GPIO pin number
    // packet[1] is what to set the pin to.
    gpio_set_level(packet[0], packet[1]);
  }
  else
  { // secplus v1, just send the byte
    if (uart_write_bytes(g_config.uart_num, packet, 1) < 0)
    {
      return ESP_FAIL;
    }
  }

  return err;
}

/**
 * @brief Decodes a packet received from the GDO and updates the status.
 * @param packet The packet received from the GDO.
 */
static void decode_v1_packet(uint8_t *packet)
{
  gdo_v1_command_t cmd = (gdo_v1_command_t)packet[0];
  uint8_t resp = packet[1];

  if (cmd == V1_CMD_QUERY_DOOR_STATUS)
  {
    gdo_door_state_t door_state = GDO_DOOR_STATE_UNKNOWN;
    uint8_t val = resp & 0x7;

    if (val == 0x2)
    {
      door_state = GDO_DOOR_STATE_OPEN;
    }
    else if (val == 0x5)
    {
      door_state = GDO_DOOR_STATE_CLOSED;
    }
    else if (val == 0x0 || val == 0x6)
    {
      door_state = GDO_DOOR_STATE_STOPPED;
    }
    else if (val == 0x1)
    {
      door_state = GDO_DOOR_STATE_OPENING;
    }
    else if (val == 0x4)
    {
      door_state = GDO_DOOR_STATE_CLOSING;
    }
    update_door_state(door_state);
  }
  else if (cmd == V1_CMD_QUERY_DOOR_STATUS_0x37)
  {
    queue_v1_command(V1_CMD_QUERY_OTHER_STATUS);
  }
  else if (cmd == V1_CMD_QUERY_OTHER_STATUS)
  {
    update_light_state((gdo_light_state_t)((resp >> 2) & 1));
    update_lock_state((gdo_lock_state_t)((~resp >> 3) & 1));
  }
  else if (cmd == V1_CMD_OBSTRUCTION)
  {
    update_obstruction_state(resp == 0 ? GDO_OBSTRUCTION_STATE_CLEAR : GDO_OBSTRUCTION_STATE_OBSTRUCTED);
  }
  else if (cmd == V1_CMD_TOGGLE_DOOR_PRESS)
  {
    update_button_state(GDO_BUTTON_STATE_PRESSED);
  }
  else if (cmd == V1_CMD_TOGGLE_DOOR_RELEASE)
  {
    update_button_state(GDO_BUTTON_STATE_RELEASED);
  }
  else
  {
    ESP_LOGD(TAG, "Unhandled command: %02x, resp: %02x", cmd, resp);
  }
}

/**
 * @brief Decodes a packet received from the GDO and updates the status.
 * @param packet The packet received from the GDO.
 */
static void decode_packet(uint8_t *packet)
{
  uint32_t rolling = 0;
  uint64_t fixed = 0;
  uint32_t data = 0;

  decode_wireline(packet, &rolling, &fixed, &data);

  data &= ~0xf000;

  if ((fixed & 0xFFFFFFFF) == g_status.client_id)
  { // my commands
    ESP_LOGE(TAG,
             "received mine: rolling=%07" PRIx32 " fixed=%010" PRIx64
             " data=%08" PRIx32,
             rolling, fixed, data);
    return;
  }
  else
  {
    ESP_LOGI(TAG,
             "received rolling=%07" PRIx32 " fixed=%010" PRIx64
             " data=%08" PRIx32,
             rolling, fixed, data);
  }

  gdo_command_t cmd = ((fixed >> 24) & 0xf00) | (data & 0xff);
  uint8_t nibble = (data >> 8) & 0x0f;
  uint8_t byte1 = (data >> 16) & 0xff;
  uint8_t byte2 = (data >> 24) & 0xff;

  ESP_LOGI(TAG, "cmd=%03x (%s) byte2=%02x byte1=%02x nibble=%01x", cmd,
           cmd_to_string(cmd), byte2, byte1, nibble);

  if (cmd == GDO_CMD_STATUS)
  {
    update_door_state((gdo_door_state_t)nibble);
    update_light_state((gdo_light_state_t)((byte2 >> 1) & 1));
    update_lock_state((gdo_lock_state_t)(byte2 & 1));
    update_learn_state((gdo_learn_state_t)((byte2 >> 5) & 1));
    if (g_config.obst_from_status)
    {
      update_obstruction_state((gdo_obstruction_state_t)((byte1 >> 6) & 1));
    }
  }
  else if (cmd == GDO_CMD_LIGHT)
  {
    handle_light_action((gdo_light_action_t)nibble);
  }
  else if (cmd == GDO_CMD_MOTOR_ON)
  {
    update_motor_state(GDO_MOTOR_STATE_ON);
  }
  else if (cmd == GDO_CMD_DOOR_ACTION)
  {
    update_button_state((gdo_button_state_t)((byte1 & 1) == 1) ? GDO_BUTTON_STATE_PRESSED : GDO_BUTTON_STATE_RELEASED);
  }
  else if (cmd == GDO_CMD_MOTION)
  {
    update_motion_state(GDO_MOTION_STATE_DETECTED);
  }
  else if (cmd == GDO_CMD_OPENINGS)
  {
    update_openings(nibble, ((byte1 << 8) | byte2));
  }
  else if (cmd == GDO_CMD_UPDATE_TTC)
  {
    update_ttc((byte1 << 8) | byte2);
    send_event(GDO_CB_EVENT_UPDATE_TTC);
  }
  else if (cmd == GDO_CMD_SET_TTC)
  {
    update_ttc((byte1 << 8) | byte2);
    send_event(GDO_CB_EVENT_SET_TTC);
  }
  else if (cmd == GDO_CMD_PAIRED_DEVICES)
  {
    update_paired_devices(nibble, byte2);
  }
  else if (cmd == GDO_CMD_BATTERY_STATUS)
  {
    update_battery_state(byte1);
  }
  else if ((g_status.door == GDO_DOOR_STATE_OPEN || g_status.door == GDO_DOOR_STATE_CLOSING) && cmd == GDO_CMD_OBST_1)
  {
    g_status.door = GDO_DOOR_STATE_OPEN; // if the obstruction sensor tripped the door will go back to open state.
    send_event(GDO_CB_EVENT_DOOR_POSITION);
  }
  else
  {
    ESP_LOGD(TAG, "Unhandled command: %03x (%s)", cmd, cmd_to_string(cmd));
  }
}

/**
 * @brief Main task that handles all the events from the UART and other tasks.
 */
static void gdo_main_task(void *arg)
{
  uint8_t rx_buffer[RX_BUFFER_SIZE * 2]; // double the size to prevent overflow
  uint16_t rx_buf_index = 0;
  uint8_t rx_pending = 0;
  gdo_tx_message_t tx_message = {};
  gdo_event_t event = {};
  gdo_cb_event_t cb_event = GDO_CB_EVENT_MAX;
  esp_err_t err = ESP_OK;
  uint32_t last_tx_time = 0;

  for (;;)
  {
    if (xQueueReceive(gdo_event_queue, (void *)&event,
                      (TickType_t)portMAX_DELAY))
    {
      cb_event = GDO_CB_EVENT_MAX;

      switch ((int)event.gdo_event)
      {
      case UART_BREAK:
        // All messages from the GDO start with a break if using V2 protocol.
        if (g_status.protocol == GDO_PROTOCOL_SEC_PLUS_V2)
        {
          ++rx_pending;
        }
        break;
      case UART_DATA:
      {
        uint16_t rx_packet_size = event.uart_event.size;
        if (!g_status.protocol)
        {
          if (rx_packet_size == 2)
          {
            ESP_LOGD(TAG, "Received 2 bytes, using protocol V1");
            g_status.protocol = GDO_PROTOCOL_SEC_PLUS_V1;
          }
          else if (rx_packet_size == 20 || rx_packet_size == 19 || rx_pending)
          {
            ESP_LOGD(TAG, "Received %u bytes, using protocol V2", rx_packet_size);
            g_status.protocol = GDO_PROTOCOL_SEC_PLUS_V2;
          }
          else
          {
            ESP_LOGD(TAG, "Received %u bytes, unknown protocol", rx_packet_size);
            uart_flush(g_config.uart_num);
          }
        }

        if (g_status.protocol == GDO_PROTOCOL_SEC_PLUS_V2)
        {
          if (!rx_pending)
          {
            // got a packet without a break first?
            ESP_LOGI(TAG, "Unexpected data; received %u bytes, %s", rx_packet_size,
                     rx_packet_size >= GDO_PACKET_SIZE ? "processing" : "ignoring");
            if (rx_packet_size >= GDO_PACKET_SIZE)
            {
              // If we have a usable packet then we should process it incase we just missed the break.
              rx_pending++;
            }
          }

          if (rx_packet_size < GDO_PACKET_SIZE)
          {
            ESP_LOGW(TAG, "Ignoring RX packet size %u, pending: %u", rx_packet_size, rx_pending);
            uart_read_bytes(g_config.uart_num, rx_buffer, rx_packet_size, 0);
            if (rx_pending)
            {
              --rx_pending;
            }
            break;
          }

          if (rx_packet_size > GDO_PACKET_SIZE)
          {
            ESP_LOGW(TAG, "Oversized packet received: %u bytes, messages pending: %u", rx_packet_size, rx_pending);
            // Sometimes the break is interperated as a 0 byte and added to the packet
            // So lets just dump the first byte(s) until we have our packet size.
            while (rx_packet_size > GDO_PACKET_SIZE)
            {
              if (uart_read_bytes(g_config.uart_num, rx_buffer, 1, 0) < 0)
              {
                ESP_LOGI(TAG, "RX buffer read error, flushing");
                uart_flush(g_config.uart_num);
                rx_pending = 0;
                break;
              }

              --rx_packet_size;
            }
          }

          while (rx_pending)
          {
            if (uart_read_bytes(g_config.uart_num, rx_buffer, GDO_PACKET_SIZE, 0) == GDO_PACKET_SIZE)
            {
              // check for the GDO packet start (0x55 0x01 0x00)
              if (memcmp(rx_buffer, "\x55\x01\x00", 3) != 0)
              {
                ESP_LOGE(TAG, "RX data signature error: 0x%02x%02x%02x", rx_buffer[0], rx_buffer[1], rx_buffer[2]);
                rx_pending--;
                continue;
              }

              print_buffer(g_status.protocol, rx_buffer, false);
              decode_packet(rx_buffer);
            }
            else
            {
              ESP_LOGE(TAG, "RX buffer read error, %u pending messages.", rx_pending);
            }
            --rx_pending;
          }
        }
        else if (g_status.protocol & GDO_PROTOCOL_SEC_PLUS_V1)
        {
          ESP_LOGV(TAG, "RX Secplus V1 data packet; %u bytes", rx_packet_size);
          int bytes_read = uart_read_bytes(g_config.uart_num, rx_buffer + rx_buf_index, rx_packet_size, 0);
          if (bytes_read < 0)
          {
            ESP_LOGE(TAG, "RX buffer read error");
            rx_buf_index = 0;
            break;
          }

          rx_buf_index += bytes_read;

          if (rx_buf_index >= GDO_PACKET_SIZE)
          {
            uint8_t i = 0;
            bool odd = rx_buf_index % 2 != 0;
            if (odd)
            {
              if (rx_buffer[0] <= V1_CMD_MIN || rx_buffer[0] >= V1_CMD_MAX)
              {
                i = 1; // Skip first byte if it's not a valid command
              }
              else
              {
                // Decrement the buffer index so we don't over read the buffer in the loop
                --rx_buf_index;
              }
            }

            uint8_t pkt[2];
            for (; i < rx_buf_index; i += 2)
            {
              pkt[0] = rx_buffer[i];
              pkt[1] = rx_buffer[i + 1];
              print_buffer(g_status.protocol, pkt, false);
              decode_v1_packet(pkt);
            }

            // if we decremented the buffer index then we need to move the last byte to the start
            if (odd && rx_buf_index % 2 == 0)
            {
              rx_buffer[0] = rx_buffer[rx_buf_index];
              rx_buf_index = 1;
            }
            else
            {
              rx_buf_index = 0;
            }
          }
        }

        break;
      }
      case UART_PARITY_ERR:
        ESP_LOGD(TAG, "Parity error, check wiring?");
        uart_flush_input(g_config.uart_num);
        rx_buf_index = 0;
        break;
      case UART_BUFFER_FULL:
        ESP_LOGE(TAG, "RX buffer full, flushing.");
        uart_flush_input(g_config.uart_num);
        xQueueReset(gdo_event_queue);
        rx_buf_index = 0;
        break;
      case UART_FIFO_OVF:
        ESP_LOGE(TAG, "RX FIFO overflow, flushing.");
        uart_flush_input(g_config.uart_num);
        rx_buf_index = 0;
        xQueueReset(gdo_event_queue);
        break;
      case GDO_EVENT_TX_PENDING:
      {
        uint32_t now = esp_timer_get_time() / 1000;
        if (now - last_tx_time < g_tx_delay_ms)
        {
          ESP_LOGD(TAG, "TX pending, waiting, %" PRIu32 "ms since last TX", now - last_tx_time);
          err = schedule_event(GDO_EVENT_TX_PENDING, (g_tx_delay_ms - (now - last_tx_time)) * 1000);
          if (err != ESP_OK)
          {
            ESP_LOGE(TAG, "Failed to schedule TX pending event, %s", esp_err_to_name(err));
          }
          break;
        }

        if ((g_status.protocol != GDO_PROTOCOL_DRY_CONTACT) && (rx_pending || gpio_get_level(g_config.uart_rx_pin)))
        {
          // If not synced yet just delete the message as the sync loop will resend it
          if (!g_status.synced)
          {
            ESP_LOGD(TAG, "Collision detected, ignoring as in middle of sync");
            xQueueReceive(gdo_tx_queue, &tx_message, 0);
            free(tx_message.packet);
            break;
          }

          ESP_LOGD(TAG, "Collision detected, requeuing command");
          // Wait 150ms for the collision to clear
          if (schedule_event(GDO_EVENT_TX_PENDING, 150 * 1000) != ESP_OK)
          {
            ESP_LOGE(TAG, "Failed to requeue command");
          }
          break;
        }

        err = ESP_OK;
        if (xQueueReceive(gdo_tx_queue, &tx_message, 0) == pdTRUE)
        {
          if (now - tx_message.sent_ms > 3000)
          {
            err = ESP_ERR_TIMEOUT;
            ESP_LOGE(TAG, "ESP_ERR_TIMEOUT: %dms, (%d - %d)", now - tx_message.sent_ms, now, tx_message.sent_ms);
          }
          else
          {
            uint8_t retry_count = 2;
            do
            {
              err = transmit_packet(tx_message.packet);
            } while (err != ESP_OK && --retry_count);
          }

          free(tx_message.packet);
        }
        else
        {
          ESP_LOGE(TAG, "TX queue empty, no message to send.");
          err = ESP_ERR_INVALID_ARG;
        }

        if (err != ESP_OK)
        {
          ESP_LOGE(TAG, "Failed to TX message: %s - %s", g_status.protocol == GDO_PROTOCOL_SEC_PLUS_V2 ? cmd_to_string(tx_message.cmd) : v1_cmd_to_string(tx_message.cmd), esp_err_to_name(err));
          // TODO: send message to app about the failure
        }
        else if (g_status.protocol != GDO_PROTOCOL_DRY_CONTACT)
        {
          // if dry contact then last_tx_time and send command is irrelevant
          if ((g_status.protocol & GDO_PROTOCOL_SEC_PLUS_V1) && (tx_message.cmd == (gdo_command_t)V1_CMD_QUERY_DOOR_STATUS ||
                                                                 tx_message.cmd == (gdo_command_t)V1_CMD_QUERY_OTHER_STATUS ||
                                                                 tx_message.cmd == (gdo_command_t)V1_CMD_OBSTRUCTION))
          {
            // Log verbose if query or obstruction request, as they are sent as part of wall panel emulation and would fill up the log
            ESP_LOGV(TAG, "Sent command: %s", v1_cmd_to_string(tx_message.cmd));
          }
          else
          {
            ESP_LOGI(TAG, "Sent command: %s", g_status.protocol == GDO_PROTOCOL_SEC_PLUS_V2 ? cmd_to_string(tx_message.cmd) : v1_cmd_to_string(tx_message.cmd));
          }
          last_tx_time = esp_timer_get_time() / 1000;
        }

        break;
      }
      case GDO_EVENT_SYNC_COMPLETE:
        cb_event = GDO_CB_EVENT_SYNCED;
        break;
      case GDO_EVENT_OBST:
        cb_event = GDO_CB_EVENT_OBSTRUCTION;
        break;
      case GDO_EVENT_DOOR_POSITION_UPDATE:
        cb_event = GDO_CB_EVENT_DOOR_POSITION;
        break;
      case GDO_EVENT_LIGHT_UPDATE:
        cb_event = GDO_CB_EVENT_LIGHT;
        break;
      case GDO_EVENT_LOCK_UPDATE:
        cb_event = GDO_CB_EVENT_LOCK;
        break;
      case GDO_EVENT_MOTOR_UPDATE:
        cb_event = GDO_CB_EVENT_MOTOR;
        break;
      case GDO_EVENT_BUTTON_UPDATE:
        cb_event = GDO_CB_EVENT_BUTTON;
        break;
      case GDO_EVENT_BATTERY_UPDATE:
        cb_event = GDO_CB_EVENT_BATTERY;
        break;
      case GDO_EVENT_LEARN_UPDATE:
        cb_event = GDO_CB_EVENT_LEARN;
        break;
      case GDO_EVENT_OPENINGS_UPDATE:
        cb_event = GDO_CB_EVENT_OPENINGS;
        break;
      case GDO_EVENT_MOTION_UPDATE:
        cb_event = GDO_CB_EVENT_MOTION;
        break;
      case GDO_EVENT_UPDATE_TTC:
        cb_event = GDO_CB_EVENT_UPDATE_TTC;
        break;
      case GDO_EVENT_SET_TTC:
        cb_event = GDO_CB_EVENT_SET_TTC;
        break;
      case GDO_EVENT_PAIRED_DEVICES_UPDATE:
        cb_event = GDO_CB_EVENT_PAIRED_DEVICES;
        break;
      case GDO_EVENT_DOOR_OPEN_DURATION_MEASUREMENT:
        cb_event = GDO_CB_EVENT_OPEN_DURATION_MEASUREMENT;
        break;
      case GDO_EVENT_DOOR_CLOSE_DURATION_MEASUREMENT:
        cb_event = GDO_CB_EVENT_CLOSE_DURATION_MEASUREMENT;
        break;
      case GDO_EVENT_TOF_TIMER:
        cb_event = GDO_CB_EVENT_TOF_TIMER;
        break;
      default:
        ESP_LOGD(TAG, "Unhandled gdo event: %d", event.gdo_event);
        break;
      }

      if (cb_event < GDO_CB_EVENT_MAX && g_event_callback)
      {
        g_event_callback(&g_status, cb_event, g_user_cb_arg);
      }
    }
  }

  vTaskDelete(NULL);
}

/*************************** STATUS FUNCTIONS ************************************/

static void update_door_state(const gdo_door_state_t door_state)
{
  static int64_t start_opening;
  static int64_t start_closing;

  if (!g_status.open_ms)
  {
    if (door_state == GDO_DOOR_STATE_OPENING &&
        g_status.door == GDO_DOOR_STATE_CLOSED)
    {
      start_opening = esp_timer_get_time();
    }
    if (door_state == GDO_DOOR_STATE_OPEN &&
        g_status.door == GDO_DOOR_STATE_OPENING && start_opening != 0)
    {
      g_status.open_ms =
          (uint16_t)((esp_timer_get_time() - start_opening) / 1000ULL);
      ESP_LOGV(TAG, "Open time: %u", g_status.open_ms);
      send_event(GDO_CB_EVENT_OPEN_DURATION_MEASUREMENT);
    }
    if (door_state == GDO_DOOR_STATE_STOPPED)
    {
      start_opening = -1;
    }
  }

  if (!g_status.close_ms)
  {
    if (door_state == GDO_DOOR_STATE_CLOSING &&
        g_status.door == GDO_DOOR_STATE_OPEN)
    {
      start_closing = esp_timer_get_time();
    }
    if (door_state == GDO_DOOR_STATE_CLOSED &&
        g_status.door == GDO_DOOR_STATE_CLOSING && start_closing != 0)
    {
      g_status.close_ms =
          (uint16_t)((esp_timer_get_time() - start_closing) / 1000ULL);
      ESP_LOGV(TAG, "Close time: %u", g_status.close_ms);
      send_event(GDO_CB_EVENT_CLOSE_DURATION_MEASUREMENT);
    }
    if (door_state == GDO_DOOR_STATE_STOPPED)
    {
      start_closing = -1;
    }
  }

  if (door_state == GDO_DOOR_STATE_OPENING || door_state == GDO_DOOR_STATE_CLOSING)
  {
    if (g_status.door_position >= 0 && g_status.close_ms > 0 && g_status.open_ms > 0 && !esp_timer_is_active(door_position_sync_timer))
    {
      g_door_start_moving_ms = (uint32_t)((esp_timer_get_time() / 1000) - 1000);
      if (esp_timer_start_periodic(door_position_sync_timer, 500 * 1000) != ESP_OK)
      {
        ESP_LOGE(TAG, "Failed to start door position sync timer");
      }
    }
  }
  else
  {
    esp_timer_stop(door_position_sync_timer);

    if (door_state == GDO_DOOR_STATE_STOPPED)
    {
      int delta = g_status.door_position - g_status.door_target;
      if (delta < -5000 || delta > 5000)
      {
        ESP_LOGE(TAG, "Door failed to reach target");
      }
    }
    else if (door_state == GDO_DOOR_STATE_OPEN)
    {
      g_status.door_position = 0;
      if ((g_status.ttc_enabled == true) && (g_status.ttc_seconds == 0))
        gdo_set_time_to_close(g_ttc_delay_s);
    }
    else if (door_state == GDO_DOOR_STATE_CLOSED)
    {
      g_status.door_position = 10000;
      if (g_status.protocol == GDO_PROTOCOL_SEC_PLUS_V2 && g_status.door != GDO_DOOR_STATE_CLOSED)
      {
        // Only update openings if status is changing to closed (ie, not steady state closed)
        get_openings();
      }
    }

    g_door_start_moving_ms = 0;
    if (g_status.motor != GDO_MOTOR_STATE_OFF)
    {
      g_status.motor = GDO_MOTOR_STATE_OFF;
      send_event(GDO_CB_EVENT_MOTOR);
    }
  }

  if (g_status.door == GDO_DOOR_STATE_UNKNOWN &&
      g_status.protocol & GDO_PROTOCOL_SEC_PLUS_V1 &&
      gdo_sync_task_handle)
  {
    xTaskNotifyGive(gdo_sync_task_handle);
  }

  static int32_t previous_door_position = -1;
  static int32_t previous_door_target = -1;
  if ((door_state != g_status.door) || (previous_door_position != g_status.door_position) || (previous_door_target != g_status.door_target))
  {
    ESP_LOGD(TAG, "Door state: %s", gdo_door_state_str[door_state]);
    g_status.door = door_state;
    previous_door_position = g_status.door_position;
    previous_door_target = g_status.door_target;
    send_event(GDO_CB_EVENT_DOOR_POSITION);
  }
  else
  {
    ESP_LOGV(TAG, "Door state: %s", gdo_door_state_str[door_state]);
  }
}

/**
 * @brief Gets the total number of paired devices with the GDO for the specified
 * type.
 * @param type The type of paired devices to get the total for.
 */
static void get_paired_devices(gdo_paired_device_type_t type)
{
  if (type >= GDO_PAIRED_DEVICE_TYPE_MAX)
  {
    ESP_LOGE(TAG, "Invalid paired device type");
    return;
  }

  queue_command(GDO_CMD_GET_PAIRED_DEVICES, type, 0, 0);
}

/********************************* INLINE UTILITIES **********************************/

/**
 * @brief Gets the current status of the GDO.
 */
inline static esp_err_t get_status()
{
  return queue_command(GDO_CMD_GET_STATUS, 0, 0, 0);
}

/**
 * @brief Gets the current total openings of the GDO.
 */
inline static esp_err_t get_openings()
{
  return queue_command(GDO_CMD_GET_OPENINGS, 0, 0, 0);
}

/**
 * @brief Sends a door action command to a dry-contact GDO.
 * @param action The action to send to the GDO.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if
 * the encoding fails.
 */
inline static esp_err_t send_door_action_dc(gdo_door_action_t action)
{
  switch (action)
  {
  case GDO_DOOR_ACTION_OPEN:
    if (g_config.dc_discrete_open_pin)
      gdo_dc_toggle_pin(g_config.dc_discrete_open_pin);
    gdo_dc_toggle_pin(g_config.uart_tx_pin);
    break;
  case GDO_DOOR_ACTION_CLOSE:
    if (g_config.dc_discrete_close_pin)
      gdo_dc_toggle_pin(g_config.dc_discrete_close_pin);
    gdo_dc_toggle_pin(g_config.uart_tx_pin);
    break;
  default:
    gdo_dc_toggle_pin(g_config.uart_tx_pin);
    break;
  }
  return ESP_OK;
}

/**
 * @brief Sends a door action command to the GDO.
 * @param action The action to send to the GDO.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if
 * the encoding fails.
 */
inline static esp_err_t send_door_action(gdo_door_action_t action)
{
  esp_err_t err = ESP_OK;
  if (g_status.protocol == GDO_PROTOCOL_DRY_CONTACT)
  {
    return send_door_action_dc(action);
  }
  else if (g_status.protocol & GDO_PROTOCOL_SEC_PLUS_V1)
  {
    return gdo_v1_toggle_cmd(V1_CMD_TOGGLE_DOOR_PRESS);
  }
  else
  {
    err = queue_command(GDO_CMD_DOOR_ACTION, action, 1, 1);
    if (err == ESP_OK)
    {
      --g_status.rolling_code; // only increment after the second command
      err = queue_command(GDO_CMD_DOOR_ACTION, action, 0, 1);
    }
  }

  return err;
}

/**
 * @brief Updates the local light state and queues an event if it has changed.
 * @param light_state The new light state to update to.
 */
inline static void update_light_state(gdo_light_state_t light_state)
{
  if (light_state != g_status.light)
  {
    ESP_LOGD(TAG, "Light state: %s", gdo_light_state_str[light_state]);
    g_status.light = light_state;
    send_event(GDO_CB_EVENT_LIGHT);
  }
  else
  {
    ESP_LOGV(TAG, "Light state: %s", gdo_light_state_str[light_state]);
  }
}

/**
 * @brief Updates the local lock state and queues an event if it has changed.
 * @param lock_state The new lock state to update to.
 */
inline static void update_lock_state(gdo_lock_state_t lock_state)
{
  if (lock_state != g_status.lock)
  {
    ESP_LOGD(TAG, "Lock state: %s", gdo_lock_state_str[lock_state]);
    g_status.lock = lock_state;
    send_event(GDO_CB_EVENT_LOCK);
  }
  else
  {
    ESP_LOGV(TAG, "Lock state: %s", gdo_lock_state_str[lock_state]);
  }
}

/**
 * @brief Updates the local obstruction state and queues an event if it has
 * changed.
 * @param obstruction_state The new obstruction state to update to.
 */
inline static void
update_obstruction_state(gdo_obstruction_state_t obstruction_state)
{
  if (obstruction_state != g_status.obstruction)
  {
    ESP_LOGD(TAG, "Obstruction state: %s", gdo_obstruction_state_str[obstruction_state]);
    g_status.obstruction = obstruction_state;
    // This function can be called from obstruction timer... therefore...
    // We use queue event rather than send event to ensure that the callback
    // function is called from the main thread (same thread as all other callbacks).
    queue_event((gdo_event_t){GDO_EVENT_OBST});
  }
  else
  {
    ESP_LOGV(TAG, "Obstruction state: %s", gdo_obstruction_state_str[obstruction_state]);
  }
}

/**
 * @brief Updates the local learn state and queues an event if it has changed.
 * Also gets the total paired devices if the learn state changed to inactive to
 * check for new paired devices.
 * @param learn_state The new learn state to update to.
 */
inline static void update_learn_state(gdo_learn_state_t learn_state)
{
  ESP_LOGD(TAG, "Learn state: %s", gdo_learn_state_str[learn_state]);
  if (learn_state != g_status.learn)
  {
    g_status.learn = learn_state;
    send_event(GDO_CB_EVENT_LEARN);
    if (learn_state == GDO_LEARN_STATE_INACTIVE && g_status.protocol == GDO_PROTOCOL_SEC_PLUS_V2)
    {
      get_paired_devices(GDO_PAIRED_DEVICE_TYPE_ALL);
    }
  }
}

/**
 * @brief Handles the light state when it was changed by a command from the GDO.
 * @param light_action The action the GDO sent to change the light state.
 */
inline static void handle_light_action(gdo_light_action_t light_action)
{
  gdo_light_state_t light_state = g_status.light;
  switch (light_action)
  {
  case GDO_LIGHT_ACTION_OFF:
    light_state = GDO_LIGHT_STATE_OFF;
    break;
  case GDO_LIGHT_ACTION_ON:
    light_state = GDO_LIGHT_STATE_ON;
    break;
  case GDO_LIGHT_ACTION_TOGGLE:
    if (light_state < GDO_LIGHT_STATE_MAX)
    {
      light_state = light_state == GDO_LIGHT_STATE_OFF ? GDO_LIGHT_STATE_ON
                                                       : GDO_LIGHT_STATE_OFF;
    }
    // We get toggle when wall panel light button pressed and released
    g_status.button = GDO_BUTTON_STATE_PRESSED;
    send_event(GDO_CB_EVENT_BUTTON);
    g_status.button = GDO_BUTTON_STATE_RELEASED;
    send_event(GDO_CB_EVENT_BUTTON);
    break;
  default:
    light_state = GDO_LIGHT_STATE_MAX;
    break;
  }

  update_light_state(light_state);
}

/**
 * @brief Handles the lock state when it was changed by a command from the GDO.
 * @param lock_action The action the GDO sent to change the lock state.
 */
inline static void handle_lock_action(gdo_lock_action_t lock_action)
{
  gdo_lock_state_t lock_state = g_status.lock;
  switch (lock_action)
  {
  case GDO_LOCK_ACTION_LOCK:
    lock_state = GDO_LOCK_STATE_LOCKED;
    break;
  case GDO_LOCK_ACTION_UNLOCK:
    lock_state = GDO_LOCK_STATE_UNLOCKED;
    break;
  case GDO_LOCK_ACTION_TOGGLE:
    if (lock_state < GDO_LOCK_STATE_MAX)
    {
      lock_state = lock_state == GDO_LOCK_STATE_LOCKED ? GDO_LOCK_STATE_UNLOCKED
                                                       : GDO_LOCK_STATE_LOCKED;
    }
    break;
  default:
    lock_state = GDO_LOCK_STATE_MAX;
    break;
  }

  update_lock_state(lock_state);
}

/**
 * @brief Updates the local motor state and queues an event if it has changed.
 * @param motor_state The new motor state to update to.
 */
inline static void update_motor_state(gdo_motor_state_t motor_state)
{
  ESP_LOGD(TAG, "Motor state: %s", gdo_motor_state_str[motor_state]);
  if (motor_state != g_status.motor)
  {
    g_status.motor = motor_state;
    send_event(GDO_CB_EVENT_MOTOR);
  }
}

/**
 * @brief Updates the local button state and queues an event if it has changed.
 * @param button_state The new button state to update to.
 */
inline static void update_button_state(gdo_button_state_t button_state)
{
  ESP_LOGD(TAG, "Button state: %s", gdo_button_state_str[button_state]);
  if (button_state == GDO_BUTTON_STATE_RELEASED)
  {
    // Why are we calling get status with every button release?
    // This creates a lot of unnecessary traffic on serial comms.
    get_status();
  }

  // Send event even if state is the same, so callback always notfied even if repeated multiple times
  g_status.button = button_state;
  send_event(GDO_CB_EVENT_BUTTON);
}

/**
 * @brief Updates the local motion state and queues an event if it has changed.
 * Also starts a timer to clear the motion state if not reset.
 * @param motion_state The new motion state to update to.
 */
inline static void update_motion_state(gdo_motion_state_t motion_state)
{
  ESP_LOGD(TAG, "Motion state: %s", gdo_motion_state_str[motion_state]);
  if (motion_state == GDO_MOTION_STATE_DETECTED)
  {
    esp_timer_stop(motion_detect_timer);
    esp_timer_start_once(motion_detect_timer, 3000 * 1000);
  }

  if (g_status.motion != motion_state)
  {
    g_status.motion = motion_state;
    // This function can be called from motion timer (started above)... therefore...
    // We use queue event rather than send event to ensure that the callback
    // function is called from the main thread (same thread as all other callbacks).
    queue_event((gdo_event_t){GDO_EVENT_MOTION_UPDATE});

    if (g_status.protocol == GDO_PROTOCOL_SEC_PLUS_V2 && motion_state == GDO_MOTION_STATE_DETECTED && g_status.light == GDO_LIGHT_STATE_OFF)
    {
      // If motion detected and light is off, then request status to update light state (which may have turned on)
      get_status();
    }
  }
}

/**
 * @brief Updates the local openings count and queues an event if it has
 * changed.
 * @param flag The flag from the GDO to indicate if the openings count is from
 * our request.
 * @param count The new openings count to update to.
 */
inline static void update_openings(uint8_t flag, uint16_t count)
{
  ESP_LOGD(TAG, "Openings: %u", count);
  if (flag == 0 || g_status.openings != 0)
  {
    if (g_status.openings != count)
    {
      g_status.openings = count;
      send_event(GDO_CB_EVENT_OPENINGS);
    }
  }
  // Ignoring openings, not from our request
}

/**
 * @brief Updates the local TTC and queues an event if it has changed.
 * @param ttc The new TTC to update to.
 */
inline static void update_ttc(uint16_t ttc)
{
  ESP_LOGI(TAG, "TTC Seconds remaining: %u", ttc);
  if (g_status.ttc_seconds != ttc)
  {
    g_status.ttc_seconds = ttc;
  }
}

/**
 * @brief Set the time to close
 * @param time_to_close The new time to close set by the UI
 */
esp_err_t gdo_set_time_to_close(uint16_t time_to_close)
{
  g_ttc_delay_s = time_to_close;
  g_status.ttc_enabled = (time_to_close > 0) ? 1 : 0;
  esp_err_t err = ESP_OK;
  uint8_t byte1 = (time_to_close >> 8);
  uint8_t byte2 = (uint8_t)time_to_close;
  uint8_t nibble = 1;
  update_ttc(time_to_close);
  queue_command(GDO_CMD_SET_TTC, nibble, byte1, byte2);
  // This can be called from unknown thread... therefore...
  // We use queue event rather than send event to ensure that the callback
  // function is called from the main thread (same thread as all other callbacks).
  queue_event((gdo_event_t){GDO_EVENT_SET_TTC});
  return err;
}

/**
 * @brief Set the tof interval timer value and enable/disable flag
 * @param interval the interval time in micro seconds
 * @param enabled the flag to enable or disable the timer on gdo_start
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the interval is less than 1000
 */
esp_err_t gdo_set_tof_timer(uint32_t interval, bool enabled)
{
  esp_err_t err = ESP_OK;
  if (interval < 1000)
  {
    ESP_LOGE(TAG, "Invalid interval, must be greater than 1000");
    err = ESP_ERR_INVALID_ARG;
  }
  g_status.tof_timer_active = enabled;
  g_status.tof_timer_usecs = interval;
  return err;
}

/**
 * @brief Set the obst test pulse interval timer value and enable/disable flag
 * @param interval the interval time in micro seconds
 * @param enabled the flag to enable or disable the timer on gdo_start
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the interval is less than 1000
 */
esp_err_t gdo_set_obst_test_pulse_timer(uint32_t interval, bool enabled)
{
  esp_err_t err = ESP_OK;
  if (interval < 1000) // prevent a race condition
  {
    ESP_LOGE(TAG, "Invalid interval, must be greater than 1000");
    err = ESP_ERR_INVALID_ARG;
  }
  g_status.obst_test_pulse_timer_active = enabled;
  g_status.obst_test_pulse_timer_usecs = interval;
  return err;
}

/**
 * @brief Updates the local paired devices count and queues an event if it has
 * changed.
 * @param type The type of paired devices to update.
 */
inline static void update_paired_devices(gdo_paired_device_type_t type,
                                         uint8_t count)
{
  ESP_LOGD(TAG, "Paired devices: %u", count);
  bool changed = false;
  if (type == GDO_PAIRED_DEVICE_TYPE_ALL &&
      g_status.paired_devices.total_all != count)
  {
    changed = true;
    g_status.paired_devices.total_all = count;
  }
  else if (type == GDO_PAIRED_DEVICE_TYPE_REMOTE &&
           g_status.paired_devices.total_remotes != count)
  {
    changed = true;
    g_status.paired_devices.total_remotes = count;
  }
  else if (type == GDO_PAIRED_DEVICE_TYPE_KEYPAD &&
           g_status.paired_devices.total_keypads != count)
  {
    changed = true;
    g_status.paired_devices.total_keypads = count;
  }
  else if (type == GDO_PAIRED_DEVICE_TYPE_WALL_CONTROL &&
           g_status.paired_devices.total_wall_controls != count)
  {
    changed = true;
    g_status.paired_devices.total_wall_controls = count;
  }
  else if (type == GDO_PAIRED_DEVICE_TYPE_ACCESSORY &&
           g_status.paired_devices.total_accessories != count)
  {
    changed = true;
    g_status.paired_devices.total_accessories = count;
  }

  if (g_status.synced && changed)
  {
    send_event(GDO_CB_EVENT_PAIRED_DEVICES);
  }
}

/**
 * @brief Updates the local battery state and queues an event if it has changed.
 * @param battery_state The new battery state to update to.
 */
inline static void update_battery_state(gdo_battery_state_t battery_state)
{
  ESP_LOGD(TAG, "Battery state: %s", gdo_battery_state_str[battery_state]);
  if (battery_state != g_status.battery)
  {
    g_status.battery = battery_state;
    send_event(GDO_CB_EVENT_BATTERY);
  }
}

/**
 * @brief Queues an event to the event queue.
 * @param event The event to queue.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the gdo event queue
 * hasn't been created yet, ESP_ERR_NO_MEM if the queue is full.
 */
inline static esp_err_t queue_event(gdo_event_t event)
{
  if (!gdo_event_queue)
  {
    ESP_LOGE(TAG, "Event Queue not created!");
    return ESP_ERR_INVALID_STATE;
  }

  if (xQueueSend(gdo_event_queue, &event, 0) == pdFALSE)
  {
    ESP_LOGE(TAG, "Event Queue Full!");
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}

/**
 * @brief Immediately calls event callback.
 * @param event The event to send.
 * @return ESP_OK
 */
inline static esp_err_t send_event(gdo_event_type_t event)
{
  if (g_event_callback)
    g_event_callback(&g_status, event, g_user_cb_arg);
  return ESP_OK;
}

/************************************ TOF SENSOR *************************************/

/**
 * @brief Set the parked threshold dynamically.
 * @param ms The minimum time in milliseconds.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the time is invalid.
 */
esp_err_t gdo_set_vehicle_parked_threshold(uint16_t vehicle_parked_threshold)
{
  esp_err_t err = ESP_OK;
  g_status.vehicle_parked_threshold = vehicle_parked_threshold;
  return err;
}

/**
 * @brief Set the parked threshold dynamically.
 * @param ms The minimum time in milliseconds.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the time is invalid.
 */
esp_err_t gdo_set_vehicle_parked_threshold_variance(uint16_t vehicle_parked_threshold_variance)
{
  esp_err_t err = ESP_OK;
  g_status.vehicle_parked_threshold_variance = vehicle_parked_threshold_variance;
  return err;
}

/**
 * @brief time of flight sensor timer callback.
 * @details Optional, tiggers on a distance measurement interval.
 * If a new value is detected an event of GDO_EVENT_TOF_TIMER is queued.
 */
inline static void tof_timer_cb(void *arg)
{
  // We use queue event rather than send event to ensure that the callback
  // function is called from the main thread (same thread as all other callbacks).
  queue_event((gdo_event_t){GDO_EVENT_TOF_TIMER});
}

/**
 * @brief Handles the dry contact interrupts
 */
static void IRAM_ATTR gdo_contact_isr_handler(void *arg)
{
  gdo_contact_t *info = (gdo_contact_t *)arg;

  portENTER_CRITICAL_ISR(&info->mux);
  if (info->inDebounce)
  {
    // if interrupt received while we are in a debounce check period,
    // keep track that it occurred and ignore it.
    info->count++;
    info->countTimestamp = esp_timer_get_time();
    portEXIT_CRITICAL_ISR(&info->mux);
    return;
  }
  else
  {
    // keep track that we are notifying task of interrupt (and therefore entering debounce period)
    info->inDebounce = true;
    info->level = gpio_get_level(info->pin);
    info->count = 0;
    info->countTimestamp = info->levelTimestamp = esp_timer_get_time();
    portEXIT_CRITICAL_ISR(&info->mux);

    BaseType_t xHigherPriorityTaskWoken;
    xTaskNotifyFromISR(info->notifyTask, 0, eNoAction, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return;
  }
}

/**
 * @brief If we received a interrupt from the GPIO pin, wait for debounce
 * mitigation and check that the pin level is stable before declaring it good
 */
static void contact_debounce_timer_cb(void *arg)
{
  gdo_contact_t *info = (gdo_contact_t *)arg;

  portENTER_CRITICAL(&info->mux);
  uint32_t pinLevel = gpio_get_level(info->pin);
  uint32_t count = info->count;
  uint32_t bounceDuration = (int32_t)(info->countTimestamp - info->levelTimestamp);
  if (pinLevel == info->level)
  {
    // Current pin level same as when timer was started, so assume that
    // state is stable and we can notify main GDO task.
    info->inDebounce = false;
    portEXIT_CRITICAL(&info->mux);
    ESP_LOGD(TAG, "Level of pin %d (%s) is %d. Interrupts during debounce: %d (%ld.%03ldms)",
             info->pin,
             (info->pin == g_config.dc_open_pin) ? "open sensor" : "closed sensor",
             pinLevel,
             count, bounceDuration / 1000UL, bounceDuration % 1000UL);
    decode_dry_contact(info->contact, pinLevel);
  }
  else
  {
    // pin level is different from when timer started, so assume that
    // state is NOT stable... wait again using whatever the state is now.
    info->inDebounce = true;
    info->level = pinLevel;
    portEXIT_CRITICAL(&info->mux);
    ESP_LOGD(TAG, "Debounce of pin %d (%s) not stable, wait for another %dms. Interrupts: %d (%ld.%03ldms)",
             info->pin,
             (info->pin == g_config.dc_open_pin) ? "open sensor" : "closed sensor",
             g_config.dc_debounce_ms,
             count, bounceDuration / 1000UL, bounceDuration % 1000UL);
    esp_timer_start_once(info->debounceTimer, g_config.dc_debounce_ms * 1000);
  }
}

/**
 * @brief This task stated to handle change in state of dry contact sensors
 * @details This task will...
 */
static void gdo_contact_task(void *arg)
{
  gdo_contact_t *info = (gdo_contact_t *)arg;
  esp_err_t err = ESP_OK;

  ESP_LOGI(TAG, "Initialize dry contact ISR and Task for pin: %d (%s)", info->pin, (info->pin == g_config.dc_open_pin) ? "open sensor" : "closed sensor");

  info->notifyTask = xTaskGetCurrentTaskHandle();
  info->level = UINT_MAX;
  portMUX_INITIALIZE(&info->mux);
  info->inDebounce = false;
  info->countTimestamp = 0;
  info->levelTimestamp = 0;

  esp_timer_create_args_t timer_args = {
      .callback = contact_debounce_timer_cb,
      .arg = info,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "contact_debounce_timer"};

  err = esp_timer_create(&timer_args, &info->debounceTimer);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to create contact debounce timer for GPIO pin %d, terminating task", info->pin);
    return;
  }

  // Install ISR handler for dry contact pin
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << info->pin),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_ANYEDGE,
  };

  err = gpio_config(&io_conf);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to configure GPIO for pin %d, terminating task", info->pin);
    return;
  }

  err = gpio_isr_handler_add(info->pin, gdo_contact_isr_handler, (void *)info);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to install ISR handler for GPIO pin %d, terminating task", info->pin);
    return;
  }

  // Notify ourselves... so we will immediately process current GPIO level to set initial state
  info->level = gpio_get_level(info->pin);
  xTaskNotify(info->notifyTask, 0, eNoAction);

  for (;;)
  {
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    ESP_LOGD(TAG, "Wake up for GPIO pin %d (%s), level is %d", info->pin, (info->pin == g_config.dc_open_pin) ? "open sensor" : "closed sensor", info->level);
    esp_timer_stop(info->debounceTimer); // Just to be super safe, should not be active.
    esp_timer_start_once(info->debounceTimer, g_config.dc_debounce_ms * 1000);
  }
}

/**
 * @brief determine door state (open/opening/closing/closed) from dry contact states
 */
void decode_dry_contact(gdo_contact_type_t contact, uint32_t level)
{
  static bool dryContactDoorOpen = false;
  static bool dryContactDoorClose = false;
  static bool previousDryContactDoorOpen = false;
  static bool previousDryContactDoorClose = false;

  if (contact == GDO_CONTACT_DOOR_OPEN)
  {
    if (level)
    {
      dryContactDoorOpen = false;
      ESP_LOGD(TAG, "Open sensor released");
    }
    else
    {
      dryContactDoorOpen = true;
      ESP_LOGD(TAG, "Open sensor pressed");
    }
  }

  if (contact == GDO_CONTACT_DOOR_CLOSE)
  {
    if (level)
    {
      dryContactDoorClose = false;
      ESP_LOGD(TAG, "Close sensor released");
    }
    else
    {
      dryContactDoorClose = true;
      ESP_LOGD(TAG, "Close sensor pressed");
    }
  }

  if (g_status.protocol == GDO_PROTOCOL_DRY_CONTACT)
  {
    if (dryContactDoorOpen)
    {
      update_door_state(GDO_DOOR_STATE_OPEN);
    }

    if (dryContactDoorClose)
    {
      update_door_state(GDO_DOOR_STATE_CLOSED);
    }

    if (!dryContactDoorClose && !dryContactDoorOpen)
    {
      if (previousDryContactDoorClose)
      {
        update_door_state(GDO_DOOR_STATE_OPENING);
      }
      else if (previousDryContactDoorOpen)
      {
        update_door_state(GDO_DOOR_STATE_CLOSING);
      }
      else
      {
        update_door_state(GDO_DOOR_STATE_UNKNOWN);
      }
    }
  }
  else
  {
    // Dry contacts are repurposed as optional door open/close when we
    // are using Sec+ 1.0 or Sec+ 2.0 door control type. You must call gdo_init() with
    // values set for dc_open_pin and dc_close_pin in the config struct.
    if (dryContactDoorOpen)
    {
      gdo_door_open();
      dryContactDoorOpen = false;
    }

    if (dryContactDoorClose)
    {

      gdo_door_close();
      dryContactDoorClose = false;
    }
  }

  previousDryContactDoorOpen = dryContactDoorOpen;
  previousDryContactDoorClose = dryContactDoorClose;
}
