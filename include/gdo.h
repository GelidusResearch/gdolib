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

#ifndef GDO_H
#define GDO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <driver/uart.h>
#include <driver/gpio.h>

    typedef enum
    {
        GDO_DOOR_STATE_UNKNOWN = 0,
        GDO_DOOR_STATE_OPEN,
        GDO_DOOR_STATE_CLOSED,
        GDO_DOOR_STATE_STOPPED,
        GDO_DOOR_STATE_OPENING,
        GDO_DOOR_STATE_CLOSING,
        GDO_DOOR_STATE_MAX,
    } gdo_door_state_t;

    typedef enum
    {
        GDO_LIGHT_STATE_OFF = 0,
        GDO_LIGHT_STATE_ON,
        GDO_LIGHT_STATE_MAX,
    } gdo_light_state_t;

    typedef enum
    {
        GDO_LOCK_STATE_UNLOCKED = 0,
        GDO_LOCK_STATE_LOCKED,
        GDO_LOCK_STATE_MAX,
    } gdo_lock_state_t;

    typedef enum
    {
        GDO_MOTION_STATE_CLEAR = 0,
        GDO_MOTION_STATE_DETECTED,
        GDO_MOTION_STATE_MAX,
    } gdo_motion_state_t;

    typedef enum
    {
        GDO_OBSTRUCTION_STATE_OBSTRUCTED = 0,
        GDO_OBSTRUCTION_STATE_CLEAR,
        GDO_OBSTRUCTION_STATE_MAX,
    } gdo_obstruction_state_t;

    typedef enum
    {
        GDO_MOTOR_STATE_OFF = 0,
        GDO_MOTOR_STATE_ON,
        GDO_MOTOR_STATE_MAX,
    } gdo_motor_state_t;

    typedef enum
    {
        GDO_BUTTON_STATE_PRESSED = 0,
        GDO_BUTTON_STATE_RELEASED,
        GDO_BUTTON_STATE_MAX,
    } gdo_button_state_t;

    typedef enum
    {
        GDO_BATT_STATE_UNKNOWN = 0,
        GDO_BATT_STATE_CHARGING = 0x6,
        GDO_BATT_STATE_FULL = 0x8,
        GDO_BATT_STATE_MAX = 0xff,
    } gdo_battery_state_t;

    typedef enum
    {
        GDO_LEARN_STATE_INACTIVE = 0,
        GDO_LEARN_STATE_ACTIVE,
        GDO_LEARN_STATE_MAX,
    } gdo_learn_state_t;

    typedef enum
    {
        GDO_PAIRED_DEVICE_TYPE_ALL = 0,
        GDO_PAIRED_DEVICE_TYPE_REMOTE,
        GDO_PAIRED_DEVICE_TYPE_KEYPAD,
        GDO_PAIRED_DEVICE_TYPE_WALL_CONTROL,
        GDO_PAIRED_DEVICE_TYPE_ACCESSORY,
        GDO_PAIRED_DEVICE_TYPE_MAX,
    } gdo_paired_device_type_t;

    typedef enum
    {
        GDO_PROTOCOL_UNKNOWN = 0,
        GDO_PROTOCOL_SEC_PLUS_V1 = 1,
        GDO_PROTOCOL_SEC_PLUS_V2,
        GDO_PROTOCOL_SEC_PLUS_V1_WITH_SMART_PANEL,
        GDO_PROTOCOL_DRY_CONTACT,
        GDO_PROTOCOL_MAX,
    } gdo_protocol_type_t;

    typedef enum
    {
        GDO_CB_EVENT_SYNCED = 0,
        GDO_CB_EVENT_OBSTRUCTION,
        GDO_CB_EVENT_DOOR_POSITION,
        GDO_CB_EVENT_LIGHT,
        GDO_CB_EVENT_LOCK,
        GDO_CB_EVENT_MOTOR,
        GDO_CB_EVENT_BUTTON,
        GDO_CB_EVENT_BATTERY,
        GDO_CB_EVENT_LEARN,
        GDO_CB_EVENT_OPENINGS,
        GDO_CB_EVENT_MOTION,
        GDO_CB_EVENT_SET_TTC,
        GDO_CB_EVENT_CANCEL_TTC,
        GDO_CB_EVENT_UPDATE_TTC,
        GDO_CB_EVENT_PAIRED_DEVICES,
        GDO_CB_EVENT_OPEN_DURATION_MEASUREMENT,
        GDO_CB_EVENT_CLOSE_DURATION_MEASUREMENT,
        GDO_CB_EVENT_TOF_TIMER,
        GDO_CB_EVENT_MAX,
    } gdo_cb_event_t;

    typedef struct
    {
        uint8_t total_remotes;
        uint8_t total_keypads;
        uint8_t total_wall_controls;
        uint8_t total_accessories;
        uint8_t total_all;
    } gdo_paired_device_t;

    typedef struct
    {
        gdo_protocol_type_t protocol;         // Protocol type
        gdo_door_state_t door;                // Door state
        gdo_light_state_t light;              // Light state
        gdo_lock_state_t lock;                // Lock state
        gdo_motion_state_t motion;            // Motion state
        gdo_obstruction_state_t obstruction;  // Obstruction state
        gdo_motor_state_t motor;              // Motor state
        gdo_button_state_t button;            // Button state
        gdo_battery_state_t battery;          // Battery state
        gdo_learn_state_t learn;              // Learn state
        gdo_paired_device_t paired_devices;   // Paired devices
        gdo_door_state_t last_move_direction; // Last move direction
        bool synced;                          // Synced state
        bool ttc_enabled;                     // ttc active
        bool toggle_only;                     // Used when the door opener only supports the toggle command.
        bool obst_override;                   // Used when the door opener has no obstruction sensors.
        bool tof_timer_active;                // ToF interval timer active
        bool obst_test_pulse_timer_active;    // Obstruction test pulse output pin timer active
        uint16_t openings;                    // Number of openings
        uint16_t ttc_seconds;                 // Time to close in seconds
        uint16_t open_ms;                     // Time door takes to open from fully closed in milliseconds
        uint16_t close_ms;                    // Time door takes to close from fully open in milliseconds
        uint16_t vehicle_parked_threshold;    // Distance thats considered a parked state
        uint16_t vehicle_parked_threshold_variance; // The variance used to determine if the vehicle is parked
        int32_t door_position;                // Door position in percentage (0-10000) [OPEN-CLOSED]
        int32_t door_target;                  // Door target position in percentage (0-10000) [OPEN-CLOSED]
        uint32_t client_id;                   // Client ID
        uint32_t rolling_code;                // Rolling code
        uint32_t tof_timer_usecs;             // ToF interval timer microseconds use to triger TOF events
        uint32_t obst_test_pulse_timer_usecs; // Obstruction test pulse output pin timer microseconds
    } gdo_status_t;

    typedef struct
    {
        uart_port_t uart_num;             // UART port number
        bool obst_from_status;            // Use obstruction status from status message
        bool invert_uart;                 // Invert UART signal
        gpio_num_t uart_tx_pin;           // UART TX pin
        gpio_num_t uart_rx_pin;           // UART RX pin
        gpio_num_t obst_in_pin;           // Obstruction input pin
        gpio_num_t obst_tp_pin;           // Obstruction test pulse pin
        gpio_num_t rf_tx_pin;             // RF TX pin
        gpio_num_t rf_rx_pin;             // RF RX pin
        gpio_num_t dc_open_pin;           // dry contact open sensor input pin
        gpio_num_t dc_close_pin;          // dry contact close sensor input pin
        gpio_num_t dc_toggle_pin;         // dry contact toggle input pin
        gpio_num_t dc_discrete_open_pin;  // dry contact open door output pin
        gpio_num_t dc_discrete_close_pin; // dry contact close door output pin
        uint32_t dc_debounce_ms;          // dry contact debounce timer duration in milliseconds.
    } gdo_config_t;

#define GDO_PAIRED_DEVICE_COUNT_UNKNOWN 0xff

    /**
     * @brief Callback function for GDO events.
     * @param status The current status of the GDO.
     * @param event The event that occurred.
     * @param user_arg optional user argument to be passed to the callback.
     */
    typedef void (*gdo_event_callback_t)(const gdo_status_t *status, gdo_cb_event_t event, void *user_arg);

    /**
     * @brief Initializes the GDO driver.
     * @param config The configuration for the GDO driver.
     * @return ESP_OK on success, ESP_ERR_NO_MEM if task creation fails, ESP_ERR_INVALID_STATE if the driver is not initialized.
     */
    esp_err_t gdo_init(const gdo_config_t *config);

    /**
     * @brief Stops and deletes the GDO driver and resets all state values to defaults.
     * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the driver is not initialized.
     */
    esp_err_t gdo_deinit(void);

    /**
     * @brief Starts the GDO driver and the UART.
     * @param event_callback The callback function to be called when an event occurs.
     * @return ESP_OK on success, ESP_ERR_NO_MEM if task creation fails, other non-zero errors.
     */
    esp_err_t gdo_start(gdo_event_callback_t event_callback, void *user_arg);

    /**
     * @brief Gets the current status of the GDO.
     * @param status a pointer to the status structure to be filled.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG if status is NULL.
     * @note This function is perfomred in a critical section and should be called with caution.
     */
    esp_err_t gdo_get_status(gdo_status_t *status);

    /**
     * @brief Starts the task that syncs the state of the GDO with the controller.
     * @return ESP_OK on success, ESP_ERR_NO_MEM if task creation fails, ESP_ERR_NOT_FINISHED if the task is already running.
     */
    esp_err_t gdo_sync(void);

    /**
     * @brief Opens the door.
     * @return ESP_OK on success, other non-zero errors.
     */
    esp_err_t gdo_door_open(void);

    /**
     * @brief Closes the door.
     * @return ESP_OK on success, other non-zero errors.
     */
    esp_err_t gdo_door_close(void);

    /**
     * @brief Stops the door.
     * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if the encoding fails.
     */
    esp_err_t gdo_door_stop(void);

    /**
     * @brief Toggles the door.
     * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if the encoding fails.
     */
    esp_err_t gdo_door_toggle(void);

    /**
     * @brief Moves the door to a specific target.
     * @param target The target position to move the door to.
     * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if the encoding fails.
     */
    esp_err_t gdo_door_move_to_target(uint32_t target);

    /**
     * @brief Turns the light on.
     * @param check if true then query the door status after setting light on.
     * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if the encoding fails.
     */
    esp_err_t gdo_light_on();
    esp_err_t gdo_light_on_check(bool check);

    /**
     * @brief Turns the light off.
     * @param check if true then query the door status after setting light off.
     * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if the encoding fails.
     */
    esp_err_t gdo_light_off();
    esp_err_t gdo_light_off_check(bool check);

    /**
     * @brief Toggles the light.
     * @return ESP_OK on success, ESP_ERR_NOT_FOUND if current state is unknown,
     * ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if the encoding fails.
     */
    esp_err_t gdo_light_toggle(void);

    /**
     * @brief Locks the door.
     * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if the encoding fails.
     */
    esp_err_t gdo_lock(void);

    /**
     * @brief Unlocks the door.
     * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if the encoding fails.
     */
    esp_err_t gdo_unlock(void);

    /**
     * @brief Toggles the lock.
     * @return ESP_OK on success, ESP_ERR_NOT_FOUND if current state is unknown,
     * ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if the encoding fails.
     */
    esp_err_t gdo_toggle_lock(void);

    /**
     * @brief Activates the learn mode.
     * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if the encoding fails.
     */
    esp_err_t gdo_activate_learn(void);

    /**
     * @brief Deactivates the learn mode.
     * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if the encoding fails.
     */
    esp_err_t gdo_deactivate_learn(void);

    /**
     * @brief Clears the paired devices.
     * @param type The type of paired devices to clear.
     * @return ESP_OK on success, ESP_ERR_NO_MEM if the queue is full, ESP_FAIL if the encoding fails.
     */
    esp_err_t gdo_clear_paired_devices(gdo_paired_device_type_t type);

    /**
     * @brief Get the door state as a string.
     * @param state The door state.
     * @return The door state as a string.
     */
    const char *gdo_door_state_to_string(gdo_door_state_t state);

    /**
     * @brief Get the light state as a string.
     * @param state The light state.
     * @return The light state as a string.
     */
    const char *gdo_light_state_to_string(gdo_light_state_t state);

    /**
     * @brief Get the lock state as a string.
     * @param state The lock state.
     * @return The lock state as a string.
     */
    const char *gdo_lock_state_to_string(gdo_lock_state_t state);

    /**
     * @brief Get the motion state as a string.
     * @param state The motion state.
     * @return The motion state as a string.
     */
    const char *gdo_motion_state_to_string(gdo_motion_state_t state);

    /**
     * @brief Get the obstruction state as a string.
     * @param state The obstruction state.
     * @return The obstruction state as a string.
     */
    const char *gdo_obstruction_state_to_string(gdo_obstruction_state_t state);

    /**
     * @brief Get the motor state as a string.
     * @param state The motor state.
     * @return The motor state as a string.
     */
    const char *gdo_motor_state_to_string(gdo_motor_state_t state);

    /**
     * @brief Get the button state as a string.
     * @param state The button state.
     * @return The button state as a string.
     */
    const char *gdo_button_state_to_string(gdo_button_state_t state);

    /**
     * @brief Get the battery state as a string.
     * @param state The battery state.
     * @return The battery state as a string.
     */
    const char *gdo_battery_state_to_string(gdo_battery_state_t state);

    /**
     * @brief Get the learn state as a string.
     * @param state The learn state.
     * @return The learn state as a string.
     */
    const char *gdo_learn_state_to_string(gdo_learn_state_t state);

    /**
     * @brief Get the paired device type as a string.
     * @param type The paired device type.
     * @return The paired device type as a string.
     */
    const char *gdo_paired_device_type_to_string(gdo_paired_device_type_t type);

    /**
     * @brief Get the protocol type as a string.
     * @param type The protocol type.
     * @return The protocol type as a string.
     */
    const char *gdo_protocol_type_to_string(gdo_protocol_type_t type);

    /**
     * @brief Sets the Security+ V2 rolling code.
     * @param rolling_code The rolling code to set.
     * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the GDO is already synced.
     */
    esp_err_t gdo_set_rolling_code(uint32_t rolling_code);

    /**
     * @brief Sets the Security+ V2 client id.
     * @param client_id The client id to set.
     * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the GDO is already synced.
     */
    esp_err_t gdo_set_client_id(uint32_t client_id);

    /**
     * @brief Sets the protocol to use to communicate with the GDO.
     * @param protocol The protocol to use.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the protocol is invalid,
     * ESP_ERR_INVALID_STATE if the protocol is already set.
     */
    esp_err_t gdo_set_protocol(gdo_protocol_type_t protocol);

    /**
     * @brief Sets the time to close in minutes
     * @param time_to_close The time to close value.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the time is invalid,
     * ESP_ERR_INVALID_STATE if the time is out of range.
     */
    esp_err_t gdo_set_time_to_close(uint16_t time_to_close);

    /**
     * @brief Sets the time the door takes to open from fully closed in milliseconds.
     * @param ms The time the door takes to open from fully closed in milliseconds.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the ms is invalid.
     */
    esp_err_t gdo_set_open_duration(uint16_t ms);

    /**
     * @brief Sets the time the door takes to close from fully open in milliseconds.
     * @param ms The time the door takes to close from fully open in milliseconds.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the ms is invalid.
     */
    esp_err_t gdo_set_close_duration(uint16_t ms);

    /**
     * @brief Sets the minimum time in milliseconds to wait between sending consecutive commands.
     * @param ms The minimum time in milliseconds.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the time is invalid.
     */
    esp_err_t gdo_set_min_command_interval(uint32_t ms);

    /**
     * @brief Enables or disables the toggle only mode, may be required by openers that do not have obstruction sensors connected.
     * @param toggle_only true to enable toggle only mode, false to disable.
     */
    void gdo_set_toggle_only(bool toggle_only);

    /**
     * @brief Enables or disables obstruction override, some openers that do not have obstruction sensors connected.
     * @param obst_override true to enable override, false to disable.
     */
    void gdo_set_obst_override(bool obst_override);

    /************************************* VEHICLE Functions *****************************************/

    /**
     * @brief Set the user interval timer 1 value and enable/disable flag
     * @param interval the interval time in micro seconds
     * @param enabled the flag to enable or disable the timer on gdo_start
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the interval is less than 1000
     */
    esp_err_t gdo_set_tof_timer(uint32_t interval, bool enabled);

    /**
     * @brief Set the obst test pulse interval timer value and enable/disable flag
     * @param interval the interval time in micro seconds
     * @param enabled the flag to enable or disable the timer on gdo_start
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the interval is less than 1000
     * @note Init config->obst_tp_pin with the desired GPIO pin
    */
    esp_err_t gdo_set_obst_test_pulse_timer(uint32_t interval, bool enabled);

    /**
     * @brief Sets the vehicle parked threshold in cm
     * @param vehicle_parked_threshold distance measure that triggers a parked state
     */
    esp_err_t gdo_set_vehicle_parked_threshold(uint16_t vehicle_parked_threshold);

    /**
     * @brief Sets the vehicle parked threshold variance in cm
     * @param vehicle_parked_threshold_variance within this variance the state will be stable
     */
    esp_err_t gdo_set_vehicle_parked_threshold_variance(uint16_t vehicle_parked_threshold_variance);

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // GDO_H
