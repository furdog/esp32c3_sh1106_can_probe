#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

static const char *TAG = "can_probe";

#undef LED_BUILTIN
#define LED_BUILTIN 8

#define SDA_PIN 5
#define SCL_PIN 6

/******************************************************************************
 * DELTA TIME
 *****************************************************************************/
struct delta_time {
	uint32_t timestamp_prev;
};

void delta_time_init(struct delta_time *self) { self->timestamp_prev = 0; }

uint32_t delta_time_update_ms(struct delta_time *self, uint32_t timestamp)
{
	uint32_t delta_time_ms = 0;

	delta_time_ms	     = timestamp - self->timestamp_prev;
	self->timestamp_prev = timestamp;

	return delta_time_ms;
}

/******************************************************************************
 * TWAI functionality (ESP-IDF)
 *****************************************************************************/
#include "esp_twai.h"
#include "esp_twai_onchip.h"

twai_node_handle_t	  node_hdl    = NULL;
twai_onchip_node_config_t node_config = {
    .io_cfg.tx		= 3,	  // TWAI TX GPIO pin
    .io_cfg.rx		= 4,	  // TWAI RX GPIO pin
    .bit_timing.bitrate = 500000, // 500 kbps bitrate (most common)
    .tx_queue_depth	= 5,	  // Transmit queue depth set to 5
    //.fail_retry_cnt 	= -1,     // Retry indefinitely on TX failure
    .fail_retry_cnt = 8, // Retry 8 times on TX failure

    .flags.enable_self_test   = true,
    .flags.enable_loopback    = true,
    .flags.enable_listen_only = false};

static volatile uint32_t esp_twai_rx_count = 0;

/** Pin probe (Short feedback pulse test)
 *  Sends a short pulse (LOW-dominant) on TX and waits for RX to go recessive.
 *
 *  @info: Only use this function when ESP-TWAI is not initialized. */
bool esp_twai_test_dominant_feedback()
{
	bool probe_result = true;

	assert(node_hdl == NULL);

	// Temporarily configure pins as standard GPIOs with Pull-ups
	gpio_set_direction(node_config.io_cfg.tx, GPIO_MODE_OUTPUT);
	gpio_set_direction(node_config.io_cfg.rx, GPIO_MODE_INPUT);
	gpio_pullup_en(node_config.io_cfg.rx);

	// Reset TX to recessive state
	//
	// Now replace digitalwrite and digitalread with esp idf equivalents
	// everywhere:
	gpio_set_level(node_config.io_cfg.tx, 1);
	esp_rom_delay_us(100);
	if (gpio_get_level(node_config.io_cfg.tx) == 1 &&
	    gpio_get_level(node_config.io_cfg.rx) == 1) {
		// Both TX and RX are HIGH (recessive state)
	} else {
		// TX or RX is not in the expected state
		probe_result = false;
	}

	if (probe_result) {
		// Change TX to dominant state and observe RX feedback
		gpio_set_level(node_config.io_cfg.tx, 0);
		esp_rom_delay_us(100);
		if (gpio_get_level(node_config.io_cfg.tx) == 0 &&
		    gpio_get_level(node_config.io_cfg.rx) == 0) {
			// Both TX and RX are LOW (dominant state)
		} else {
			probe_result = false;
		}
	}

	// Restore TX to recessive state
	gpio_set_level(node_config.io_cfg.tx, 1);
	esp_rom_delay_us(100);

	return probe_result;
}

/** Pin probe (Transciever dominant timeout function test)
 * ISO-11898-2 states it must be between 0.8ms and 10.0ms.
 * It is also optional and can not be observed in some devices.
 *
 * @warning: Only use this function when ESP-TWAI is not initialized. */
bool esp_twai_test_dominant_timeout()
{
	assert(node_hdl == NULL);

	// Temporarily configure pins as standard GPIOs with Pull-ups
	gpio_set_direction(node_config.io_cfg.tx, GPIO_MODE_OUTPUT);
	gpio_set_direction(node_config.io_cfg.rx, GPIO_MODE_INPUT);
	gpio_pullup_en(node_config.io_cfg.rx);

	gpio_set_level(node_config.io_cfg.tx, 0);
	esp_rom_delay_us(14000); /** 14ms (off-limits) */
	if (gpio_get_level(node_config.io_cfg.tx) == 0 &&
	    gpio_get_level(node_config.io_cfg.rx) == 1) {
		// Transceiver dominant timeout function test passed
		return true;
	} else {
		return false;
	}
}

static bool twai_rx_probe(twai_node_handle_t		   handle,
			  const twai_rx_done_event_data_t *edata,
			  void				  *user_ctx)
{
	uint8_t	     recv_buff[8];
	twai_frame_t rx_frame = {
	    .buffer	= recv_buff,
	    .buffer_len = sizeof(recv_buff),
	};

	if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
		esp_twai_rx_count++;
	}

	return false;
}

void esp_twai_init(uint32_t bitrate)
{
	if (node_hdl != NULL) {
		return;
	}

	node_config.bit_timing.bitrate = bitrate;

	twai_event_callbacks_t user_cbs = {
	    .on_rx_done = twai_rx_probe,
	};

	// Create a new TWAI controller driver instance
	ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_hdl));

	ESP_ERROR_CHECK(
	    twai_node_register_event_callbacks(node_hdl, &user_cbs, NULL));

	// Start the TWAI controller
	ESP_ERROR_CHECK(twai_node_enable(node_hdl));
}

void esp_twai_deinit()
{
	if (node_hdl != NULL) {
		twai_node_disable(node_hdl);
		ESP_ERROR_CHECK(twai_node_delete(node_hdl));
		node_hdl = NULL;
	};

	esp_twai_rx_count = 0;
}

esp_err_t esp_twai_transmit_once()
{
	uint8_t	     send_buff[8] = {0};
	twai_frame_t tx_msg	  = {
	    .header.id	= 0x1,		     /* Message ID */
	    .header.ide = false,	     /* Use standard ID format */
	    .buffer	= send_buff,	     /* Pointer to data to transmit */
	    .buffer_len = sizeof(send_buff), /* Length of data to transmit */
	};
	twai_node_transmit(
	    node_hdl, &tx_msg,
	    0); /* Timeout = 0: returns immediately if queue is full */

	return twai_node_transmit_wait_all_done(
	    node_hdl, 100); /* Wait for transmission to finish (up to 100ms) */
}

/******************************************************************************
 * Generic CAN probing device (ESP-IDF)
 * May depend on specific implementation calls.
 *****************************************************************************/
#define ACTION_DELAY_MS 500

/** Various (commonly used) bitrates for the CAN probing. */
enum can_common_bitrates {
	CAN_COMMON_BITRATE_10K = 0,
	CAN_COMMON_BITRATE_20K,
	CAN_COMMON_BITRATE_50K,
	CAN_COMMON_BITRATE_100K,
	CAN_COMMON_BITRATE_125K,
	CAN_COMMON_BITRATE_200K,
	CAN_COMMON_BITRATE_250K,
	CAN_COMMON_BITRATE_400K,
	CAN_COMMON_BITRATE_500K,
	CAN_COMMON_BITRATE_800K,
	CAN_COMMON_BITRATE_1000K,

	CAN_COMMON_BITRATE_MAX
};

/** List of common CAN bitrates. Must match the enum can_common_bitrates. */
const uint32_t can_common_bitrates_list[] = {10000,  20000,  50000,  100000,
					     125000, 200000, 250000, 400000,
					     500000, 800000, 1000000};

/** Twai probe state machine state. */
enum can_probe_state {
	CAN_PROBE_STATE_TRANSCEIVER_TEST,
	CAN_PROBE_STATE_LISTENING_TEST,
	CAN_PROBE_STATE_LOOPBACK_TEST
};

const char *can_probe_state_names[] = {
    "TJA_Test",
    "RX_Test",
    "LO_Test",
};

struct can_probe {
	bool can_bus_is_ok;

	/** Results of the dominant feedback and timeout tests. */
	bool dominant_feedback_test_passed;
	bool dominant_timeout_test_passed;
	bool listening_test_passed;
	bool loopback_test_passed;

	uint8_t bitrate_idx;

	uint8_t state;
};

void can_probe_init(struct can_probe *self)
{
	/* At startup, assume the CAN bus is not OK. */
	self->can_bus_is_ok = false;

	/* Results of the dominant feedback and timeout tests. */
	self->dominant_feedback_test_passed = false;
	self->dominant_timeout_test_passed  = false;
	self->listening_test_passed	    = false;
	self->loopback_test_passed	    = false;

	self->bitrate_idx = CAN_COMMON_BITRATE_125K; /**< Default to 125K. */

	/* Start with the transceiver test state.
	 * We'll also test other states, until we prove the CAN bus is OK. */
	self->state = CAN_PROBE_STATE_TRANSCEIVER_TEST;
}

void can_probe_fsm_update(struct can_probe *self, uint32_t delta_time_ms)
{
	switch (self->state) {
	case CAN_PROBE_STATE_TRANSCEIVER_TEST:
		ESP_LOGI(TAG, "Performing transceiver test.");
		self->can_bus_is_ok		    = false;
		self->dominant_feedback_test_passed = false;
		self->dominant_timeout_test_passed  = false;

		/** Active line check (may disrupt bus integrity) */
		self->dominant_feedback_test_passed =
		    esp_twai_test_dominant_feedback();
		self->dominant_timeout_test_passed =
		    esp_twai_test_dominant_timeout();

		/* If both tests pass, assume the bus is OK. */
		if (self->dominant_feedback_test_passed &&
		    self->dominant_timeout_test_passed) {
			self->can_bus_is_ok = true;
		} else {
			ESP_LOGI(TAG,
				 "Transceiver test failed. "
				 "dominant_feedback=%d, dominant_timeout=%d",
				 self->dominant_feedback_test_passed,
				 self->dominant_timeout_test_passed);

			/* Dominant feedback and timeout tests may fail if
			 * there's any activity on the bus. So we don't assume
			 * the bus is OK yet. */
		}

		vTaskDelay(pdMS_TO_TICKS(ACTION_DELAY_MS));

		self->state = CAN_PROBE_STATE_LISTENING_TEST;
		break;

	case CAN_PROBE_STATE_LISTENING_TEST:
		self->listening_test_passed = false;

		ESP_LOGI(TAG, "Performing passive listen test. Bitrate: %d",
			 can_common_bitrates_list[self->bitrate_idx]);

		node_config.flags.enable_listen_only = true;

		esp_twai_init(can_common_bitrates_list[self->bitrate_idx]);

		vTaskDelay(pdMS_TO_TICKS(ACTION_DELAY_MS));
		if (esp_twai_rx_count > 0) {
			self->can_bus_is_ok	    = true;
			self->listening_test_passed = true;

			esp_twai_rx_count = 0;

			/* Don't move anywhere, stay in this state.
			 * until there's no more activity on the bus. */
			break;
		}

		esp_twai_deinit();

		ESP_LOGI(TAG, "Passive listen test failed.");
		self->listening_test_passed = false;
		self->state		    = CAN_PROBE_STATE_LOOPBACK_TEST;
		break;

	case CAN_PROBE_STATE_LOOPBACK_TEST:
		ESP_LOGI(TAG, "Performing loopback test.");

		self->loopback_test_passed = false;

		node_config.flags.enable_listen_only = false;

		if (!self->can_bus_is_ok) {
			ESP_LOGI(TAG, "Self-test enabled.");
			node_config.flags.enable_self_test = true;
		} else {
			node_config.flags.enable_self_test = false;
		}

		esp_twai_init(can_common_bitrates_list[self->bitrate_idx]);

		esp_twai_transmit_once();
		bool tx_failed = true;

		twai_node_status_t node_status;
		if (twai_node_get_info(node_hdl, &node_status, NULL) ==
		    ESP_OK) {
			tx_failed = node_status.tx_error_count > 0;
			ESP_LOGI(TAG, "tx_error_count: %d",
				 node_status.tx_error_count);
		}

		vTaskDelay(pdMS_TO_TICKS(ACTION_DELAY_MS));
		if (esp_twai_rx_count > 0 ||
		    (!node_config.flags.enable_self_test && !tx_failed)) {
			self->can_bus_is_ok	    = true;
			self->listening_test_passed = true;

			ESP_LOGI(TAG, "Loopback test passed.");
			self->loopback_test_passed = true;
		} else {
			ESP_LOGI(TAG,
				 "Loopback test failed. Changing bitrate.");

			/* Move to the next bitrate if loopback test failed,
			 * wrapping around if necessary. */
			self->bitrate_idx =
			    (self->bitrate_idx + 1) % CAN_COMMON_BITRATE_MAX;
		}

		esp_twai_deinit();

		vTaskDelay(pdMS_TO_TICKS(ACTION_DELAY_MS));

		if (self->can_bus_is_ok) {
			self->state = CAN_PROBE_STATE_LISTENING_TEST;
		} else {
			/* All tests failed, move to transceiver test. */
			self->state = CAN_PROBE_STATE_TRANSCEIVER_TEST;
		}
		break;
	default:
		break;
	}
}

/******************************************************************************
 * DISPLAY
 *****************************************************************************/
#include "u8g2.h"
#include "u8g2_esp32_hal.h"

#define PIN_SDA GPIO_NUM_5
#define PIN_SCL GPIO_NUM_6

u8g2_t u8g2;

/******************************************************************************
 * Display functionality
 *****************************************************************************/
void display_init(void)
{
	ESP_LOGI(TAG, "Initializing U8g2 HAL...");

	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.bus.i2c.sda	= PIN_SDA;
	u8g2_esp32_hal.bus.i2c.scl	= PIN_SCL;
	u8g2_esp32_hal_init(u8g2_esp32_hal);

	u8g2_Setup_ssd1306_i2c_72x40_er_2(&u8g2, U8G2_R0,
					  u8g2_esp32_i2c_byte_cb,
					  u8g2_esp32_gpio_and_delay_cb);

	u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);

	u8g2_ClearBuffer(&u8g2);
	u8g2_SetFont(&u8g2, u8g2_font_unifont_t_chinese3);
}

void display_update(struct can_probe *self)
{
	char buf[32];

	u8g2_FirstPage(&u8g2);
	do {
		u8g2_DrawStr(&u8g2, 5, 19, can_probe_state_names[self->state]);

		snprintf(buf, sizeof(buf), "%lu",
			 can_common_bitrates_list[self->bitrate_idx]);
		u8g2_DrawStr(&u8g2, 5, 34, buf);
	} while (u8g2_NextPage(&u8g2));
}

/******************************************************************************
 * SETUP/LOOP
 *****************************************************************************/
struct delta_time delta_time;
struct can_probe  can_probe;
static bool	  led_state = false;

void app_main(void)
{
	delta_time_init(&delta_time);
	can_probe_init(&can_probe);
	display_init();

	while (1) {
		uint32_t delta_time_ms = delta_time_update_ms(
		    &delta_time, esp_timer_get_time() / 1000);

		gpio_set_level(LED_BUILTIN, led_state);
		led_state = !led_state;

		display_update(&can_probe);
		can_probe_fsm_update(&can_probe, delta_time_ms);

		vTaskDelay(pdMS_TO_TICKS(1));
	}
}
