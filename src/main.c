#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/sht4x.h>

#include <math.h>

/* NTC thermistors */

#define NTC_TEMP_SIZE 10

#define SUPPLY_MILLIVOLTS 3300 /* Make sure this is accurate to regulator voltage */

#define REF_RESISTANCE 47000

#define NTC_REF_TEMPERATURE 25
#define NTC_REF_RESISTANCE 50000
#define NTC_BETA 3950

#define ADC_DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     ADC_DT_SPEC_AND_COMMA)
};

static uint16_t NTC_value[(ARRAY_SIZE(adc_channels))][NTC_TEMP_SIZE] = {0};
static int8_t last_NTC_value[(ARRAY_SIZE(adc_channels))] = {0};

static float NTC_temperature[(ARRAY_SIZE(adc_channels))] = {0};

static float ntc_get_temperature_mV(float mV)
{
	float NTC_resistance = SUPPLY_MILLIVOLTS * REF_RESISTANCE / mV - REF_RESISTANCE;
	float temperature = 1.0 / (1.0 / (NTC_REF_TEMPERATURE + 273.15) + log(NTC_resistance / NTC_REF_RESISTANCE) / NTC_BETA) - 273.15;
	return temperature;
}

static void adc_thread(void)
{
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

	for (size_t i = 0U; i < (ARRAY_SIZE(adc_channels)); i++)
		adc_channel_setup_dt(&adc_channels[i]);

	while (1) {
		for (size_t i = 0U; i < (ARRAY_SIZE(adc_channels)); i++)
		{
			int32_t val_mv;

			adc_sequence_init_dt(&adc_channels[i], &sequence);

			adc_read_dt(&adc_channels[i], &sequence);

			val_mv = (int32_t)buf;
			adc_raw_to_millivolts_dt(&adc_channels[i], &val_mv);
			NTC_value[i][last_NTC_value[i]] = val_mv;
			last_NTC_value[i] = (last_NTC_value[i] + 1) % NTC_TEMP_SIZE;

			val_mv = 0;
			for (int j = 0; j < NTC_TEMP_SIZE; j++)
				val_mv += NTC_value[i][j];
			NTC_temperature[i] = ntc_get_temperature_mV(val_mv / (float)NTC_TEMP_SIZE);
		}

		k_sleep(K_MSEC(1));
	}
}

K_THREAD_DEFINE(adc_thread_id, 512, adc_thread, NULL, NULL, NULL, 6, 0, 0);

/* Fan tachometers */

#define TACHOMETER_TIME_SIZE 10

#define GPIO_DT_SPEC_AND_COMMA(node_id, prop, idx) \
	GPIO_DT_SPEC_GET_BY_IDX(node_id, prop, idx),

static const struct gpio_dt_spec tachometers[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), tachometer_gpios,
			     GPIO_DT_SPEC_AND_COMMA)
};

static struct gpio_callback tachometer_cb_data;

static int64_t tachometer_time[(ARRAY_SIZE(tachometers))][TACHOMETER_TIME_SIZE] = {0};
static int8_t last_tachometer_time[(ARRAY_SIZE(tachometers))] = {0};

static float fan_tachometer[(ARRAY_SIZE(tachometers))] = {0};

static void tachometer_pulsed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	for (size_t i = 0U; i < (ARRAY_SIZE(tachometers)); i++)
	{
		if (pins & BIT(tachometers[i].pin))
		{
			last_tachometer_time[i] = (last_tachometer_time[i] + 1) % TACHOMETER_TIME_SIZE;
			tachometer_time[i][last_tachometer_time[i]] = k_uptime_ticks();
		}
	}
}

static void fan_tachometer_thread(void)
{
	uint32_t tachometer_pins = 0;

	for (size_t i = 0U; i < (ARRAY_SIZE(tachometers)); i++)
	{
		gpio_pin_configure_dt(&tachometers[i], GPIO_INPUT);
		gpio_pin_interrupt_configure_dt(&tachometers[i], GPIO_INT_EDGE_FALLING);
		tachometer_pins |= BIT(tachometers[i].pin);
	}
	gpio_init_callback(&tachometer_cb_data, tachometer_pulsed, tachometer_pins);
	gpio_add_callback(tachometers[0].port, &tachometer_cb_data);

	while (1)
	{
		for (size_t i = 0U; i < (ARRAY_SIZE(tachometers)); i++)
		{
			if (k_uptime_ticks() - tachometer_time[i][last_tachometer_time[i]] > CONFIG_SYS_CLOCK_TICKS_PER_SEC / 20)
			{
				fan_tachometer[i] = 0;
				continue;
			}
			float dt =
				(tachometer_time[i][last_tachometer_time[i]] -
				 tachometer_time[i][(last_tachometer_time[i] + 1) % TACHOMETER_TIME_SIZE]) *
				2.0f / (float)TACHOMETER_TIME_SIZE / (float)CONFIG_SYS_CLOCK_TICKS_PER_SEC;
			fan_tachometer[i] = 60.0f / dt;
		}
		k_sleep(K_MSEC(10));
	}
}

K_THREAD_DEFINE(fan_tachometer_thread_id, 512, fan_tachometer_thread, NULL, NULL, NULL, 6, 0, 0);

/* PWM controls */

#define FAN_PWM_MIN 0.25f
#define FAN_PWM_MAX 0.65f

#define PELTIER_PWM_MIN 0.0f
#define PELTIER_PWM_MAX 1.0f

#define PWM_DT_SPEC_AND_COMMA(node_id, prop, idx) \
	PWM_DT_SPEC_GET_BY_IDX(node_id, idx),

static const struct pwm_dt_spec pwm_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), pwms,
			     PWM_DT_SPEC_AND_COMMA)
};

static float pwm_duty[(ARRAY_SIZE(pwm_channels))] = {0};

static void fan_set_pwm(uint32_t id, float duty_cycle)
{
	duty_cycle = duty_cycle < 0 ? 0 : duty_cycle;
	duty_cycle = duty_cycle > 1 ? 1 : duty_cycle;
	duty_cycle = FAN_PWM_MIN + (FAN_PWM_MAX - FAN_PWM_MIN) * duty_cycle;
	pwm_duty[id] = duty_cycle;
	pwm_set_pulse_dt(&pwm_channels[id], pwm_channels[id].period * duty_cycle);
}

static void peltier_set_pwm(uint32_t id, float duty_cycle)
{
	id += 3;
	duty_cycle = duty_cycle < 0 ? 0 : duty_cycle;
	duty_cycle = duty_cycle > 1 ? 1 : duty_cycle;
	duty_cycle = PELTIER_PWM_MIN + (PELTIER_PWM_MAX - PELTIER_PWM_MIN) * duty_cycle;
	pwm_duty[id] = duty_cycle;
	pwm_set_pulse_dt(&pwm_channels[id], pwm_channels[id].period * duty_cycle);
}

/* SHT4X */

#if !DT_HAS_COMPAT_STATUS_OKAY(sensirion_sht4x)
#error "No sensirion,sht4x compatible node found in the device tree"
#endif

static float sht_temperature = 0;
static float sht_humidity = 0;

static void sht_thread(void)
{
	const struct device *const sht = DEVICE_DT_GET_ANY(sensirion_sht4x);
	struct sensor_value temp, hum;

	while (1)
	{
		if (sensor_sample_fetch(sht))
		{
			printf("Failed to fetch sample from SHT4X device\n");
			return;
		}

		sensor_channel_get(sht, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(sht, SENSOR_CHAN_HUMIDITY, &hum);

		sht_temperature = sensor_value_to_float(&temp);
		sht_humidity = sensor_value_to_float(&hum);
	}
}

K_THREAD_DEFINE(sht_thread_id, 1024, sht_thread, NULL, NULL, NULL, 6, 0, 0);

/* MCU temperature */

#if !DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_temp)
#error "No nordic,nrf-temp compatible node found in the device tree"
#endif

static float mcu_temperature = 0;

static void temp_thread(void)
{
	const struct device *const dev = DEVICE_DT_GET_ANY(nordic_nrf_temp);
	struct sensor_value temp;

	while (1)
	{
		if (sensor_sample_fetch(dev))
		{
			printf("Failed to fetch sample from nRF temperature device\n");
			return;
		}

		sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp);

		mcu_temperature = sensor_value_to_float(&temp);
	}
}

K_THREAD_DEFINE(temp_thread_id, 512, temp_thread, NULL, NULL, NULL, 6, 0, 0);

/* Dryer loop */

#define AMBIENT_TEMPERATURE 25.0f

#define CHAMBER_TARGET_TEMPERATURE 43.0f
#define CHAMBER_TEMPERATURE_TOLERANCE 0.5f
#define CHAMBER_TARGET_HUMIDITY 10.0f
#define CHAMBER_HUMIDITY_TOLERANCE 0.25f

#define CHAMBER_PELTIER_TARGET_TEMPERATURE (CHAMBER_TARGET_TEMPERATURE + 14.0f)
#define CHAMBER_PELTIER_TEMPERATURE_TOLERANCE 2.0f

#define EXTERNAL_PELTIER_TARGET_TEMPERATURE (AMBIENT_TEMPERATURE + 12.0f)
#define EXTERNAL_PELTIER_TEMPERATURE_TOLERANCE 4.0f

#define CHAMBER_PELTIER_HEATSINK_NTC 1
#define EXTERNAL_PELTIER_HEATSINK_NTC 3

#define CHAMBER_HEATING_FAN 0
#define EXTERNAL_COOLING_FAN 1
#define DEHUMIDIFIER_FAN 2

#define CHAMBER_PELTIER 0
#define EXTERNAL_PELTIER 1

void dryer_thread(void)
{
	float duty;
	while (1)
	{
		float duty_int_temp = (sht_temperature - CHAMBER_TARGET_TEMPERATURE) / CHAMBER_TEMPERATURE_TOLERANCE;
		float duty_int_hum = (sht_humidity - CHAMBER_TARGET_HUMIDITY) / CHAMBER_HUMIDITY_TOLERANCE;
		float duty_int_ntc = (NTC_temperature[CHAMBER_PELTIER_HEATSINK_NTC] - CHAMBER_PELTIER_TARGET_TEMPERATURE) / CHAMBER_PELTIER_TEMPERATURE_TOLERANCE;
		float duty_ext_ntc = (NTC_temperature[EXTERNAL_PELTIER_HEATSINK_NTC] - EXTERNAL_PELTIER_TARGET_TEMPERATURE) / EXTERNAL_PELTIER_TEMPERATURE_TOLERANCE;
	
		duty = (1.0f - duty_int_temp) / 2.0f;
		fan_set_pwm(CHAMBER_HEATING_FAN, duty);

//		duty = (1.0f + MAX(duty_int_temp, duty_int_hum)) / 2.0f;
//		fan_set_pwm(DEHUMIDIFIER_FAN, duty);
		fan_set_pwm(DEHUMIDIFIER_FAN, 0);

		duty = (1.0f + duty_ext_ntc) / 2.0f;
		fan_set_pwm(EXTERNAL_COOLING_FAN, duty);

		duty = (1.0f - MAX(duty_int_temp, duty_int_ntc)) / 2.0f;
		peltier_set_pwm(CHAMBER_PELTIER, duty);

		duty = (1.0f + MAX(duty_int_temp - 1.0f, duty_int_hum)) / 2.0f;
		peltier_set_pwm(EXTERNAL_PELTIER, duty);
//		fan_set_pwm(EXTERNAL_COOLING_FAN, duty);

		k_sleep(K_MSEC(1));
	}
}

K_THREAD_DEFINE(dryer_thread_id, 512, dryer_thread, NULL, NULL, NULL, 6, 0, 0);

/* Debugging */

static const char *ntc_labels[4] = {
	"Internal Peltier Condenser",
	"Internal Peltier Heatsink",
	"External Peltier Condenser",
	"External Peltier Heatsink"};

static const char *tach_labels[5] = {
	"Chamber Heating Fan 1",
	"Chamber Heating Fan 2",
	"External Cooling Fan 1",
	"External Cooling Fan 2",
	"Dehumidifier Fan"};

static const char *pwm_labels[5] = {
	"Chamber Heating Fans",
	"External Cooling Fans",
	"Dehumidifier Fan",
	"Internal Peltier",
	"External Peltier"}; 

int main(void)
{
	while (1)
	{
		printk("\nMCU Temperature: %.2f C\n", (double)mcu_temperature);
		printk("Chamber Temperature: %.2f C\n", (double)sht_temperature);
		printk("Chamber Humidity: %.2f%%\n", (double)sht_humidity);
		for (int i = 0; i < 4; i++)
			printk("%s: %.2f C\n", ntc_labels[i], (double)NTC_temperature[i]);
		for (int i = 0; i < 5; i++)
			printk("%s: %.0f rpm\n", tach_labels[i], (double)fan_tachometer[i]);
		for (int i = 0; i < 5; i++)
			printk("%s: %.2f%%\n", pwm_labels[i], (double)(pwm_duty[i] * 100));
		k_sleep(K_MSEC(1000));
	}
	return 0;
}
