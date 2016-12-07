/*
 * Copyright 2011-2013 Ayla Networks, Inc.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of Ayla Networks, Inc.
 */

#include <string.h>
#include <ayla/mcu_platform.h>
#include <mcu_io.h>
#include <stm32.h>

#ifdef DEMO_CONF
#include <ayla/conf_token.h>
#include <ayla/conf_access.h>
#endif

#include <ayla/ayla_proto_mcu.h>
#include <ayla/props.h>
#include <ayla/serial_msg.h>

#ifdef DEMO_POST
#include <post.h>
#endif /* DEMO_POST */

#ifdef DEMO_CONSOLE
#include <stdio.h>
#undef putchar
#undef getchar
#endif /* DEMO_CONSOLE */

#ifdef DEMO_UART
#include <ayla/uart.h>
#else
#include <spi_platform_arch.h>
#endif /* DEMO_UART */

#ifdef DEMO_FILE_PROP
#include <ayla/prop_dp.h>
#include <demo_stream.h>
#endif /* DEMO_FILE_PROP */
#ifdef DEMO_IR
#include <ir_io.h>
#endif /* DEMO_IR */

#ifdef DEMO_FILE_PROP
#define VERSION "demo_dp 1.6"
#else
#define VERSION "demo 1.6"
#endif /* DEMO_FILE_PROP */

#ifdef DEMO_SCHED_LIB
#include <ayla/schedeval.h>
#include <ayla/sched.h>
#include <ayla/cmp.h>
#endif /* DEMO_SCHED_LIB */

#ifdef DEMO_CONSOLE
#include <stdio.h>
#include <ayla/atty.h>
#include <ayla/console.h>
#endif /* DEMO_CONSOLE */

#ifdef DEMO_IMG_MGMT
#include <flash_layout.h>
#ifdef AYLA_BUILD_VERSION
#include <build.h>
#else
#define BUILD_DATE ""
#endif /* AYLA_BUILD_VERSION */
#ifdef AYLA_KEIL
/*
 * Image header location is fixed.
 */
#define IMG_HDR_LOC		(MCU_IMG_ACTIVE + IMAGE_HDR_OFF)
#define IMG_HDR_VER_LOC		(IMG_HDR_LOC + sizeof(struct image_hdr))

const struct image_hdr __img_hdr
			__attribute__((used))
			__attribute((at(IMG_HDR_LOC)));
const char version[72] __attribute((at(IMG_HDR_VER_LOC))) =
	 VERSION " " BUILD_DATE;
#else
const char version[] __attribute__((section(".version"))) =
	VERSION " " BUILD_DATE;
#endif /* AYLA_KEIL */
#elif AYLA_BUILD_VERSION
#include <build.h>
const char version[] = VERSION " " BUILD_DATE;
#else
const char version[] = VERSION;
#endif /* DEMO_IMG_MGMT || AYLA_BUILD_VERSION */

static u8 factory_reset;

#ifdef DEMO_CONSOLE
#define SOURCES {				\
		[SRC_CLOUD] = "cloud",		\
		[SRC_APP1] = "app1",		\
		[SRC_APP2] = "app2",		\
		[SRC_APP3] = "app3",		\
		[SRC_APP4] = "app4",		\
		[SRC_APP5] = "app5",		\
		[SRC_APP6] = "app6",		\
		[SRC_APP7] = "app7",		\
		[SRC_MCU] = "mcu",		\
		[SRC_USER] = "user",		\
	}
static const char const *sources[] = SOURCES;
#endif

static void set_input(struct prop *, void *arg, void *valp, size_t len);
static void set_cmd(struct prop *, void *arg, void *valp, size_t len);
static void set_dec_in(struct prop *prop, void *arg, void *valp, size_t len);

#ifdef DEMO_IMG_MGMT
extern u8 boot2inactive;
extern u8 template_req;
void mcu_img_mgmt_init(void);
int send_inactive_version(struct prop *, void *arg);
void set_boot2inactive(struct prop *, void *arg, void *valp, size_t len);
int send_template_version(struct prop *, void *arg);
void template_version_sent(void);
#endif

static s32 input;
static s32 output;
static s32 decimal_in;
static s32 decimal_out;
static u8 blue_button;
static char cmd_buf[TLV_MAX_STR_LEN + 1];

#ifdef DEMO_SCHED_LIB
static u8 sched_buf[256];
static u8 sched_out_length;
static const u8 *sched_out;
static struct sched_prop schedule_in;
static void set_schedule_in(struct prop *prop, void *arg, void *valp,
    size_t len);
#endif

#ifdef DEMO_POWER_MGMT
/*
 * Minumum awake time before putting Ayla Module
 * in StandBy if connectivity is not needed
 */
#define MCU_SEND_STANDBY	1 /* Allow MCU to put Ayla Module in standby */
#define MIN_CONNECTIVITY_TIME	60 /* seconds */

static u8 standby_needs_reset;
static u32 next_standby_tick;
static void demo_set_standby(void);
#endif

enum demo_val_err {
	VAL_NO_ERR = 0,
	VAL_BAD_LEN,
	VAL_OUT_OF_RNG
};

static void set_led(struct prop *prop, void *arg, void *valp, size_t len)
{
	u8 val = *(u8 *)valp;
	u32 pin = (u32)arg;

	stm32_set_led(pin, val);
#ifdef DEMO_CONSOLE
	printf("%s set %s to %u", sources[console_src], prop->name, val);
#endif
}

static int send_led(struct prop *prop, void *arg)
{
	u8 val;
	u32 pin = (u32)prop->arg;

	val = stm32_get_led(pin);
	return prop_send(prop, &val, sizeof(val), arg);
}

static int send_version(struct prop *prop, void *arg)
{
	return prop_send(prop, version, strlen(version), arg);
}

static u8 send_prop_mask(void)
{
	/*
	 * For any property updates coming in while the
	 * Ayla Module is in standby or not yet up remember to
	 * send the property update to ADS
	 */
	return valid_dest_mask | ADS_BIT;
}

static int send_prop_with_meta(struct prop *prop, void *arg)
{
	struct datapoint_meta meta[DP_META_MAX_ENTRIES + 1];
	int rc;

	memset(&meta, 0, sizeof(meta));
	meta[0].key = "key0";
	meta[0].val = "val0";
	meta[1].key = "key1";
	meta[1].val = "val1";

	rc = prop_validate_meta(meta);
	if (rc < 0) {
		return rc;
	}
	return prop_send_meta(prop, meta, arg);
}

static int send_file_prop_with_meta(struct prop *prop, void *arg)
{
	struct datapoint_meta meta[DP_META_MAX_ENTRIES + 1];
	int rc;

	memset(&meta, 0, sizeof(meta));
	meta[0].key = "key0";
	meta[0].val = "val0";
	meta[1].key = "key1";
	meta[1].val = "val1";

	rc = prop_validate_meta(meta);
	if (rc < 0) {
		return rc;
	}
	return prop_dp_send_meta(prop, meta, arg);
}

#ifdef DEMO_SCHED_LIB
static int send_schedule(struct prop *prop, void *arg)
{
	return prop_send(prop, sched_out, sched_out_length, arg);
}
#endif

#ifdef DEMO_IR
char ir_encode_proto[20];
u32 ir_encode;

static void set_ir_encode_proto(struct prop *prop, void *arg,
	void *valp, size_t len)
{
	if (len >= sizeof(ir_encode_proto) - 1) {
		return;
	}
	memcpy(ir_encode_proto, valp, len);
	ir_encode_proto[len] = '\0';
}

static void set_ir_encode(struct prop *prop, void *arg, void *valp, size_t len)
{
	if (len != sizeof(u32)) {
		return;
	}
	ir_encode = *(u32 *)valp;
	if (ir_encode_proto[0]) {
		ir_send(ir_encode_proto, ir_encode);
	}
}
#endif /* DEMO_IR */

struct prop prop_table[] = {
	{ "Blue_button", ATLV_BOOL, NULL, send_prop_with_meta,
	    &blue_button, sizeof(blue_button), AFMT_READ_ONLY},
#define PROP_BUTTON 0
	{ "output", ATLV_INT, NULL, prop_send_generic, &output,
	    sizeof(output), AFMT_READ_ONLY},
#define PROP_OUTPUT 1
	{ "log", ATLV_UTF8, NULL, prop_send_generic, &cmd_buf[0],
	    0, AFMT_READ_ONLY},
#define PROP_LOG 2
	{ "decimal_out", ATLV_CENTS, NULL, prop_send_generic, &decimal_out,
	    sizeof(decimal_out), AFMT_READ_ONLY},
#define PROP_DEC_OUT 3
	{ "decimal_in", ATLV_CENTS, set_dec_in, prop_send_generic,
	    &decimal_in, sizeof(decimal_in)},
	{ "Blue_LED", ATLV_BOOL, set_led, send_led,
	    (void *)(1 << LED0_PIN), 1},
	{ "Green_LED", ATLV_BOOL, set_led, send_led,
	    (void *)(1 << LED1_PIN), 1},
#ifdef DEMO_SCHED_LIB
	{ "schedule_in", ATLV_SCHED, set_schedule_in, NULL, &schedule_in},
	{ "schedule_out", ATLV_SCHED, NULL, send_schedule, NULL, 0,
	  AFMT_READ_ONLY},
#define PROP_SCHED_OUT 8
#endif
	{ "cmd", ATLV_UTF8, set_cmd, prop_send_generic, &cmd_buf[0]},
	{ "input", ATLV_INT, set_input, prop_send_generic,
	    &input, sizeof(input)},
	{ "version", ATLV_UTF8, NULL, send_version, NULL, 0, AFMT_READ_ONLY},
#ifdef DEMO_IMG_MGMT
	{ "inactive_version", ATLV_UTF8, NULL, send_inactive_version, NULL,
	  0, AFMT_READ_ONLY },
	{ "boot_to_inactive", ATLV_BOOL, set_boot2inactive, prop_send_generic,
	  &boot2inactive, sizeof(boot2inactive) },
	{ "oem_host_version", ATLV_UTF8, NULL, send_template_version },
#endif
#ifdef DEMO_FILE_PROP
	/*
	 * Long data points must use property type ATLV_LOC in this table,
	 * even though they have type ATLV_BIN in the protocol.
	 */
	{ "stream_up_len", ATLV_INT, set_length_up, prop_send_generic,
	    &stream_up_len, sizeof(stream_up_len)},
	{ "stream_up", ATLV_LOC, NULL, send_file_prop_with_meta, &stream_up_state, 0},
	{ "stream_down", ATLV_LOC, prop_dp_set, prop_dp_send,
	    &stream_down_state, 0},
	{ "stream_down_len", ATLV_UINT, NULL, prop_send_generic,
	   &stream_down_state.next_off, sizeof(stream_down_state.next_off)},
	{ "stream_down_match_len", ATLV_UINT, NULL, prop_send_generic,
	   &stream_down_patt_match_len, sizeof(stream_down_patt_match_len)},
#endif /* DEMO_FILE_PROP */
#ifdef DEMO_IR
	{ "ir_encode_proto", ATLV_UTF8, set_ir_encode_proto, prop_send_generic,
	   &ir_encode_proto, sizeof(ir_encode_proto) - 1},
	{ "ir_encode", ATLV_UINT, set_ir_encode, prop_send_generic,
	   &ir_encode, sizeof(ir_encode)},
#endif /* DEMO_IR */
	{ NULL }
};
u8 prop_count = (sizeof(prop_table) / sizeof(prop_table[0])) - 1;

static void set_input(struct prop *prop, void *arg, void *valp, size_t len)
{
	s32 i = *(s32 *)valp;

	if (len != sizeof(s32)) {
		/*
		 * Demo for end-to-end datapoint acks. Enable acks for
		 * input property in the template.
		 * ack_status = 1 (failure), it is 0 by default.
		 */
		prop->ack.ack_status = 1;
		prop->ack.ack_message = VAL_BAD_LEN;
		return;
	}
	input = i;
	if (i > 0x7fff || i < -0x8000) {
		output = -1;		/* square would overflow */
		/*
		 * Demo for end-to-end datapoint acks.
		 */
		prop->ack.ack_status = 1;
		prop->ack.ack_message = VAL_OUT_OF_RNG;
	} else {
		output = i * i;
	}
#ifdef DEMO_CONSOLE
	printf("%s set %s to %ld", sources[console_src], prop->name, input);
	printf("%s set output to %ld", sources[SRC_MCU], output);
#else
	prop_table[PROP_OUTPUT].send_mask = send_prop_mask();
#endif
}

static void set_dec_in(struct prop *prop, void *arg, void *valp, size_t len)
{
	s32 i = *(s32 *)valp;

	if (len != sizeof(s32)) {
		return;
	}
	decimal_in = i;
	decimal_out = i;
#ifdef DEMO_CONSOLE
	printf("%s set decimal_in to %ld", sources[console_src], decimal_in);
	printf("%s set decimal_out to %ld", sources[SRC_MCU], decimal_out);
#else
	prop_table[PROP_DEC_OUT].send_mask = send_prop_mask();
#endif
}

static void set_cmd(struct prop *prop, void *arg, void *valp, size_t len)
{
	if (len >= sizeof(cmd_buf)) {
		len = sizeof(cmd_buf) - 1;
	}
	memcpy(cmd_buf, valp, len);
	cmd_buf[len] = '\0';
#ifdef DEMO_CONSOLE
	printf("%s set cmd to %s", sources[console_src], cmd_buf);
	printf("%s set log to %s", sources[SRC_MCU], cmd_buf);
#else
	prop_table[PROP_LOG].send_mask = send_prop_mask();
#endif
}

#ifdef DEMO_SCHED_LIB
static void set_schedule_in(struct prop *prop, void *arg, void *valp,
    size_t len)
{
	if (len > sizeof(sched_buf)) {
		len = sizeof(sched_buf);
	}
	memcpy(sched_buf, valp, len);
	sched_out = sched_buf;
	sched_out_length = len;
	prop_table[PROP_SCHED_OUT].send_mask = send_prop_mask();
#ifdef DEMO_CONSOLE
	printf("%s set %s", sources[console_src], prop->name);
	printf("%s set %s", sources[SRC_MCU], prop_table[PROP_SCHED_OUT].name);
#endif
	memcpy(schedule_in.tlvs, valp, len);
	schedule_in.len = sched_out_length;
	sched_run_all(NULL);
}
#endif

#ifdef DEMO_PROP_TEST
static u16 prop_test_duration = 30;

static void prop_test_read_cb(void *buf, size_t len)
{
	struct ayla_tlv *tlv;
	u8 status;
	enum conf_token prop_test[] = {CT_server, CT_prop, CT_time_limit};
	u16 val;

	tlv = (struct ayla_tlv *)buf;
	if (tlv->len != sizeof(status)) {
		return;
	}
	memcpy(&status, tlv + 1, sizeof(status));

	/* toggle the status */
	status ^= 1;

	if (status) {
		/* if duration not set, send without, but set for next time. */
		val = htons(prop_test_duration);
		prop_test_duration += 10;
		prop_test_duration %= 70;
	} else {
		val = htons(0);
	}
	conf_write(1, prop_test, 3, ATLV_UINT, &val, sizeof(val));
}

static void prop_test_toggle(void)
{
	enum conf_token server_prop[] = {CT_server, CT_prop};

	conf_read(server_prop, 2, prop_test_read_cb);
}
#endif

/*
 * Blue button push observed by interrupt handler.
 * Callers are in stm32.c
 */
void demo_set_button_state(u8 button_value)
{
	blue_button = button_value;
	prop_table[PROP_BUTTON].send_mask = send_prop_mask();
#ifdef DEMO_PROP_TEST
	if (blue_button) {
		prop_test_toggle();
	}
#endif
}

static void demo_event_cb(u8 ev_type, void *ev_datap, u8 ev_datalen)
{
	u8 event;

	event = *(u8 *)(ev_datap);
	switch (ev_type) {
	case ATLV_REGINFO:
		if (ev_datalen == sizeof(u8)) {
			if (event) {
				/* user registered */
			} else {
				/* user deregistered */
			}
#ifdef DEMO_UREG_LED
			stm32_set_led((1 << UREG_LED_PIN), event);
#endif
		}
		break;
	case ATLV_EVENT_MASK:
#ifdef DEMO_POWER_MGMT
		if (ev_datalen == sizeof(u8) && event & AYLA_EVENT_STANDBY) {
			/* Ayla module in standby mode */
			standby_needs_reset = 1;
		}
#endif
		break;
	default:
		/* ignore unknown events */
		break;
	}
}

#ifdef DEMO_POWER_MGMT
static inline void set_next_standby_tick(void)
{
	next_standby_tick = tick + MIN_CONNECTIVITY_TIME * SYSTICK_HZ;
}

/*
 * Send MCU standby request
 */
static void demo_set_standby(void)
{
	enum conf_token standby[] = {CT_power, CT_standby};
	u8 val = 1;

	conf_write(1, standby, 2, ATLV_BOOL, &val, sizeof(val));
}

/*
 * Determine if wireless connectivity is required
 */
static int connectivity_needed(void)
{
	if (prop_is_busy() || prop_pending() || conf_is_busy() ||
	    (!standby_needs_reset && !(valid_dest_mask & ADS_BIT))) {
		set_next_standby_tick();
		return 1;
	}

	/*
	 * If connectivity is not needed wait min
	 * time before putting Ayla Module in StandBy
	 */
	if (!standby_needs_reset && cmp_gt(next_standby_tick, tick)) {
		return 1;
	}
	return 0;
}
#endif

/*
 * Demo module reset recovery handler. Used to carry out
 * actions required upon module resets.
 */
static void demo_module_reset_cb(void)
{
#ifdef DEMO_POWER_MGMT
	/*
	 * Reset conf_pwr_mode to receive the latest power
	 * mode setting on module reset.
	 */
	conf_pwr_mode = -1;
#endif
}

int main(int argc, char **argv)
{
	struct prop *prop;

	feature_mask |= MCU_LAN_SUPPORT;
#ifdef DEMO_IMG_MGMT
	mcu_img_mgmt_init();
	feature_mask |= MCU_OTA_SUPPORT;
#endif
#ifdef DEMO_SCHED_LIB
	feature_mask |= MCU_TIME_SUBSCRIPTION;
#endif
	mcu_io_init();
#ifdef DEMO_UART
	feature_mask |= MCU_DATAPOINT_CONFIRM;
	uart_init();
#else
	spi_platform_init();
#endif
#ifdef DEMO_CONSOLE
	atty_init(console_cli);
#endif
	stm32_reset_module();
	stm32_init();
#ifdef DEMO_IR
	ir_init();
#endif /* DEMO_IR */
#ifdef DEMO_POST
	if (post_enabled()) {
		post();
	}
#endif /* DEMO_POST */
	factory_reset = stm32_factory_reset_detect();
#ifdef DEMO_FILE_PROP
	demo_stream_init();
#endif /* DEMO_FILE_PROP */
#ifdef DEMO_POWER_MGMT
	set_next_standby_tick();
#endif
#ifdef DEMO_CONSOLE
	printf("\n\nmcu init done");
#endif
	/*
	 * Set callback handlers if needed.
	 */
	module_reset_cb = &demo_module_reset_cb;
	prop_event_cb = &demo_event_cb;

	for (;;) {
		if (stm32_ready()) {
			if (factory_reset &&
			    !serial_tx_cmd(ACMD_LOAD_FACTORY, NULL, 0)) {
				factory_reset = 0;
				stm32_set_factory_rst_led(0);
				while (stm32_ready()) {
					serial_poll();
				}
			}
#ifdef DEMO_CONF
			conf_poll();
#endif
			prop_poll();
			serial_poll();
#ifdef DEMO_CONSOLE
			console_platform_poll();
#endif
#ifdef DEMO_SCHED_LIB
			if (sched_next_event_tick &&
			    (tick == sched_next_event_tick ||
			    cmp_gt(tick, sched_next_event_tick))) {
				sched_run_all(&sched_next_event_tick);
			}
#endif
#ifdef DEMO_POWER_MGMT
			/* Put Ayla Module if connectivity not required */
			if (!standby_needs_reset && !connectivity_needed() &&
			    MCU_SEND_STANDBY && (conf_pwr_mode == CT_standby)) {
				demo_set_standby();
				standby_needs_reset = 1;
			}
#endif
#ifdef DEMO_POWER_MGMT
		} else if (standby_needs_reset) {
			/*
			 * Ayla module is in Stand-By. Reset is
			 * needed to resume
			 */
			if (connectivity_needed()) {
				/* Reset the module */
				stm32_reset_module();
				standby_needs_reset = 0;

				/*
				 * Send features and get connectivity
				 * mask again on a reset
				 */
				prop_update_connectivity(0);
				features_sent = 0;
			}
#endif
		}
#ifdef DEMO_IMG_MGMT
		if (template_req &&
		    prop_send_done(prop_lookup("oem_host_version")) == 0) {
			/*
			 * Template version number has been sent.
			 */
			template_version_sent();
		}
#endif
		prop = prop_lookup_error();
		if (prop != NULL) {
			/*
			 * Property send has failed with error code.
			 * Error code is available in prop->send_err
			 *
			 * Insert logic here to handle the failure.
			 */
			prop->send_err = 0;
		}
	}
}
