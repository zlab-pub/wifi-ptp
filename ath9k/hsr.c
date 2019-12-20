/*
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Kirill Berezin
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <linux/io.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/bitops.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include <asm/unaligned.h>

#include "hw.h"
#include "ath9k.h"

#define HSR_GPIO_CSN 8
#define HSR_GPIO_CLK 6
#define HSR_GPIO_DOUT 7
#define HSR_GPIO_DIN 5

/* delays are in useconds */
#define HSR_DELAY_HALF_TICK 100
#define HSR_DELAY_PRE_WRITE 75
#define HSR_DELAY_FINAL 20000
#define HSR_DELAY_TRAILING 200

void ath9k_hsr_init(struct ath_hw *ah)
{
	ath9k_hw_gpio_request_in(ah, HSR_GPIO_DIN, NULL);
	ath9k_hw_gpio_request_out(ah, HSR_GPIO_CSN, NULL,
				  AR_GPIO_OUTPUT_MUX_AS_OUTPUT);
	ath9k_hw_gpio_request_out(ah, HSR_GPIO_CLK, NULL,
				  AR_GPIO_OUTPUT_MUX_AS_OUTPUT);
	ath9k_hw_gpio_request_out(ah, HSR_GPIO_DOUT, NULL,
				  AR_GPIO_OUTPUT_MUX_AS_OUTPUT);

	ath9k_hw_set_gpio(ah, HSR_GPIO_CSN, 1);
	ath9k_hw_set_gpio(ah, HSR_GPIO_CLK, 0);
	ath9k_hw_set_gpio(ah, HSR_GPIO_DOUT, 0);

	udelay(HSR_DELAY_TRAILING);
}

static u32 ath9k_hsr_write_byte(struct ath_hw *ah, int delay, u32 value)
{
	struct ath_common *common = ath9k_hw_common(ah);
	int i;
	u32 rval = 0;

	udelay(delay);

	ath9k_hw_set_gpio(ah, HSR_GPIO_CLK, 0);
	udelay(HSR_DELAY_HALF_TICK);

	ath9k_hw_set_gpio(ah, HSR_GPIO_CSN, 0);
	udelay(HSR_DELAY_HALF_TICK);

	for (i = 0; i < 8; ++i) {
		rval = rval << 1;

		/* pattern is left to right, that is 7-th bit runs first */
		ath9k_hw_set_gpio(ah, HSR_GPIO_DOUT, (value >> (7 - i)) & 0x1);
		udelay(HSR_DELAY_HALF_TICK);

		ath9k_hw_set_gpio(ah, HSR_GPIO_CLK, 1);
		udelay(HSR_DELAY_HALF_TICK);

		rval |= ath9k_hw_gpio_get(ah, HSR_GPIO_DIN);

		ath9k_hw_set_gpio(ah, HSR_GPIO_CLK, 0);
		udelay(HSR_DELAY_HALF_TICK);
	}

	ath9k_hw_set_gpio(ah, HSR_GPIO_CSN, 1);
	udelay(HSR_DELAY_HALF_TICK);

	ath_dbg(common, CONFIG, "ath9k_hsr_write_byte: write byte %d return value is %d %c\n",
		value, rval, rval > 32 ? rval : '-');

	return rval & 0xff;
}

static int ath9k_hsr_write_a_chain(struct ath_hw *ah, char *chain, int items)
{
	int status = 0;
	int i = 0;
	int err;

	/* a preamble */
	ath9k_hsr_write_byte(ah, HSR_DELAY_PRE_WRITE, 0);
	status = ath9k_hsr_write_byte(ah, HSR_DELAY_PRE_WRITE, 0);

	/* clear HSR's reply buffer */
	if (status) {
		int loop = 0;

		for (loop = 0; (loop < 42) && status; ++loop)
			status = ath9k_hsr_write_byte(ah, HSR_DELAY_PRE_WRITE,
						      0);

		if (loop >= 42) {
			ATH_DBG_WARN(1,
				     "ath9k_hsr_write_a_chain: can't clear an output buffer after a 42 cycles.\n");
			return -1;
		}
	}

	for (i = 0; (i < items) && (chain[i] != 0); ++i)
		ath9k_hsr_write_byte(ah, HSR_DELAY_PRE_WRITE, (u32)chain[i]);

	ath9k_hsr_write_byte(ah, HSR_DELAY_PRE_WRITE, 0);
	mdelay(HSR_DELAY_FINAL / 1000);

	/* reply */
	memset(chain, 0, items);

	ath9k_hsr_write_byte(ah, HSR_DELAY_PRE_WRITE, 0);
	udelay(HSR_DELAY_TRAILING);

	for (i = 0; i < (items - 1); ++i) {
		u32 ret;

		ret = ath9k_hsr_write_byte(ah, HSR_DELAY_PRE_WRITE, 0);
		if (ret != 0)
			chain[i] = (char)ret;
		else
			break;

		udelay(HSR_DELAY_TRAILING);
	}

	if (i <= 1)
		return 0;

	err = kstrtoint(chain + 1, 10, &i);
	if (err)
		return err;

	return i;
}

int ath9k_hsr_disable(struct ath_hw *ah)
{
	char cmd[10] = {'b', '4', '0', 0, 0, 0, 0, 0, 0, 0};
	int ret;

	ret = ath9k_hsr_write_a_chain(ah, cmd, sizeof(cmd));
	if ((ret > 0) && (*cmd == 'B'))
		return 0;

	return -1;
}

int ath9k_hsr_enable(struct ath_hw *ah, int bw, int fq)
{
	char cmd[10];
	int ret;

	/* Bandwidth argument is 0 sometimes. Assume default 802.11bgn
	 * 20MHz on invalid values
	 */
	if ((bw != 5) && (bw != 10) && (bw != 20) && (bw != 40))
		bw = 20;

	memset(cmd, 0, sizeof(cmd));
	*cmd = 'b';
	snprintf(cmd + 1, 3, "%02d", bw);

	ret = ath9k_hsr_write_a_chain(ah, cmd, sizeof(cmd));
	if ((*cmd != 'B') || (ret != bw)) {
		ATH_DBG_WARN(1,
			     "ath9k_hsr_enable: failed changing bandwidth -> set (%d,%d) reply (%d, %d)\n",
			     'b', bw, *cmd, ret);
		return -1;
	}

	memset(cmd, 0, sizeof(cmd));
	*cmd = 'x';
	ret = ath9k_hsr_write_a_chain(ah, cmd, sizeof(cmd));
	if (*cmd != 'X') {
		ATH_DBG_WARN(1,
			     "ath9k_hsr_enable: failed 'x' command -> reply (%d, %d)\n",
			     *cmd, ret);
		return -1;
	}

	memset(cmd, 0, sizeof(cmd));
	*cmd = 'm';
	ret = ath9k_hsr_write_a_chain(ah, cmd, sizeof(cmd));
	if (*cmd != 'M') {
		ATH_DBG_WARN(1,
			     "ath9k_hsr_enable: failed 'm' command -> reply (%d, %d)\n",
			     *cmd, ret);
		return  -1;
	}

	memset(cmd, 0, sizeof(cmd));
	*cmd = 'f';
	snprintf(cmd + 1, 6, "%05d", fq);
	ret = ath9k_hsr_write_a_chain(ah, cmd, sizeof(cmd));
	if ((*cmd != 'F') && (ret != fq)) {
		ATH_DBG_WARN(1,
			     "ath9k_hsr_enable: failed set frequency -> reply (%d, %d)\n",
			     *cmd, ret);
		return -1;
	}

	return 0;
}

int ath9k_hsr_status(struct ath_hw *ah)
{
	char cmd[10] = {'s', 0, 0, 0, 0, 0, 0, 0, 0, 0};
	int ret;

	ret = ath9k_hsr_write_a_chain(ah, cmd, sizeof(cmd));
	if (*cmd != 'S') {
		ATH_DBG_WARN(1, "ath9k_hsr_status: returned %d,%d\n", *cmd,
			     ret);
		return -1;
	}

	return 0;
}
