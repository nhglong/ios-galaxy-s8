/* SPDX-License-Identifier: GPL-2.0-only
 * Galaxy S8 PMIC (S2MPS17) access over the Exynos SPEEDY bus, MMU off.
 *
 * touch_probe v25 found the touch controller unpowered under our boot: SDA
 * held low (HSI2C master state LOSE) and the gpa1-0 interrupt line at 0.
 * Its rails are S2MPS17 LDO34 "tsp_io" 1.85 V and LDO35 "tsp_avdd" 3.30 V;
 * S-Boot leaves them off and Samsung's Linux turns them on. The SPEEDY
 * transfer and the LDO values follow uniLoader (board-dreamlte.c,
 * drivers/samsung/exynos-speedy.c); the RTC layout follows Samsung's
 * rtc-s2mp.h (RTC on SPEEDY slave 2, binary counters from 0x04).
 */
#include <stdint.h>
#include "pmic.h"

#define SPEEDY			0x15b50000UL
#define SP_FIFO_CTRL		0x004
#define SP_CMD			0x008
#define SP_INT_ENABLE		0x00c
#define SP_INT_STATUS		0x010
#define SP_TX_DATA		0x034
#define SP_RX_DATA		0x038
#define SP_FIFO_RESET		(1u << 31)
#define SP_LEN(rx, tx)		((rx) | (tx) << 8)
#define SP_RANDOM		(1u << 19)
#define SP_WRITE		(1u << 20)
#define SP_SLAVE(x)		(((x) & 0xf) << 15)
#define SP_ADDR(x)		(((x) & 0xff) << 7)
#define SP_DONE			(1u << 0)

#define SLAVE_PMIC		1
#define SLAVE_RTC		2
#define L34CTRL			0x5f
#define L35CTRL			0x60
#define LDO_EN			(3u << 6)
#define L34_1V85		(LDO_EN | (1850000 - 700000) / 25000)
#define L35_3V3			(LDO_EN | (3300000 - 1800000) / 25000)
#define RTC_UPDATE		0x02
#define RTC_RUDR		0x01
#define RTC_SEC			0x04

#define TIMER_HZ		26000000UL

static void (*kv)(const char *, uint64_t);
static void (*out)(const char *);

static inline uint32_t rd(uintptr_t a) { return *(volatile uint32_t *)a; }
static inline void wr(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = v; }

static uint64_t ticks(void)
{
	uint64_t v;
	__asm__ volatile("isb; mrs %0, cntpct_el0" : "=r"(v));
	return v;
}

static void delay_us(unsigned us)
{
	uint64_t end = ticks() + TIMER_HZ / 1000000 * us;
	while (ticks() < end)
		;
}

/* One random-access byte transfer; returns the byte read, or < 0. */
static int speedy(unsigned slave, unsigned reg, int write, uint8_t val)
{
	wr(SPEEDY + SP_FIFO_CTRL, SP_FIFO_RESET);
	delay_us(10);
	wr(SPEEDY + SP_FIFO_CTRL, SP_LEN(1, 1));
	wr(SPEEDY + SP_INT_ENABLE, write ? 0x80300011u : 0x80070301u);
	wr(SPEEDY + SP_INT_STATUS, 0xffffffffu);
	wr(SPEEDY + SP_CMD, SP_RANDOM | (write ? SP_WRITE : 0) |
	   SP_SLAVE(slave) | SP_ADDR(reg));
	if (write)
		wr(SPEEDY + SP_TX_DATA, val);
	uint64_t end = ticks() + TIMER_HZ / 100;
	while (ticks() < end) {
		uint32_t s = rd(SPEEDY + SP_INT_STATUS);
		if (s & SP_DONE) {
			wr(SPEEDY + SP_INT_STATUS, 0xffffffffu);
			return write ? 0 : (int)(rd(SPEEDY + SP_RX_DATA) & 0xff);
		}
	}
	return -1;
}

static void show(const char *k, unsigned slave, unsigned reg)
{
	int v = speedy(slave, reg, 0, 0);
	kv(k, v < 0 ? 0xdead : (uint64_t)v);
}

/* Cycles of this core per 10 ms of the 26 MHz system timer. */
static void cpu_freq(void)
{
	uint64_t pmcr;
	__asm__ volatile("mrs %0, pmcr_el0" : "=r"(pmcr));
	__asm__ volatile("msr pmcr_el0, %0" :: "r"(pmcr | 1 | 4));	/* E, reset C */
	__asm__ volatile("msr pmcntenset_el0, %0" :: "r"(1UL << 31));
	__asm__ volatile("msr pmccfiltr_el0, %0" :: "r"(1UL << 27));	/* count EL2 too */
	uint64_t c0, c1, t0 = ticks();
	__asm__ volatile("isb; mrs %0, pmccntr_el0" : "=r"(c0));
	while (ticks() - t0 < TIMER_HZ / 100)
		;
	__asm__ volatile("isb; mrs %0, pmccntr_el0" : "=r"(c1));
	kv("cpu MHz", (c1 - c0) / 10000);
}

uint64_t pmic_init(void (*kv_fn)(const char *, uint64_t), void (*out_fn)(const char *))
{
	kv = kv_fn;
	out = out_fn;
	out("PMIC (S2MPS17 over SPEEDY 0x15b50000)\n");
	cpu_freq();
	show("pmic id", SLAVE_PMIC, 0x00);
	show("L34", SLAVE_PMIC, L34CTRL);
	show("L35", SLAVE_PMIC, L35CTRL);
	out("\n");
	int r1 = speedy(SLAVE_PMIC, L34CTRL, 1, L34_1V85);
	int r2 = speedy(SLAVE_PMIC, L35CTRL, 1, L35_3V3);
	kv("write rc", (uint64_t)(uint32_t)(r1 | r2));
	show("L34", SLAVE_PMIC, L34CTRL);
	show("L35", SLAVE_PMIC, L35CTRL);
	out("\n");

	/* RTC: latch the counters (RUDR), then sec..year at 0x04..0x0a. */
	int u = speedy(SLAVE_RTC, RTC_UPDATE, 0, 0);
	speedy(SLAVE_RTC, RTC_UPDATE, 1, (uint8_t)((u < 0 ? 0 : u) | RTC_RUDR));
	delay_us(200);
	uint64_t raw = 0;
	out("RTC");
	show(" ctrl", SLAVE_RTC, 0x00);
	for (unsigned i = 0; i < 8; i++) {
		int v = speedy(SLAVE_RTC, RTC_SEC + i, 0, 0);
		raw |= (uint64_t)(v < 0 ? 0xff : v) << (8 * i);
	}
	kv(" sec..year", raw);
	out("\n");
	return raw;
}
