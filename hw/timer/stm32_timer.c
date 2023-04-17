/*
 * STM32 Microcontroller Timer module
 *
 * Copyright (C) 2010 Andrew Hankins
 *
 * Source code based on pl011.c
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "sysemu/sysemu.h"
#include "hw/arm/stm32.h"
#include "hw/ptimer.h"

#include "hw/stm32-pcb/pcb.h"

/* DEFINITIONS*/

/* See the README file for details on these settings. */
#define DEBUG_STM32_TIMER

#ifdef DEBUG_STM32_TIMER
#define DPRINTF2(fmt, ...)                                       \
    do { fprintf(stderr, "STM32_TIMER: " fmt , ## __VA_ARGS__); } while (0)
#define DPRINTF(fmt, ...) PCB_DPRINTF("STM32_TIMER " fmt, ## __VA_ARGS__)
#else
#define DPRINTF(fmt, ...)
#endif

#define TIMER_CR1_OFFSET   0x00
#define TIMER_CR2_OFFSET   0x04
#define TIMER_SMCR_OFFSET  0x08
#define TIMER_DIER_OFFSET  0x0c
#define TIMER_SR_OFFSET    0x10
#define TIMER_EGR_OFFSET   0x14
#define TIMER_CCMR1_OFFSET 0x18
#define TIMER_CCMR2_OFFSET 0x1c
#define TIMER_CCER_OFFSET  0x20
#define TIMER_CNT_OFFSET   0x24
#define TIMER_PSC_OFFSET   0x28
#define TIMER_ARR_OFFSET   0x2c
#define TIMER_RCR_OFFSET   0x30
#define TIMER_CCR1_OFFSET  0x34
#define TIMER_CCR2_OFFSET  0x38
#define TIMER_CCR3_OFFSET  0x3c
#define TIMER_CCR4_OFFSET  0x40
#define TIMER_BDTR_OFFSET  0x44
#define TIMER_DCR_OFFSET   0x48
#define TIMER_DMAR_OFFSET  0x4C

#define TIMER_CR1_OPM      0x8
#define TIMER_CR1_CEN      0x1
#define TIMER_CR1_DIR_DOWN 0x10
#define TIMER_CR1_CMS_MASK 0x60

#define TIMER_CCER_CC4E   (1<<12)
#define TIMER_CCER_CC3E   (1<<8)
#define TIMER_CCER_CC2E   (1<<4)
#define TIMER_CCER_CC1E   (1<<0)

#define TIM_OC1M(a) ((a->ccmr1>>4)&7)
#define TIM_OC2M(a) ((a->ccmr1>>12)&7)
#define TIM_OC3M(a) ((a->ccmr2>>4)&7)
#define TIM_OC4M(a) ((a->ccmr2>>12)&7)

#define TIM_CC1P(a) ((a->ccer>>1)&1)
#define TIM_CC2P(a) ((a->ccer>>5)&1)
#define TIM_CC3P(a) ((a->ccer>>9)&1)
#define TIM_CC4P(a) ((a->ccer>>13)&1)

#define TIM_CC1S(a) ((a->ccmr1) &3)
#define TIM_CC2S(a) ((a->ccmr1>>8) &3)
#define TIM_CC3S(a) ((a->ccmr2) &3)
#define TIM_CC4S(a) ((a->ccmr2>>8) &3)

#define TIM_CC1E(a) ((a->ccer) & 1)
#define TIM_CC2E(a) ((a->ccer>>4) & 1)
#define TIM_CC3E(a) ((a->ccer>>8) & 1)
#define TIM_CC4E(a) ((a->ccer>>12) & 1)

#define TIM_IC1PSC(a) (((a->ccmr1)>>2) & 3)
#define TIM_IC2PSC(a) (((a->ccmr1)>>10) & 3)
#define TIM_IC3PSC(a) (((a->ccmr2)>>2) & 3)
#define TIM_IC4PSC(a) (((a->ccmr2)>>10) & 3)

#define TIM_IC1F(a) ((a->ccmr1>>4) & 0xf)
#define TIM_IC2F(a) ((a->ccmr1>>12) & 0xf)
#define TIM_IC3F(a) ((a->ccmr2>>4) & 0xf)
#define TIM_IC4F(a) ((a->ccmr2>>12) & 0xf)

#define CC1IF_MASK (1<<1)
#define CC2IF_MASK (1<<2)
#define CC3IF_MASK (1<<3)
#define CC4IF_MASK (1<<4)

#define CC1OF_MASK (1<<9)
#define CC2OF_MASK (1<<10)
#define CC3OF_MASK (1<<11)
#define CC4OF_MASK (1<<12)

#define TIM3_CH1_PORT 0
#define TIM3_CH2_PORT 0
#define TIM3_CH3_PORT 1
#define TIM3_CH4_PORT 1

#define TIM3_CH1_PIN 6
#define TIM3_CH2_PIN 7
#define TIM3_CH3_PIN 0
#define TIM3_CH4_PIN 1

#define GPIOx_BSRR_OFFSET 0x10
#define GPIOx_IDR_OFFSET 0x08
enum
{
    TIMER_UP_COUNT     = 0,
    TIMER_DOWN_COUNT   = 1
};

struct Stm32Timer {

    /* Inherited */
    SysBusDevice busdev;

    MemoryRegion  iomem;
    ptimer_state *timer;
    struct QEMUTimer *timer_cc1;
    struct QEMUTimer *timer_cc2;
    struct QEMUTimer *timer_cc3;
    struct QEMUTimer *timer_cc4;

    uint8_t channel_gpio_port[4];
    uint8_t channel_gpio_pin[4];
    uint8_t pin_value[4];

    uint8_t ocref;
    qemu_irq      irq;

    /* Properties */
    stm32_periph_t periph;
    void *stm32_rcc_prop;
    void *stm32_gpio_prop;
    void *stm32_afio_prop;

    Stm32Rcc *stm32_rcc;
    Stm32Gpio **stm32_gpio;
    Stm32Afio *stm32_afio;

    int running;
    int countMode;
    int itr;

    uint32_t cr1;
    /* uint32_t cr2; Extended modes not supported */
    /* uint32_t smcr; Slave mode not supported */
    uint32_t dier;
    uint32_t sr;
    uint32_t egr;
    uint32_t ccmr1;
    uint32_t ccmr2;
    uint32_t ccer;
    /* uint32_t cnt; Handled by ptimer */
    uint32_t psc;
    uint32_t arr;
    /* uint32_t rcr; Repetition count not supported */
    uint32_t ccr1;
    uint32_t ccr2;
    uint32_t ccr3;
    uint32_t ccr4;
    /* uint32_t bdtr; Break and deadtime not supported */
    /* uint32_t dcr;  DMA mode not supported */
    /* uint32_t dmar; DMA mode not supported */

};

static void stm32_timer_freq(Stm32Timer *s)
{
    // Why do we need to multiply the frequency by 2?  This is how real hardware
    // behaves.
    uint32_t clk_freq = 2*stm32_rcc_get_periph_freq(s->stm32_rcc, s->periph) / (s->psc + 1);
    uint32_t period_ns = 1000000000ll / clk_freq;
    DPRINTF
    (
        "%s Update freq = 2 * %d / %d = %d (period = %d)\n",
        stm32_periph_name(s->periph),
        stm32_rcc_get_periph_freq(s->stm32_rcc, s->periph),
        (s->psc + 1),
        clk_freq,
        period_ns
    );
    if(clk_freq != 0) {
        ptimer_set_freq(s->timer, clk_freq);
        s->timer_cc1->scale = period_ns;
        s->timer_cc2->scale = period_ns;
        s->timer_cc3->scale = period_ns;
        s->timer_cc4->scale = period_ns;
    }
}

static uint32_t stm32_timer_get_count(Stm32Timer *s)
{
    uint64_t cnt = ptimer_get_count(s->timer);
    if (s->countMode == TIMER_UP_COUNT)
    {
        return s->arr - (cnt & 0xfffff);
    }
    else
    {
        return (cnt & 0xffff);
    }
}

static void stm32_timer_update_ocref(Stm32Timer *s, uint8_t channel, uint32_t ccr, uint8_t mode, uint8_t polarity){
  uint32_t elapsed = stm32_timer_get_count(s);
  uint32_t oc = ccr & 0xffff;
  Stm32Gpio *gpio = s->stm32_gpio[s->channel_gpio_port[channel]];
  uint8_t pin = s->channel_gpio_pin[channel];
  uint8_t old_ocref = s->ocref;

//  DPRINTF("[%d] mode=%d elapsed=%d, ccr=%d\n", channel, mode, elapsed, ccr);
  switch(mode){
    case 0:
      break;
    case 1:
      if (elapsed >= oc) s->ocref |= (1<<channel);
      break;
    case 2:
      if (elapsed >= oc) s->ocref &= ~(1<<channel);
      break;
    case 3:
      if (elapsed >= oc) s->ocref ^= (1<<channel);
      break;
    case 4:
      s->ocref &= ~(1<<channel);
      break;
    case 5:
      s->ocref |= (1<<channel);
      break;
    case 6:
      if (elapsed <= oc){
          s->ocref |= (1<<channel);
      }else{
          s->ocref &= ~(1<<channel);
      }
      break;
    case 7:
      if (elapsed <= oc){
          s->ocref &= ~(1<<channel);
      }else{
          s->ocref |= (1<<channel);
      }
      break;
  }

  MemoryRegion* region = sysbus_mmio_get_region(SYS_BUS_DEVICE(gpio), 0);

  uint32_t changed = old_ocref ^ s->ocref;
  if (changed & (1<<channel)){
    if ((s->ocref & (1<<channel)) && !polarity){
      io_mem_write(region, GPIOx_BSRR_OFFSET, (1<<pin), 4);
    }else if (!(s->ocref & (1<<channel)) && polarity){
      io_mem_write(region, GPIOx_BSRR_OFFSET, (1<<pin), 4);
    }else{
      io_mem_write(region, GPIOx_BSRR_OFFSET, (1<<(pin+16)), 4);
    }
  }
}

static void stm32_timer_update_ic(Stm32Timer* s, uint8_t channel){
  uint32_t elapsed = stm32_timer_get_count(s);
//  uint8_t prescaler;
//  uint8_t filter;

  DPRINTF("Update IC%d elapsed = %d\n", channel+1, elapsed);
  switch(channel){
  case 0:
//    prescaler = TIM_IC1PSC(s);
//    filter = TIM_IC1F(s);
    s->ccr1 = elapsed & 0xffff;
    if (s->sr & CC1IF_MASK){
      s->sr |= CC1OF_MASK;
    }else{
      s->sr |= CC1IF_MASK;
    }
    break;
  case 1:
//    prescaler = TIM_IC2PSC(s);
//    filter = TIM_IC2F(s);
    s->ccr2 = elapsed & 0xffff;
    if (s->sr & CC2IF_MASK){
      s->sr |= CC2OF_MASK;
    }else{
      s->sr |= CC2IF_MASK;
    }
    break;
  case 2:
//    prescaler = TIM_IC3PSC(s);
//    filter = TIM_IC3F(s);
    s->ccr3 = elapsed & 0xffff;
    if (s->sr & CC3IF_MASK){
      s->sr |= CC3OF_MASK;
    }else{
      s->sr |= CC3IF_MASK;
    }
    break;
  case 3:
//    prescaler = TIM_IC4PSC(s);
//    filter = TIM_IC4F(s);
    s->ccr4 = elapsed & 0xffff;
    if (s->sr & CC4IF_MASK){
      s->sr |= CC4OF_MASK;
    }else{
      s->sr |= CC4IF_MASK;
    }
    break;
  }
}

static void stm32_timer_gpio_trigger(void* opaque, int n, int level){
    Stm32Timer* s = (Stm32Timer*)opaque;
    uint8_t new_level = level ? 1 : 0;
    if (s->pin_value[n] == new_level){
      return;
    }

    uint8_t falling = new_level ? 0 : 1;
    s->pin_value[n] = new_level;

    DPRINTF("Timer IC trigger [%d] %x @ %d\n", n, level, stm32_timer_get_count(s));

    if       (n == 0 && TIM_CC1E(s) && TIM_CC1S(s) == 1 && TIM_CC1P(s) == falling){
      stm32_timer_update_ic(s, 0);
    }else if (n == 1 && TIM_CC1E(s) && TIM_CC1S(s) == 2 && TIM_CC1P(s) == falling){
      stm32_timer_update_ic(s, 0);
    }else if (n == 1 && TIM_CC2E(s) && TIM_CC2S(s) == 1 && TIM_CC2P(s) == falling){
      stm32_timer_update_ic(s, 1);
    }else if (n == 0 && TIM_CC2E(s) && TIM_CC2S(s) == 2 && TIM_CC2P(s) == falling){
      stm32_timer_update_ic(s, 1);
    }else if (n == 2 && TIM_CC3E(s) && TIM_CC3S(s) == 1 && TIM_CC3P(s) == falling){
      stm32_timer_update_ic(s, 2);
    }else if (n == 3 && TIM_CC3E(s) && TIM_CC3S(s) == 2 && TIM_CC3P(s) == falling){
      stm32_timer_update_ic(s, 2);
    }else if (n == 3 && TIM_CC4E(s) && TIM_CC4S(s) == 1 && TIM_CC4P(s) == falling){
      stm32_timer_update_ic(s, 3);
    }else if (n == 2 && TIM_CC4E(s) && TIM_CC4S(s) == 2 && TIM_CC4P(s) == falling){
      stm32_timer_update_ic(s, 3);
    }
}
static void stm32_timer_check_cc_state(Stm32Timer *s){

    if (TIM_CC1E(s) && TIM_CC1S(s) == 0){
        stm32_timer_update_ocref(s, 0, s->ccr1, TIM_OC1M(s), TIM_CC1P(s));
    }
    if (TIM_CC2E(s) && TIM_CC2S(s) == 0){
        stm32_timer_update_ocref(s, 1, s->ccr2, TIM_OC2M(s), TIM_CC2P(s));
    }
    if (TIM_CC3E(s) && TIM_CC3S(s) == 0){
        stm32_timer_update_ocref(s, 2, s->ccr3, TIM_OC3M(s), TIM_CC3P(s));
    }
    if (TIM_CC4E(s) && TIM_CC4S(s) == 0){
        stm32_timer_update_ocref(s, 3, s->ccr4, TIM_OC4M(s), TIM_CC4P(s));
    }
}
static void stm32_timer_set_count(Stm32Timer *s, uint32_t cnt)
{
    if (s->countMode == TIMER_UP_COUNT)
    {
        ptimer_set_count(s->timer, s->arr - (cnt & 0xfffff));
    }
    else
    {
        ptimer_set_count(s->timer, cnt & 0xffff);
    }
    if (cnt != 0){
    stm32_timer_check_cc_state(s);
    }
}

static void stm32_timer_clk_irq_handler(void *opaque, int n, int level)
{
    Stm32Timer *s = (Stm32Timer *)opaque;

    assert(n == 0);

    stm32_timer_freq(s);
}

static void stm32_timer_update(Stm32Timer *s)
{
    stm32_timer_freq(s);

    if (s->cr1 & TIMER_CR1_DIR_DOWN) /* dir bit */
    {
        s->countMode = TIMER_DOWN_COUNT;
    }
    else
    {
        s->countMode = TIMER_UP_COUNT;
    }

    if (s->cr1 & TIMER_CR1_CMS_MASK) /* CMS */
    {
        s->countMode = TIMER_UP_COUNT;
    }

    if (s->cr1 & TIMER_CR1_CEN) /* timer enable */
    {
        if (TIM_CC1E(s) && TIM_CC1S(s) == 0){
            DPRINTF("%s Enabling capture/compare 1 timer\n", stm32_periph_name(s->periph));
            timer_mod_ns(s->timer_cc1, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + (s->ccr1 & 0xffff) * s->timer_cc1->scale);
        }
        if (TIM_CC2E(s) && TIM_CC2S(s) == 0){
            DPRINTF("%s Enabling capture/compare 2 timer\n", stm32_periph_name(s->periph));
            timer_mod_ns(s->timer_cc2, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + (s->ccr2 & 0xffff) * s->timer_cc2->scale);
        }
        if (TIM_CC3E(s) && TIM_CC3S(s) == 0){
            DPRINTF("%s Enabling capture/compare 3 timer\n", stm32_periph_name(s->periph));
            timer_mod_ns(s->timer_cc3, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + (s->ccr3 & 0xffff) * s->timer_cc3->scale);
        }
        if (TIM_CC4E(s) && TIM_CC4S(s) == 0){
            DPRINTF("%s Enabling capture/compare 4 timer\n", stm32_periph_name(s->periph));
            timer_mod_ns(s->timer_cc4, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + (s->ccr4 & 0xffff) * s->timer_cc4->scale);
        }
        DPRINTF("%s Enabling timer\n", stm32_periph_name(s->periph));
        ptimer_run(s->timer, (s->cr1 & TIMER_CR1_OPM));
    }
    else
    {
        DPRINTF("%s Disabling timer\n", stm32_periph_name(s->periph));
        ptimer_stop(s->timer);
        timer_mod_ns(s->timer_cc1, 0);
        timer_mod_ns(s->timer_cc2, 0);
        timer_mod_ns(s->timer_cc3, 0);
        timer_mod_ns(s->timer_cc4, 0);
    }
}

static void stm32_timer_update_UIF(Stm32Timer *s, uint8_t value) {
    s->sr &= ~0x1; /* update interrupt flag in status reg */
    s->sr |= (value & 0x1);

    qemu_set_irq(s->irq, value);
}

static void stm32_timer_cc1_tick(void *opaque){
    Stm32Timer *s = (Stm32Timer *)opaque;
    uint32_t elapsed = stm32_timer_get_count(s);
    DPRINTF("%s CC 1 Alarm raised @ %d\n", stm32_periph_name(s->periph), elapsed);
    stm32_timer_check_cc_state(s);
}
static void stm32_timer_cc2_tick(void *opaque){
    Stm32Timer *s = (Stm32Timer *)opaque;
    uint32_t elapsed = stm32_timer_get_count(s);
    DPRINTF("%s CC 2 Alarm raised @ %d\n", stm32_periph_name(s->periph), elapsed);
    stm32_timer_check_cc_state(s);
}
static void stm32_timer_cc3_tick(void *opaque){
    Stm32Timer *s = (Stm32Timer *)opaque;
    uint32_t elapsed = stm32_timer_get_count(s);
    DPRINTF("%s CC 3 Alarm raised @ %d\n", stm32_periph_name(s->periph), elapsed);
    stm32_timer_check_cc_state(s);
}
static void stm32_timer_cc4_tick(void *opaque){
    Stm32Timer *s = (Stm32Timer *)opaque;
    uint32_t elapsed = stm32_timer_get_count(s);
    DPRINTF("%s CC 3 Alarm raised @ %d\n", stm32_periph_name(s->periph), elapsed);
    stm32_timer_check_cc_state(s);
}
static void stm32_timer_tick(void *opaque)
{
    Stm32Timer *s = (Stm32Timer *)opaque;
    DPRINTF("%s Alarm raised\n", stm32_periph_name(s->periph));
    s->itr = 1;
    stm32_timer_update_UIF(s, 1);

//    stm32_timer_check_cc_state(s);

    if (s->countMode == TIMER_UP_COUNT)
    {
        stm32_timer_set_count(s, 0);
    }
    else
    {
        stm32_timer_set_count(s, s->arr);
    }

    if (s->cr1 & TIMER_CR1_CMS_MASK) /* CMS */
    {
        if (s->countMode == TIMER_UP_COUNT)
        {
            s->countMode = TIMER_DOWN_COUNT;
        }
        else
        {
            s->countMode = TIMER_UP_COUNT;
        }
    }

    if (s->cr1 & TIMER_CR1_OPM) /* one shot */
    {
        s->cr1 &= 0xFFFE;
    }
    else
    {
DPRINTF("Restart\n");
        stm32_timer_update(s);
    }
}

static uint64_t stm32_timer_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    Stm32Timer *s = (Stm32Timer *)opaque;

    switch (offset) {
    case TIMER_CR1_OFFSET:
//        DPRINTF("%s cr1 = %x\n", stm32_periph_name(s->periph), s->cr1);
        return s->cr1;
    case TIMER_CR2_OFFSET:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: CR2 not supported");
        return 0;
    case TIMER_SMCR_OFFSET:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: SMCR not supported");
        return 0;
    case TIMER_DIER_OFFSET:
        DPRINTF("%s dier = %x\n", stm32_periph_name(s->periph), s->dier);
        return s->dier;
    case TIMER_SR_OFFSET:
        DPRINTF("%s sr = %x\n", stm32_periph_name(s->periph), s->sr);
        return s->sr;
    case TIMER_EGR_OFFSET:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: EGR write only");
        return 0;
    case TIMER_CCMR1_OFFSET:
        DPRINTF("%s ccmr1 = %x\n", stm32_periph_name(s->periph), s->ccmr1);
        return s->ccmr1;
    case TIMER_CCMR2_OFFSET:
        DPRINTF("%s ccmr2 = %x\n", stm32_periph_name(s->periph), s->ccmr2);
        return s->ccmr2;
    case TIMER_CCER_OFFSET:
        DPRINTF("%s ccer = %x\n", stm32_periph_name(s->periph), s->ccer);
        return s->ccer;
    case TIMER_CNT_OFFSET:
        //DPRINTF("%s cnt = %x\n", stm32_periph_name(s->periph), stm32_timer_get_count(s));
        return stm32_timer_get_count(s);
    case TIMER_PSC_OFFSET:
        DPRINTF("%s psc = %x\n", stm32_periph_name(s->periph), s->psc);
        return s->psc;
    case TIMER_ARR_OFFSET:
        DPRINTF("%s arr = %x\n", stm32_periph_name(s->periph), s->arr);
        return s->arr;
    case TIMER_RCR_OFFSET:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: RCR not supported");
        return 0;
    case TIMER_CCR1_OFFSET:
        DPRINTF("%s ccr1 = %x\n", stm32_periph_name(s->periph), s->ccr1);
        s->sr &= ~CC1IF_MASK;
        return s->ccr1;
    case TIMER_CCR2_OFFSET:
        DPRINTF("%s ccr2 = %x\n", stm32_periph_name(s->periph), s->ccr2);
        s->sr &= ~CC2IF_MASK;
        return s->ccr2;
    case TIMER_CCR3_OFFSET:
        DPRINTF("%s ccr3 = %x\n", stm32_periph_name(s->periph), s->ccr3);
        s->sr &= ~CC3IF_MASK;
        return s->ccr3;
    case TIMER_CCR4_OFFSET:
        DPRINTF("%s ccr4 = %x\n", stm32_periph_name(s->periph), s->ccr4);
        s->sr &= ~CC4IF_MASK;
        return s->ccr4;
    case TIMER_BDTR_OFFSET:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: BDTR not supported");
        return 0;
    case TIMER_DCR_OFFSET:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: DCR not supported");
        return 0;
    case TIMER_DMAR_OFFSET:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: DMAR not supported");
        return 0;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "stm32_read: Bad offset 0x%x\n", (int)offset);
        break;
    }

    return 0;
}

static void stm32_timer_write(void * opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    Stm32Timer *s = (Stm32Timer *)opaque;
    switch (offset) {
    case TIMER_CR1_OFFSET:
    {
        uint32_t  timer_start = !(s->cr1 & 1) && (value & 1) ;

        s->cr1 = value & 0x3FF;
        DPRINTF("%s cr1 = %x\n", stm32_periph_name(s->periph), s->cr1);
        stm32_timer_update(s);
        if(timer_start){
          stm32_timer_check_cc_state(s);
        }
        break;
    }
    case TIMER_CR2_OFFSET:
        /* s->cr2 = value & 0xF8; */
        qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: CR2 not supported");
        break;
    case TIMER_SMCR_OFFSET:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: SMCR not supported");
        break;
    case TIMER_DIER_OFFSET:
        s->dier = value & 0x5F5F;
        DPRINTF("%s dier = %x\n", stm32_periph_name(s->periph), s->dier);
        break;
    case TIMER_SR_OFFSET:
        s->sr ^= (value ^ 0xFFFF);
        s->sr &= 0x1eFF;
        stm32_timer_update_UIF(s, s->sr & 0x1);
        DPRINTF("%s sr = %x\n", stm32_periph_name(s->periph), s->sr);
        break;
    case TIMER_EGR_OFFSET:
        s->egr = value & 0x1E;
        if (value & 0x40) {
            /* TG bit */
            s->sr |= 0x40;
        }
        if (value & 0x1) {
             /* UG bit - reload count */
            ptimer_set_limit(s->timer, s->arr, 1);
        }
        DPRINTF("%s egr = %x\n", stm32_periph_name(s->periph), s->egr);
        break;
    case TIMER_CCMR1_OFFSET:
        s->ccmr1 = value & 0xffff;
        DPRINTF("%s ccmr1 = %x\n", stm32_periph_name(s->periph), s->ccmr1);
        break;
    case TIMER_CCMR2_OFFSET:
        s->ccmr2 = value & 0xffff;
        DPRINTF("%s ccmr2 = %x\n", stm32_periph_name(s->periph), s->ccmr2);
        break;
    case TIMER_CCER_OFFSET:
        s->ccer = value & 0x3333;
        DPRINTF("%s ccer = %x\n", stm32_periph_name(s->periph), s->ccer);
        break;
    case TIMER_CNT_OFFSET:
        stm32_timer_set_count(s, value & 0xffff);
        DPRINTF("%s cnt = %x\n", stm32_periph_name(s->periph), stm32_timer_get_count(s));
        break;
    case TIMER_PSC_OFFSET:
        s->psc = value & 0xffff;
        DPRINTF("%s psc = %x\n", stm32_periph_name(s->periph), s->psc);
        stm32_timer_freq(s);
        break;
    case TIMER_ARR_OFFSET:
        s->arr = value & 0xffff;
        ptimer_set_limit(s->timer, s->arr, 1);
        DPRINTF("%s arr = %x\n", stm32_periph_name(s->periph), s->arr);
        break;
    case TIMER_RCR_OFFSET:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: RCR not supported");
        /* s->rcr = value & 0xff; */
        break;
    case TIMER_CCR1_OFFSET:
        s->ccr1 = value & 0xffff;
        DPRINTF("%s ccr1 = %x\n", stm32_periph_name(s->periph), s->ccr1);
        break;
    case TIMER_CCR2_OFFSET:
        s->ccr2 = value & 0xffff;
        DPRINTF("%s ccr2 = %x\n", stm32_periph_name(s->periph), s->ccr2);
        break;
    case TIMER_CCR3_OFFSET:
        s->ccr3 = value & 0xffff;
        DPRINTF("%s ccr3 = %x\n", stm32_periph_name(s->periph), s->ccr3);
        break;
    case TIMER_CCR4_OFFSET:
        s->ccr4 = value & 0xffff;
        DPRINTF("%s ccr4 = %x\n", stm32_periph_name(s->periph), s->ccr4);
        break;
    case TIMER_BDTR_OFFSET:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: BDTR not supported");
        break;
    case TIMER_DCR_OFFSET:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: DCR not supported");
        break;
    case TIMER_DMAR_OFFSET:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: DMAR not supported");
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "stm32_read: Bad offset 0x%x\n", (int)offset);
        break;
    }

}

static const MemoryRegionOps stm32_timer_ops = {
    .read = stm32_timer_read,
    .write = stm32_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};
static void stm32_timer_channel_gpio_connect(Stm32Timer* s, int channel, qemu_irq irq){
    if (s->channel_gpio_port[channel] >= STM32_GPIO_COUNT || s->channel_gpio_pin[channel] >= STM32_GPIO_PIN_COUNT){
        return;
    }
    SysBusDevice * sd = SYS_BUS_DEVICE(s->stm32_gpio[s->channel_gpio_port[channel]]);
    uint8_t pin = s->channel_gpio_pin[channel];

    qemu_irq new_irq = irq;

    /* HACK to mutiplex IRQ */
    if (sd->irqp[pin] && *sd->irqp[pin]){
      qemu_irq old_irq = *sd->irqp[pin];
      new_irq = qemu_irq_split(old_irq, irq);
    }

    sysbus_connect_irq(sd, pin, new_irq);
}
static int stm32_timer_init(SysBusDevice *dev)
{
    QEMUBH *bh;
    qemu_irq *clk_irq;
    Stm32Timer *s = STM32_TIMER(dev);

    s->stm32_rcc = (Stm32Rcc *)s->stm32_rcc_prop;
    s->stm32_gpio = (Stm32Gpio **)s->stm32_gpio_prop;
    s->stm32_afio = (Stm32Afio *)s->stm32_afio_prop;

    memory_region_init_io(&s->iomem, OBJECT(s), &stm32_timer_ops, s, "stm32-timer", 0x3ff);
    sysbus_init_mmio(dev, &s->iomem);

    sysbus_init_irq(dev, &s->irq);

    qemu_irq* channel_irqs = qemu_allocate_irqs(stm32_timer_gpio_trigger, s, 4);
    if (s->periph == STM32_TIM3){
      s->channel_gpio_port[0] = 0;
      s->channel_gpio_port[1] = 0;
      s->channel_gpio_port[2] = 1;
      s->channel_gpio_port[3] = 1;
      s->channel_gpio_pin[0] = 6;
      s->channel_gpio_pin[1] = 7;
      s->channel_gpio_pin[2] = 0;
      s->channel_gpio_pin[3] = 1;
    }else{
      s->channel_gpio_port[0] = STM32_GPIO_COUNT;
      s->channel_gpio_port[1] = STM32_GPIO_COUNT;
      s->channel_gpio_port[2] = STM32_GPIO_COUNT;
      s->channel_gpio_port[3] = STM32_GPIO_COUNT;

      s->channel_gpio_pin[0] = STM32_GPIO_PIN_COUNT;
      s->channel_gpio_pin[1] = STM32_GPIO_PIN_COUNT;
      s->channel_gpio_pin[2] = STM32_GPIO_PIN_COUNT;
      s->channel_gpio_pin[3] = STM32_GPIO_PIN_COUNT;
    }
    stm32_timer_channel_gpio_connect(s, 0, channel_irqs[0]);
    stm32_timer_channel_gpio_connect(s, 1, channel_irqs[1]);
    stm32_timer_channel_gpio_connect(s, 2, channel_irqs[2]);
    stm32_timer_channel_gpio_connect(s, 3, channel_irqs[3]);

    /* Register handlers to handle updates to the TIM's peripheral clock. */
    clk_irq = qemu_allocate_irqs(stm32_timer_clk_irq_handler, (void *)s, 1);
    stm32_rcc_set_periph_clk_irq(s->stm32_rcc, s->periph, clk_irq[0]);

    bh = qemu_bh_new(stm32_timer_tick, s);
    s->timer = ptimer_init(bh);

    s->timer_cc1 = timer_new(QEMU_CLOCK_VIRTUAL, SCALE_US, stm32_timer_cc1_tick, s);
    s->timer_cc2 = timer_new(QEMU_CLOCK_VIRTUAL, SCALE_US, stm32_timer_cc2_tick, s);
    s->timer_cc3 = timer_new(QEMU_CLOCK_VIRTUAL, SCALE_US, stm32_timer_cc3_tick, s);
    s->timer_cc4 = timer_new(QEMU_CLOCK_VIRTUAL, SCALE_US, stm32_timer_cc4_tick, s);

    s->cr1   = 0;
    s->dier  = 0;
    s->sr    = 0;
    s->egr   = 0;
    s->ccmr1 = 0;
    s->ccmr2 = 0;
    s->ccer  = 0;
    s->psc   = 0;
    s->arr   = 0;
    s->ccr1  = 0;
    s->ccr2  = 0;
    s->ccr3  = 0;
    s->ccr4  = 0;

    return 0;
}

static void stm32_timer_pre_save(void *opaque)
{
    //Stm32Timer *s = opaque;

    /* tick_offset is base_time - rtc_clock base time.  Instead, we want to
     * store the base time relative to the vm_clock for backwards-compatibility.  */
    //int64_t delta = qemu_get_clock_ns(rtc_clock) - qemu_get_clock_ns(vm_clock);
    //s->tick_offset_vmstate = s->tick_offset + delta / get_ticks_per_sec();
}

static int stm32_timer_post_load(void *opaque, int version_id)
{
    //Stm32Timer *s = opaque;

    //int64_t delta = qemu_get_clock_ns(rtc_clock) - qemu_get_clock_ns(vm_clock);
    //s->tick_offset = s->tick_offset_vmstate - delta / get_ticks_per_sec();
    //stm32_timer_set_alarm(s);
    return 0;
}

static Property stm32_timer_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32Timer, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_PTR("stm32_rcc",   Stm32Timer, stm32_rcc_prop),
    DEFINE_PROP_PTR("stm32_gpio",  Stm32Timer, stm32_gpio_prop),
    DEFINE_PROP_PTR("stm32_afio",  Stm32Timer, stm32_afio_prop),
    DEFINE_PROP_END_OF_LIST()
};

static const VMStateDescription vmstate_stm32 = {
    .name = "stm32-timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = stm32_timer_pre_save,
    .post_load = stm32_timer_post_load,
    .fields = (VMStateField[]) {
        //VMSTATE_UINT32(tick_offset_vmstate, stm32_timer_tm_state),
        //VMSTATE_UINT32(mr, Stm32Timer),
        //VMSTATE_UINT32(lr, Stm32Timer),
        //VMSTATE_UINT32(cr, Stm32Timer),
        //VMSTATE_UINT32(im, Stm32Timer),
        //VMSTATE_UINT32(is, Stm32Timer),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32_timer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32_timer_init;
    //dc->no_user = 1;
    dc->vmsd = &vmstate_stm32;
    dc->props = stm32_timer_properties;
}

static const TypeInfo stm32_timer_info = {
    .name          = "stm32-timer",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32Timer),
    .class_init    = stm32_timer_class_init,
};

static void stm32_timer_register_types(void)
{
    type_register_static(&stm32_timer_info);
}

type_init(stm32_timer_register_types)
