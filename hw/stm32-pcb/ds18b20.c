#include "hw/stm32-pcb/pcb.h"
#include "hw/sysbus.h"
#include "hw/arm/stm32.h"
#include "qapi-event.h"
#include "hw/ptimer.h"
#include "sysemu/sysemu.h"

#include <inttypes.h>
#define STM32_PC13 STM32_GPIO_INDEX(STM32_GPIOC_INDEX, 13)

static const char* states[] = {
  "IDLE",
  "RESET_START",
  "RESET_END",
  "PRESENCE_START",
  "READ_START",
  "READ_END",
};
#define STATE_IDLE           0
#define STATE_RESET_START    1
#define STATE_RESET_END      2
#define STATE_PRESENCE_START 3
#define STATE_READ_START     4
#define STATE_READ_END       5


#define EVT_TIMER_RESET  1
#define EVT_TIMER_PULSE  2
#define EVT_EDGE_RAISING 3
#define EVT_EDGE_FALLING 4

#define RESET_THRESHOLD 450

#define RESET_RESPONSE_THRESHOLD 60

#define RESET_DETECT_TIMEOUT 1000
typedef struct  {
    /* Inherited */
    PCBDevice busdev;

    /* Properties */
    uint16_t data_gpio;
    ptimer_state *pulse_timer;
    uint8_t state;

    qemu_timeval tv_fall;
    qemu_timeval tv_raise;

    /* Private */

    uint8_t gpio_value;
} Ds18b20State;


#define TYPE_STM32_DS18B20 "stm32-periph-ds18b20"
#define STM32_DS18B20(obj) OBJECT_CHECK(Ds18b20State, (obj), TYPE_STM32_DS18B20)

static uint32_t get_pulse_len(Ds18b20State *s){
  if (s->tv_raise.tv_sec < s->tv_fall.tv_sec)
    return 0;

  uint32_t len = s->tv_raise.tv_sec - s->tv_fall.tv_sec;
  if (len > 2){
    return 2000000;
  }
  len += (s->tv_raise.tv_usec - s->tv_fall.tv_usec);

  return len;
}
static void stm32_ds18b20_set_pin(Ds18b20State * s, int level){
    PCBBus * bus = PCB_BUS(DEVICE(&s->busdev)->parent_bus);
    PCB_DPRINTF("DS18B20 0x%02x: pin => %d\n", s->busdev.addr, level);
    bus->gpio_set_value(bus, s->data_gpio, level ? 3300: 0);
}

static void stm32_ds18b20_delay(Ds18b20State* s, int delay){
    ptimer_stop(s->pulse_timer);
    ptimer_set_limit(s->pulse_timer, delay, 1);
    ptimer_set_count(s->pulse_timer, delay);
    ptimer_run(s->pulse_timer, 1);
}

static void stm32_ds18b20_fsm_reset(Ds18b20State *s){
    PCB_DPRINTF("DS18B20 0x%02x: reset detected\n", s->busdev.addr);
    s->state = STATE_RESET_START;
    ptimer_stop(s->pulse_timer);
    ptimer_set_count(s->pulse_timer, 0);
}

static void stm32_ds18b20_fsm(Ds18b20State* s, uint8_t event){

    if(s->state == STATE_RESET_END && event == EVT_TIMER_PULSE){
      s->state = STATE_PRESENCE_START;
      stm32_ds18b20_set_pin(s, 0);
      stm32_ds18b20_delay(s, 120);
    }else if (s->state == STATE_RESET_START && event == EVT_EDGE_RAISING){
      s->state = STATE_RESET_END;
      stm32_ds18b20_delay(s, 30);
    }else if (s->state == STATE_PRESENCE_START && event == EVT_TIMER_PULSE){
      s->state = STATE_READ_START;
      stm32_ds18b20_set_pin(s, 1);
    }else if (s->state == STATE_READ_START && event == EVT_EDGE_FALLING){
      stm32_ds18b20_delay(s, 500);
    }else if (s->state == STATE_READ_START && event == EVT_EDGE_RAISING){
      uint32_t len = get_pulse_len(s);

      if (len <= 15){
        PCB_DPRINTF("DS18B20 0x%02x: ONE detected @ %d\n", s->busdev.addr, len);
      }else if (len < 450){
        PCB_DPRINTF("DS18B20 0x%02x: ZERO detected @ %d\n", s->busdev.addr, len);
      }else{
        stm32_ds18b20_fsm_reset(s);
      }
      ptimer_stop(s->pulse_timer);
    }
    PCB_DPRINTF("DS18B20 0x%02x: state => %s!\n", s->busdev.addr, states[s->state]);
}
static void stm32_ds18b20_pulse_tick(void *opaque){
    stm32_ds18b20_fsm((Ds18b20State*)opaque, EVT_TIMER_PULSE);
}
static void stm32_ds18b20_irq_handler(void *opaque, int n, int level)
{
    Ds18b20State *s = (Ds18b20State *)opaque;
    PCBDevice *pd = PCB_DEVICE(s);

    uint8_t new_value = s->gpio_value;

    assert(n == 0);

    if(level){
      new_value |= (1<<n);
    }else{
      new_value &= ~(1<<n);
    }

    uint8_t changed = s->gpio_value ^ new_value;
    uint8_t active = new_value;
    if (changed && active){
      qemu_gettimeofday(&s->tv_raise);
//      qapi_event_send_x_pcb(pd->addr, "DS18B20", 1, &error_abort);
      PCB_DPRINTF("DS18B20 0x%02x: on\n", s->busdev.addr);

      uint32_t len = get_pulse_len(s);
      if (len > 450){
        stm32_ds18b20_fsm_reset(s);
      }
      stm32_ds18b20_fsm(s, EVT_EDGE_RAISING);
    }else if (changed){
      qemu_gettimeofday(&s->tv_fall);
//      qapi_event_send_x_pcb(pd->addr, "DS18B20", 0, &error_abort);
      PCB_DPRINTF("DS18B20 0x%02x: off\n", s->busdev.addr);

      stm32_ds18b20_fsm(s, EVT_EDGE_FALLING);
    }

    s->gpio_value = new_value;
}

static int ds18b20_get_state(PCBDevice* dev, const char* unit, Error **errp){
  Ds18b20State *s = STM32_DS18B20(dev);

  if (strncmp("DS18B20", unit, 3) || unit[3] != 0){
    error_setg(errp, "Unsupported unit: %s", unit);
    return 0;
  }
  return s->gpio_value ? 0 : 1;
}
static void stm32_ds18b20_reset(DeviceState *dev)
{
    Ds18b20State *s = STM32_DS18B20(dev);

    s->gpio_value = 0;
    stm32_ds18b20_fsm_reset(s);
}

static void stm32_ds18b20_realize(DeviceState *dev, Error **errp)
{
    Ds18b20State *s = STM32_DS18B20(dev);
    PCBBus* bus = PCB_BUS(dev->parent_bus);

    PCBDeviceClass *pc = PCB_DEVICE_GET_CLASS(dev);
    if (pc->parent_realize){
      pc->parent_realize(dev, errp);
    }

    s->busdev.get_state = ds18b20_get_state;

    if (STM32_PORT_INDEX(s->data_gpio) >= STM32_GPIO_COUNT){
      error_setg(errp, "Unsupported GPIO port for DATA: 0x%02x", s->data_gpio);
      return;
    }
    if (STM32_PIN_INDEX(s->data_gpio) >= STM32_GPIO_PIN_COUNT){
      error_setg(errp, "Unsupported GPIO pin for DATA: 0x%02x", s->data_gpio);
      return;
    }

    qemu_irq* gpio_irq = qemu_allocate_irqs(stm32_ds18b20_irq_handler, (void *)s, 1);

    bus->gpio_connect(bus, s->data_gpio, gpio_irq[0]);

    QEMUBH *bh;
    bh = qemu_bh_new(stm32_ds18b20_pulse_tick, s);
    s->pulse_timer = ptimer_init(bh);

    // Ticks in microseconds
    ptimer_set_freq(s->pulse_timer, 1000000);

    stm32_ds18b20_reset((DeviceState *)s);
}
static Property stm32_ds18b20_properties[] = {
    DEFINE_PROP_UINT16("data", Ds18b20State, data_gpio, STM32_PC13),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32_ds18b20_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    PCBDeviceClass * pc = PCB_DEVICE_CLASS(klass);
    pc->parent_realize = dc->realize;
    dc->realize = stm32_ds18b20_realize;
    dc->reset = stm32_ds18b20_reset;
    dc->props = stm32_ds18b20_properties;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}
static TypeInfo stm32_ds18b20_info = {
    .name  = TYPE_STM32_DS18B20,
    .parent = TYPE_PCB_DEVICE,
    .instance_size  = sizeof(Ds18b20State),
    .class_init = stm32_ds18b20_class_init
};

static void stm32_ds18b20_register_types(void)
{
    type_register_static(&stm32_ds18b20_info);
}

type_init(stm32_ds18b20_register_types)
