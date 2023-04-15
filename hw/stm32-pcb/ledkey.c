#include "hw/stm32-pcb/pcb.h"
#include "hw/sysbus.h"
#include "hw/arm/stm32.h"
#include "qapi-event.h"
#include <inttypes.h>

#define STATE_CMD 0
#define STATE_ADDR 1
#define STATE_DATA 2
#define STATE_READ 3

#define MODE_WRITE 0
#define MODE_READ  1

#define INCREMENT_ON 0
#define INCREMENT_OFF 1

#define BIT_STB  0
#define BIT_CLK  1
#define BIT_DIO 2

#define MASK_STB (1<<BIT_STB)
#define MASK_CLK (1<<BIT_CLK)
#define MASK_DIO (1<<BIT_DIO)

#define STM32_PB12 STM32_GPIO_INDEX(STM32_GPIOB_INDEX, 12)
#define STM32_PB13 STM32_GPIO_INDEX(STM32_GPIOB_INDEX, 13)
#define STM32_PB15 STM32_GPIO_INDEX(STM32_GPIOB_INDEX, 15)

typedef struct  {
    /* Inherited */
    PCBDevice busdev;

    /* Properties */
    uint16_t stb_gpio;
    uint16_t clk_gpio;
    uint16_t dio_gpio;

    /* Private */

    uint8_t gpio_value;
    uint8_t spi_byte;
    uint8_t spi_cnt;
    uint8_t buffer[16];
    uint32_t buttons;

    uint8_t state;
    uint8_t addr;
    uint8_t write_mode;
    uint8_t autoincrement;
} LedkeyState;


#define TYPE_STM32_LEDKEY "stm32-periph-ledkey"
#define STM32_LEDKEY(obj) OBJECT_CHECK(LedkeyState, (obj), TYPE_STM32_LEDKEY)



#if 0
static void print_led(LedkeyState *s){
  int i;
  printf("\e[1;1H\e[2J");
  printf("BUF:");
  for(i=1; i<16; i+=2) printf(" %c   ", s->buffer[i]&1?'*':' ');
  printf("\n");
  printf("BUF:");
  for(i=0; i<16; i+=2) printf("%s  " , s->buffer[i]&1 ?"+-+":"   ");
  printf("\n");
  printf("BUF:");
  for(i=0; i<16; i+=2) printf("%c %c  " , s->buffer[i]&0x20 ?'|':' ',s->buffer[i]&2 ?'|':' ');
  printf("\n");
  printf("BUF:");
  for(i=0; i<16; i+=2) printf("%s  " , s->buffer[i]&0x40 ?"+-+":"   ");
  printf("\n");
  printf("BUF:");
  for(i=0; i<16; i+=2) printf("%c %c  " , s->buffer[i]&0x10 ?'|':' ',s->buffer[i]&4 ?'|':' ');
  printf("\n");
  printf("BUF:");
  for(i=0; i<16; i+=2) printf("%s  " , s->buffer[i]&0x8 ?"+-+":"   ");
  printf("\n");
  printf("BUF:");
  for(i=0; i<16; i+=2) printf("      ");
  printf("\n");
}
#endif
static void lk_parse(LedkeyState *s){
  PCBDevice *pd = PCB_DEVICE(s);

  uint8_t data = s->spi_byte;
//  printf("SPI:0x%02x state: %d\n", data, s->state);
  if (s->state == STATE_CMD){
    switch(data & 0xc0){
      case 0:
        printf("Display mode: ");
        switch(data & 3){
          case 0: printf("4 GRIDs, 13 SEGs\n"); break;
          case 1: printf("5 GRIDs, 12 SEGs\n"); break;
          case 2: printf("6 GRIDs, 11 SEGs\n"); break;
          case 3: printf("7 GRIDs, 10 SEGs (default)\n"); break;
        }
        break;
      case 0x40:
        s->write_mode = (data & 2) ? MODE_READ : MODE_WRITE;
        s->autoincrement = (data & 4) ? INCREMENT_OFF : INCREMENT_ON;
//        printf("Data settings: mode=%s, autoincrement=%s, mode=%s\n", ((data&8)?"test":"normal"), (s->autoincrement == INCREMENT_OFF ? "off":"on"), (s->write_mode == MODE_READ?"read":"write"));
        if (s->write_mode == MODE_READ){
          s->addr = 0;
          s->spi_byte = s->buttons & 0xff;
          s->spi_cnt = 0;
          s->state = STATE_READ;
        }
        break;
      case 0x80:
        printf ("Display control: %s intensity=%d\n",((data & 8) ? "ON" : "OFF"), data & 7);
        break;
      case 0xc0:
        s->addr = data & 0xf;
//        printf("Set address: 0x%02x\n", s->addr);
        s->state = STATE_DATA;
        break;
    }
  }else if (s->state == STATE_READ){
//    printf ("State read: 0x%02x\n", s->addr);
    s->addr = (s->addr + 1) & 0x3;
    s->spi_byte = (s->buttons >> (s->addr << 3)) & 0xff;
    s->spi_cnt = 0;
  }else if (s->state == STATE_DATA){
//    printf("Data: @%x=0x%02x\n", s->addr, data);
    if (s->write_mode == MODE_WRITE){
      s->buffer[s->addr] = data;
      if (s->addr == 0){
        qapi_event_send_x_pcb(pd->addr, "SEG0", s->buffer[s->addr], &error_abort);
      }else if (s->addr == 1){
        qapi_event_send_x_pcb(pd->addr, "LED0", s->buffer[s->addr] & 1, &error_abort);
      }else if (s->addr == 2){
        qapi_event_send_x_pcb(pd->addr, "SEG1", s->buffer[s->addr], &error_abort);
      }else if (s->addr == 3){
        qapi_event_send_x_pcb(pd->addr, "LED1", s->buffer[s->addr] & 1, &error_abort);
      }else if (s->addr == 4){
        qapi_event_send_x_pcb(pd->addr, "SEG2", s->buffer[s->addr], &error_abort);
      }else if (s->addr == 5){
        qapi_event_send_x_pcb(pd->addr, "LED2", s->buffer[s->addr] & 1, &error_abort);
      }else if (s->addr == 6){
        qapi_event_send_x_pcb(pd->addr, "SEG3", s->buffer[s->addr], &error_abort);
      }else if (s->addr == 7){
        qapi_event_send_x_pcb(pd->addr, "LED3", s->buffer[s->addr] & 1, &error_abort);
      }else if (s->addr == 8){
        qapi_event_send_x_pcb(pd->addr, "SEG4", s->buffer[s->addr], &error_abort);
      }else if (s->addr == 9){
        qapi_event_send_x_pcb(pd->addr, "LED4", s->buffer[s->addr] & 1, &error_abort);
      }else if (s->addr == 10){
        qapi_event_send_x_pcb(pd->addr, "SEG5", s->buffer[s->addr], &error_abort);
      }else if (s->addr == 11){
        qapi_event_send_x_pcb(pd->addr, "LED5", s->buffer[s->addr] & 1, &error_abort);
      }else if (s->addr == 12){
        qapi_event_send_x_pcb(pd->addr, "SEG6", s->buffer[s->addr], &error_abort);
      }else if (s->addr == 13){
        qapi_event_send_x_pcb(pd->addr, "LED6", s->buffer[s->addr] & 1, &error_abort);
      }else if (s->addr == 14){
        qapi_event_send_x_pcb(pd->addr, "SEG7", s->buffer[s->addr], &error_abort);
      }else if (s->addr == 15){
        qapi_event_send_x_pcb(pd->addr, "LED7", s->buffer[s->addr] & 1, &error_abort);
      }
//      print_led(s);
    }
    if (s->autoincrement == INCREMENT_ON){
      s->addr = (s->addr + 1) & 0xf;
    }
  }
}
static void stm32_ledkey_irq_handler(void *opaque, int n, int level)
{

    LedkeyState *s = (LedkeyState *)opaque;
    DeviceState* dev = DEVICE(opaque);
    PCBBus* bus = PCB_BUS(dev->parent_bus);

    uint8_t new_value = s->gpio_value;

    assert(n >= BIT_STB && n <= BIT_DIO);
    if(level){
      new_value |= (1<<n);
    }else{
      new_value &= ~(1<<n);
    }
//    printf("SPI(irq): <= %02x state: %d\n", new_value,s->state);
    uint8_t changed = s->gpio_value ^ new_value;
    if (!(new_value & MASK_STB)){
      if ((changed & MASK_CLK) && (new_value & MASK_CLK)){
        if (s->write_mode == MODE_WRITE){
          s->spi_byte >>= 1;
          if(new_value & MASK_DIO){
            s->spi_byte |= 0x80;
          }
          s->spi_cnt++;
        }
      }else if ((changed & MASK_CLK) && !(new_value & MASK_CLK)){
        if (s->write_mode == MODE_READ){
         int v = (s->buttons >> ((s->addr<<3)+s->spi_cnt)) & 1 ? 3300 : 0;
         bus->gpio_set_value(bus, s->dio_gpio, v);
//    printf("SPI(irq): (%x,%d,%d) => %02x state: %d\n", s->buttons, s->addr, s->spi_cnt, v,s->state);
         s->spi_byte >>= 1;
         s->spi_cnt++;
        }
      }

      if (s->spi_cnt == 8){
        lk_parse(s);
        s->spi_byte = 0;
        s->spi_cnt = 0;
      }

    }else if (s->state == STATE_CMD && s->write_mode == MODE_READ){
        s->state = STATE_READ;
        s->addr = 0;
        s->spi_cnt = 0;
        s->spi_byte = s->buttons & 0xff;
    }else if (s->state == STATE_READ && s->write_mode == MODE_READ){
        s->state = STATE_CMD;
        s->write_mode = MODE_WRITE;
        s->spi_cnt = 0;
        s->addr = 0;
        s->spi_byte = 0;
    }else{
      s->spi_byte = 0;
      s->spi_cnt = 0;
      s->state = STATE_CMD;
    }
    s->gpio_value = new_value;
}

static void stm32_ledkey_reset(DeviceState *dev)
{
    LedkeyState *s = STM32_LEDKEY(dev);

    s->gpio_value = 0;
    s->spi_byte = 0;
    s->spi_cnt = 0;
    s->buttons = 0;
    s->state = STATE_CMD;
    s->addr = 0;
    s->write_mode = MODE_WRITE;
    s->autoincrement = INCREMENT_OFF;
}
static int ledkey_get_state(PCBDevice* dev, const char* unit, Error **errp){
  LedkeyState *s = STM32_LEDKEY(dev);

  if (!strncmp("SEG", unit, 3) && unit[3]>='0' && unit[3]<='7' && unit[4] == 0){
    int index = unit[3]-'0';
    return s->buffer[index<<1];
  }
  if (!strncmp("LED", unit, 3) && unit[3]>='0' && unit[3]<='7' && unit[4] == 0){
    int index = unit[3]-'0';
    return s->buffer[(index<<1)+1] & 1;
  }
  error_setg(errp, "Unsupported unit: %s", unit);
  return 0;

}
static void ledkey_set_state(PCBDevice* dev, const char* unit, int64_t state, Error **errp){
  LedkeyState *s = STM32_LEDKEY(dev);
  uint32_t mask = 0;
  if (!strncmp("BTN0", unit, 4) && unit[4] == 0){
    mask = 0x00000001;
  }else if (!strncmp("BTN1", unit, 4) && unit[4] == 0){
    mask = 0x00000100;
  }else if (!strncmp("BTN2", unit, 4) && unit[4] == 0){
    mask = 0x00010000;
  }else if (!strncmp("BTN3", unit, 4) && unit[4] == 0){
    mask = 0x01000000;
  }else if (!strncmp("BTN4", unit, 4) && unit[4] == 0){
    mask = 0x00000010;
  }else if (!strncmp("BTN5", unit, 4) && unit[4] == 0){
    mask = 0x00001000;
  }else if (!strncmp("BTN6", unit, 4) && unit[4] == 0){
    mask = 0x00100000;
  }else if (!strncmp("BTN7", unit, 4) && unit[4] == 0){
    mask = 0x10000000;
  }else{
    error_setg(errp, "Unsupported unit: %s", unit);
    return;
  }

  if (state){
    s->buttons |= mask;
  }else{
    s->buttons &= ~mask;
  }
}
static void stm32_ledkey_realize(DeviceState *dev, Error **errp)
{
    LedkeyState *s = STM32_LEDKEY(dev);
    PCBBus* bus = PCB_BUS(dev->parent_bus);
    PCBDeviceClass *pc = PCB_DEVICE_GET_CLASS(dev);
    if (pc->parent_realize){
      pc->parent_realize(dev, errp);
    }
    s->busdev.get_state = ledkey_get_state;
    s->busdev.set_state = ledkey_set_state;

    if (STM32_PORT_INDEX(s->stb_gpio) >= STM32_GPIO_COUNT){
      error_setg(errp, "Unsupported GPIO port for STB: 0x%02x", s->stb_gpio);
      return;
    }
    if (STM32_PORT_INDEX(s->clk_gpio) >= STM32_GPIO_COUNT){
      error_setg(errp, "Unsupported GPIO port for CLK: 0x%02x", s->clk_gpio);
      return;
    }
    if (STM32_PORT_INDEX(s->dio_gpio) >= STM32_GPIO_COUNT){
      error_setg(errp, "Unsupported GPIO port for CLK: 0x%02x", s->dio_gpio);
      return;
    }
    if (STM32_PIN_INDEX(s->stb_gpio) >= STM32_GPIO_PIN_COUNT){
      error_setg(errp, "Unsupported GPIO pin for STB: 0x%02x", s->stb_gpio);
      return;
    }
    if (STM32_PIN_INDEX(s->clk_gpio) >= STM32_GPIO_PIN_COUNT){
      error_setg(errp, "Unsupported GPIO pin for CLK: 0x%02x", s->stb_gpio);
      return;
    }
    if (STM32_PIN_INDEX(s->dio_gpio) >= STM32_GPIO_PIN_COUNT){
      error_setg(errp, "Unsupported GPIO pin for DIO: 0x%02x", s->dio_gpio);
      return;
    }

    qemu_irq *gpio_irq = qemu_allocate_irqs(stm32_ledkey_irq_handler, (void *)s, 3);

    bus->gpio_connect(bus, s->stb_gpio, gpio_irq[BIT_STB]);
    bus->gpio_connect(bus, s->clk_gpio, gpio_irq[BIT_CLK]);
    bus->gpio_connect(bus, s->dio_gpio, gpio_irq[BIT_DIO]);

    stm32_ledkey_reset((DeviceState *)s);
}


static Property stm32_ledkey_properties[] = {
    DEFINE_PROP_UINT16("stb", LedkeyState, stb_gpio, STM32_PB12),
    DEFINE_PROP_UINT16("clk", LedkeyState, clk_gpio, STM32_PB13),
    DEFINE_PROP_UINT16("dio", LedkeyState, dio_gpio, STM32_PB15),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32_ledkey_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    PCBDeviceClass * pc = PCB_DEVICE_CLASS(klass);
    pc->parent_realize = dc->realize;
    dc->realize = stm32_ledkey_realize;
    dc->reset = stm32_ledkey_reset;
    dc->props = stm32_ledkey_properties;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}
static TypeInfo stm32_ledkey_info = {
    .name  = TYPE_STM32_LEDKEY,
    .parent = TYPE_PCB_DEVICE,
    .instance_size  = sizeof(LedkeyState),
    .class_init = stm32_ledkey_class_init
};

static void stm32_ledkey_register_types(void)
{
    type_register_static(&stm32_ledkey_info);
}

type_init(stm32_ledkey_register_types)
