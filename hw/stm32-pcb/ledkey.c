#include "hw/stm32-pcb/pcb.h"
#include "hw/sysbus.h"
#include "hw/arm/stm32.h"
#include <inttypes.h>

#define STATE_CMD 0
#define STATE_ADDR 1
#define STATE_DATA 2

#define MODE_WRITE 0
#define MODE_READ  1

#define INCREMENT_ON 0
#define INCREMENT_OFF 1

#define BIT_NSS  0
#define BIT_SCK  1
#define BIT_MISO 2
#define BIT_MOSI 3

#define MASK_NSS (1<<BIT_NSS)
#define MASK_SCK (1<<BIT_SCK)
#define MASK_MISO (1<<BIT_MISO)
#define MASK_MOSI (1<<BIT_MOSI)

#define STM32_GPIO_INDEX(port,pin) (((port & 0xf)<<4) | (pin & 0xf))
#define STM32_PORT_INDEX(gpio) ((gpio>>4) & 0xf)
#define STM32_PIN_INDEX(gpio) (gpio & 0xf)

#define STM32_PB12 STM32_GPIO_INDEX(STM32_GPIOB_INDEX, 12)
#define STM32_PB13 STM32_GPIO_INDEX(STM32_GPIOB_INDEX, 13)
#define STM32_PB14 STM32_GPIO_INDEX(STM32_GPIOB_INDEX, 14)
#define STM32_PB15 STM32_GPIO_INDEX(STM32_GPIOB_INDEX, 15)

typedef struct  {
    /* Inherited */
    PCBDevice busdev;

    /* Properties */
    uint8_t nss_gpio;
    uint8_t sck_gpio;
    uint8_t miso_gpio;
    uint8_t mosi_gpio;

    /* Private */

    uint8_t gpio_value;
    uint8_t spi_byte;
    uint8_t spi_cnt;
    uint8_t buffer[16];

    uint8_t state;
    uint8_t addr;
    uint8_t write_mode;
    uint8_t autoincrement;
} LedkeyState;


#define TYPE_STM32_LEDKEY "stm32-periph-ledkey"
#define STM32_LEDKEY(obj) OBJECT_CHECK(LedkeyState, (obj), TYPE_STM32_LEDKEY)



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

static void lk_parse(LedkeyState *s){
  uint8_t data = s->spi_byte;
//  printf("SPI:0x%02x\n", data);
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
  }else if (s->state == STATE_DATA){
//    printf("Data: @%x=0x%02x\n", s->addr, data);
    if (s->write_mode == MODE_WRITE){
      s->buffer[s->addr] = data;
      print_led(s);
    }
    if (s->autoincrement == INCREMENT_ON){
      s->addr = (s->addr + 1) & 0xf;
    }
  }
}
static void stm32_ledkey_irq_handler(void *opaque, int n, int level)
{

    LedkeyState *s = (LedkeyState *)opaque;

    uint8_t new_value = s->gpio_value;

    assert(n >= BIT_NSS && n <= BIT_MOSI);
    if(level){
      new_value |= (1<<n);
    }else{
      new_value &= ~(1<<n);
    }

    uint8_t changed = s->gpio_value ^ new_value;
    if (!(new_value & MASK_NSS)){
/*
printf("%c%c%c%c vs %c%c%c%c\n", 
    s->gpio_value & MASK_NSS?'N':'n',
    s->gpio_value & MASK_SCK?'C':'c',
    s->gpio_value & MASK_MISO?'I':'i',
    s->gpio_value & MASK_MOSI?'O':'o',
    new_value & MASK_NSS?'N':'n',
    new_value & MASK_SCK?'C':'c',
    new_value & MASK_MISO?'I':'i',
    new_value & MASK_MOSI?'O':'o'
);
*/
      if ((changed & MASK_SCK) && (new_value & MASK_SCK)){
        s->spi_byte >>= 1;
        if(new_value & MASK_MOSI){
          s->spi_byte |= 0x80;
        }
        s->spi_cnt++;
      }

      if (s->spi_cnt == 8){
        lk_parse(s);
        s->spi_byte = 0;
        s->spi_cnt = 0;
      }

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

    s->state = STATE_CMD;
    s->addr = 0;
    s->write_mode = MODE_WRITE;
    s->autoincrement = INCREMENT_OFF;
}


static DeviceState* find_gpio(uint8_t gpio_param, DeviceState* gpio_a,DeviceState* gpio_b,DeviceState* gpio_c){
    if (STM32_PORT_INDEX(gpio_param) == 0){
      return gpio_a;
    }else if (STM32_PORT_INDEX(gpio_param) == 1){
      return gpio_b;
    }else if (STM32_PORT_INDEX(gpio_param) == 2){
      return gpio_c;
    }else{
      return 0;
    }
}
static void stm32_ledkey_realize(DeviceState *dev, Error **errp)
{
    qemu_irq *gpio_irq;
    LedkeyState *s = STM32_LEDKEY(dev);

    DeviceState *gpio_a = DEVICE(object_resolve_path("/machine/stm32/gpio[a]", NULL));
    DeviceState *gpio_b = DEVICE(object_resolve_path("/machine/stm32/gpio[b]", NULL));
    DeviceState *gpio_c = DEVICE(object_resolve_path("/machine/stm32/gpio[c]", NULL));


    DeviceState *gpio_nss = find_gpio(s->nss_gpio, gpio_a, gpio_b, gpio_c);
    DeviceState *gpio_sck = find_gpio(s->sck_gpio, gpio_a, gpio_b, gpio_c);
    DeviceState *gpio_miso = find_gpio(s->miso_gpio, gpio_a, gpio_b, gpio_c);
    DeviceState *gpio_mosi = find_gpio(s->mosi_gpio, gpio_a, gpio_b, gpio_c);

    if (!gpio_nss){
      error_setg(errp, "Unsupported GPIO port for NSS: 0x%02x", s->nss_gpio);
      return;
    }
    if (!gpio_sck){
      error_setg(errp, "Unsupported GPIO port for SCK: 0x%02x", s->sck_gpio);
      return;
    }
    if (!gpio_miso){
      error_setg(errp, "Unsupported GPIO port for MISO: 0x%02x", s->miso_gpio);
      return;
    }
    if (!gpio_mosi){
      error_setg(errp, "Unsupported GPIO port for SCK: 0x%02x", s->mosi_gpio);
      return;
    }
    if (STM32_PIN_INDEX(s->nss_gpio) >= STM32_GPIO_PIN_COUNT){
      error_setg(errp, "Unsupported GPIO pin for NSS: 0x%02x", s->nss_gpio);
      return;
    }
    if (STM32_PIN_INDEX(s->sck_gpio) >= STM32_GPIO_PIN_COUNT){
      error_setg(errp, "Unsupported GPIO pin for SCK: 0x%02x", s->nss_gpio);
      return;
    }
    if (STM32_PIN_INDEX(s->miso_gpio) >= STM32_GPIO_PIN_COUNT){
      error_setg(errp, "Unsupported GPIO pin for MISO: 0x%02x", s->miso_gpio);
      return;
    }
    if (STM32_PIN_INDEX(s->mosi_gpio) >= STM32_GPIO_PIN_COUNT){
      error_setg(errp, "Unsupported GPIO pin for MOSI: 0x%02x", s->mosi_gpio);
      return;
    }

    gpio_irq = qemu_allocate_irqs(stm32_ledkey_irq_handler, (void *)s, 4);

    qdev_connect_gpio_out(gpio_nss, STM32_PIN_INDEX(s->nss_gpio), gpio_irq[BIT_NSS]);
    qdev_connect_gpio_out(gpio_sck, STM32_PIN_INDEX(s->sck_gpio), gpio_irq[BIT_SCK]);
    qdev_connect_gpio_out(gpio_miso, STM32_PIN_INDEX(s->miso_gpio), gpio_irq[BIT_MISO]);
    qdev_connect_gpio_out(gpio_mosi, STM32_PIN_INDEX(s->mosi_gpio), gpio_irq[BIT_MOSI]);

    stm32_ledkey_reset((DeviceState *)s);

}


static Property stm32_ledkey_properties[] = {
    DEFINE_PROP_UINT8("nss-in", LedkeyState, nss_gpio, STM32_PB12),
    DEFINE_PROP_UINT8("sck-in", LedkeyState, sck_gpio, STM32_PB13),
    DEFINE_PROP_UINT8("miso-in", LedkeyState, miso_gpio, STM32_PB14),
    DEFINE_PROP_UINT8("mosi-out", LedkeyState, mosi_gpio, STM32_PB15),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32_ledkey_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

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
