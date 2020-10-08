// Very basic support via a Linux gpiod device
//
// Copyright (C) 2017-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#include "autoconf.h"
#include "gpio.h"
#include "command.h" // shutdown
#include "sched.h" // shutdown

#include <fcntl.h> // open
#include <stdio.h> // snprintf
#include <string.h> // memset
#include <stdlib.h> // atexit

#include "internal.h" // report_errno

#include </usr/include/linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <sys/mman.h>


#define CHIP_FILE_NAME "/dev/gpiochip%u"
#define GPIO_CONSUMER "klipper"

#define BCM2708_PERI_BASE        0x3F000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd = -1;
void *gpio_map;

// I/O access
volatile unsigned *gpio;


// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock


DECL_ENUMERATION_RANGE("pin", "gpio0", GPIO(0, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip0/gpio0", GPIO(0, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip1/gpio0", GPIO(1, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip2/gpio0", GPIO(2, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip3/gpio0", GPIO(3, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip4/gpio0", GPIO(4, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip5/gpio0", GPIO(5, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip6/gpio0", GPIO(6, 0), MAX_GPIO_LINES);
DECL_ENUMERATION_RANGE("pin", "gpiochip7/gpio0", GPIO(7, 0), MAX_GPIO_LINES);


//
// Set up a memory regions to access GPIO
//
void setup_io()
{
    if(mem_fd != -1)
        return;

    /* open /dev/mem */
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        shutdown("can't open /dev/mem");
    }

    /* mmap GPIO */
    gpio_map = mmap(
            NULL,             //Any adddress in our space will do
            BLOCK_SIZE,       //Map length
            PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
            MAP_SHARED,       //Shared with other processes
            mem_fd,           //File to map
            GPIO_BASE         //Offset to GPIO peripheral
    );

    close(mem_fd); //No need to keep mem_fd open after mmap

    if (gpio_map == MAP_FAILED) {
        printf("mmap error %d\n", (int)gpio_map);//errno also set!
        exit(-1);
    }

    // Always use volatile pointer!
    gpio = (volatile unsigned *)gpio_map;


} // setup_io



struct gpio_line {
    int chipid;
    int offset;
    int fd;
    int state;
};

static struct gpio_line lines[8*MAX_GPIO_LINES];

static int gpio_chip_fd[8] = { -1 };

static int
get_chip_fd(uint8_t chipId) {
    char chipFilename[64],errorMessage[256];
    int i = 0;
    if (gpio_chip_fd[chipId] <= 0) {
        snprintf(chipFilename,sizeof(chipFilename), CHIP_FILE_NAME, chipId);

        if(access(chipFilename, F_OK) < 0){
            snprintf(errorMessage,sizeof(errorMessage),
                "%s not found!",chipFilename);
            report_errno(errorMessage,-1);
            shutdown("GPIO chip device not found");
        }
        gpio_chip_fd[chipId] = open(chipFilename,O_RDWR | O_CLOEXEC);
        if (gpio_chip_fd[chipId] < 0) {
            snprintf(errorMessage,sizeof(errorMessage),
                "Unable to open GPIO %s",chipFilename);
            report_errno(errorMessage,-1);
            shutdown("Unable to open GPIO chip device");
        }

        for (i=0; i<MAX_GPIO_LINES; ++i) {
            lines[GPIO(chipId,i)].offset = i;
            lines[GPIO(chipId,i)].fd = -1;
            lines[GPIO(chipId,i)].chipid = chipId;
        }



    }
    return gpio_chip_fd[chipId];
}

struct gpio_out
gpio_out_setup(uint32_t pin, uint8_t val)
{
    struct gpio_line* line = &lines[pin];
    line->offset = GPIO2PIN(pin);
    line->chipid = GPIO2PORT(pin);
    struct gpio_out g = { .line = line };
    gpio_out_reset(g,val);
    return g;
}

static void
gpio_release_line(struct gpio_line* line)
{
    if(line->chipid == 0) {

    } else {
        if (line->fd > 0) {
            close(line->fd);
            line->fd = -1;
        }
    }
}

void
gpio_out_write_generic(struct gpio_out g, uint8_t val) {
    struct gpiohandle_data data;
    memset(&data, 0, sizeof(data));
    data.values[0] = !!val;
    ioctl(g.line->fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
    g.line->state = !!val;
}

void
gpio_out_write_bcm2835(struct gpio_out g, uint8_t val)
{
//    fprintf(stderr, "writing pin %d val %d\n", g.line->offset, !!val);
    if(val) {
        GPIO_SET = 1 << g.line->offset;
    } else {
        GPIO_CLR = 1 << g.line->offset;
    }
    g.line->state = !!val;
}

void
gpio_out_write(struct gpio_out g, uint8_t val)
{
    if(g.line->chipid == 0) {
        gpio_out_write_bcm2835(g,val);
    } else {
        gpio_out_write_generic(g,val);
    }
}

void
gpio_out_reset_generic(struct gpio_out g, uint8_t val)
{
    int rv;
    struct gpiohandle_request req;
    gpio_release_line(g.line);
    memset(&req, 0, sizeof(req));
    req.lines = 1;
    req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    req.lineoffsets[0] = g.line->offset;
    req.default_values[0] = !!val;
    strncpy(req.consumer_label,GPIO_CONSUMER,sizeof(req.consumer_label) - 1);
    rv = ioctl(get_chip_fd(g.line->chipid), GPIO_GET_LINEHANDLE_IOCTL, &req);
    if (rv < 0) {
        report_errno("gpio_out_reset get line",rv);
        shutdown("Unable to open out GPIO chip line");
    }
    set_close_on_exec(req.fd);
    g.line->fd = req.fd;
    g.line->state = !!val;
}

void
gpio_out_reset_bcm2835(struct gpio_out g, uint8_t val)
{
    setup_io();

    INP_GPIO(g.line->offset);
    OUT_GPIO(g.line->offset);

    gpio_out_write_bcm2835(g, val);
    fprintf(stderr, "Selecting pin %d for output done\n", g.line->offset);
}

void
gpio_out_reset(struct gpio_out g, uint8_t val)
{
    if(g.line->chipid == 0) {
        gpio_out_reset_bcm2835(g,val);
    } else {
        gpio_out_reset_generic(g,val);
    }
}





void
gpio_out_toggle(struct gpio_out g)
{
    gpio_out_write(g,!g.line->state);
}

void
gpio_out_toggle_noirq(struct gpio_out g)
{
    gpio_out_toggle(g);
}

struct gpio_in
gpio_in_setup(uint32_t pin, int8_t pull_up)
{
    struct gpio_line* line = &lines[pin];
    line->offset = GPIO2PIN(pin);
    line->chipid = GPIO2PORT(pin);
    struct gpio_in g = { .line = line };
    gpio_in_reset(g,pull_up);
    return g;
}

void
gpio_in_reset_generic(struct gpio_in g, int8_t pull_up)
{
    int rv;
    struct gpiohandle_request req;
    gpio_release_line(g.line);
    memset(&req, 0, sizeof(req));
    req.lines = 1;
    req.flags = GPIOHANDLE_REQUEST_INPUT;
#if defined(GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP)
    if (pull_up > 0) {
        req.flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
    } else if (pull_up < 0) {
        req.flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
    }
#endif
    req.lineoffsets[0] = g.line->offset;
    strncpy(req.consumer_label,GPIO_CONSUMER,sizeof(req.consumer_label) - 1);
    rv = ioctl(
        get_chip_fd(g.line->chipid),GPIO_GET_LINEHANDLE_IOCTL,&req);
    if (rv < 0) {
        report_errno("gpio_in_reset get line",rv);
        shutdown("Unable to open in GPIO chip line");
    }
    set_close_on_exec(req.fd);
    g.line->fd = req.fd;
}

void
gpio_in_reset_bcm2835(struct gpio_in g, int8_t pull_up)
{
    setup_io();

    fprintf(stderr, "Selecting pin %d for input done\n", g.line->offset);

    //  with a pullup
#if defined(GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP)
    if (pull_up > 0) {
        //bcm2835_gpio_set_pud(g.line->offset, BCM2835_GPIO_PUD_UP);
        fprintf(stderr, "pin %d pullup done\n", g.line->offset);
    } else if (pull_up < 0) {
        //bcm2835_gpio_set_pud(g.line->offset, BCM2835_GPIO_PUD_DOWN);
        fprintf(stderr, "pin %d pulldown done\n", g.line->offset);
    }
#endif

}

void
gpio_in_reset(struct gpio_in g, int8_t pull_up)
{
    gpio_in_reset_generic(g,pull_up);
    return;

    if(g.line->chipid == 0) {
        gpio_in_reset_bcm2835(g,pull_up);
    } else {
        gpio_in_reset_generic(g,pull_up);
    }
}

uint8_t
gpio_in_read_generic(struct gpio_in g)
{
    struct gpiohandle_data data;
    memset(&data, 0, sizeof(data));
    ioctl(g.line->fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
    return data.values[0];
}
uint8_t
gpio_in_read_bcm2835(struct gpio_in g)
{
    return GET_GPIO(g.line->offset);
}
uint8_t
gpio_in_read(struct gpio_in g)
{
    return gpio_in_read_generic(g);
    if(g.line->chipid == 0) {
        return gpio_in_read_bcm2835(g);
    } else {
        return gpio_in_read_generic(g);
    }
}
