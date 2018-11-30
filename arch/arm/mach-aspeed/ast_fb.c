#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/tty.h>

extern void power_putstr(const char *ptr);
void fb_printf(const char *fmt, ...)
{
    static char buf[1024];
    va_list args;
    int r;


    va_start(args, fmt);

    r = vsnprintf(buf, sizeof(buf), fmt, args);
    power_putstr(buf);

  va_end(args);

}

int fb_initialized = 0;

void fb_init(void)
{
    fb_initialized = 1;
}

