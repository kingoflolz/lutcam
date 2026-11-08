/* Standalone RV32EC camera supervisor. No interrupts, heap or host services. */
#include <stdint.h>
#include "sensor_mode.h"
#include "ae.h"

#ifdef FIRMWARE_TEST
uint32_t mmio_read(uint32_t address);
void mmio_write(uint32_t address, uint32_t value);
#else
static uint32_t mmio_read(uint32_t a) { return *(volatile uint32_t *)(uintptr_t)a; }
static void mmio_write(uint32_t a, uint32_t v) { *(volatile uint32_t *)(uintptr_t)a = v; }
#endif
#define MAIL 0x30028000u
#define CTL  0x30024000u
#define PAIR (CTL + 0x200)
#ifdef FIRMWARE_TEST
#define RD(o) mmio_read(MAIL + (o))
#define WR(o,v) mmio_write(MAIL + (o), (v))
#else
/* Shared directly with the host aperture; no hardware mailbox register bank. */
volatile uint32_t mailbox[14] __attribute__((section(".mailbox"))) = {0};
#define RD(o) ((o)<0x40 ? mailbox[((o)-8)/4] : mmio_read(MAIL+(o)))
#define WR(o,v) do { if ((o)<0x40) mailbox[((o)-8)/4]=(v); \
                     else mmio_write(MAIL+(o),(v)); } while (0)
#endif
#define CLOCK_HZ 120000000u
enum { READY=1, BUSY=2, FAILED=4 };
enum { INIT=1, ISP=2, STREAM_ON=3, STREAM_OFF=4, MANUAL=5, AUTO=6 };
enum { E_I2C=1, E_STATE=4, E_ARG=5, E_HIST=6 };
/* 0xc0 encodes zero compensation; each code step is 1/4 EV, range +/-2 EV.
   This lies outside the old mean-target range and needs no wider mailbox. */
static uint32_t lines=400, gain=16, enabled=1, compensation=0xc0;
static AeHistory ae_history;
/* Cooperative tasks share their temporary storage. Camera work starts only
   after the UDP reader has finished using its outgoing packet. */
static union {
    uint32_t packet[67];
    struct { uint32_t options[8],word,index,input; } reader;
    struct { AeHistogram a,b; int32_t scratch[6]; } ae;
} work;
#include "network.h"

/* RV32I compiler helpers keep the core free of multiply/divide hardware. */
uint32_t __mulsi3(uint32_t a, uint32_t b) {
    uint32_t r=0;
    while (b) { if (b&1) r+=a; a<<=1; b>>=1; }
    return r;
}
uint32_t __udivsi3(uint32_t a, uint32_t b) {
    uint32_t q=0;
    /* The only firmware division is bounded exposure energy / 885: fewer
       than 362 subtractions. Prevent conversion back to a recursive divide. */
    while (a>=b) { a-=b; ++q; __asm__("" : "+r"(a)); }
    return q;
}
static void delay(uint32_t cycles) {
    uint32_t start=RD(0x40);
    while (RD(0x40)-start < cycles) {}
}
/* Every operation is a complete trusted engine record; no mutation retries. */
static __attribute__((noreturn)) void restart(uint32_t reason) {
    WR(0x0c,reason); WR(8,FAILED);
    NW(0x4c,1); /* Revoke capture before reloading the current slot. */
    uint32_t boot=0x30026000u;
    uint32_t command=(mmio_read(boot+0x24)+4)&~3u;
    mmio_write(boot+0x24,command|mmio_read(boot+8));
    for (;;) {}
}
static void sensor(uint32_t cam, uint32_t reg, uint32_t value) {
    uint32_t base=0x30020000u+(cam<<12), start=RD(0x40);
    if (mmio_read(base+0x14)&1) restart(E_I2C);
    uint32_t id=mmio_read(base+0x24);
    mmio_write(base+0x80, id);
    /* These are the board's strapped seven-bit addresses, not shifted bytes. */
    mmio_write(base+0x84, (cam ? 0x10 : 0x60) | (reg<<8));
    mmio_write(base+0x88, value);
    mmio_write(base+0x8c, id);
    while (mmio_read(base+0x20)!=id) {
        if (RD(0x40)-start > CLOCK_HZ/5) restart(E_I2C);
    }
    if (mmio_read(base+0x28)&255) restart(E_I2C);
}
static void sensors(uint32_t r, uint32_t v) {
    sensor(0,r,v); sensor(1,r,v);
}
static void idle(void) {
    if (mmio_read(PAIR+8) || (mmio_read(PAIR+12)&15)) restart(E_STATE);
}
static void power(uint32_t value) {
    for (uint32_t c=0;c<2;c++) mmio_write(CTL+(c<<8)+12,value);
    delay(CLOCK_HZ/1000);
}
static void initialize(void) {
    idle();
    ae_history.valid=0;
    mmio_write(PAIR+0x6c,0);
    power(4); power(5);
    uint32_t page=0;
    for (const uint8_t *p=sensor_mode;p<sensor_mode+sizeof(sensor_mode);) {
        uint32_t reg=*p++;
        if (reg&128) { page=(reg&127)<<8; reg=*p++; }
        sensors(page|reg,*p++);
    }
    delay(CLOCK_HZ/1000*66);
    lines=400; gain=16;
}
static void isp_initialize(void) {
    idle();
    for (uint32_t c=0;c<2;c++) {
        uint32_t base=0x30000000u+(c<<16), start=RD(0x40);
        mmio_write(base+4,0);
        while (mmio_read(base+8)&2) {
            if (RD(0x40)-start>CLOCK_HZ/10) restart(E_STATE);
        }
        mmio_write(base+12,0); /* no defect calibration installed */
        mmio_write(base+16,3);
        while (mmio_read(base+8)&8) {
            if (RD(0x40)-start>CLOCK_HZ/10) restart(E_STATE);
        }
        mmio_write(base+4,13); /* RAW/4, histogram, complete-frame snapshots */
    }
}
static void streaming(uint32_t on) {
    idle();
    sensors(0x0100,on);
}
static void exposure(uint32_t next_lines, uint32_t next_gain) {
    if (next_lines<1 || next_lines>885 || next_gain<16 || next_gain>255) restart(E_ARG);
    static const uint8_t offsets[]={0,1,2,8,9};
    uint8_t values[]={0,next_lines>>4,(next_lines<<4)&255,0,next_gain};
    sensors(0x3208,0);
    for (uint32_t i=0;i<5;i++) sensors(0x3500+offsets[i],values[i]);
    sensors(0x3208,0x10);
    /* Failed staging or END restarts before either group can launch. */
    sensors(0x3208,0xa0);
    delay(CLOCK_HZ/40);
    lines=next_lines; gain=next_gain;
}
static void meter(uint32_t c, AeHistogram *h) {
    uint32_t base=0x30022000u+(c<<12);
    h->count=h->p50=h->p90=h->p99=h->clipped=0;
    mmio_write(base,1);
    for (uint32_t i=0;i<512;i++) {
        uint32_t word=mmio_read(base+8), n=word&0xfffff;
        if (((word>>20)&511)!=i || (word>>30)!=(i==0 ? 2u : i==511 ? 1u : 0u)) restart(E_HIST);
        ae_bin(h,i,n);
    }
    if (h->count!=512000) restart(E_HIST);
}
static void publish(void) {
    WR(0x24,lines); WR(0x28,gain); WR(0x2c,compensation); WR(0x38,enabled);
    WR(8,READY);
}
static __attribute__((noinline)) void camera_poll(void) {
    static uint32_t discard=1;
    WR(0x3c,RD(0x3c)+1);
    uint32_t seq=RD(0x10);
    if (seq!=RD(0x20)) {
        uint32_t op=RD(0x14), a=RD(0x18), b=RD(0x1c);
        WR(8,BUSY);
        if (op==INIT) initialize();
        else if (op==ISP) isp_initialize();
        else if (op==STREAM_ON || op==STREAM_OFF) streaming(op==STREAM_ON);
        else if (op==MANUAL) { enabled=0; exposure(a,b); }
        else if (op==AUTO && a<=1 && b>=0xb8 && b<=0xc8) { enabled=a; compensation=b; }
        else restart(E_ARG);
        publish(); WR(0x20,seq); discard=1;
    }
    if (!mmio_read(PAIR+8)) return;
    if (!(mmio_read(0x30022004)&1) || !(mmio_read(0x30023004)&1)) return;
    meter(0,&work.ae.a); meter(1,&work.ae.b);
    WR(0x30,work.ae.a.p90>>1); WR(0x34,work.ae.b.p90>>1);
    /* Discard the already-frozen frame after each update; the next
       complete frame is then guaranteed to follow the settling delay. */
    if (discard) { discard=0; return; }
    int32_t correction=ae_error(&ae_history,&work.ae.a,&work.ae.b,lines*gain,
                                (int32_t)compensation-0xc0,work.ae.scratch);
    if (enabled) {
        uint32_t settings=ae_settings(lines,gain,correction,ae_history.exposure);
        uint32_t l=settings&1023, g=settings>>10;
        if (l!=lines || g!=gain) { exposure(l,g); discard=1; }
    }
    publish();
}
int main(void) {
    WR(8,BUSY);
    initialize(); isp_initialize(); publish();
    while (1) {
        network_poll();
        if (!(NR(4)&2)) camera_poll();
    }
}
