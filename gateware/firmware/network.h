/* DHCPv4 policy lives in the RV32 firmware. The UDP aperture only transports
   bytes; TX DMA reads this persistent buffer from the supervisor SRAM. */
#define NET 0x30029000u
#define NR(o) mmio_read(NET+(o))
#define NW(o,v) mmio_write(NET+(o),(v))
enum { N_SELECT=1, N_REQUEST, N_BOUND, N_RENEW };
#define net_packet work.packet
_Static_assert(sizeof(net_packet) == 268, "DHCP aperture requires a 268-byte packet");
static uint32_t net_state, net_xid, net_clock, net_seconds, net_start, net_due;
static uint32_t net_ip, net_server, net_lease, net_t1, net_epoch;
/* Parsing can reuse the already validated BOOTP scalars. Overloaded option
   areas begin at byte 44 and remain intact. */
#define net_word work.reader.word
#define net_byte_index work.reader.index
#define net_input work.reader.input
typedef union {
    struct { uint32_t type, server, lease, mask, router, t1, overload; };
    uint32_t value[7];
} NetOptions;
static __attribute__((noinline)) uint32_t net_swap(uint32_t value) {
    return __builtin_bswap32(value);
}
static uint32_t net_byte(void) {
    uint32_t i=net_byte_index++;
    if (net_input) return ((uint8_t *)net_packet)[net_input+i];
    if (!(i&3)) net_word=NR(0x1c);
    uint32_t value=net_word&255; net_word>>=8; return value;
}
/* Parse bounded TLVs, including PAD/END and overloaded file/sname fields.
   Unused options are skipped; truncated options cannot change configuration. */
static uint32_t net_options(uint32_t p, uint32_t length) {
    NetOptions *o=(NetOptions *)net_packet;
    net_input=p; net_byte_index=0;
    while (length--) {
        uint32_t code=net_byte();
        if (code==255) return 1;
        if (!code) continue;
        if (!length--) return 0;
        uint32_t n=net_byte(), value=0, field=0;
        if (n>length) return 0;
        length-=n;
        static const uint8_t codes[7]={53,54,51,1,3,58,52};
        static const uint8_t sizes[7]={1,4,4,4,4,4,1};
        while (field<7 && codes[field]!=code) ++field;
        if (field<7 && n<sizes[field]) return 0;
        for (uint32_t j=0;j<n;j++) {
            uint32_t b=net_byte();
            if (j<4) value=(value<<8)|b;
        }
        if (field<7 && !o->value[field]) o->value[field]=value;
    }
    return 0; /* Every option area needs an END marker. */
}
static void net_configure(uint32_t ip, uint32_t mask, uint32_t router) {
    if (NR(8)!=ip || NR(12)!=mask || NR(16)!=router) {
        /* Revoking/changing a lease revokes capture admission too. A host
           must resolve the new route before arming the cameras again. */
        NW(0x4c,1);
        NW(8,ip); NW(12,mask); NW(16,router);
    }
}
static void net_restart(void) {
    net_configure(0,0,0);
    /* chaddr distinguishes boards; the free-running timer identifies this
       transaction. Retransmissions retain the same XID. */
    net_xid=RD(0x40);
    net_state=N_SELECT; net_start=net_seconds; net_due=net_seconds;
    net_ip=net_server=net_lease=0;
}
static void net_send(void) {
    for (uint32_t i=0;i<sizeof(net_packet)/4;i++) net_packet[i]=0;
    net_packet[0]=0x00060101; net_packet[1]=net_xid;
    uint32_t seconds=net_seconds-net_start;
    uint32_t renewing=net_state==N_RENEW;
    net_packet[2]=(seconds>>8)|((seconds&255)<<8)|(net_state!=N_RENEW ? 0x00800000 : 0);
    if (renewing) net_packet[3]=net_swap(net_ip);
    net_packet[7]=NR(0x34); net_packet[8]=NR(0x38);
    net_packet[59]=0x63538263;
    /* PAD octets align option values to words. chaddr identifies the client;
       no duplicate client-id or redundant maximum-size option is necessary. */
    net_packet[60]=0x00000135|((net_state==N_SELECT ? 1u : 3u)<<16);
    net_packet[61]=0x03010237; /* request subnet mask and router */
    uint32_t end=62;
    if (!renewing) {
        net_packet[62]=0x04320000;
        net_packet[63]=net_swap(net_state==N_SELECT ? NR(0x3c) : net_ip); end=64;
        if (net_state==N_REQUEST) {
            net_packet[64]=0x04360000; net_packet[65]=net_swap(net_server); end=66;
        }
    }
    net_packet[end]=255;
    NW(0x24,(uint32_t)(uintptr_t)net_packet); NW(0x28,sizeof(net_packet));
    NW(0x2c,net_state==N_RENEW ? net_server : 0xffffffffu); NW(0x30,1);
}
static void net_receive(void) {
    uint32_t length=NR(0x14);
    for (uint32_t i=0;i<60;i++) net_packet[i]=NR(0x1c);
    uint32_t ip=net_swap(net_packet[4]);
    if ((net_packet[0]&0xffffff)!=0x060102 || net_packet[1]!=net_xid
            || net_packet[7]!=NR(0x34) || (net_packet[8]&65535)!=NR(0x38)
            || net_packet[59]!=0x63538263) goto done;
    NetOptions *o=(NetOptions *)net_packet;
    for (uint32_t i=0;i<7;i++) o->value[i]=0;
    uint32_t valid=net_options(0,length-240);
    if (o->overload>3) valid=0;
    if (o->overload&1) valid &= net_options(108,128);
    if (o->overload&2) valid &= net_options(44,64);
    if (!valid || !o->server) goto done;
    if (net_state==N_SELECT && o->type==2 && ip) {
        net_ip=ip; net_server=o->server; net_state=N_REQUEST;
        net_due=net_start=net_seconds;
    } else if ((net_state==N_REQUEST || net_state==N_RENEW) && o->server==net_server) {
        if (o->type==6) net_restart();
        else if (o->type==5 && ip==net_ip && o->lease>net_seconds-net_start) {
            uint32_t mask=o->mask ? o->mask : NR(12), router=o->router ? o->router : NR(16);
            if (!mask) goto done;
            net_lease=o->lease;
            net_t1=o->t1;
            if (!net_t1 || net_t1>=net_lease) net_t1=(net_lease+1)/2;
            net_epoch=net_start; net_server=o->server; net_state=N_BOUND;
            net_configure(ip,mask,router);
        }
    }
done:
    NW(0x20,1); /* Release any padding or rejected remainder. */
}
static __attribute__((noinline)) void network_poll(void) {
    if (!net_state) {
        net_clock=RD(0x40); NW(0x50,1); net_restart();
    }
    if (RD(0x40)-net_clock>=CLOCK_HZ) { net_clock+=CLOCK_HZ; ++net_seconds; }
    if (net_state!=N_BOUND && net_seconds-net_start>=16) net_restart();
    uint32_t age=net_seconds-net_epoch;
    if (net_state>=N_BOUND && net_lease!=0xffffffffu) {
        /* Failed transactions restart after 16 s. Expiry still revokes a
           short lease immediately; no additional recovery states. */
        if (age>=net_lease) net_restart();
        else if (age>=net_t1 && net_state==N_BOUND) {
            net_state=N_RENEW; net_due=net_start=net_seconds;
            ++net_xid;
        }
    }
    if (!(NR(4)&2)) { /* RX and TX reuse the same persistent buffer. */
        if (NR(4)&1) net_receive();
        if (net_state!=N_BOUND && (int32_t)(net_seconds-net_due)>=0) {
            net_send();
            net_due=net_seconds+4+(net_xid&1);
        }
    }
    NW(0x40,net_state); NW(0x44,net_lease); NW(0x48,net_server);
}
