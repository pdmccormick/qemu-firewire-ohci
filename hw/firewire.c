/*
 * QEMU IEEE1394 OHCI emulation
 *
 * Copyright (c) 2009 Peter McCormick
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include "hw.h"
#include "pci.h"
#include "qemu-char.h"

#define OHCI1394_DEBUG         1

#define PCI_REGISTER_BLOCK_SIZE      0x800

#define PCI_CLASS_SERIAL_FIREWIRE_OHCI      0x0c0010
#define PCI_VENDOR_ID_TI                    0x104c
#define PCI_DEVICE_ID_TI_TSB43AB22          0x8023


#define REG_SET(f,v)    ((f) | (v))
#define REG_CLEAR(f,v)  ((f) & ~(v))

#define FLAG_SET(d,r,f)     { (d)->regs.r |=  ( r ## _ ## f ); }
#define FLAG_UNSET(d,r,f)   { (d)->regs.r &= ~( r ## _ ## f ); }

#define FIELD_SET(s,f,v)    ( ((s) & ~(f)) | ( ((v) << ( f ## _shift )) & (f)))
#define FIELD_GET(s,f)      ( ((s) & (f)) >> (f ## _shift) )
#define FIELD_UPDATE(s,f,v) { (s) = FIELD_SET(s,f,v); }

#define SHADOW(f,t)     \
    union {             \
        uint32_t raw;   \
        t;              \
    } f;

typedef uint32_t quadlet_t;

enum ohci1349_pci_offs {
    PCI_Status      = 0x04,
    PCI_Command     = 0x06,
    PCI_HCI_Control = 0x40,
    PCI_Base_Addr_0 = 0x10
};

/* Numeric enum constants for OHCI registers */
enum ohci1394_regs_offs {
#define BEGIN_REGISTER(r,a,m)               r = a,
#define END_REGISTER
#define BEGIN_REGISTER_SET_CLEAR(r,a,m)     r ## _Set   = a,        \
                                            r ## _Clear = a + 4,
#define END_REGISTER_SET_CLEAR
#define FLAG(...)
#define FIELD(...)
#include "ohci1394-regs.h"
#undef BEGIN_REGISTER
#undef END_REGISTER
#undef BEGIN_REGISTER_SET_CLEAR
#undef END_REGISTER_SET_CLEAR
#undef FLAG
#undef FIELD
};

/* Labels for OHCI registers by address */
#define MAX_REG_FIELDS  32
typedef struct {
    uint32_t addr;
    int is_pair;
    const char *label;
    struct ohci1394_reg_field_t {
        int is_flag;
        const char *name;
        int val;
        int shift;
    } fields[MAX_REG_FIELDS];
} ohci1394_reg_desc_t;

ohci1394_reg_desc_t ohci1394_regs_desc[] = {
#define BEGIN_REGISTER(r,a,m)  { a, 0, #r, {
#define END_REGISTER  { -1, NULL, -1, -1 }   } },
#define BEGIN_REGISTER_SET_CLEAR(r,a,m) { a, 1, #r, {
#define END_REGISTER_SET_CLEAR END_REGISTER
#define FLAG(r,f,p)     { 1, #f, p, 0 },
#define FIELD(r,f,m,s)  { 0, #f, m, s },
#include "ohci1394-regs.h"
#undef BEGIN_REGISTER
#undef END_REGISTER
#undef BEGIN_REGISTER_SET_CLEAR
#undef END_REGISTER_SET_CLEAR
#undef FLAG
#undef FIELD
    { -1, -1, NULL, {} }
};

enum ohci1394_regs_mask {
    /* 3.1.1  ContextControl */
    MASK_ContextControl     = 0x00009cff,
    ContextControl_run      = (1 << 15),
    ContextControl_wake     = (1 << 12),
    ContextControl_dead     = (1 << 11),
    ContextControl_active   = (1 << 10),
    ContextControl_spd      = 0x000000e0, ContextControl_spd_shift   = 5,
    ContextControl_event    = 0x0000001f, ContextControl_event_shift = 0,

    /* 3.1.2  CommandPtr */
    MASK_CommandPtr             = 0xffffffff,
    CommandPtr_descriptorAddr   = 0xfffffff0, CommandPtr_descriptorAddr_shift = 4,
    CommandPtr_Z                = 0x0000000f, CommandPtr_Z_shift = 0,

#define BEGIN_REGISTER(r,a,m)   MASK_ ## r = m,
#define END_REGISTER
#define BEGIN_REGISTER_SET_CLEAR(r,a,m) BEGIN_REGISTER(r,a,m)
#define END_REGISTER_SET_CLEAR
#define FLAG(r,f,p)             r ## _ ## f = 1 << p,
#define FIELD(r,f,m,s)          r ## _ ## f = m, r ## _ ## f ## _shift = s,
#include "ohci1394-regs.h"
#undef BEGIN_REGISTER
#undef END_REGISTER
#undef BEGIN_REGISTER_SET_CLEAR
#undef END_REGISTER_SET_CLEAR
#undef FLAG
#undef FIELD
};

enum ohci1394_event_code {
    evt_no_status       = 0x00,
    
    evt_long_packet     = 0x02,
    evt_missing_ack     = 0x03,
    evt_underrun        = 0x04,
    evt_overrun         = 0x05,
    evt_descriptor_read = 0x06,
    evt_data_read       = 0x07,
    evt_data_write      = 0x08,
    evt_bus_reset       = 0x09,
    evt_timeout         = 0x0a,
    evt_tcode_err       = 0x0b,
    
    evt_unknown         = 0x0e,
    evt_flushed         = 0x0f,
    
    ack_complete        = 0x11,
    ack_pending         = 0x12,

    ack_busy_X          = 0x14,
    ack_busy_A          = 0x15,
    ack_busy_B          = 0x16,
    
    ack_tardy           = 0x1b,
    
    ack_data_error      = 0x1d,
    ack_type_error      = 0x1e,
};




typedef struct {
    uint32_t Version;
    uint32_t GUID_ROM;
    uint32_t ATRetries;
    // CSR
    uint32_t ConfigROMhdr;
    uint32_t BusID;
    uint32_t BusOptions;
    uint32_t GUIDHi, GUIDLo;
    uint32_t ConfigROMmap;
    uint32_t PostedWriteAddressLo, PostedWriteAddressHi;
    uint32_t VendorID;
    uint32_t HCControl;
    uint32_t SelfIDBuffer, SelfIDCount;
    uint32_t IRMultiChanMaskHi, IRMultiChanMaskLo;
    uint32_t IntEvent, IntMask;
    uint32_t IsoXmitIntEvent, IsoXmitIntMask;
    uint32_t IsoRecvIntEvent, IsoRecvIntMask;
    uint32_t InitialBandwidthAvailable;
    uint32_t InitialChannelsAvailableHi, InitialChannelsAvailableLo;
    uint32_t FairnessControl;
    uint32_t LinkControl;
    uint32_t NodeID;
    uint32_t PhyControl;
    uint32_t IsoCycleTimer;
    uint32_t AsyncReqFilterHi, AsyncReqFilterLo;
    uint32_t PhyReqFilterHi, PhyReqFilterLo;
    uint32_t PhysicalUpperBound;
} ohci1394_controller_registers;

enum ohci1394_phy_regs_mask {
    /* 0000_2 */
    PHY_Physical_ID = 0x3f, PHY_Physical_ID_shift = 0,
    PHY_R           = (1 << 6),
    PHY_PS          = (1 << 7),

    /* 0001_2 */
    PHY_RHB         = (1 << 0),
    PHY_IBR         = (1 << 1),
    PHY_Gap_count   = 0xfc, PHY_Gap_count_shift = 2,

    /* 0010_2 */
    PHY_Extended    = 0x07, PHY_Extended_shift = 0,
    PHY_Total_ports = 0x08, PHY_Total_ports_shift = 3,

    /* 0011_2 */
    PHY_Max_speed   = 0x07, PHY_Max_speed_shift = 0,
    PHY_Delay       = 0xf0, PHY_Delay_shift = 4,

    /* 0100_2 */
    PHY_LCtrl       = (1 << 0),
    PHY_Contender   = (1 << 1),
    PHY_Jitter      = 0x1c, PHY_Jitter_shift = 2,
    PHY_Pwr_class   = 0xe0, PHY_Pwr_class_shift = 5,

    /* 0101_2 */
    PHY_Watchdog    = (1 << 0),
    PHY_ISBR        = (1 << 1),
    PHY_Loop        = (1 << 2),
    PHY_Pwr_fail    = (1 << 3),
    PHY_Timeout     = (1 << 4),
    PHY_Port_event  = (1 << 5),
    PHY_Enab_accel  = (1 << 6),
    PHY_Enab_multi  = (1 << 7),

    /* 0111_2 */
    PHY_Page_select = 0x07, PHY_Page_select_shift = 0,
    PHY_Port_select = 0xf0, PHY_Port_select_shift = 4,
};

typedef struct {
    uint8_t page_select;
    uint8_t port_select;

    uint8_t ISBR;
    uint8_t Enab_accel;
} ohci1394_phy_registers;

typedef struct {
    PCIDevice dev;
    
    int host_fd;

    int regs_index;
    uint32_t regs_addr;
    
    /* OHCI register values */
    ohci1394_controller_registers regs;

    /* IEEE1394 PHY registers */
    ohci1394_phy_registers phy_regs;

    /* Internal device state */
    int softReset;
    int selfIDBufferPtr;
} OHCI1394State;


static char *ohci1394_display_reg(ohci1394_reg_desc_t *regs_desc, uint32_t addr, uint32_t val, int is_write)
{
#define DISPLAY_SIZE 1024
    static char disp[DISPLAY_SIZE];
    char *cur = disp;
    int i, size = DISPLAY_SIZE, len;
    ohci1394_reg_desc_t *reg = NULL;
    struct ohci1394_reg_field_t *field;

    i = 0;
    while(regs_desc[i].addr != -1) {
        reg = &regs_desc[i];

        if(reg->addr == addr || ((reg->addr + 4) == addr && reg->is_pair))
            break;

        i++;
    }
    
    disp[0] = '\0';
    cur = disp;

    /* nothing found */
    if(regs_desc[i].addr == -1 || reg == NULL) {
        len = snprintf(cur, size, "addr=0x%08x val=0x%08x", addr, val);

        return disp;
    }

    len = snprintf(cur, size, "@%s", reg->label);
    cur += len;
    size -= len;

    /* Only if writing a set & clear pair */
    if(is_write && reg->is_pair) {
        int setter = (reg->addr == addr);   /* ... as opposed to clearing (since *Set reg precedes *Clear */

        for(i = 0; i < MAX_REG_FIELDS; i++) {
            field = &reg->fields[i];

            if(field->is_flag == -1)
                break;

            if(!field->is_flag)
                continue;

            /* If this flag is set... */
            if(val & (1 << field->val)) {
                len = snprintf(cur, size, " %c%s", setter ? '+' : '-', field->name);
                cur += len;
                size -= len;
            }
        }

        return disp;
    }

    /* Otherwise normal register with flags and fields */
    for(i = 0; i < MAX_REG_FIELDS; i++) {
        field = &reg->fields[i];

        if(field->is_flag == -1)
            break;

        /* If this is a flag... */
        if(field->is_flag) {
            /* .. show if set */
            if(val & (1 << field->val)) {
                len = snprintf(cur, size, " %s", field->name);
                cur += len;
                size -= len;
            }
        }
        /* .. or just the field */
        else {
            len = snprintf(cur, size, " %s=0x%x", field->name, (val & field->val) >> field->shift);
            cur += len;
            size -= len;
        }
    }

    return disp;
}

static void ohci1394_interrupt_update(OHCI1394State *d)
{
    int level = 0;

    /* Update iso xmit/recv interrupt status bits */
    if((d->regs.IsoXmitIntEvent & d->regs.IsoXmitIntMask) != 0)
        FLAG_SET(d, IntEvent, isochTx);
    
    if((d->regs.IsoRecvIntEvent & d->regs.IsoRecvIntMask) != 0)
        FLAG_SET(d, IntEvent, isochRx);

    /* Determine if interrupt should be fired */
    if((d->regs.IntEvent & IntMask_masterIntEnable) != 0 && (d->regs.IntEvent & d->regs.IntMask) != 0)
        level = 1;

    qemu_set_irq(d->dev.irq[0], level);
}

static void ohci1394_reset_hard(OHCI1394State *d)
{
}

static void ohci1394_reset_soft(OHCI1394State *d)
{
    /* Do soft reset */
    
    /* Clear status */
    FLAG_UNSET(d, HCControl, softReset);
}

static void ohci1394_reset_bus(OHCI1394State *d)
{
}

/* OHCI register write handlers */
static void ohci1394_reg_write_HCControl(OHCI1394State *d, uint32_t addr, uint32_t val, int is_clear)
{
    if(is_clear)
        return;

    // val & HCControl_linkEnable  -> linkEnable set to 1
    // val & HCControl_linkEnable  -> linkEnable cleared to 0
    if(val & HCControl_linkEnable)
        printf("error! LPS cleared by software\n");
    
    // p46. Software shall not clear LPS

    if(val & HCControl_softReset)
        ohci1394_reset_soft(d);
}

static void ohci1394_reg_write_SelfIDBuffer(OHCI1394State *d, uint32_t addr, uint32_t val)
{
    d->regs.SelfIDBuffer = val;
    d->selfIDBufferPtr = FIELD_GET(val, SelfIDBuffer_selfIDBufferPtr);

#ifdef OHCI1394_DEBUG
    printf("ohci1394_reg_write_SelfIDBuffer bufferPtr=0x%08x\n", d->selfIDBufferPtr);
#endif
}

static void ohci1394_reg_write_SelfIDCount(OHCI1394State *d, uint32_t addr, uint32_t val)
{
}

static void ohci1394_phy_reg_write(OHCI1394State *d, uint8_t addr, uint8_t val)
{
#ifdef OHCI1394_DEBUG
    printf("ohci1394_phy_reg_write page=%d addr=0x%02x val=0x%02x\n", d->phy_regs.page_select, addr, val);
#endif
    
    switch(d->phy_regs.page_select) {
        /* Page 0 */
        case 0:
            switch(addr) {
                case 5: /* 0101_2 */
                    d->phy_regs.ISBR = val & PHY_ISBR;
                    d->phy_regs.Enab_accel = val & PHY_Enab_accel;
                    break;

                case 7: /* 0111_2 */
                    d->phy_regs.page_select = FIELD_GET(val, PHY_Page_select);
                    break;
            }
            break;
    }
    
    FLAG_SET(d, PhyControl, rdDone);
}

static void ohci1394_phy_reg_read(OHCI1394State *d, uint8_t addr)
{
    uint8_t val = 0;

    /* Cable environment extended PHY register map */
    if(addr >= 0 && addr <= 7) {
        switch(addr) {
            case 0: /* 0000_2 */
                break;

            case 1: /* 0001_2 */
                break;

            case 2: /* 0010_2 */
                val |= PHY_Extended;
                FIELD_UPDATE(val, PHY_Total_ports, 1);
                break;

            case 3: /* 0011_2 */
                FIELD_UPDATE(val, PHY_Max_speed, 2);
                break;

            case 4: /* 0100_2 */
                break;

            case 5: /* 0101_2 */
                val |= d->phy_regs.ISBR ? PHY_ISBR : 0;
                val |= d->phy_regs.Enab_accel ? PHY_Enab_accel : 0;
                break;

            case 6: break; /* 0110_2 reserved */

            case 7: /* 0111_2 */
                val = d->phy_regs.page_select;
                break;
        }
    }
    else
        switch(d->phy_regs.page_select) {
            /* Page 0 */
            case 0:
                switch(addr) {
                    case 8:
                        val = 0xff;
                        break;
                }
                break;

            /* Page 1 */
            case 1:
                switch(addr) {
                }
                break;
        }

#ifdef OHCI1394_DEBUG
    if(addr & ~0x07)
        printf("ohci1394_phy_reg_read page=%d addr=0x%02x val=0x%02x\n", d->phy_regs.page_select, addr, val);
    else
        printf("ohci1394_phy_reg_read addr=0x%x val=0x%02x\n", addr, val);
#endif

    FIELD_UPDATE(d->regs.PhyControl, PhyControl_rdAddr, addr);
    FIELD_UPDATE(d->regs.PhyControl, PhyControl_rdData, val);

    FLAG_SET(d, PhyControl, rdDone);
    FLAG_SET(d, IntEvent, phyRegRcvd);
    ohci1394_interrupt_update(d);
}

static void ohci1394_reg_write_PhyControl(OHCI1394State *d, uint32_t addr, uint32_t val)
{
    uint8_t phy_addr, data;

    d->regs.PhyControl = val;
    phy_addr = FIELD_GET(val, PhyControl_regAddr);

    if(val & PhyControl_rdReg) {
        FLAG_UNSET(d, PhyControl, rdReg);
        FLAG_UNSET(d, PhyControl, rdDone);

        ohci1394_phy_reg_read(d, phy_addr);
    }
    else if(val & PhyControl_wrReg) {
        FLAG_UNSET(d, PhyControl, wrReg);
        FLAG_UNSET(d, PhyControl, rdDone);

        data = FIELD_GET(val, PhyControl_wrData);

        ohci1394_phy_reg_write(d, phy_addr, data);
    }
}

/* OHCI register space */
static void ohci1394_regs_map(PCIDevice *pci_dev, int region_num, uint32_t addr, uint32_t size, int type)
{
    OHCI1394State *d = (OHCI1394State *)pci_dev;
    d->regs_addr = addr;
    cpu_register_physical_memory(addr, PCI_REGISTER_BLOCK_SIZE, d->regs_index);
}

static uint32_t ohci1394_regs_readl(void *opaque, target_phys_addr_t addr)
{
    OHCI1394State *d = opaque;
    uint32_t val;
    enum ohci1394_regs_offs reg = (enum ohci1394_regs_offs) (addr & 0xffff);

    val = 0;

#define READ_DIRECT(k)  case k: val = d->regs.k; break;

/* Set and Clear registers both return same value */
#define READ_SET_AND_CLEAR(k)       \
        case k ## _Set:             \
        case k ## _Clear:           \
            val = d->regs.k;        \
            break;

/* Set returns value, clear returns a masked one */
#define READ_SET_AND_MASK_CLEAR(k,m)        \
        case k ## _Set:                     \
            val = d->regs.k;                \
            break;                          \
        case k ## _Clear:                   \
            val = d->regs.k & d->regs.m;    \
            break;

    switch(reg) {
        READ_SET_AND_CLEAR      (HCControl)
        READ_DIRECT             (SelfIDBuffer)
        READ_DIRECT             (SelfIDCount)
        READ_SET_AND_CLEAR      (LinkControl)
        READ_DIRECT             (PhyControl)
        READ_SET_AND_MASK_CLEAR (IntEvent, IntMask)
        READ_SET_AND_CLEAR      (IntMask)
        READ_SET_AND_MASK_CLEAR (IsoXmitIntEvent, IsoXmitIntMask)
        READ_SET_AND_CLEAR      (IsoXmitIntMask)
        READ_SET_AND_MASK_CLEAR (IsoRecvIntEvent, IsoRecvIntMask)
        READ_SET_AND_CLEAR      (IsoRecvIntMask)

        default:
            break;
    }

#ifdef OHCI1394_DEBUG
    printf("ohci1394_regs_readl %s\n", ohci1394_display_reg(ohci1394_regs_desc, addr, val, 0));
#endif

    return val;
}

static void ohci1394_regs_writel(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    OHCI1394State *d = opaque;
    enum ohci1394_regs_offs reg = (enum ohci1394_regs_offs) (addr & 0xffff);

#ifdef OHCI1394_DEBUG
    printf("ohci1394_regs_writel %s\n", ohci1394_display_reg(ohci1394_regs_desc, addr, val, 1));
#endif

#define WRITE_HANDLE(f)     case f: d->regs.f = val; ohci1394_reg_write_ ## f (d, addr, val); break;

#define WRITE_SET(f)        \
        case f ## _Set:     \
            d->regs.f = REG_SET(d->regs.f, val) & MASK_ ## f;

#define WRITE_CLEAR(f)      \
        case f ## _Clear:   \
            d->regs.f = REG_CLEAR(d->regs.f, val) & MASK_ ## f;

#define WRITE_SET_AND_CLEAR_INTERRUPT(f)    \
        WRITE_SET(f)                        \
            ohci1394_interrupt_update(d);   \
            break;                          \
        WRITE_CLEAR(f)                      \
            ohci1394_interrupt_update(d);   \
            break;

#define WRITE_SET_AND_CLEAR_HANDLE(f)       \
        WRITE_SET(f)                        \
            ohci1394_reg_write_ ## f (d, addr, val, 0);  \
            break;                          \
        WRITE_CLEAR(f)                      \
            ohci1394_reg_write_ ## f (d, addr, val, 1);  \
            break;


    switch(reg) {
        /* 5.7  HCControl */
        WRITE_SET_AND_CLEAR_HANDLE(HCControl)

        /* 5.12  PhyControl */
        WRITE_HANDLE(PhyControl)

        /* 6.1  IntEvent */
        WRITE_SET_AND_CLEAR_INTERRUPT(IntEvent)
        WRITE_SET_AND_CLEAR_INTERRUPT(IntMask)

        /* 6.3.1  IsoXmitIntEvent */
        WRITE_SET_AND_CLEAR_INTERRUPT(IsoXmitIntEvent)
        
        /* 6.3.2  IsoXmitIntMask */
        WRITE_SET_AND_CLEAR_INTERRUPT(IsoXmitIntMask)

        /* 6.4.1  IsoRecvIntEvent */
        WRITE_SET_AND_CLEAR_INTERRUPT(IsoRecvIntEvent)

        /* 6.4.2  IsoRecvIntMask */
        WRITE_SET_AND_CLEAR_INTERRUPT(IsoRecvIntMask)

        /* 11.1  SelfIDBuffer */
        WRITE_HANDLE(SelfIDBuffer)
        
        /* 11.2  SelfIDCount */
        WRITE_HANDLE(SelfIDCount)

        default:
            break;
    }
}

static CPUReadMemoryFunc *ohci1394_regs_read[] = {
    NULL,
    NULL,
    (CPUReadMemoryFunc *) &ohci1394_regs_readl
};

static CPUWriteMemoryFunc *ohci1394_regs_write[] = {
    NULL,
    NULL,
    (CPUWriteMemoryFunc *) &ohci1394_regs_writel
};



/* Host interface */
static void ohci1394_host_raw_fd_read(void *opaque)
{
    OHCI1394State *d = opaque;
    uint8_t buf[1024];
    int len, size;

    len = sizeof(buf) - 1;
    size = read(d->host_fd, buf, len);

    buf[size] = '\0';

    printf("raw: %s\n", buf);
}

static void ohci1394_common_init(OHCI1394State *d)
{
    int fd;
    const char *filename = "raw";

    d->host_fd = -1;

    fd = open(filename, O_RDWR);
    if(fd < 0) {
        fprintf(stderr, "unable to open %s\n", filename);
        return;
    }

    qemu_set_fd_handler(fd, ohci1394_host_raw_fd_read, NULL, d);

    d->host_fd = fd;

    d->phy_regs.page_select = 0;
    d->phy_regs.port_select = 0;
}

/* PCI device init */
PCIDevice *pci_firewire_init(PCIBus *bus, int devfn)
{
    OHCI1394State *d;
    uint8_t *pci_conf;

    d = (OHCI1394State *) pci_register_device(bus, "Firewire OHCI", sizeof(OHCI1394State), devfn, NULL, NULL);

    pci_conf = d->dev.config;

    pci_config_set_vendor_id(pci_conf, PCI_VENDOR_ID_TI);
    pci_config_set_device_id(pci_conf, PCI_DEVICE_ID_TI_TSB43AB22);
    pci_config_set_class(pci_conf, PCI_CLASS_SERIAL_FIREWIRE);

    *(uint8_t *)(pci_conf + 0x09) = 0x10;    /* IEEE1394 OHCI - programming interface */
    *(uint8_t *)(pci_conf + 0x3d) = 0x01;    /* Interrupt pin 0 */

    *(uint16_t *)(pci_conf + PCI_Command) = cpu_to_le16(0x0046);
    *(uint16_t *)(pci_conf + PCI_Status)  = cpu_to_le16(0x0000);
    *(uint32_t *)(pci_conf + PCI_HCI_Control) = cpu_to_le16(0x0000);

    d->regs_index = cpu_register_io_memory(0, ohci1394_regs_read, ohci1394_regs_write, d);
    pci_register_io_region(&d->dev, 0, PCI_REGISTER_BLOCK_SIZE, PCI_ADDRESS_SPACE_MEM, ohci1394_regs_map);

    *(uint32_t *)(pci_conf + PCI_Base_Addr_0) = d->regs_addr;

    ohci1394_common_init(d);

    return (PCIDevice *) d;
}

