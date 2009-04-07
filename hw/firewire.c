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


#define MASK_SET(f,v)    ((f) | (v))
#define MASK_CLEAR(f,v)  ((f) & ~(v))

#define REG_SET(f,v)     { (f) = MASK_SET((f), (v)); }
#define REG_CLEAR(f,v)   { (f) = MASK_CLEAR((f), (v)); }

#define FIELD_SET(s,f,v)    ( ((s) & ~(f)) | ( ((v) << ( f ## _shift )) & f))
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

enum ohci1394_regs_offs {
    Version         = 0x000,
    GUID_ROM        = 0x004,
    ATRetries       = 0x008,
    
    CSRReadData     = 0x00c,
    CSRCompareDate  = 0x010,
    CSRControl      = 0x014,

    ConfigROMhdr    = 0x018,
    BusID           = 0x01c,
    BusOptions      = 0x020,
    GUIDHi          = 0x024,
    GUIDLo          = 0x028,

    ConfigROMmap        = 0x034,
    PostedWriteAddrLo   = 0x038,
    PostedWriteAddrHi   = 0x03c,
    VendorID            = 0x040,
    
    HCControl_Set       = 0x050,
    HCControl_Clear     = 0x054,
    
    SelfIDBuffer    = 0x064,
    SelfIDCount     = 0x068,

    IRMultiChanMaskHi_Set   = 0x070,
    IRMultiChanMaskHi_Clear = 0x074,
    
    IRMultiChanMaskLo_Set   = 0x078,
    IRMultiChanMaskLo_Clear = 0x07c,

    IntEvent_Set    = 0x080,
    IntEvent_Clear  = 0x084,

    IntMask_Set     = 0x088,
    IntMask_Clear   = 0x08c,

    IsoXmitIntEvent_Set     = 0x090,
    IsoXmitIntEvent_Clear   = 0x094,
    IsoXmitIntMask_Set      = 0x098,
    IsoXmitIntMask_Clear    = 0x09c,
    
    IsoRecvIntEvent_Set     = 0x0a0,
    IsoRecvIntEvent_Clear   = 0x0a4,
    IsoRecvIntMask_Set      = 0x0a8,
    IsoRecvIntMask_Clear    = 0x0ac,

    InitialBandwidthAvailable   = 0x0b0,
    InitialChannelsAvailableHi  = 0x0b4,
    InitialChannelsAvailableLo  = 0x0b8,

    FairnessControl     = 0x0dc,
    LinkControl_Set     = 0x0e0,
    LinkControl_Clear   = 0x0e4,
    NodeID              = 0x0e8,
    PhyControl          = 0x0ec,
    IsoCycleTimer       = 0x0f0,
    
    AsyncReqFilterHi_Set   = 0x100,
    AsyncReqFilterHi_Clear = 0x104,
    AsyncReqFilterLo_Set   = 0x108,
    AsyncReqFilterLo_Clear = 0x10c,

    PhyReqFilterHi_Set     = 0x110,
    PhyReqFilterHi_Clear   = 0x114,
    PhyReqFilterLo_Set     = 0x118,
    PhyReqFilterLo_Clear   = 0x11c,

    PhysicalUpperBound     = 0x120,
    
    /* Async Request Transmit */ 
    AsyncReqXmitContextControl_Set   = 0x180,
    AsyncReqXmitContextControl_Clear = 0x184,
    AsyncReqXmitContextCommandPtr    = 0x18c,
    
    /* Async Response Transmit */ 
    AsyncRespXmitContextControl_Set   = 0x1a0,
    AsyncRespXmitContextControl_Clear = 0x1a4,
    AsyncRespXmitContextCommandPtr    = 0x1ac,
    
    /* Async Request Receive */ 
    AsyncReqRecvContextControl_Set   = 0x1c0,
    AsyncReqRecvContextControl_Clear = 0x1c4,
    AsyncReqRecvContextCommandPtr    = 0x1cc,
    
    /* Async Response Receive */ 
    AsyncRespRecvContextControl_Set   = 0x1e0,
    AsyncRespRecvContextControl_Clear = 0x1e4,
    AsyncRepsRecvContextCommandPtr    = 0x1ec,

    /* Iso Transmit */    
    IsoXmitContextControl_Set   = 0x200,  // + 16*n
    IsoXmitContextControl_Clear = 0x204,  // + 16*n
    IsoXmitContextCommandPtr    = 0x20C,  // + 16*n

    /* Iso Receive */    
    IsoRecvContextControl_Set   = 0x400,  // + 32*n
    IsoRecvContextControl_Clear = 0x404,  // + 32*n
    IsoRecvContextCommandPtr    = 0x40C,  // + 32*n
    IsoRecvContextContextMatch  = 0x410,  // + 32*n
};

enum ohci1394_regs_mask {
    /* 3.1.1  ContextControl */
    ContextControl_run      = (1 << 15),
    ContextControl_wake     = (1 << 12),
    ContextControl_dead     = (1 << 11),
    ContextControl_active   = (1 << 10),
    ContextControl_spd      = 0x000000e0, ContextControl_spd_shift   = 5,
    ContextControl_event    = 0x0000001f, ContextControl_event_shift = 0,

    /* 3.1.2  CommandPtr */
    CommandPtr_descriptorAddr   = 0xfffffff0, CommandPtr_descriptorAddr_shift = 4,
    CommandPtr_Z                = 0x0000000f, CommandPtr_Z_shift = 0,

    /* 5.2  Version */
    Version_GUID_ROM    = (1 << 24),
    Version_version     = 0x00ff0000, Version_version_shift  = 16,
    Version_revision    = 0x000000ff, Version_revision_shift = 0,

    /* 5.3  GUID_ROM */
    GUID_ROM_addrReset  = (1 << 31),
    GUID_ROM_rdStart    = (1 << 25),
    GUID_ROM_rdData     = 0x00ff0000, GUID_ROM_rdData_shift  = 16,
    GUID_ROM_miniROM    = 0x000000ff, GUID_ROM_miniROM_shift = 0,

    /* 5.4  ATRetries */
    ATRetries_secondLimit        = 0xe0000000, ATRetries_secondLimit_shift        = 29,
    ATRetries_cycleLimit         = 0x1fff0000, ATRetries_cycleLimit_shift         = 16,
    ATRetries_maxPhysRespRetries = 0x00000f00, ATRetries_maxPhysRespRetries_shift = 8,
    ATRetries_maxATRespRetries   = 0x000000f0, ATRetries_maxATRespRetries_shift   = 4,
    ATRetries_maxATReqRetries    = 0x0000000f, ATRetries_maxATReqRetries_shift    = 0,

    /* 5.5  p56 */
    
    /* 5.6  VendorID */
    VendorID_vendorUnique    = 0xff000000, VendorID_vendorUnique_shift    = 24,
    VendorID_vendorCompanyID = 0x00ffffff, VendorID_vendorCompanyID_shift = 0,
    
    /* 5.7  HCControl */
    HCControl_BIBimageValid     = (1 << 31),
    HCControl_noByteSwapData    = (1 << 30),
    HCControl_ackTardyEnable    = (1 << 29),
    HCControl_programPhyEnable  = (1 << 23),
    HCControl_aPhyEnhanceEnable = (1 << 22),
    HCControl_LPS               = (1 << 19),
    HCControl_postedWriteEnable = (1 << 18),
    HCControl_linkEnable        = (1 << 17),
    HCControl_softReset         = (1 << 16),

    /* 5.10  LinkControl */
    LinkControl_cycleSource        = (1 << 22),
    LinkControl_cycleMastre        = (1 << 21),
    LinkControl_cycleTimerEnable   = (1 << 20),
    LinkControl_rcvPhyPkt          = (1 << 10),
    LinkControl_rcvSelfID          = (1 << 9),
    LinkControl_tag1SyncFilterLock = (1 << 6),

    /* 5.11  Node ID */
    NodeID_iDValid    = (1 << 31),
    NodeID_root       = (1 << 30),
    NodeID_CPS        = (1 << 27),
    NodeID_busNumber  = 0x0000ffc0, NodeID_busNumber_shift  = 6,
    NodeID_nodeNumber = 0x0000003f, NodeID_nodeNumber_shift = 0,

    /* 5.12  PhyControl */
    PhyControl_rdDone  = (1 << 31),
    PhyControl_rdAddr  = 0x0f000000, PhyControl_rdAddr_shift  = 24,
    PhyControl_rdData  = 0x00ff0000, PhyControl_rdData_shift  = 16,
    PhyControl_rdReg   = (1 << 15),
    PhyControl_wrReg   = (1 << 14),
    PhyControl_regAddr = 0x00000f00, PhyControl_regAddr_shift = 8,
    PhyControl_wrData  = 0x000000ff, PhyControl_wrData_shift  = 0,

    /* 5.13  IsoCycleTimer */
    IsoCycleTimer_cycleSeconds = 0xfe000000, IsoCycleTimer_cycleSeconds_shift = 25,
    IsoCycleTimer_cycleCount   = 0x01fff000, IsoCycleTimer_cycleCount_shift   = 12,
    IsoCycleTimer_cycleOffest  = 0x00000fff, IsoCycleTimer_cycleOffset_shift  = 0,

    /* 5.14.1  AsyncReqFilter */
    /* 5.14.2  PhyReqFilter */

    /* 6.1  IntEvent */
    IntEvent_,

    /* 6.2  IntMask */



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
    int _reserved0 : 16;
    int run : 1;
    int _reserved1 : 2;
    int wake : 1;
    int dead : 1;
    int active : 1;
    int _reserved2 : 2;
    int spd : 3;
    enum ohci1394_event_code event : 5;
} ohci1394_context_control_t;

typedef struct {
    int descriptorAddress : 28;
    int z : 4;
} ohci1394_command_ptr_t;

struct ohci1394_reg_HCControl {
    int BIBimageValid : 1;
    int noByteSwapData : 1;
    int ackTardyEnable : 1;
    int _reserved0 : 5;
    int programPhyEnable : 1;
    int aPhyEnhanceEnable : 1;
    int _reserved1 : 2;
    int LPS : 1;
    int postedWriteEnable : 1;
    int linkEnable : 1;
    int softReset : 1;
    int _reserved2 : 16;
};


typedef struct {
    uint32_t Version;
    uint32_t GUID_ROM;
    uint32_t ATRetries;
    uint32_t BusID;

    uint32_t HCControl;

    uint32_t LinkControl;
} ohci1394_controller_registers;

typedef struct {
    PCIDevice dev;
    
    int host_fd;

    int regs_index;
    uint32_t regs_addr;

    ohci1394_controller_registers regs;
} OHCI1394State;


static void ohci1394_regs_map(PCIDevice *pci_dev, int region_num, uint32_t addr, uint32_t size, int type)
{
    OHCI1394State *d = (OHCI1394State *)pci_dev;

    d->regs_addr = addr;

#ifdef OHCI1394_DEBUG
    printf("ohci1394_regs_map addr=0x%08x size=0x%08x\n", addr, size);
#endif

    cpu_register_physical_memory(addr, PCI_REGISTER_BLOCK_SIZE, d->regs_index);
}

static uint32_t ohci1394_regs_readl(void *opaque, target_phys_addr_t addr)
{
    OHCI1394State *d = opaque;
    uint32_t val;
    enum ohci1394_regs_offs reg = (enum ohci1394_regs_offs) (addr & 0xffff);

    switch(reg) {
        case HCControl_Set:
        case HCControl_Clear:
            val = d->regs.HCControl;
            break;

        case LinkControl_Set:
        case LinkControl_Clear:
            val = d->regs.LinkControl;
            break;

default: return -1;
    }

    val = 0;

#ifdef OHCI1394_DEBUG
    printf("ohci1394_regs_readl addr=0x" TARGET_FMT_plx " val=0x%08x\n", addr, val);
#endif

    return val;
}

static void ohci1394_regs_writel(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    OHCI1394State *d = opaque;
    enum ohci1394_regs_offs reg = (enum ohci1394_regs_offs) (addr & 0xffff);

#ifdef OHCI1394_DEBUG
    printf("ohci1394_regs_writel addr=0x" TARGET_FMT_plx " val=0x%08x\n", addr, val);
#endif

    switch(reg) {
        case HCControl_Set:
            REG_SET(d->regs.HCControl, val);

            // val & HCControl_linkEnable  -> linkEnable set to 1
            break;
        
        case HCControl_Clear:
            REG_CLEAR(d->regs.HCControl, val);

            // val & HCControl_linkEnable  -> linkEnable cleared to 0
            if(val & HCControl_linkEnable)
                printf("error! LPS cleared by software");
            
            // p46. Software shall not clear LPS
            break;

        /* 5.12  PhyControl */
        case PhyControl:
//    PhyControl_rdData  = 0x00ff0000, PhyControl_rdData_shift  = 16,

            break;

default: return;
    }
}

static CPUReadMemoryFunc *ohci1394_regs_read[] = {
    NULL,
    NULL,
    (CPUReadMemoryFunc *)&ohci1394_regs_readl
};

static CPUWriteMemoryFunc *ohci1394_regs_write[] = {
    NULL,
    NULL,
    (CPUWriteMemoryFunc *)&ohci1394_regs_writel
};

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
}

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

    *(uint16_t *)(pci_conf + PCI_Command) = cpu_to_le16(0x0046);
    *(uint16_t *)(pci_conf + PCI_Status)  = cpu_to_le16(0x0000);
    *(uint32_t *)(pci_conf + PCI_HCI_Control) = cpu_to_le16(0x0000);

    d->regs_index = cpu_register_io_memory(0, ohci1394_regs_read, ohci1394_regs_write, d);
    pci_register_io_region(&d->dev, 0, PCI_REGISTER_BLOCK_SIZE, PCI_ADDRESS_SPACE_MEM, ohci1394_regs_map);

    *(uint32_t *)(pci_conf + PCI_Base_Addr_0) = d->regs_addr;

    ohci1394_common_init(d);

    return (PCIDevice *)d;
}

