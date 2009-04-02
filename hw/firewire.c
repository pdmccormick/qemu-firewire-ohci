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
#include "qemu-timer.h"
#include "qemu_socket.h"

#define OHCI1394_DEBUG         1

#define PCI_CLASS_SERIAL_FIREWIRE_OHCI   0x0c0010
#define PCI_VENDOR_ID_TI        0x104c
#define PCI_DEVICE_ID_TI_TSB43AB22  0x8023

#define OHCI1394_PCI_Status_Reg     0x04
#define OHCI1394_PCI_Command_Reg    0x06
#define OHCI1394_PCI_HCI_Control    0x40
#define OHCI1394_PCI_Base_Adr_0     0x10

#define OHCI1394_REGISTER_SIZE      0x800

typedef struct OHCI1394State_st {
    PCIDevice dev;

    int regs_index;
    uint32_t regs_addr;
} OHCI1394State;

static void ohci1394_regs_map(PCIDevice *pci_dev, int region_num, uint32_t addr, uint32_t size, int type)
{
    OHCI1394State *d = (OHCI1394State *)pci_dev;

    d->regs_addr = addr;

#ifdef OHCI1394_DEBUG
    printf("ohci1394_regs_map addr=0x%08x size=0x%08x\n", addr, size);
#endif

    cpu_register_physical_memory(addr, OHCI1394_REGISTER_SIZE, d->regs_index);
}

static uint32_t ohci1394_regs_readl(void *opaque, target_phys_addr_t addr)
{
    OHCI1394State *d = opaque;
    uint32_t val;

    val = 0;

#ifdef OHCI1394_DEBUG
    printf("ohci1394_regs_readl addr=0x" TARGET_FMT_plx " val=0x%08x\n", addr, val);
#endif

    return val;
}

static void ohci1394_regs_writel(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    OHCI1394State *d = opaque;

#ifdef OHCI1394_DEBUG
    printf("ohci1394_regs_writel addr=0x" TARGET_FMT_plx " val=0x%08x\n", addr, val);
#endif

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



PCIDevice *pci_firewire_init(PCIBus *bus, int devfn)
{
    OHCI1394State *d;
    uint8_t *pci_conf;

    printf("firewire init\n");

    d = (OHCI1394State *)pci_register_device(bus, "Firewire OHCI", sizeof(OHCI1394State), devfn, NULL, NULL);

    pci_conf = d->dev.config;

    pci_config_set_vendor_id(pci_conf, PCI_VENDOR_ID_TI);
    pci_config_set_device_id(pci_conf, PCI_DEVICE_ID_TI_TSB43AB22);
    pci_config_set_class(pci_conf, PCI_CLASS_SERIAL_FIREWIRE);

    // seems to be safest, working way I have so far!!
//    *(uint32_t *)(pci_conf+0x08) = cpu_to_le32((PCI_CLASS_SERIAL_FIREWIRE_OHCI << 8) + 1);
//
    *(uint8_t *)(pci_conf+0x09) = 0x10;    /* IEEE1394 OHCI - programming interface */
//    cpu_to_le32((PCI_CLASS_SERIAL_FIREWIRE_OHCI << 8) + 1);

    *(uint16_t *)(pci_conf+OHCI1394_PCI_Command_Reg) = cpu_to_le16(0x0046);
    *(uint16_t *)(pci_conf+OHCI1394_PCI_Status_Reg) = cpu_to_le16(0x0000);
    *(uint32_t *)(pci_conf+OHCI1394_PCI_HCI_Control) = cpu_to_le16(0x0000);

    /*
     * allocate 2kB region in PCI host memory
     * set base_adr_0 (via pci_conf) as ptr to said region
     */

    d->regs_index = cpu_register_io_memory(0, ohci1394_regs_read, ohci1394_regs_write, d);
    pci_register_io_region(&d->dev, 0, OHCI1394_REGISTER_SIZE, PCI_ADDRESS_SPACE_MEM, ohci1394_regs_map);

    *(uint32_t *)(pci_conf+OHCI1394_PCI_Base_Adr_0) = d->regs_addr;

    return (PCIDevice *)d;
}

