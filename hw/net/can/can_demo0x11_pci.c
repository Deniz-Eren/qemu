/*
 * Demo PCIe CAN device (SJA1000 based) emulation
 *
 * Currently there are no MSI-X (PCI Capability 0x11) CAN devices, so this
 * template was made to provide future device emulation developers a starting
 * point to support devices that utilize MSI-X, MSI and regular IRQs.
 * This demo registers itself as an Advantech iDoor Module: 2-Ports Isolated
 * CANBus mPCIe, DB9, however with MSI-X, which the actual device does not
 * support.
 *
 * Copyright (c) 2023 Deniz Eren (deniz.eren@icloud.com)
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

#include "qemu/units.h"
#include "qemu/osdep.h"
#include "qemu/event_notifier.h"
#include "qemu/module.h"
#include "qemu/thread.h"
#include "qemu/sockets.h"
#include "qapi/error.h"
#include "chardev/char.h"
#include "hw/irq.h"
#include "hw/pci/pci_device.h"
#include "hw/qdev-properties.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "migration/vmstate.h"
#include "net/can_emu.h"

#include "can_sja1000.h"
#include "qom/object.h"

#define TYPE_CAN_PCI_DEV "demo0x11_pci"

typedef struct Demo0x11PCIeState Demo0x11PCIeState;
DECLARE_INSTANCE_CHECKER(Demo0x11PCIeState, DEMO0X11_PCI_DEV,
                         TYPE_CAN_PCI_DEV)

/* the PCI device and vendor IDs */
#ifndef DEMO0X11_PCI_VENDOR_ID1
#define DEMO0X11_PCI_VENDOR_ID1    0x13fe
#endif

#ifndef DEMO0X11_PCI_DEVICE_ID1
#define DEMO0X11_PCI_DEVICE_ID1    0x00d7
#endif

#define DEMO0X11_PCI_SJA_COUNT     2
#define DEMO0X11_PCI_SJA_RANGE     0x400

#define DEMO0X11_PCI_BYTES_PER_SJA 0x80

#define DEMO0X11_IO_IDX            0
#define DEMO0X11_MSIX_IDX          1

#define DEMO0X11_MSIX_SIZE         (16 * KiB)

#define DEMO0X11_MSIX_TABLE        (0x0000)
#define DEMO0X11_MSIX_PBA          (0x2000)

#define DEMO0X11_MSI_VEC_NUM       (8)
#define DEMO0X11_MSIX_VEC_NUM      DEMO0X11_MSI_VEC_NUM
#define DEMO0X11_MSI_RI_ENTRY      (0) /* Receive interrupt */
#define DEMO0X11_MSI_TI_ENTRY      (1) /* Transmit interrupt */
#define DEMO0X11_MSI_EI_ENTRY      (2) /* Error warning interrupt */
#define DEMO0X11_MSI_DOI_ENTRY     (3) /* Data overrun interrupt */
#define DEMO0X11_MSI_WUI_ENTRY     (4) /* Wakeup interrupt */
#define DEMO0X11_MSI_EPI_ENTRY     (5) /* Error passive */
#define DEMO0X11_MSI_ALI_ENTRY     (6) /* Arbitration lost */
#define DEMO0X11_MSI_BEI_ENTRY     (7) /* Bus error interrupt */

struct Demo0x11PCIeState {
    /*< private >*/
    PCIDevice       dev;
    /*< public >*/
    MemoryRegion    io;
    MemoryRegion    msix;

    CanSJA1000State sja_state[DEMO0X11_PCI_SJA_COUNT];
    qemu_irq        irq;

    char            *model; /* The model that support, only SJA1000 now. */
    CanBusState     *canbus[DEMO0X11_PCI_SJA_COUNT];
};

static void demo0x11_pci_reset(DeviceState *dev)
{
    Demo0x11PCIeState *d = DEMO0X11_PCI_DEV(dev);
    int i;

    for (i = 0 ; i < DEMO0X11_PCI_SJA_COUNT; i++) {
        can_sja_hardware_reset(&d->sja_state[i]);
    }

    msix_reset(&d->dev);
}

static uint64_t demo0x11_pci_io_read(void *opaque, hwaddr addr, unsigned size)
{
    Demo0x11PCIeState *d = opaque;
    CanSJA1000State *s = &d->sja_state[0];
    hwaddr _addr = addr;

    if (addr >= DEMO0X11_PCI_SJA_RANGE) {
        s = &d->sja_state[1];
        _addr -= DEMO0X11_PCI_SJA_RANGE;
    }

    if (_addr >= DEMO0X11_PCI_BYTES_PER_SJA) {
        return 0;
    }

    return can_sja_mem_read(s, _addr >> 2, size);
}

static void demo0x11_pci_io_write(void *opaque, hwaddr addr, uint64_t data,
                                  unsigned size)
{
    Demo0x11PCIeState *d = opaque;
    CanSJA1000State *s = &d->sja_state[0];
    hwaddr _addr = addr;

    if (addr >= DEMO0X11_PCI_SJA_RANGE) {
        s = &d->sja_state[1];
        _addr -= DEMO0X11_PCI_SJA_RANGE;
    }

    if (_addr >= DEMO0X11_PCI_BYTES_PER_SJA) {
        return;
    }

    can_sja_mem_write(s, _addr >> 2, data, size);
}

static const MemoryRegionOps demo0x11_pci_io_ops = {
    .read = demo0x11_pci_io_read,
    .write = demo0x11_pci_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .max_access_size = 1,
    },
};

static void
demo0x11_unuse_msix_vectors(Demo0x11PCIeState *s, int num_vectors)
{
    int i;
    for (i = 0; i < num_vectors; i++) {
        msix_vector_unuse(PCI_DEVICE(s), i);
    }
}

static void
demo0x11_use_msix_vectors(Demo0x11PCIeState *s, int num_vectors)
{
    int i;
    for (i = 0; i < num_vectors; i++) {
        msix_vector_use(PCI_DEVICE(s), i);
    }
}

static void
demo0x11_init_msix(Demo0x11PCIeState *s)
{
    Error *err = NULL;

    int res = msix_init(PCI_DEVICE(s), DEMO0X11_MSIX_VEC_NUM,
                        &s->msix,
                        DEMO0X11_MSIX_IDX, DEMO0X11_MSIX_TABLE,
                        &s->msix,
                        DEMO0X11_MSIX_IDX, DEMO0X11_MSIX_PBA,
                        0xA0, &err);

    if (res < 0) {
        error_setg(&err, "demo0x11_init_msix failed");
        return;
    } else {
        demo0x11_use_msix_vectors(s, DEMO0X11_MSIX_VEC_NUM);
    }
}

static void
demo0x11_cleanup_msix(Demo0x11PCIeState *s)
{
    if (msix_present(PCI_DEVICE(s))) {
        demo0x11_unuse_msix_vectors(s, DEMO0X11_MSIX_VEC_NUM);
        msix_uninit(PCI_DEVICE(s), &s->msix, &s->msix);
    }
}

static void demo0x11_pci_realize(PCIDevice *pci_dev, Error **errp)
{
    static const uint16_t pcie_offset = 0x0E0;
    Demo0x11PCIeState *d = DEMO0X11_PCI_DEV(pci_dev);
    uint8_t *pci_conf;
    Error *err = NULL;
    int i;
    int ret;

    /* Map MSI and MSI-X vector entries one-to-one for each interrupt */
    uint8_t msi_map[DEMO0X11_MSI_VEC_NUM] = {
        DEMO0X11_MSI_RI_ENTRY,  /* Receive interrupt */
        DEMO0X11_MSI_TI_ENTRY,  /* Transmit interrupt */
        DEMO0X11_MSI_EI_ENTRY,  /* Error warning interrupt */
        DEMO0X11_MSI_DOI_ENTRY, /* Data overrun interrupt */
        DEMO0X11_MSI_WUI_ENTRY, /* Wakeup interrupt */
        DEMO0X11_MSI_EPI_ENTRY, /* Error passive */
        DEMO0X11_MSI_ALI_ENTRY, /* Arbitration lost */
        DEMO0X11_MSI_BEI_ENTRY  /* Bus error interrupt */
    };

    pci_conf = pci_dev->config;
    pci_conf[PCI_INTERRUPT_PIN] = 0x01; /* interrupt pin A */

    d->irq = pci_allocate_irq(&d->dev);

    for (i = 0 ; i < DEMO0X11_PCI_SJA_COUNT; i++) {
        can_sja_cap_init(&d->sja_state[i], d->irq, pci_dev, msi_map, msi_map);
    }

    for (i = 0 ; i < DEMO0X11_PCI_SJA_COUNT; i++) {
        if (can_sja_connect_to_bus(&d->sja_state[i], d->canbus[i]) < 0) {
            error_setg(errp, "can_sja_connect_to_bus failed");
            return;
        }
    }

    memory_region_init_io(&d->io, OBJECT(d), &demo0x11_pci_io_ops,
                          d, "demo0x11_pci-io", 2*DEMO0X11_PCI_SJA_RANGE);
    pci_register_bar(&d->dev, DEMO0X11_IO_IDX,
                     PCI_BASE_ADDRESS_SPACE_MEMORY, &d->io);

    memory_region_init(&d->msix, OBJECT(d), "demo0x11_pci-msix",
                       DEMO0X11_MSIX_SIZE);
    pci_register_bar(&d->dev, DEMO0X11_MSIX_IDX,
                     PCI_BASE_ADDRESS_SPACE_MEMORY, &d->msix);

    demo0x11_init_msix(d);

    if (pcie_endpoint_cap_v1_init(pci_dev, pcie_offset) < 0) {
        error_setg(errp, "Failed to initialize PCIe capability");
        return;
    }

    ret = msi_init( PCI_DEVICE(d), 0xD0, DEMO0X11_MSI_VEC_NUM,
            true, false, NULL );

    if (ret) {
        error_setg(errp, "msi_init failed (%d)", ret);
        return;
    }

    error_free(err);
}

static void demo0x11_pci_exit(PCIDevice *pci_dev)
{
    Demo0x11PCIeState *d = DEMO0X11_PCI_DEV(pci_dev);
    int i;

    for (i = 0 ; i < DEMO0X11_PCI_SJA_COUNT; i++) {
        can_sja_disconnect(&d->sja_state[i]);
    }

    qemu_free_irq(d->irq);
    demo0x11_cleanup_msix(d);
    msi_uninit(pci_dev);
}

static const VMStateDescription vmstate_demo0x11_pci = {
    .name = TYPE_CAN_PCI_DEV,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(dev, Demo0x11PCIeState),
        VMSTATE_STRUCT(sja_state[0], Demo0x11PCIeState, 0, vmstate_can_sja,
                       CanSJA1000State),
        VMSTATE_STRUCT(sja_state[1], Demo0x11PCIeState, 0, vmstate_can_sja,
                       CanSJA1000State),
        VMSTATE_END_OF_LIST()
    }
};

static void demo0x11_pci_instance_init(Object *obj)
{
    Demo0x11PCIeState *d = DEMO0X11_PCI_DEV(obj);

    object_property_add_link(obj, "canbus0", TYPE_CAN_BUS,
                             (Object **)&d->canbus[0],
                             qdev_prop_allow_set_link_before_realize,
                             0);
    object_property_add_link(obj, "canbus1", TYPE_CAN_BUS,
                             (Object **)&d->canbus[1],
                             qdev_prop_allow_set_link_before_realize,
                             0);
}

static void demo0x11_pci_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = demo0x11_pci_realize;
    k->exit = demo0x11_pci_exit;
    k->vendor_id = DEMO0X11_PCI_VENDOR_ID1;
    k->device_id = DEMO0X11_PCI_DEVICE_ID1;
    k->revision = 0x00;
    k->class_id = 0x000c09;
    k->subsystem_vendor_id = DEMO0X11_PCI_VENDOR_ID1;
    k->subsystem_id = DEMO0X11_PCI_DEVICE_ID1;
    dc->desc = "Template or example device supporting MSI-X capability";
    dc->vmsd = &vmstate_demo0x11_pci;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->reset = demo0x11_pci_reset;
}

static const TypeInfo demo0x11_pci_info = {
    .name          = TYPE_CAN_PCI_DEV,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(Demo0x11PCIeState),
    .class_init    = demo0x11_pci_class_init,
    .instance_init = demo0x11_pci_instance_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static void demo0x11_pci_register_types(void)
{
    type_register_static(&demo0x11_pci_info);
}

type_init(demo0x11_pci_register_types)
