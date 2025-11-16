/*
 * QEMU RISC-V VirtIO machine interface
 *
 * Copyright (c) 2017 SiFive, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_UR_DP1000_H
#define HW_UR_DP1000_H

#include "hw/boards.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/sysbus.h"
#include "hw/block/flash.h"
#include "hw/intc/riscv_imsic.h"
#include "hw/pci-host/designware.h"
#include "hw/gpio/sifive_gpio.h"

#define UR_DP1000_CPUS_MAX_BITS             9
#define UR_DP1000_CPUS_MAX                  (1 << UR_DP1000_CPUS_MAX_BITS)
#define UR_DP1000_SOCKETS_MAX_BITS          2
#define UR_DP1000_SOCKETS_MAX               (1 << UR_DP1000_SOCKETS_MAX_BITS)

#define TYPE_UR_DP1000_MACHINE MACHINE_TYPE_NAME("ur_dp1000")
typedef struct UltraRISCState UltraRISCState;
DECLARE_INSTANCE_CHECKER(UltraRISCState, UR_DP1000_MACHINE,
                         TYPE_UR_DP1000_MACHINE)

typedef enum UltraRISCAIAType {
    UR_DP1000_AIA_TYPE_NONE = 0,
    UR_DP1000_AIA_TYPE_APLIC,
    UR_DP1000_AIA_TYPE_APLIC_IMSIC,
} UltraRISCAIAType;

struct UltraRISCState {
    /*< private >*/
    MachineState parent;

    /*< public >*/
    Notifier machine_done;
    DeviceState *platform_bus_dev;
    RISCVHartArrayState soc[UR_DP1000_SOCKETS_MAX];
    DeviceState *irqchip[UR_DP1000_SOCKETS_MAX];
    PFlashCFI01 *flash[2];
    FWCfgState *fw_cfg;

    int fdt_size;
    bool have_aclint;
    UltraRISCAIAType aia_type;
    int aia_guests;
    char *oem_id;
    char *oem_table_id;
    OnOffAuto acpi;
    const MemMapEntry *memmap;
    DesignwarePCIEHost pcie;
    SIFIVEGPIOState gpio;
};

enum {
    UR_DP1000_DEBUG,
    UR_DP1000_MROM,
    UR_DP1000_TEST,
    UR_DP1000_RTC,
    UR_DP1000_CLINT,
    UR_DP1000_ACLINT_SSWI,
    UR_DP1000_PLIC,
    UR_DP1000_APLIC_M,
    UR_DP1000_APLIC_S,
    UR_DP1000_UART0,
    UR_DP1000_VIRTIO,
    UR_DP1000_FW_CFG,
    UR_DP1000_IMSIC_M,
    UR_DP1000_IMSIC_S,
    UR_DP1000_FLASH,
    UR_DP1000_DRAM,
    UR_DP1000_PCIE_PIO0,
    UR_DP1000_PCIE_MMIO_0,
    UR_DP1000_PCIE_MMIO64_0,
    UR_DP1000_PCIE_PIO1,
    UR_DP1000_PCIE_MMIO_1,
    UR_DP1000_PCIE_MMIO64_1,
    UR_DP1000_PCIE_PIO2,
    UR_DP1000_PCIE_MMIO_2,
    UR_DP1000_PCIE_MMIO64_2,
    UR_DP1000_PLATFORM_BUS,
    UR_DP1000_GPIO,
    UR_DP1000_PCIE_DBI_0,
    UR_DP1000_PCIE_DBI_1,
    UR_DP1000_PCIE_DBI_2,
    UR_DP1000_PCIE_CFG0,
    UR_DP1000_PCIE_CFG1,
    UR_DP1000_PCIE_CFG2,
    UR_DP1000_PINMUX,
    UR_DP1000_UART1,
    UR_DP1000_UART2,
    UR_DP1000_UART3,
    UR_DP1000_UART4,
    UR_DP1000_SPI0,
    UR_DP1000_SPI1,
    UR_DP1000_I2C0,
    UR_DP1000_I2C1,
    UR_DP1000_I2C2,
    UR_DP1000_I2C3,
};

#if 0
enum {
    UR_DP1000_DEBUG,
    UR_DP1000_MROM,
    UR_DP1000_TEST,
    UR_DP1000_RTC,
    UR_DP1000_CLINT,
    UR_DP1000_ACLINT_SSWI,
    UR_DP1000_PLIC,
    UR_DP1000_APLIC_M,
    UR_DP1000_APLIC_S,
    UR_DP1000_UART0,
    UR_DP1000_VIRTIO,
    UR_DP1000_FW_CFG,
    UR_DP1000_IMSIC_M,
    UR_DP1000_IMSIC_S,
    UR_DP1000_FLASH,
    UR_DP1000_DRAM,
    UR_DP1000_PCIE_MMIO,
    UR_DP1000_PCIE_PIO,
    UR_DP1000_PLATFORM_BUS,
    UR_DP1000_PCIE_ECAM,
    UR_DP1000_PCIE_DBI_0
};
#endif

enum {
    UART0_IRQ = 10,
    RTC_IRQ = 11,
    VIRTIO_IRQ = 1, /* 1 to 8 */
    VIRTIO_COUNT = 8,
    PCIE_IRQ = 0x20, /* 32 to 35 */
    UR_DP1000_PLATFORM_BUS_IRQ = 64, /* 64 to 95 */
};

#define UR_DP1000_PLATFORM_BUS_NUM_IRQS 32

#define UR_DP1000_IRQCHIP_NUM_MSIS 255
#define UR_DP1000_IRQCHIP_NUM_SOURCES 96
#define UR_DP1000_IRQCHIP_NUM_PRIO_BITS 3
#define UR_DP1000_IRQCHIP_MAX_GUESTS_BITS 3
#define UR_DP1000_IRQCHIP_MAX_GUESTS ((1U << UR_DP1000_IRQCHIP_MAX_GUESTS_BITS) - 1U)

#define UR_DP1000_PLIC_PRIORITY_BASE 0x00
#define UR_DP1000_PLIC_PENDING_BASE 0x1000
#define UR_DP1000_PLIC_ENABLE_BASE 0x2000
#define UR_DP1000_PLIC_ENABLE_STRIDE 0x80
#define UR_DP1000_PLIC_CONTEXT_BASE 0x200000
#define UR_DP1000_PLIC_CONTEXT_STRIDE 0x1000
#define UR_DP1000_PLIC_SIZE(__num_context) \
    (UR_DP1000_PLIC_CONTEXT_BASE + (__num_context) * UR_DP1000_PLIC_CONTEXT_STRIDE)

#define FDT_PCI_ADDR_CELLS    3
#define FDT_PCI_INT_CELLS     1
#define FDT_PLIC_ADDR_CELLS   0
#define FDT_PLIC_INT_CELLS    1
#define FDT_APLIC_INT_CELLS   2
#define FDT_APLIC_ADDR_CELLS  0
#define FDT_IMSIC_INT_CELLS   0
#define FDT_MAX_INT_CELLS     2
#define FDT_MAX_INT_MAP_WIDTH (FDT_PCI_ADDR_CELLS + FDT_PCI_INT_CELLS + \
                                 1 + FDT_MAX_INT_CELLS)
#define FDT_PLIC_INT_MAP_WIDTH  (FDT_PCI_ADDR_CELLS + FDT_PCI_INT_CELLS + \
                                 1 + FDT_PLIC_INT_CELLS)
#define FDT_APLIC_INT_MAP_WIDTH (FDT_PCI_ADDR_CELLS + FDT_PCI_INT_CELLS + \
                                 1 + FDT_APLIC_INT_CELLS)

uint32_t imsic_num_bits(uint32_t count);

/*
 * The virt machine physical address space used by some of the devices
 * namely ACLINT, PLIC, APLIC, and IMSIC depend on number of Sockets,
 * number of CPUs, and number of IMSIC guest files.
 *
 * Various limits defined by UR_DP1000_SOCKETS_MAX_BITS, UR_DP1000_CPUS_MAX_BITS,
 * and UR_DP1000_IRQCHIP_MAX_GUESTS_BITS are tuned for maximum utilization
 * of virt machine physical address space.
 */

#define UR_DP1000_IMSIC_GROUP_MAX_SIZE      (1U << IMSIC_MMIO_GROUP_MIN_SHIFT)
#if UR_DP1000_IMSIC_GROUP_MAX_SIZE < \
    IMSIC_GROUP_SIZE(UR_DP1000_CPUS_MAX_BITS, UR_DP1000_IRQCHIP_MAX_GUESTS_BITS)
#error "Can't accommodate single IMSIC group in address space"
#endif

#define UR_DP1000_IMSIC_MAX_SIZE            (UR_DP1000_SOCKETS_MAX * \
                                        UR_DP1000_IMSIC_GROUP_MAX_SIZE)
#if 0x4000000 < UR_DP1000_IMSIC_MAX_SIZE
#error "Can't accommodate all IMSIC groups in address space"
#endif

#endif
