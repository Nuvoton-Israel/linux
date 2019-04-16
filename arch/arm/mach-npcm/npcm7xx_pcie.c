// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2017-2019 Nuvoton Technology corporation.
 
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/fwnode.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/io.h>
#include <linux/spinlock.h>
#include <asm/signal.h>
#include <asm/mach/pci.h>
#include <linux/platform_device.h>

#include <linux/msi.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>
#include <asm/cputype.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_address.h>


#undef NPCM7XX_PCIE_RC_INTERRUPT
#define NPCM7XX_PCIE_RC_INTERRUPT pcie_rc_irq
int pcie_rc_irq;
#endif

/* #define PCIE_DEBUG */
#define KERN_DEBUG_PCIE  KERN_INFO

#ifdef PCIE_DEBUG
#define PCIE_DBG(fmt,args...)   printk(fmt ,##args)
#else
#define PCIE_DBG(fmt,args...)
#endif

#define DRIVER_NAME "npcm7xx-pcie-rc"

/* PCIe Root Complex Register */
#define LINKSTAT			0x92
#define RCCFGNUM			0x140
#define IMSI_ADDR			0x190
#define PCIERC_IMASK_LOCAL_ADDR		0x180
#define PCIERC_ISTATUS_LOCAL_ADDR	0x184
#define PCIERC_ISTATUS_MSI_ADDR		0x194

#define PCIERC_ISTATUS_LOCAL_MSI_BIT	BIT(28)
#define PCIERC_ISTATUS_LOCAL_INTA_BIT	BIT(24)
#define NPCM7XX_CORE_SELECT		(15)

/* PCIe-to-AXI Window 0 Registers */
#define RCPA0SAL	0x600
#define RCPA0SAH	0x604
#define RCPA0TAL	0x608
#define RCPA0TAH	0x60C
#define RCPA0TP		0x610
#define RCPA0TM		0x618

/* AXI-to-PCIe Window 1 to 4 Registers */
#define RCAPnSAL(n) (0x800 + (0x20 * (n)))
#define RCAPnSAH(n) (0x804 + (0x20 * (n)))
#define RCAPnTAL(n) (0x808 + (0x20 * (n)))
#define RCAPnTAH(n) (0x80C + (0x20 * (n)))
#define RCAPnTP(n)  (0x810 + (0x20 * (n)))

/* RCAPnSAL register fields */
#define ATR_SIZE_4K     11
#define ATR_SIZE_8K     12
#define ATR_SIZE_16K    13
#define ATR_SIZE_32K    14
#define ATR_SIZE_64K    15
#define ATR_SIZE_128K   16
#define ATR_SIZE_256K   17
#define ATR_SIZE_512K   18
#define ATR_SIZE_1M     19
#define ATR_SIZE_2M     20
#define ATR_SIZE_4M     21
#define ATR_SIZE_8M     22
#define ATR_SIZE_16M    23
#define ATR_SIZE_32M    24
#define ATR_SIZE_64M    25
#define ATR_SIZE_128M   26
#define ATR_SIZE_256M   27
#define ATR_SIZE_512M   28
#define ATR_SIZE_1G     29
#define ATR_SIZE_2G     30

/* RCAPnTP register fields */
#define TRSF_PARAM_MEMORY    (0L << 16)
#define TRSF_PARAM_CONFIG    (1L << 16)
#define TRSF_PARAM_IO        (2L << 16)
#define TRSL_ID_PCIE_TX_RX   0 
#define TRSL_ID_PCIE_CONFIG  1 


#define PLDA_XPRESS_RICH_MEMORY_WINDOW 	0
#define PLDA_XPRESS_RICH_CONFIG_WINDOW 	1
#define PLDA_XPRESS_RICH_IO_WINDOW 		2
//#define PLDA_XPRESS_RICH_IO_WINDOW 		1
#define PLDA_XPRESS_RICH_MESSAGE_WINDOW	4

#define PLDA_XPRESS_RICH_TARGET_PCI_TX_RX	0
#define PLDA_XPRESS_RICH_TARGET_PCI_CONFIG	1
#define PLDA_XPRESS_RICH_TARGET_AXI_MASTER	4

#define PCI_RC_ATTR_WIN_EN_POS			0
#define PCI_RC_ATTR_WIN_SIZE_POS		1
#define PCI_RC_ATTR_AP_ADDR_L_POS		12

#define PCI_RC_ATTR_TRSF_PARAM_POS		16
#define PCI_RC_ATTR_TRSL_ID_POS			0

#define IRQ_REQUEST
#define PCI_BAR_REG 0x10
#define PCI_CMD_STATUS_REG  0x4

#define  PCI_COMMAND_IO		0x1
#define  PCI_COMMAND_MEMORY	0x2
#define  PCI_COMMAND_MASTER	0x4

/*
 * Windows definitions
 * !! important note !! - Window base
 *  must be align to window size.
 */
#define CONFIG_WIN_BASE    (0xE8000000 + 0)
#define IO_WIN_BASE        (0xE8000000 + SZ_1M)
#define MEM_WIN_BASE       (0xE8000000 + SZ_32M)  

#define CONFIG_WIN_SIZE    SZ_4K
#define IO_WIN_SIZE        SZ_1M
#define MEM_WIN_SIZE       SZ_32M

#define CONFIG_WIN_V_BASE  npcm7xx_pcie_rc_config_area_virtual_address
//#define DBG_DELAY

/* Other rgister for initialzing root complex */
#define MFSEL1			0x0C
#define MFSEL3			0x64
#define MFSEL3_PCIEPUSE_BIT	BIT(17)

#define INTCR3			0x9c
#define INTCR3_RCCORER_BIT	BIT(22)

#define I2CSEGSEL		0xE0

#define IPSRST3			0x34
#define IPSRST3_PCIERC_BIT	BIT(15)

#define LINK_UP_FIELD		(0x3F << 20)
#define LINK_RETRAIN_BIT	BIT(27)

#define RC_TO_EP_DELAY		15

/* Types & Variable */  
void __iomem *npcm7xx_pcie_rc_config_area_virtual_address;

static struct resource pcie_rc_io;

#ifdef CONFIG_PCI_MSI 

/* Number of MSI IRQs */
#define NPCM750_NUM_MSI_IRQS		32

#define NPCM750_MAX_NUM_RESOURCES	1

struct npcm750_pcie_port {
	void __iomem *reg_base;
	u32 irq;
	unsigned long msi_pages;
	u8 root_busno;
	struct device *dev;
	struct irq_domain *irq_domain;
	struct resource bus_range;
	struct list_head resources;
	struct regmap *rst_regmap;
	struct regmap *gcr_regmap;
	unsigned rst_ep_gpio;
	unsigned rst_rc_gpio;
};

static DECLARE_BITMAP(msi_irq_in_use, NPCM750_NUM_MSI_IRQS);

static inline struct npcm750_pcie_port *sys_to_pcie(struct pci_sys_data *sys)
{
	return sys->private_data;
}

static u32 bar0;

/* extern functions */
extern void npcm7xx_spin_lock_irqsave(unsigned long *flags);
extern void npcm7xx_spin_unlock_irqrestore(unsigned long flags);

/*
 * npcm750_pcie_destroy_msi - Free MSI number
 * @irq: IRQ to be freed
 */
static void npcm750_pcie_destroy_msi(unsigned int irq)
{
	struct msi_desc *msi;
	struct npcm750_pcie_port *port;
	struct irq_data *d = irq_get_irq_data(irq);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

		
	if (!test_bit(hwirq, msi_irq_in_use)){
		msi = irq_get_msi_desc(irq);		
		port = msi_desc_to_pci_sysdata(msi);
		dev_err(port->dev, "Trying to free unused MSI#%d\n", irq);
	} else {
		clear_bit(hwirq, msi_irq_in_use);
	}
}

/*
 * npcm750_pcie_assign_msi - Allocate MSI number
 * @port: PCIe port structure
 *
 * Return: A valid IRQ on success and error value on failure.
 */
static int npcm750_pcie_assign_msi(void)
{
	int pos;

	pos = find_first_zero_bit(msi_irq_in_use, NPCM750_NUM_MSI_IRQS);
	if (pos < NPCM750_NUM_MSI_IRQS)
		set_bit(pos, msi_irq_in_use);
	else
		return -ENOSPC;

	return pos;
}

static irqreturn_t npcm7xx_msi_handler(int irq, void* dev_id)
{
	struct npcm750_pcie_port *port = (struct npcm750_pcie_port *)dev_id;
	unsigned int dt_irq;
	unsigned int index;
	unsigned long status;
	
	u32 global_pcie_status = ioread32(port->reg_base + PCIERC_ISTATUS_LOCAL_ADDR );
	if( global_pcie_status & PCIERC_ISTATUS_LOCAL_MSI_BIT )
	{

		status = ioread32(port->reg_base + PCIERC_ISTATUS_MSI_ADDR);
		if (!status)
			return IRQ_HANDLED;
		do {
			index = find_first_bit(&status, 32);
			iowrite32(1 << index, port->reg_base +
				  PCIERC_ISTATUS_MSI_ADDR);
	
			dt_irq = irq_find_mapping(port->irq_domain, index);
			if (test_bit(index, msi_irq_in_use))
				generic_handle_irq(dt_irq);
			else
				dev_info(port->dev, "unhandled MSI\n");
			
			status = ioread32(port->reg_base +
					  PCIERC_ISTATUS_MSI_ADDR);
			
		} while (status);
	}
	iowrite32(global_pcie_status,
		  port->reg_base + PCIERC_ISTATUS_LOCAL_ADDR);

	return IRQ_HANDLED;
}


/* 
 * function : set_translation_window
 * type - 0-memory
 * size - should be 2^n bytes
 */
static int set_translation_window(void __iomem * config_address_base,dma_addr_t *source_addr,
		u32 size,dma_addr_t *dest_addr,u8 win_type,u8 target)
{
	u8 win_size = 11 ;
	u32 val;

	if (size < 4096)
		printk("%s : ERROR : window size should be greater then 4KB\n " , __FUNCTION__);

	size=(size>>(win_size+2));
	while(size) {
		size= (size >> 1);
		win_size++;
	}

#ifdef __LP64__
	writel( ((uint64_t)source_addr & 0xffffffff) +
			(win_size << PCI_RC_ATTR_WIN_SIZE_POS)+( 1<< PCI_RC_ATTR_WIN_EN_POS) , config_address_base);

	writel( ((uint64_t)source_addr >> 32 ) & 0xffffffff ,config_address_base + 0x4 );
	writel(((uint64_t)dest_addr & 0xffffffff) , config_address_base + 0x8);
	writel( ((uint64_t)dest_addr >> 32 ) & 0xffffffff , config_address_base + 0xc );
#else
	writel( ((u32)source_addr  ) +
			(win_size << PCI_RC_ATTR_WIN_SIZE_POS)+( 1<< PCI_RC_ATTR_WIN_EN_POS) , config_address_base);


	writel( 0 ,config_address_base + 0x4 );
	writel( (u32)dest_addr  , config_address_base + 0x8);
	writel( 0, config_address_base + 0xc );
#endif

	val=(win_type << PCI_RC_ATTR_TRSF_PARAM_POS) + (target << PCI_RC_ATTR_TRSL_ID_POS);
	writel( val , config_address_base + 0x10);
	return 0;
}

static struct irq_chip npcm7xx_msi_chip = {
	.name = "NPCM750 PCI-MSI",
	.irq_enable = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

int npcm750_pcie_msi_setup_irq(struct msi_controller *chip,
				     struct pci_dev *pdev,
				     struct msi_desc *desc)
{
	struct npcm750_pcie_port *port = sys_to_pcie(pdev->bus->sysdata);
	unsigned int irq,id;
	int hwirq;
	struct msi_msg msg;

	hwirq = npcm750_pcie_assign_msi();
	if (hwirq < 0)
		return hwirq;
	
	irq = irq_create_mapping(port->irq_domain, hwirq);
	if (!irq)
		return -EINVAL;

	irq_set_msi_desc(irq, desc);
	
	pr_info("npcm750_pcie_msi_setup_irq hwirq %d irq %d \n",(int)hwirq,irq);

	msg.address_hi = 0x0;
	msg.address_lo = (bar0 + IMSI_ADDR) & 0xfffffff0;

	id = read_cpuid_id();
	msg.data = (id << NPCM7XX_CORE_SELECT) | hwirq;

	pci_write_msi_msg(irq, &msg);

	irq_set_chip_and_handler(irq, &npcm7xx_msi_chip, handle_simple_irq);

	return 0;
}

void npcm750_msi_teardown_irq(struct msi_controller *chip,
				    unsigned int irq)
{
	pr_debug("%s : irq = %d \n " , __FUNCTION__ , irq);
	npcm750_pcie_destroy_msi(irq);
}

/* MSI Chip Descriptor */
static struct msi_controller npcm750_pcie_msi_chip = {
	.setup_irq = npcm750_pcie_msi_setup_irq,
	.teardown_irq = npcm750_msi_teardown_irq,
};

/*
 * npcm750_pcie_msi_map - Set the handler for the MSI and mark IRQ as valid
 * @domain: IRQ domain
 * @irq: Virtual IRQ number
 * @hwirq: HW interrupt number
 *
 * Return: Always returns 0.
 */
static int npcm750_pcie_msi_map(struct irq_domain *domain, unsigned int irq,
			       irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &npcm7xx_msi_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

/* IRQ Domain operations */
static const struct irq_domain_ops msi_domain_ops = {
	.map = npcm750_pcie_msi_map,
};

/*
 * npcm750_pcie_free_irq_domain - Free IRQ domain
 * @port: PCIe port information
 */
static void npcm750_pcie_free_irq_domain(struct npcm750_pcie_port *port)
{
	int i;
	u32 irq, num_irqs;

	/* Free IRQ Domain */
	if (IS_ENABLED(CONFIG_PCI_MSI)) {

		num_irqs = NPCM750_NUM_MSI_IRQS;
	} 

	for (i = 0; i < num_irqs; i++) {
		irq = irq_find_mapping(port->irq_domain, i);
		if (irq > 0)
			irq_dispose_mapping(irq);
	}

	irq_domain_remove(port->irq_domain);
}

/*
 * npcm750_pcie_init_irq_domain - Initialize IRQ domain
 * @port: PCIe port information
 *
 * Return: '0' on success and error value on failure
 */
static int npcm750_pcie_init_irq_domain(struct npcm750_pcie_port *port)
{
	struct device *dev = port->dev;
	struct device_node *node = dev->of_node;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		port->irq_domain = irq_domain_add_linear(node,
							 NPCM750_NUM_MSI_IRQS,
							 &msi_domain_ops,
							 &npcm750_pcie_msi_chip);
		if (!port->irq_domain) {
			dev_err(dev, "Failed to get a MSI IRQ domain\n");
			return PTR_ERR(port->irq_domain);
		}
	}

	return 0;
}

static int npcm7xx_msi_init(struct npcm750_pcie_port *port)
{
	struct device *dev = port->dev;
	int ret = 0;
	struct device_node *node = dev->of_node;
	int err;

	iowrite32(PCIERC_ISTATUS_LOCAL_MSI_BIT, port->reg_base + PCIERC_IMASK_LOCAL_ADDR);/* enable interrupts  */
	set_translation_window((void __iomem *)port->reg_base + RCPA0SAL  ,  0, SZ_512M ,
	                     0 , PLDA_XPRESS_RICH_MEMORY_WINDOW , PLDA_XPRESS_RICH_TARGET_AXI_MASTER) ;

	pcie_rc_irq = irq_of_parse_and_map(node, 0);
	if (!pcie_rc_irq) {
		printk(KERN_ERR "%s - failed to map irq\n", __FUNCTION__);
		ret= -1;
		goto done;
	}

	port->irq = pcie_rc_irq;
	err = devm_request_irq(dev, port->irq, npcm7xx_msi_handler,
			       IRQF_SHARED | IRQF_NO_THREAD, "npcm750-pcie",
			       port);
	if (err) {
		dev_err(dev, "unable to request irq %d\n", port->irq);
		return err;
	}
	
	err = npcm750_pcie_init_irq_domain(port);
	if (err) {
		dev_err(dev, "Failed creating IRQ Domain\n");
		return err;
	}

done:
	return ret;
}

static void npcm7xx_msi_update_rc_bar0(u32 bar, struct npcm750_pcie_port *port)
{
	u32 val;
	bar0 = bar;

	__raw_writel(0x1F0000 , (void __iomem *) port->reg_base + RCCFGNUM);
	val = __raw_readl(CONFIG_WIN_V_BASE + PCI_CMD_STATUS_REG);
	val |= (PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);
	__raw_writel(val , CONFIG_WIN_V_BASE + PCI_CMD_STATUS_REG);
	pr_debug( "%s , bar = 0x%x   \n",__FUNCTION__,bar);

}
#endif // CONFIG_PCI_MSI

/* Functions */  
static void npcm7xx_gpio_reset(struct npcm750_pcie_port *port, int reset_clear)
{
	if (reset_clear){
		gpio_set_value(port->rst_rc_gpio, 1);
		/* 
		 * There is a timeout after which the LTSSM 
		 * is moving from DETECT.QUIET to DETECT.ACTIVE
		 */
		msleep(RC_TO_EP_DELAY);
		gpio_set_value(port->rst_ep_gpio, 1);
	}
	else {
		gpio_set_value(port->rst_ep_gpio, 0);		
		gpio_set_value(port->rst_rc_gpio, 0);
	}
}

/*
 * Function:        npcm7xx_pcie_rc_device_connected.                                                   
 * Description:     Check is there is a device on the other side of the bridge.                                                                                    
 * Returns:         1 - device exist (link connection)                                                  
 *                  0 - no device    (no connection)                                                    
 */
static int npcm7xx_pcie_rc_device_connected(struct npcm750_pcie_port *port)
{
	u32 val;

	/*
	 * Check the Link status register on the bridge 
	 * configuration space at:
	 * Bus 0, Device 0, function 0 offset 0x92 bit 4
	 */   
	__raw_writel(0x1F0000 , port->reg_base + RCCFGNUM);
	val = __raw_readl(CONFIG_WIN_V_BASE + 0x90);
	return  ((val & LINK_UP_FIELD) >> 20); 
}

static int npcm7xx_pcie_rc_config_read(struct pci_bus *bus, 
                                     unsigned int devfn, 
                                     int where, 
                                     int size, 
                                     u32 *value)
{
	struct npcm750_pcie_port *port = sys_to_pcie(bus->sysdata);

	if ((bus->number > 0) && (npcm7xx_pcie_rc_device_connected(port) == 0)) {
		 printk(KERN_INFO "npcm7xx_pcie_rc_config_read - NO LINK\n");        
		 *value = 0xFFFFFFFF;
		 return PCIBIOS_SUCCESSFUL;
	}

	if ((bus->number == 0) && (devfn == 0) && ((where & 0xFFFFFFFC) == 8))
		*value = 0x6040001;
	else {
		__raw_writel(0x1F0000 | (((unsigned int)(bus->number)) << 8) | (devfn & 0xFF), port->reg_base + RCCFGNUM);
		*value = __raw_readl(CONFIG_WIN_V_BASE + (where & 0xFFFFFFFC));
	}
	 
	if (size == 1)
		*value = (*value >> (8 * (where & 3))) & 0xff;
	else if (size == 2)
		*value = (*value >> (8 * (where & 3))) & 0xffff;

	PCIE_DBG(KERN_DEBUG_PCIE "npcm7xx_pcie_rc_config_read (b:%d,df:%d,w:%x, v:%x)\n",(int)(bus->number),devfn,where, *value);        

	return PCIBIOS_SUCCESSFUL;
}

static int npcm7xx_pcie_rc_config_write(struct pci_bus *bus, unsigned int devfn, 
					int where, int size, u32 value)
{

	struct npcm750_pcie_port *port = sys_to_pcie(bus->sysdata);
	u32 org_val, new_val;    
	int ret = PCIBIOS_SUCCESSFUL;

	PCIE_DBG(KERN_DEBUG_PCIE "npcm7xx_pcie_rc_config_write (b:%d,df:%d,w:%x,s:%d v:%x) - Start.\n", (int)(bus->number),devfn,where, size, value);

	if ((bus->number > 0) && (npcm7xx_pcie_rc_device_connected(port) == 0)) {
		printk(KERN_DEBUG_PCIE "npcm7xx_pcie_rc_config_write - NO LINK\n");        
		return ret;
	}

#ifdef CONFIG_PCI_MSI
	if((bus->number == 0) && (0 == devfn) && (0x10 == where))
		npcm7xx_msi_update_rc_bar0(value, port);

#endif    
	__raw_writel(0x1F0000 | (((unsigned int)(bus->number)) << 8) |
		     (devfn & 0xFF), port->reg_base + RCCFGNUM);

	if (size == 4) 
		__raw_writel(value, CONFIG_WIN_V_BASE + (where & 0xFFFFFFFC));
	else if (size == 2) {
		org_val = __raw_readl(CONFIG_WIN_V_BASE + (where & 0xFFFFFFFC));
		value   = (value & 0x0000FFFF) << ((where & 0x3) * 8);
		new_val = (org_val & ~(0x0000FFFF << ((where & 0x3) * 8))) | value;
		__raw_writel(new_val, CONFIG_WIN_V_BASE + (where & 0xFFFFFFFC));
	}
	else if (size == 1) {
		org_val = __raw_readl(CONFIG_WIN_V_BASE + (where & 0xFFFFFFFC));
		value   = (value & 0x000000FF) << ((where & 0x3) * 8);
		new_val = (org_val & ~(0x000000FF << ((where & 0x3) * 8))) | value;
		__raw_writel(new_val, CONFIG_WIN_V_BASE + (where & 0xFFFFFFFC));
	}
	else 
		ret = PCIBIOS_BAD_REGISTER_NUMBER;

	return ret;
}

static struct pci_ops npcm7xx_pcie_rc_ops = {
	.read	= npcm7xx_pcie_rc_config_read,
	.write	= npcm7xx_pcie_rc_config_write,
};

static void wait_for_L0_state(struct npcm750_pcie_port *port, u32 timeout)
{
	__raw_writel(0x1F0000 , port->reg_base + RCCFGNUM);

	/* Wait for link up and link retrain done as L0 State */         
	while ( ((LINK_UP_FIELD & __raw_readl(CONFIG_WIN_V_BASE + 0x90)) == 0) 
		|| ((LINK_RETRAIN_BIT & __raw_readl(CONFIG_WIN_V_BASE + 0x90)) 
		    != 0) ) {
		if (timeout == 0) {
			printk(KERN_ERR DRIVER_NAME " Not in L0 State !!!- Link Control Register and Status (Offset 10h) = 0x%x \n",__raw_readl(CONFIG_WIN_V_BASE + 0x90));     
			break;
		}
		timeout--;
		udelay(100);
	}
}

int npcm7xx_pcie_rc_scan_bus(int nr, struct pci_host_bridge *bridge)
{
	struct pci_sys_data *sys = pci_host_bridge_priv(bridge);
	struct npcm750_pcie_port *port = sys_to_pcie(sys);
	int ret;
	u32 val = 0;

	// wait 100 ms for EP to be ready
	msleep(100);

	PCIE_DBG(KERN_DEBUG_PCIE "npcm7xx_pcie_rc_scan_bus - Start.\n");

	__raw_writel(0x1F0000 , port->reg_base + RCCFGNUM);

	PCIE_DBG(KERN_DEBUG_PCIE "Link Capabilities Register (Offset 0Ch) = 0x%x \n", __raw_readl(CONFIG_WIN_V_BASE + 0x8C));        
	PCIE_DBG(KERN_DEBUG_PCIE "Link Control and Status Register (Offset 10h) = 0x%x \n", __raw_readl(CONFIG_WIN_V_BASE + 0x90));        
	     
	wait_for_L0_state(port, 400);

#ifdef DBG_DELAY
	{		
	    int i;
		printk("before retrain:\n");
		for(i=0; i<30; i++) {
			printk(" %d", i);
			msleep(1000);
		}
	}
#endif
	/* Set link Retraining bit */
	val = __raw_readw(CONFIG_WIN_V_BASE + 0x90);
	val |= 0x20;
	__raw_writew(val, CONFIG_WIN_V_BASE + 0x90);

	wait_for_L0_state(port, 400);       // timeout ~400*100usec

	PCIE_DBG(KERN_DEBUG_PCIE "Link Capabilities Register (Offset 0Ch) = 0x%x \n", __raw_readl(CONFIG_WIN_V_BASE + 0x8C));        
	PCIE_DBG(KERN_DEBUG_PCIE "Link Control and Status Register(Offset 10h) = 0x%x \n", __raw_readl(CONFIG_WIN_V_BASE + 0x90));        

	list_splice_init(&sys->resources, &bridge->windows);
	bridge->dev.parent = NULL;
	bridge->sysdata = sys;
	bridge->busnr = sys->busnr;
	bridge->ops = &npcm7xx_pcie_rc_ops;
	bridge->msi = &npcm750_pcie_msi_chip;

	ret = pci_scan_root_bus_bridge(bridge);

	PCIE_DBG(KERN_DEBUG_PCIE "npcm7xx_pcie_rc_scan_bus - End.\n");

	return ret;
}

static void npcm7xx_pcie_rc_init_config_window(struct npcm750_pcie_port *port)
{
	/* Translation address of 0 */
	writel(0, port->reg_base + RCAPnTAL(1));
	writel(0, port->reg_base + RCAPnTAH(1));

	/* TRSF_PARAM=1, TRSL_ID=1 */
	writel(TRSF_PARAM_CONFIG | TRSL_ID_PCIE_CONFIG,
	       port->reg_base + RCAPnTP(1));

	/* Src addr high to 0 */
	writel(0, port->reg_base + RCAPnSAH(1));
	writel((CONFIG_WIN_BASE & 0xFFFFF000) | (ATR_SIZE_4K << 1) | 0x01,
	       port->reg_base + RCAPnSAL(1));
}


static void npcm7xx_initialize_as_root_complex(struct npcm750_pcie_port *port)
{ 
	PCIE_DBG(KERN_DEBUG_PCIE "npcm7xx_initialize_as_root_complex - Start\n");  

	/* put RC core to reset (write 0 to enter reset and 1 to release) */
	regmap_write_bits(port->gcr_regmap, INTCR3, INTCR3_RCCORER_BIT, 0x0);
		
	/* put RC to reset (write 1 to enter reset and 0 to enable module) */
	regmap_write_bits(port->rst_regmap, INTCR3,
			  IPSRST3_PCIERC_BIT, IPSRST3_PCIERC_BIT);
		
	/* enable rootport access */
	regmap_write_bits(port->gcr_regmap, MFSEL3,
			  MFSEL3_PCIEPUSE_BIT, MFSEL3_PCIEPUSE_BIT);

	/* release RC from reset */
	regmap_write_bits(port->gcr_regmap, INTCR3,
			  INTCR3_RCCORER_BIT, INTCR3_RCCORER_BIT);
	       
	/* enable RC */
	regmap_write_bits(port->rst_regmap, IPSRST3, IPSRST3_PCIERC_BIT, 0x0);

	PCIE_DBG(KERN_DEBUG_PCIE "npcm7xx_initialize_as_root_complex - End\n");
}

static int __init npcm7xx_pcie_rc_setup(int nr, struct pci_sys_data *sys)
{
	struct npcm750_pcie_port *port = sys_to_pcie(sys);
	struct device *dev = port->dev;
	struct device_node *node = dev->of_node;
	struct of_pci_range range;
	struct of_pci_range_parser parser;
	struct resource *mem;
	int err = 1, mem_resno = 0;
	int ret;

	PCIE_DBG(KERN_DEBUG_PCIE "npcm7xx_pcie_rc_setup - Start (nr = %d).\n",nr);    

	if (nr > 0)
		return 0;

#ifdef DBG_DELAY
	{
	    int i;		
		printk("before RC enable:\n");
		for(i=0; i<30; i++) {
			printk(" %d", i);
			msleep(1000);
		}
	}
#endif

	ret = devm_gpio_request_one(dev, port->rst_rc_gpio, GPIOF_OUT_INIT_LOW,
			     "rst-rc-pci");
	if (ret < 0) {
		dev_err(dev,"failed to configure rst-rc-pci-gpio (%d)\n", ret);
		return ret;
	}
	ret = devm_gpio_request_one(dev, port->rst_ep_gpio, GPIOF_OUT_INIT_LOW,
				    "rst-ep-pci");
	if (ret == -EBUSY) {
		dev_info(dev, "reset ep gpio using reset rc gpio .\n");
	} else if (ret) {
		dev_err(dev, "%d unable to get reset ep.\n", ret);
		return ret;
	}

	/* 
	 * Set GPIO for Endpoint reset release on DRB/SVB,
	 * Done before everything.
	 */ 
	npcm7xx_gpio_reset(port, 0);
	/* 
	 * This function might not be needed
	 * in case this system initialzations 
	 * will be done in a different place
	 */ 
	npcm7xx_initialize_as_root_complex(port); 
#ifdef DBG_DELAY		
	{
	    int i;	    
		printk("After RC enable before window config:\n");
		for(i=0; i<30; i++)
			printk(" %d", i);	
	}
#endif

	npcm7xx_pcie_rc_init_config_window(port);

	pcie_rc_io.name   = "PCI IO space";
	pcie_rc_io.start  = IO_WIN_BASE;
	pcie_rc_io.end    = pcie_rc_io.start + (IO_WIN_SIZE - 1);
	pcie_rc_io.flags  = IORESOURCE_IO;

	request_resource(&ioport_resource, &pcie_rc_io);
	pci_add_resource_offset(&sys->resources, &pcie_rc_io, sys->io_offset);
	
	/* Config I/O window */
	writel(pcie_rc_io.start, port->reg_base + RCAPnTAL(3));
	writel(0, port->reg_base + RCAPnTAH(3));

	writel(TRSF_PARAM_IO| TRSL_ID_PCIE_CONFIG, port->reg_base + RCAPnTP(3));

	writel(0, port->reg_base + RCAPnSAH(3));
	writel((pcie_rc_io.start & 0xFFFFF000) | (ATR_SIZE_1M << 1) | 0x01,
	       port->reg_base + RCAPnSAL(3));

	if (of_pci_range_parser_init(&parser, node)) {
		dev_err(dev, "missing \"ranges\" property\n");
		return -EINVAL;
	}
	
	/* Parse the ranges and add the resources found to the list */
	for_each_of_pci_range(&parser, &range) {

		if (mem_resno >= NPCM750_MAX_NUM_RESOURCES) {
			dev_err(dev, "Maximum memory resources exceeded\n");
			return -EINVAL;
		}

		mem = devm_kmalloc(dev, sizeof(*mem), GFP_KERNEL);
		if (!mem) {
			err = -ENOMEM;
			//goto free_resources;
		}

		of_pci_range_to_resource(&range, node, mem);

		switch (mem->flags & IORESOURCE_TYPE_BITS) {
			case IORESOURCE_MEM:
				request_resource(&iomem_resource, mem);
				pci_add_resource_offset(&sys->resources, mem, sys->mem_offset);
				
				/* Config memory window */
				writel(mem->start,
				       port->reg_base + RCAPnTAL(2));
				writel(0, port->reg_base + RCAPnTAH(2));
				writel(TRSF_PARAM_MEMORY| TRSL_ID_PCIE_TX_RX,
				       port->reg_base + RCAPnTP(2));
				writel(0, port->reg_base + RCAPnSAH(2));
				writel((mem->start & 0xFFFFF000) |
						   (ATR_SIZE_32M << 1) |
						   0x01,	
						   port->reg_base + RCAPnSAL(2));
				mem_resno++;
				break;
		default:
			err = -EINVAL;
			break;
		}

		if (err < 0) {
			dev_warn(dev, "Invalid resource found %pR\n", mem);
			continue;
		}

	}

	/* 
	 * Set Endpoint reset release on DRB/SVB,
	 * Done right after PCI_RC enable on GCR.
	 */
	npcm7xx_gpio_reset(port, 1);

	gpio_free(port->rst_ep_gpio);
	gpio_free(port->rst_rc_gpio);

#ifdef PCIE_DEBUG
	__raw_writel(0x1F0000 , port->reg_base + RCCFGNUM);
#endif
 
#ifdef CONFIG_PCI_MSI
	npcm7xx_msi_init(port);
#endif

	PCIE_DBG(KERN_DEBUG_PCIE "Link Capabilities Register (Offset 0Ch) = 0x%x \n", __raw_readl(CONFIG_WIN_V_BASE + 0x8C));        
	PCIE_DBG(KERN_DEBUG_PCIE "Link Control and Status Register(Offset 10h) = 0x%x \n", __raw_readl(CONFIG_WIN_V_BASE + 0x90));        

	PCIE_DBG(KERN_DEBUG_PCIE "npcm7xx_pcie_rc_setup - End.\n");    

	return err;
}

static int npcm750_pcie_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
        struct npcm750_pcie_port *port = sys_to_pcie(dev->bus->sysdata);
	int irq;

	irq = of_irq_parse_and_map_pci(dev, slot, pin);
	if (!irq)
		irq = port->irq;

	return irq;
}

static int npcm7xx_pcie_rc_probe(struct platform_device *pdev)
{    
	struct npcm750_pcie_port *port;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct device_node *np = dev->of_node;
	int error = 0;
	struct hw_pci hw;
	const char *type;
	int ret;
	
	PCIE_DBG(KERN_DEBUG_PCIE "npcm7xx_pcie_rc_probe - Start.\n");    

	port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;
	
	port->dev = dev;
	
	type = of_get_property(np, "device_type", NULL);
	if (!type || strcmp(type, "pci")) {
		dev_err(dev, "invalid \"device_type\" %s\n", type);
		return -EINVAL;
	}

 	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_ERR DRIVER_NAME " failed to get io memory region resouce.\n");
		error = -ENXIO;
		goto failed;
	}

	port->reg_base = ioremap(res->start, resource_size(res));
	if (port->reg_base == NULL) {
		printk(KERN_ERR DRIVER_NAME "failed to remap I/O memory\n");
		error = -ENXIO;
		goto failed;
	}

	port->rst_regmap = syscon_regmap_lookup_by_compatible("nuvoton,npcm750-rst");
	if (IS_ERR(port->rst_regmap)) {
		dev_err(&pdev->dev, "Failed to find nuvoton,npcm750-rst\n");
		return PTR_ERR(port->rst_regmap);
	}

	port->gcr_regmap = syscon_regmap_lookup_by_compatible("nuvoton,npcm750-gcr");
	if (IS_ERR(port->gcr_regmap)) {
		dev_err(&pdev->dev, "Failed to find nuvoton,npcm750-gcr\n");
		return PTR_ERR(port->gcr_regmap);
	}

	if (!request_mem_region(CONFIG_WIN_BASE, CONFIG_WIN_SIZE , pdev->name)) {
		printk(KERN_ERR DRIVER_NAME "failed to request config window area\n");
		error = -EBUSY;
		goto failed;
	}

	npcm7xx_pcie_rc_config_area_virtual_address = ioremap(CONFIG_WIN_BASE, CONFIG_WIN_SIZE);  
	if (npcm7xx_pcie_rc_config_area_virtual_address == NULL) {
		printk(KERN_ERR DRIVER_NAME "failed to map config window area\n");
		error = -ENXIO;
		goto failed1;
	}
	ret = of_get_named_gpio(np, "rst-rc-pci", 0);
	if (ret < 0) {
		dev_err(dev, "failed to get rst-rc-pci-gpio (%d)\n", ret);
		goto failed2;
	}
	port->rst_rc_gpio = ret;
	dev_dbg(dev, "rst_rc_gpio = %u\n", port->rst_rc_gpio);

	ret = of_get_named_gpio(np, "rst-ep-pci", 0);
	if (ret < 0) {
		dev_err(dev, "failed to get rst-ep-pci-gpio (%d)\n", ret);
		goto failed2;
	}
	port->rst_ep_gpio = ret;
	dev_dbg(dev, "rst_ep_gpio = %u\n", port->rst_ep_gpio);

	/* Register the device */
	memset(&hw, 0, sizeof(hw));
	hw = (struct hw_pci) {
		.nr_controllers	= 1,
		.private_data	= (void **)&port,
		.setup		= npcm7xx_pcie_rc_setup,
		.map_irq	= npcm750_pcie_map_irq,
		.scan		= npcm7xx_pcie_rc_scan_bus,
		.ops		= &npcm7xx_pcie_rc_ops,
	};

#ifdef CONFIG_PCI_MSI
	npcm750_pcie_msi_chip.dev = port->dev;
	hw.msi_ctrl = &npcm750_pcie_msi_chip;
#endif

	/* Init the device */
	pci_common_init_dev(dev,&hw);
	release_mem_region(IO_WIN_BASE, IO_WIN_SIZE);
	PCIE_DBG(KERN_DEBUG_PCIE "npcm7xx_pcie_rc_probe - End.\n");  

	return 0;
failed2:
	iounmap(npcm7xx_pcie_rc_config_area_virtual_address);
failed1:    
	release_mem_region(CONFIG_WIN_BASE, CONFIG_WIN_SIZE);
failed:
	return error;
}


static int npcm7xx_pcie_rc_remove(struct platform_device *pdev)
{
	struct npcm750_pcie_port *port = platform_get_drvdata(pdev);

	PCIE_DBG(KERN_DEBUG_PCIE "npcm7xx_pcie_rc_remove - Start.\n");        

	iounmap(npcm7xx_pcie_rc_config_area_virtual_address);
	release_mem_region(CONFIG_WIN_BASE, CONFIG_WIN_SIZE);

	npcm750_pcie_free_irq_domain(port);

	PCIE_DBG(KERN_DEBUG_PCIE "npcm7xx_pcie_rc_remove - End.\n");        

	return 0;
}

static struct of_device_id npcm750_pcie_of_match[] = {
	{ .compatible = "nuvoton,npcm750-pcirc", },
	{}
};

static struct platform_driver npcm7xx_pcie_rc_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = npcm750_pcie_of_match,
		.owner	= THIS_MODULE,
	},
	.probe		= npcm7xx_pcie_rc_probe,
	.remove		= npcm7xx_pcie_rc_remove,
};

module_platform_driver(npcm7xx_pcie_rc_driver);

MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_DESCRIPTION("NPCM7xx PCIe Root Complex driver");
MODULE_LICENSE("GPL");

