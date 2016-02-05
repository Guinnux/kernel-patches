/*
 * Driver for ICPlus IP175D PHY
 *
 * Copyright (c) 2010 Keystone Electronic Solutions
 * Copyright (c) 2007 Freescale Semiconductor, Inc.
 * 
 * Derived from icplus.c - Michael Barkowski
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/of.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include <misc/icplus.h>

MODULE_DESCRIPTION("ICPlus IP175D PHY driver");
MODULE_AUTHOR("Willie Victor");
MODULE_LICENSE("GPL");

/* Valid for PHY 0-4 */
#define IP175D_REG_ST 			1		/* Status Register */
#define IP175D_REG_ST_ANEG		(1<<5)	/* Autonegotiation: 1=Complete, 0=In Progress */
#define IP175D_REG_ST_LINK		(1<<2)	/* Link Status: 1=Link Pass, 0=Link Fail */

#define IP175D_REG_SS			18		/* Special Status Register */
#define IP175D_REG_SS_SPD		(1<<11)	/* Speed: 1=100Mbps, 0=10Mbps */
#define IP175D_REG_SS_DPX		(1<<10) /* Duplex: 1=FDX, 0=HDX */

#define IP175D_PHY_SWCON		20		/* Switch Control Registers */
#define IP175D_REG_CHIPID		0		/* Chip Identification (def: 0x175D) */
#define IP175D_REG_SOFT_RST		2		/* Software Reset Register */

#define IP175D_PHY_VLCON		22		/* VLAN Group Control Registers */
#define IP175D_PHY_VLCON2		23
#define IP175D_REG_VLCLASS		0		/* VLAN Classification Register */
#define IP175D_REG_VLCLASS_P0	(1<<6)	/* Per-port classification */
#define IP175D_REG_VLCLASS_P1	(1<<7)	/*     (tag-based only)    */
#define IP175D_REG_VLCLASS_P2	(1<<8)	/* 0*: Use VID if tagged,   */
#define IP175D_REG_VLCLASS_P3	(1<<9)	/*    or PVID if untagged  */
#define IP175D_REG_VLCLASS_P4	(1<<10)	/*                         */
#define IP175D_REG_VLCLASS_P5	(1<<11)	/* 1: Use only PVID        */

#define IP175D_REG_VLCLASS_M0	(1<<0)	/* Per-port mode setting */
#define IP175D_REG_VLCLASS_M1	(1<<1)	/*                       */
#define IP175D_REG_VLCLASS_M2	(1<<2)  /* 0*: Port based VLAN    */
#define IP175D_REG_VLCLASS_M3	(1<<3)	/*                       */
#define IP175D_REG_VLCLASS_M4	(1<<4)  /* 1: Tag based VLAN     */
#define IP175D_REG_VLCLASS_M5	(1<<5)  /*                       */

#define IP175D_REG_VLINGR		1		/* VLAN Ingress Rule */
#define IP175D_REG_VLEGR		2		/* VLAN Egress Rule */
#define IP175D_REG_VLEGR_KEEP0	(1<<0)	/* Keep VLAN Tag Header */			
#define IP175D_REG_VLEGR_KEEP1	(1<<1)	/* (per-port)			*/
#define IP175D_REG_VLEGR_KEEP2	(1<<2)	/* 0*: Disabled			*/
#define IP175D_REG_VLEGR_KEEP3	(1<<3)	/* 1: Keep VLAN tag header */
#define IP175D_REG_VLEGR_KEEP4	(1<<4)	/*    from frame.		*/
#define IP175D_REG_VLEGR_KEEP5	(1<<5)	/*						*/

/* VLAN Identifiers (bits [11:0])       *
 * Runs from 14 (VLAN_0) to 29 (VLAN_F) */
#define IP175D_REG_VID0			14
#define IP175D_REG_VID1			15
#define IP175D_REG_VID2			16
#define IP175D_REG_VID3			17
#define IP175D_REG_VID4			18


/* MII Force Mode */
#define IP175D_PHY_FORCE		20
#define IP175D_REG_FORCE		4
#define IP175D_MAC5_FORCE_100	(1<<15)
#define IP175D_MAC5_FORCE_FULL	(1<<13)
#define IP175D_MAC4_FORCE_100	(1<<14)
#define IP175D_MAC4_FORCE_FULL	(1<<12)

/* MII Status Report */
#define IP175D_PHY_MIISTAT		21
#define IP175D_REG_MIISTAT		0
#define IP175D_MII0_FULL		(1<<7)
#define IP175D_MII0_SPEED10		(1<<6)
#define IP175D_MII0_FLOW		(1<<5)


static void print_phy_details(struct phy_device *phydev,int phy_addr)
{
    int phy_data;
    printk("PHY%d details:\n",phy_addr);
    phy_data = mdiobus_read(phydev->bus,phy_addr,0x00);

    printk("Test 0x%04X value %04X\n",(phy_data & (1<<14)),(1<<14));
    if ( (phy_data & (1<<14)) == (1<<14) )
    {
        printk("PHY Loopback enabled...disabling it\n");
        mdiobus_write(phydev->bus,phy_addr,0x00, (phy_data & ~(1<<14)));
    }
    printk("Register 0: %04X\n",phy_data);
    if (phy_addr==4)
    {
        //mdiobus_write(phydev->bus,phy_addr,0x00,(phy_data | 0x0800));
        phy_data = mdiobus_read(phydev->bus,phy_addr,0x00);
        printk("Register 0 on PHY4 now: %04X\n",phy_data);

        

    }

    phy_data = mdiobus_read(phydev->bus,phy_addr,0x01);
    printk("Status: %04X\n",phy_data);
    phy_data = mdiobus_read(phydev->bus,phy_addr,0x04);
    printk("MII Register 4: %04X\n",phy_data);
    
}

static int ip175d_probe(struct phy_device *phydev)
{
    printk("Probing for 175d address: %d\n",phydev->addr);
    if (phydev->addr==4)
    {

		u16 reg_data = IP175D_MAC5_FORCE_100 | IP175D_MAC5_FORCE_FULL;
    	int err = mdiobus_write(phydev->bus, IP175D_PHY_SWCON, IP175D_REG_FORCE, reg_data);
    	printk("Disable autonegotiate on PHY5\n");

    	if(err < 0)
			return err;

		mdiobus_write(phydev->bus, phydev->addr, MII_BMCR, 0x2000);

        phydev->state = PHY_READY;
		phydev->speed = SPEED_100;
		phydev->duplex = DUPLEX_FULL;
		phydev->link = 1;

        reg_data = mdiobus_read(phydev->bus, phydev->addr, MII_BMCR);
        printk("PHY5 details 0x%04X\n",reg_data);

    }
    return 0;
}

static struct mii_bus * m_mii_bus = NULL;

static int ip175d_config_init_simple(struct phy_device *phydev)
{
	int err, i;
	static int full_reset_performed = 0;
	u16 reg_data;

    printk("Name %s, state %d, interface %d\n",phydev->drv->name,(int)phydev->state,(int)phydev->interface);
	if (full_reset_performed == 0) {
        m_mii_bus = phydev->bus;

		/* master reset */
        printk("Resetting master bus...\n");
		err = mdiobus_write(phydev->bus, IP175D_PHY_SWCON, IP175D_REG_SOFT_RST, 0x175D);
		if (err < 0)
        {
            printk("Could not reset master bus %d\n",err);
			return err;
        }

		/* ensure no bus delays overlap reset period */
		err = mdiobus_read(phydev->bus, IP175D_PHY_SWCON, IP175D_REG_SOFT_RST);
		/* data sheet specifies reset period is 2 msec */
		mdelay(2);

		/* Force mode for MAC5 (connected to CPU) */
		reg_data = IP175D_MAC5_FORCE_100 | IP175D_MAC5_FORCE_FULL;
		err = mdiobus_write(phydev->bus, IP175D_PHY_SWCON, IP175D_REG_FORCE, reg_data);
		if(err < 0)
			return err;
		mdelay(10);

		mdiobus_write(phydev->bus, 22, 0, 0x9000);

        /* reset switch ports */
		for (i = 0; i < 5; i++) {
			err = mdiobus_write(phydev->bus, i,
						 MII_BMCR, BMCR_RESET);
			if (err < 0)
				return err;
		}

		for (i = 0; i < 5; i++)
			err = mdiobus_read(phydev->bus, i, MII_BMCR);

    }
	if (phydev->addr == 4) {
		phydev->state = PHY_RUNNING;
		phydev->speed = SPEED_100;
		phydev->duplex = DUPLEX_FULL;
		phydev->link = 1;
		netif_carrier_on(phydev->attached_dev);
	}

    return 0;
}
static int ip175d_config_init(struct phy_device *phydev)
{
	int err, i;
	static int full_reset_performed = 0;
	u16 reg_data;
	struct device_node *np;
	int vlan_setup = 0;
	int count;

	if (full_reset_performed == 0) {

		/* master reset */
        printk("Resetting master bus...\n");

      np = of_find_compatible_node(NULL,NULL,"icplus,ic175d");
      if(np)
      {
         const int * prop;
         int len;

         printk("Found my OF Node\n");
         count = of_get_child_count(np);
         prop = of_get_property(np,"vlan",&len);
         if(prop)
         {
            vlan_setup = ntohl(*prop);
         }


      }
      printk("VLAN Setup is %d\n",vlan_setup);


		err = mdiobus_write(phydev->bus, IP175D_PHY_SWCON, IP175D_REG_SOFT_RST, 0x175D);
		if (err < 0)
        {
            printk("Could not reset master bus %d\n",err);
			return err;
        }

		/* ensure no bus delays overlap reset period */
		err = mdiobus_read(phydev->bus, IP175D_PHY_SWCON, IP175D_REG_SOFT_RST);
		/* data sheet specifies reset period is 2 msec */
		mdelay(2);

		/* Force mode for MAC5 (connected to CPU) */
		reg_data = IP175D_MAC5_FORCE_100 | IP175D_MAC5_FORCE_FULL;
		err = mdiobus_write(phydev->bus, IP175D_PHY_SWCON, IP175D_REG_FORCE, reg_data);
		if(err < 0)
			return err;
		mdelay(10);
	
		if(vlan_setup)
		{
         /* set tag-based VLAN, drop packets that connot be classified */
         mdiobus_write(phydev->bus, 22, 0, 0x003f);

         /* drop packets with nonzero CFI, NULL VID as untagged, discard
          * VID=0xfff, enable ingress filters */
         mdiobus_write(phydev->bus, 22, 1, 0x0c3f);

         /* defaults */
         mdiobus_write(phydev->bus, 22, 2, 0x0000);

         /* PVID for ports 0, 1, 2, 3, 4 */
         mdiobus_write(phydev->bus, 22, 4, 0x0003);
         mdiobus_write(phydev->bus, 22, 5, 0x0004);
         mdiobus_write(phydev->bus, 22, 6, 0x0004);
         mdiobus_write(phydev->bus, 22, 7, 0x0004);
         mdiobus_write(phydev->bus, 22, 8, 0x0004);

         /* set VLAN VID values */
         err = mdiobus_write(phydev->bus, 22, 14, 0x0002); /* VLAN 0 */
         err = mdiobus_write(phydev->bus, 22, 15, 0x0003); /* VLAN 1 */
         err = mdiobus_write(phydev->bus, 22, 16, 0x0004); /* VLAN 2 */
         err = mdiobus_write(phydev->bus, 22, 17, 0x0005); /* VLAN 3 */
         err = mdiobus_write(phydev->bus, 22, 18, 0x0006); /* VLAN 4 */
         err = mdiobus_write(phydev->bus, 22, 19, 0x0007); /* VLAN 5 */

         /* set port membership*/
         reg_data = 0x00;		/* VLAN 0: */
         reg_data = (0x21 << 8);	/* VLAN 1: 5, 0 */
         err = mdiobus_write(phydev->bus, 23, 0, reg_data);
         reg_data = 0x3e; 		/* VLAN 2: 5, 4, 3, 2, 1 */
         err = mdiobus_write(phydev->bus, 23, 1, reg_data);

         /* port 5 (to CPU) should add tags to outgoing packets *
          * for all VLANs */
         reg_data = 0x20;		/* VLAN 0 */
         reg_data |= (0x20 << 8);/* VLAN 1 */
         err = mdiobus_write(phydev->bus, 23, 8, reg_data);
         reg_data = 0x20;		/* VLAN 2 */
         reg_data |= (0x20 << 8);/* VLAN 3 */
         err = mdiobus_write(phydev->bus, 23, 9, reg_data);
         reg_data = 0x20;		/* VLAN 4 */
         reg_data |= (0x20 << 8);/* VLAN 5 */
         err = mdiobus_write(phydev->bus, 23, 10, reg_data);


           /* ports 0-4 should strip tags from outgoing packets
          * for all VLANs */
         reg_data = 0x1f | (0x1f << 8);
         err = mdiobus_write(phydev->bus, 23, 16, reg_data); /* VLAN 0, 1 */
         err = mdiobus_write(phydev->bus, 23, 17, reg_data); /* VLAN 2, 3 */
         err = mdiobus_write(phydev->bus, 23, 18, reg_data); /* VLAN 4, 5 */

         /* filters for VLAN 0-5 valid */
         mdiobus_write(phydev->bus, 22, 10, 0x003e);
		}
		else
		{
		   /* Disable VLAN Goodies */
		   mdiobus_write(phydev->bus, 22, 0, 0x9000);

		}

		
		/* Force the LED setup regardless of the Strap Configuration */

		/* Disble MII2 */
		mdiobus_write(phydev->bus, 21, 3, 0x90f0);
		
		/* Set LED Mode */
      mdiobus_write(phydev->bus, 20, 24, 0x3);

		
		/* reset switch ports */
		for (i = 0; i < 5; i++) {
			err = mdiobus_write(phydev->bus, i,
						 MII_BMCR, BMCR_RESET);
			if (err < 0)
				return err;
		}

		for (i = 0; i < 5; i++)
			err = mdiobus_read(phydev->bus, i, MII_BMCR);

		mdelay(2);

		full_reset_performed = 1;
        printk("Full reset performed\n");
	}

    /* PHY 4 not connected to switch in this case*/
	if (phydev->addr != 4) {
		phydev->state = PHY_UP;
		phydev->speed = SPEED_UNKNOWN;
		phydev->duplex = DUPLEX_UNKNOWN;
		phydev->link = 0;
		phydev->irq = PHY_POLL;
		netif_carrier_on(phydev->attached_dev);
	}

	return 0;
}

static int ip175d_read_status(struct phy_device *phydev)
{
	
   phydev->state = PHY_RUNNING;
   phydev->speed = SPEED_100;
   phydev->duplex = DUPLEX_FULL;
   phydev->link = 1;
   phydev->irq = PHY_POLL;
	return 0;
}

static int ip175d_config_aneg(struct phy_device *phydev)
{

   phydev->state = PHY_RUNNING;
   phydev->speed = SPEED_100;
   phydev->duplex = DUPLEX_FULL;
   phydev->link = 1;
   phydev->irq = PHY_POLL;

   return 0;
}

static struct phy_driver ip175d_driver = 
{
    .phy_id         = 0x02430d80,
    .name           = "ICPlus IP175D",
    .phy_id_mask    = 0x0ffffff0,
    .features       = (PHY_BASIC_FEATURES)&(~(SUPPORTED_Autoneg)),
    .probe          = &ip175d_probe,
    .config_init    = &ip175d_config_init,
//    .config_init    = &ip175d_config_init_simple,
  .config_aneg    = &ip175d_config_aneg,
//    .config_aneg    = genphy_config_aneg,
  .read_status    = &ip175d_read_status,
//  .read_status    = genphy_read_status,
    .driver         = { .owner = THIS_MODULE, },
};

static int major; 
static char msg[200];

static ssize_t device_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset)
{
    return simple_read_from_buffer(buffer, length, offset, msg, 200);
}

static ssize_t device_write(struct file *filp, const char __user *buff, size_t len, loff_t *off)
{
    if (len > 199)
        return -EINVAL;
    copy_from_user(msg, buff, len);
    msg[len] = '\0';
    return len;
}

char buf[200];
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
int device_ioctl(struct inode *inode, struct file *filep, unsigned int cmd, unsigned long arg) 
#else
long device_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
#endif
{
    int len = 200;
    switch(cmd) 
    {
        case READ_IOCTL:    
            copy_to_user((char *)arg, buf, 200);
        break;

        case WRITE_IOCTL:
            copy_from_user(buf, (char *)arg, len);
        break;
        case CONFIG_VLAN:
        {
            u8 unvid_mode = 0;
            u16 vlan_setup = 0;
            s_vlan_config vlan_config;
            if (copy_from_user(&vlan_config,(s_vlan_config*)arg,sizeof(s_vlan_config)))
            {
                return -EACCES;   
            }
            printk("Configuring VLAN setup...\n");

            if (vlan_config.clear)
                vlan_setup |= BIT_N(15);

            switch (vlan_config.unkvid_mode)
            {
                case UNKVID_DISCARD:
                    unvid_mode = 0;
                break;  
                case UNKVID_FORDWARD_TO_CPU:
                    unvid_mode = 1;
                break;  
                case UNKVID_FLOOD_PACKET:
                    unvid_mode = 2;
                break;  
                case UNKVID_RESERVED:
                    unvid_mode = 3;
                break;  
                default:
                    unvid_mode = 0;
                break;
            }
            vlan_setup |= (unvid_mode << 12);

            // Add VLAN classification
            vlan_setup |= (vlan_config.vlan_classification << 6);
            // Add VLAN mode
            vlan_setup |= (vlan_config.vlan_mode << 0);
            // Struct 4 bytes port
            // 4 bytes lan
            printk("Configuring to 0x%04X\n",vlan_setup);
            mdiobus_read(m_mii_bus, 22, 0);

        }
        break;
        case MAP_PORT_TO_VLAN:        
        {
            s_portmap map;
            if (copy_from_user(&map,(s_portmap*)arg,sizeof(s_portmap)))
            {
                return -EACCES;
            }

        }
        break;
        case MAP_IO_PLATFORM:
        {
            int reg_data = 0x00;        /* VLAN 0: */
            int err = 0, i = 0;
            u32 reg;
            void __iomem *io;
            u32 temp;
            u32 reg_int_status;
            u32 reg_network_status;
            /* set tag-based VLAN, drop packets that connot be classified */
            mdiobus_write(m_mii_bus, 22, 0, 0x003f);

            /* drop packets with nonzero CFI, NULL VID as untagged, discard
             * VID=0xfff, enable ingress filters */
            mdiobus_write(m_mii_bus, 22, 1, 0x0c3f);

            /* defaults */
            mdiobus_write(m_mii_bus, 22, 2, 0x0000);

            /* PVID for ports 0, 1, 2, 3, 4 */
            mdiobus_write(m_mii_bus, 22, 4, 0x0003);
            mdiobus_write(m_mii_bus, 22, 5, 0x0004);
            mdiobus_write(m_mii_bus, 22, 6, 0x0004);
            mdiobus_write(m_mii_bus, 22, 7, 0x0004);
            mdiobus_write(m_mii_bus, 22, 8, 0x0004);

            /* set VLAN VID values */
            err = mdiobus_write(m_mii_bus, 22, 14, 0x0002); /* VLAN 0 */
            err = mdiobus_write(m_mii_bus, 22, 15, 0x0003); /* VLAN 1 */
            err = mdiobus_write(m_mii_bus, 22, 16, 0x0004); /* VLAN 2 */
            err = mdiobus_write(m_mii_bus, 22, 17, 0x0005); /* VLAN 3 */
            err = mdiobus_write(m_mii_bus, 22, 18, 0x0006); /* VLAN 4 */
            err = mdiobus_write(m_mii_bus, 22, 19, 0x0007); /* VLAN 5 */
            
            /* set port membership*/
            reg_data = (0x21 << 8); /* VLAN 1: 5, 0 */
            err = mdiobus_write(m_mii_bus, 23, 0, reg_data);
            reg_data = 0x3e;        /* VLAN 2: 5, 4, 3, 2, 1 */
            //~ reg_data |= (0x24 << 8);/* VLAN 3: 5, 2*/
            err = mdiobus_write(m_mii_bus, 23, 1, reg_data);
            //~ reg_data = 0x28;        /* VLAN 4: 5, 3 */
            //~ reg_data |= (0x30 << 8);/* VLAN 5: 5, 4*/
            //~ err = mdiobus_write(phydev->bus, 23, 2, reg_data);
            
            /* port 5 (to CPU) should add tags to outgoing packets *
             * for all VLANs */
            reg_data = 0x20;        /* VLAN 0 */
            reg_data |= (0x20 << 8);/* VLAN 1 */
            err = mdiobus_write(m_mii_bus, 23, 8, reg_data);
            reg_data = 0x20;        /* VLAN 2 */
            reg_data |= (0x20 << 8);/* VLAN 3 */
            err = mdiobus_write(m_mii_bus, 23, 9, reg_data);
            reg_data = 0x20;        /* VLAN 4 */
            reg_data |= (0x20 << 8);/* VLAN 5 */
            err = mdiobus_write(m_mii_bus, 23, 10, reg_data);
            
            reg = 0xFFFC4000;

            io = ioremap(reg, SZ_1K);
            //at91_set_gpio_input
            temp = __raw_readl(io + 0xC0);
            printk("User IO Register %d\n",temp);
            reg_int_status = __raw_readl(io + 0x24);
            printk("Interrupt status register %08X\n",reg_int_status);
            reg_network_status =  __raw_readl(io + 0x24);
            printk("Network status register %08X\n",reg_network_status);
            temp = ioread32(&reg);    
            printk("User IO Register %d\n",temp);
            iounmap(io);

            reg = 0xFFFFFC00;
            io = ioremap(reg,SZ_1K);
            temp = __raw_readl(io + 0x18);
            printk("PMC Clock status %08X\n",temp);
            iounmap(io);

            
            /* ports 0-4 should strip tags from outgoing packets
             * for all VLANs */
            reg_data = 0x1f | (0x1f << 8);  
            err = mdiobus_write(m_mii_bus, 23, 16, reg_data); /* VLAN 0, 1 */
            err = mdiobus_write(m_mii_bus, 23, 17, reg_data); /* VLAN 2, 3 */
            err = mdiobus_write(m_mii_bus, 23, 18, reg_data); /* VLAN 4, 5 */
            
            /* filters for VLAN 0-5 valid */
            mdiobus_write(m_mii_bus, 22, 10, 0x003e);

            /* reset switch ports */
            for (i = 0; i < 5; i++) {
                err = mdiobus_write(m_mii_bus, i,
                             MII_BMCR, BMCR_RESET);
                if (err < 0)
                    return err;
            }

            for (i = 0; i < 5; i++)
                err = mdiobus_read(m_mii_bus, i, MII_BMCR);

            mdelay(2);
            break;
        }
        default:
        return -ENOTTY;
    }
    return len;
}

static struct file_operations fops = 
{
    .read = device_read, 
    .write = device_write,

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
    .ioctl = device_ioctl,
#else
    .unlocked_ioctl = device_ioctl
#endif

};

static int __init ip175d_init(void)
{
    printk("Register IOCTL device...\n");
    major = register_chrdev(0,"icplus",&fops);
    if (major < 0)
    {
        printk("Registering of device failed %d\n",major);
    }
    else
    {
        printk("created ioctl device with major %d\n",major);
        printk("Craete node with: mknod /dev/tempdev c %d 0\n",major);
    }
    return phy_driver_register(&ip175d_driver);
}

static void __exit ip175d_exit(void)
{
    phy_driver_unregister(&ip175d_driver);
}

module_init(ip175d_init);
module_exit(ip175d_exit);

#if defined(CONFIG_OF)
static const struct of_device_id icplus_dt_ids[] = 
{
    { .compatible = "icplus,ic175d" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, icplus_dt_ids);
#endif

