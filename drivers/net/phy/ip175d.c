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



static int ip175d_probe(struct phy_device *phydev)
{
    printk("Probing for 175d address: %d\n",phydev->addr);
    if (phydev->addr==0)
    {

		u16 reg_data = IP175D_MAC5_FORCE_100 | IP175D_MAC5_FORCE_FULL;
    	int err = mdiobus_write(phydev->bus, IP175D_PHY_SWCON, IP175D_REG_FORCE, reg_data);
    	printk("Disable autonegotiate on PHY5\n");

    	if(err < 0)
			return err;

		mdiobus_write(phydev->bus, phydev->addr, MII_BMCR, 0x2000);

        phydev->state = PHY_READY;
        phydev->supported = PHY_BASIC_FEATURES;
        phydev->advertising = PHY_BASIC_FEATURES;

        reg_data = mdiobus_read(phydev->bus, phydev->addr, MII_BMCR);
        printk("PHY5 details 0x%04X\n",reg_data);

    }
    else if(phydev->addr<=4)
    {
        phydev->state = PHY_READY;
        phydev->supported = PHY_BASIC_FEATURES;
        phydev->advertising = PHY_BASIC_FEATURES;
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

	return 0;
}

static int ip175d_read_status(struct phy_device *phydev)
{
	
	uint16_t addr;
	int link = 0;
	for(addr = 0; addr <= 4; addr++)
   {
	  uint16_t phy_status = mdiobus_read(phydev->bus,addr,IP175D_REG_ST);
	  if(phy_status & IP175D_REG_ST_LINK)
	  {
		  link = 1;
		  if(!phydev->link)
		  {
			  printk("Link is OK ");
			  if(phy_status & (1<<14))
			  {
				  printk("100M/Full\n");
			  }
			  else if(phy_status & (1<<13))
			  {
				  printk("100M/Half\n");
			  }
			  else if(phy_status & (1<<12))
			  {
				  printk("10M/Full\n");
			  }
			  else if(phy_status & (1<<11))
			  {
				  printk("10M/Half\n");
			  }
			  else
			  {
				  printk("Link with Nothing WTF\n");
			  }
		  }
	  }
   }

     phydev->link = link;
     if(link)
     {
       phydev->speed = SPEED_100;
   	   phydev->duplex = DUPLEX_FULL;
       phydev->pause = 0;
		phydev->asym_pause = 0;
     }
	return 0;
}

static int ip175d_config_aneg(struct phy_device *phydev)
{
	return 0;
}

static int ip175d_aneg_done(struct phy_device *phydev)
{
	int aneg = 0;
	uint16_t addr;

   for(addr = 0; addr <= 4; addr++)
   {
	  uint16_t phy_status = mdiobus_read(phydev->bus,addr,IP175D_REG_ST);
	  if(phy_status & IP175D_REG_ST_ANEG)
	  {
		  aneg = BMSR_ANEGCOMPLETE;
	  }
   }
   return aneg;
}

static struct phy_driver ip175d_driver = 
{
    .phy_id         = 0x02430d80,
    .name           = "ICPlus IP175D",
    .phy_id_mask    = 0x0ffffff0,
    .features       = (PHY_BASIC_FEATURES),
    .probe          = &ip175d_probe,
    .config_init    = &ip175d_config_init,
    .config_aneg    = &ip175d_config_aneg,
	.aneg_done      = &ip175d_aneg_done,
    .read_status    = &ip175d_read_status,
    .driver         = { .owner = THIS_MODULE, },
};


static int __init ip175d_init(void)
{
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

