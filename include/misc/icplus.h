#ifndef _H_ICPLUS_IO
#define _H_ICPLUS_IO

#define MY_MACIG 'G'
#define READ_IOCTL _IOR(MY_MACIG, 0, int)
#define WRITE_IOCTL _IOW(MY_MACIG, 1, int)
#define CONFIG_VLAN _IOW(MY_MACIG, 2, int)
#define MAP_PORT_TO_VLAN _IOW(MY_MACIG, 3, int)
#define MAP_IO_PLATFORM _IOW(MY_MACIG,4,int)

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;


// General shit bit macro
#define BIT_N(n)  (1<<(n))
#define PORT_N(n)  BIT_N(n)
#define PORT_0	(1<<0)
#define PORT_1 	(1<<1)
#define PORT_2	(1<<2)
#define PORT_3	(1<<3)
#define	PORT_4	(1<<4)

// Enable/Disable bit on byte
#define DISABLE_BIT(a,b)	((a) = (a) & ~PORT_N(b))
#define ENABLE_BIT(a,b) 	((a) = (a) | PORT_N(b))

/* Define VLAN Classification
 *	-	Use VID to classify VLAN
 *	-	Use PVID to classify VLAN
 */
#define USE_VID(a,p)	DISABLE_BIT(a,p)
#define USE_PVID(a,p) 	ENABLE_BIT(a,p)

/* Define VLAN Mode.
 * 	-	Port based
 * 	-	Tagged based
 */
#define VM_PORT_VLAN(a,p) DISABLE_BIT(a,p)
#define VM_TAGGED_VLAN(a,p) ENABLE_BIT(a,p)

typedef enum 
{
	UNKVID_DISCARD = 0,
	UNKVID_FORDWARD_TO_CPU,
	UNKVID_FLOOD_PACKET,
	UNKVID_RESERVED,
}e_unkvid_mode;

typedef struct
{
	u8 clear;
	e_unkvid_mode unkvid_mode;
	u8 vlan_classification;
	u8 vlan_mode;
	/* data */
}s_vlan_config;

typedef struct
{
    u8 port;
    u8 pvid;
    u8 vid;
    
}s_portmap;



#endif
