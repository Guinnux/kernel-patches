/*
 * gpio-gnxio.c
 *
 *  Created on: 16 Apr 2014
 *      Author: john
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <misc/gnxio.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <linux/moduleparam.h>

static char * gpiomap = "rsm";
module_param(gpiomap,charp,S_IRUGO);

enum e_io
{
	INPUT,
	OUTPUT,
};

enum e_level
{
	LOW=0,
	HIGH=1
};


struct gnx_io_dev
{
	struct cdev cdev;
	struct class *cl;
	struct s_pindef * gpio_mapping;
	int n_io;
	spinlock_t list_lock;
	struct list_head list;
};

struct s_pindef
{
	uint32_t pin;
	const char * pin_desc;
	enum e_io in_out;
	enum e_level default_level;
};

struct squishy_data
{
	struct list_head list;
	struct semaphore sem;
	uint32_t change_cnt;
	struct gnx_io_dev * dev;
};

static void gnx_io_init_squishy(struct squishy_data * sq,struct gnx_io_dev * dev)
{
	sema_init(&sq->sem,0);
	sq->change_cnt = 0;
	sq->dev = dev;
	spin_lock(&dev->list_lock);
	list_add(&sq->list,&dev->list);
	spin_unlock(&dev->list_lock);
}
static void gnx_io_release_squishy(struct squishy_data * sq,struct gnx_io_dev * dev)
{
	spin_lock(&dev->list_lock);
	list_del(&sq->list);
	spin_unlock(&dev->list_lock);
	kfree(sq);
}



static irqreturn_t gnx_io_interrupt_handler(int irq, void *dev_id)
{
	struct gnx_io_dev * dev = (struct gnx_io_dev *)dev_id;
	struct squishy_data * sq;

	pr_debug("gnxio: Got Interrupt\n");

	spin_lock(&dev->list_lock);

	/* Service squishy list */
	list_for_each_entry(sq,&dev->list,list)
	{
		sq->change_cnt++;
		up(&sq->sem);
	}

	spin_unlock(&dev->list_lock);
	return IRQ_HANDLED;
}


#if defined(CONFIG_OF_GPIO)
static void gnx_io_set_mapping(struct gnx_io_dev * dev)
{
	struct device_node *np,*child;
	int count,idx;
	pr_debug("Using OF mapping\n");

	np = of_find_compatible_node(NULL,NULL,"kses,gnxio");
	if(np)
	{
		pr_debug("Found my OF Node\n");
		count = of_get_child_count(np);
		pr_debug("Found %d children\n",count);
		idx = 0;
		dev->gpio_mapping = (struct s_pindef *)kmalloc((sizeof(struct s_pindef)*count),GFP_KERNEL);
		dev->n_io = count;
		for_each_child_of_node(np, child)
		{
        	const int * prop;
          int len;
          dev->gpio_mapping[idx].pin_desc = of_get_property(child, "label", NULL);
          dev->gpio_mapping[idx].pin = of_get_gpio(child,0);
          prop = of_get_property(child, "output", &len );
          if(prop)
          {
        	  dev->gpio_mapping[idx].in_out = ntohl(*prop);
          }
          else
        	 dev->gpio_mapping[idx].in_out = INPUT;

          prop = of_get_property(child, "def_state", &len);
          if(prop)
        	  dev->gpio_mapping[idx].default_level = ntohl(*prop);
          else
        	  dev->gpio_mapping[idx].default_level = LOW;

          pr_debug("Found PIN %s(%d) %s Default %s\n",dev->gpio_mapping[idx].pin_desc,dev->gpio_mapping[idx].pin,(dev->gpio_mapping[idx].in_out == INPUT)?"Input":"Output",(dev->gpio_mapping[idx].default_level == LOW)?"LOW":"HIGH");
          idx++;
		}

	}
	else
	{
     	pr_debug("OF node is missing\n");
    	dev->n_io = 0;
    	dev->gpio_mapping = NULL;
	}
}

#else
static void gnx_io_set_mapping(struct gnx_io_dev * dev)
{
	/* Check which mapping we need to load by using the insert parameter */

	if(!strcmp(gpiomap,"gnxio"))
	{
		dev->gpio_mapping = gnx_io_platform_gpio_mapping;
		dev->n_io = (sizeof(gnx_io_platform_gpio_mapping)/sizeof(struct s_pindef)) - 1;
		pr_debug ("  gnxio: mapped guinnux_io_platform (%d pins)\n",dev->n_io);
	}
	else if(!strcmp(gpiomap,"rsm"))
	{
		dev->gpio_mapping = rsm_controller_gpio_mapping;
		dev->n_io = (sizeof(rsm_controller_gpio_mapping)/sizeof(struct s_pindef)) - 1;

		pr_debug ("  gnxio: mapped rsm_controller (%d pins)\n",dev->n_io);
	}
	else
	{
		pr_debug ("  gnxio unknown map %s\n",gpiomap);
		dev->n_io = 0;
		dev->gpio_mapping = NULL;
	}

	//For now there is only one
}
#endif

static void gnx_io_init_mapping(struct gnx_io_dev * dev)
{
	int ctr;
	int res;
	for(ctr = 0;ctr<dev->n_io;ctr++)
	{
		pr_debug ("  gnxio: init pin %d %s as %d:%d",ctr,dev->gpio_mapping[ctr].pin_desc,dev->gpio_mapping[ctr].in_out,dev->gpio_mapping[ctr].default_level);
		gpio_request(dev->gpio_mapping[ctr].pin,dev->gpio_mapping[ctr].pin_desc);
		if(dev->gpio_mapping[ctr].in_out != INPUT)
		{
			pr_debug("OUTPUT\n");
			gpio_direction_output(dev->gpio_mapping[ctr].pin,(dev->gpio_mapping[ctr].default_level == LOW) ? 0 : 1);
		}
		else
		{
			pr_debug("INPUT\n");
			gpio_direction_input(dev->gpio_mapping[ctr].pin);
			res = request_irq(gpio_to_irq(dev->gpio_mapping[ctr].pin),gnx_io_interrupt_handler,IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,dev->gpio_mapping[ctr].pin_desc,dev);
			if(res)
			{
				pr_debug ("  gnxio: Could not registerinterrupt %s\n",dev->gpio_mapping[ctr].pin_desc );
				pr_debug ("  gnxio: Retrying....\n");
				free_irq(dev->gpio_mapping[ctr].pin,NULL);
				res = request_irq(gpio_to_irq(dev->gpio_mapping[ctr].pin),gnx_io_interrupt_handler,IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,dev->gpio_mapping[ctr].pin_desc,dev);
				if(res)
				{
					pr_debug("  gnxio: Really not working\n");
				}
				else
				{
					pr_debug("  gnxio: forced irg OK\n");
				}
			}
		}
		gpio_export(dev->gpio_mapping[ctr].pin,0);
	}
}

static void gnx_io_release_mapping(struct gnx_io_dev * dev)
{
	int ctr;
	for (ctr=0;ctr<dev->n_io;ctr++)
	{
		gpio_unexport(dev->gpio_mapping[ctr].pin);
		if(dev->gpio_mapping[ctr].in_out == INPUT)
		{
			pr_debug("  gnxio: Removing interrupt %s\n",dev->gpio_mapping[ctr].pin_desc);
			free_irq(gpio_to_irq(dev->gpio_mapping[ctr].pin),dev);
		}
		gpio_free(dev->gpio_mapping[ctr].pin);
	}
}


static void gnx_io_set_output(struct gnx_io_dev * dev, struct gnx_io_pin_status * pin_stat)
{
	int p_idx = pin_stat->pin_id;
	if((p_idx >= 0) && (p_idx < dev->n_io) && (dev->gpio_mapping[p_idx].in_out == OUTPUT ))
	{
		gpio_set_value(dev->gpio_mapping[p_idx].pin,(pin_stat->pin_level) ? 1: 0);
	}
}

static void gnx_io_get_value(struct gnx_io_dev * dev,struct gnx_io_pin_status * pin_stat,int p_idx)
{
	if(p_idx >= 0 && p_idx < dev->n_io )
	{
		pin_stat->pin_id = p_idx;
		pin_stat->pin_level = gpio_get_value(dev->gpio_mapping[p_idx].pin);
	}
	else
	{
		pin_stat->pin_id = -1;
		pin_stat->pin_level = -1;
	}
}


static loff_t gnx_io_llseek(struct file * filp, loff_t off,int whence)
{
	pr_debug (" Guinnux IO llseek\n");
    return -EINVAL;
}
static ssize_t gnx_io_read(struct file * filp, char __user * buf, size_t count, loff_t * pos)
{
	int d_len;
		int err;
		int num_pins;
		int pin_ctr;
		struct gnx_io_pin_status * stat_buff;
		struct squishy_data * sq = (struct squishy_data *)filp->private_data;
		struct gnx_io_dev * dev = sq->dev;

		if((filp->f_flags&O_NONBLOCK) == 0)
		{
			err = down_interruptible(&sq->sem);
		}

	    num_pins = count/sizeof(struct gnx_io_pin_status);
	    if(num_pins > dev->n_io)
	    {
	    	num_pins = dev->n_io;
	    }

	    sq->change_cnt = 0;

	    stat_buff = (struct gnx_io_pin_status *)kmalloc(num_pins*sizeof(struct gnx_io_pin_status),GFP_KERNEL);

	    for(pin_ctr = 0;pin_ctr < num_pins;pin_ctr++)
	    {
	    	gnx_io_get_value(dev,&stat_buff[pin_ctr],pin_ctr);
	    }

	    d_len = num_pins*sizeof(struct gnx_io_pin_status);

		err = copy_to_user(buf,stat_buff,d_len);
		if(err);
	    kfree(stat_buff);

	    return d_len;
}

static ssize_t gnx_io_write(struct file * filp, const char __user * buf, size_t count, loff_t * pos)
{
	int d_len;
    int err;
	int num_pins;
	int pin_ctr;
	struct gnx_io_pin_status * stat_buff;
	struct squishy_data * sq = (struct squishy_data *)filp->private_data;
	struct gnx_io_dev * dev = sq->dev;

	num_pins = count/sizeof(struct gnx_io_pin_status);

	stat_buff = (struct gnx_io_pin_status *)kmalloc(num_pins*sizeof(struct gnx_io_pin_status),GFP_KERNEL);

	d_len = num_pins*sizeof(struct gnx_io_pin_status);

	err = copy_from_user(stat_buff,buf,d_len);

    for(pin_ctr = 0;pin_ctr < num_pins;pin_ctr++)
	{
	  	gnx_io_set_output(dev,&stat_buff[pin_ctr]);
	}
    if(err);
    kfree(stat_buff);

    return d_len;

}

static unsigned int gnx_io_poll(struct file * filp, struct poll_table_struct *poll_table)
{
	unsigned int flags = (POLLOUT | POLLWRNORM); // You can always write
	struct squishy_data * sq = (struct squishy_data *)filp->private_data;

	pr_debug (" Guinnux IO Poll\n");
    if(sq->change_cnt)
	{
		flags |= (POLLIN | POLLRDNORM);
	}
    return flags;
}

static long gnx_io_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
	int type;
	int err;

	struct gnx_io_dev * dev;
	struct squishy_data * sq;
	sq = (struct squishy_data *)filp->private_data;
	dev = sq->dev;


	pr_debug (" Guinnux IO IOCTL\n");
	type = _IOC_TYPE(cmd);
	if(type == GNX_IOCTL_MAGIC)
	{
		pr_debug (" gnxio: Definitely one of mine\n");
	}
	else
	{
		pr_debug("  gnxio: Unknown type %d\n",type);
	}
	switch (cmd)
	{
	  case GNX_IO_IOCTLGNIO:
	  {
		  pr_debug("  gnxio: Requesting num IO\n");
		  return dev->n_io;
	  }
	  case GNX_IO_IOCTLGMAP:
	  {
		  int ctr;
		  struct gnx_io_pin_cfg * pin_cfg = (struct gnx_io_pin_cfg *)kmalloc(sizeof(struct gnx_io_pin_cfg)*dev->n_io,GFP_KERNEL);
		  pr_debug ("  gnxio: Requesting IO Map\n");
		  for(ctr=0;ctr<dev->n_io;ctr++)
		  {
			  pin_cfg[ctr].pin_id = ctr;
			  pin_cfg[ctr].pin_in_out = dev->gpio_mapping[ctr].in_out;
			  pin_cfg[ctr].def_level = dev->gpio_mapping[ctr].default_level;
			  strcpy(pin_cfg[ctr].pin_desc,dev->gpio_mapping[ctr].pin_desc);
		  }
		  err = copy_to_user((void __user *)arg,pin_cfg,sizeof(struct gnx_io_pin_cfg)*dev->n_io);
		  kfree(pin_cfg);
		  if(err)
		  {
			  return -ENOMEM;
		  }
		  return dev->n_io;
	  }
	  default:
		  break;
	}

	return -ENOTTY;

}
static int gnx_io_open(struct inode * inode, struct file * filp)
{

	struct gnx_io_dev * dev;
    struct squishy_data * sq;
	pr_debug (" Guinnux IO Open\n");

	dev = container_of(inode->i_cdev, struct gnx_io_dev, cdev);
	sq = (struct squishy_data *)kmalloc(sizeof(struct squishy_data),GFP_KERNEL);

	gnx_io_init_squishy(sq,dev);

	filp->private_data = sq;

	return 0;
}

static int gnx_io_release(struct inode * inode, struct file * filp)
{

   struct squishy_data * sq = (struct squishy_data *)filp->private_data;
   struct gnx_io_dev * dev = sq->dev;
   gnx_io_release_squishy(sq,dev);
   pr_debug (" Guinnux IO Release\n");
   return 0;

}


struct file_operations gnx_io_fops =
{
    .owner = THIS_MODULE,
    .llseek = gnx_io_llseek,
    .read = gnx_io_read,
    .write = gnx_io_write,
    .poll = gnx_io_poll,
    .unlocked_ioctl = gnx_io_ioctl,
    .open = gnx_io_open,
    .release = gnx_io_release
};

static dev_t first_store = 0;
struct gnx_io_dev * gnx_io_dev = NULL;


static int gnx_io_setup(void)
{
	int err = 0;
	dev_t first;
	err = alloc_chrdev_region(&first,0,1,"gnxio");
	if(!err)
	{
		first_store = first;

		gnx_io_dev = kmalloc(sizeof(struct gnx_io_dev),GFP_KERNEL);
		gnx_io_dev->cl = class_create(THIS_MODULE, "gnxio");
		device_create(gnx_io_dev->cl, NULL, first, NULL, "gnxio0");
		cdev_init(&gnx_io_dev->cdev,&gnx_io_fops);
		gnx_io_dev->cdev.owner = THIS_MODULE;
		gnx_io_dev->cdev.ops = &gnx_io_fops;
		err = cdev_add(&gnx_io_dev->cdev,first,1);
		INIT_LIST_HEAD(&gnx_io_dev->list);
		spin_lock_init(&gnx_io_dev->list_lock);
		gnx_io_set_mapping(gnx_io_dev);
		gnx_io_init_mapping(gnx_io_dev);
	}
	return err;
}


static void gnx_io_cleanup(void)
{
	if(gnx_io_dev)
	{
		gnx_io_release_mapping(gnx_io_dev);
		cdev_del(&gnx_io_dev->cdev);
		device_destroy(gnx_io_dev->cl, first_store);
		class_destroy(gnx_io_dev->cl);
		unregister_chrdev_region(first_store,1);
		kfree(gnx_io_dev);
	}
}


static int __init guinnuxio_init(void)
{
  int err = 0;

  pr_debug ("Loading Guinnux IO\n");

  err = gnx_io_setup();
  if(err)
    goto alloc_err;

  return err;

  alloc_err:
    pr_debug("Could not allocate the device numbers\n");
    gnx_io_cleanup();
    return err;
}

static void __exit guinnuxio_exit(void)
{
  gnx_io_cleanup();

  pr_debug ("Unloading Guinnux IO.\n");
  return;
}



module_init(guinnuxio_init);
module_exit(guinnuxio_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Keystone Electronic Solutions");
MODULE_DESCRIPTION("gnxio abstracts the GPIO for the Guinnux based hardware");
MODULE_INFO(alias,"gnxio");

#if defined(CONFIG_OF)
static const struct of_device_id gnxio_dt_ids[] =
{
    { .compatible = "kses,gnxio" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gnxio_dt_ids);
#endif
