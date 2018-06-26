#include<linux/module.h> /*Needed by all modules*/
#include<linux/kernel.h> /*Needed for KERN_INFO*/
#include<linux/init.h>   /*Needed for the macros*/
#include<linux/interrupt.h>
#include<linux/sched.h>
#include<linux/platform_device.h>
#include<linux/io.h>
#include<linux/of.h>
#include<linux/miscdevice.h>
#include<linux/fs.h>
#include<linux/uaccess.h>
#include<linux/slab.h>
#include<linux/dma-mapping.h>

#include<linux/of_gpio.h>
#include<linux/gpio/consumer.h>

#define DEVNAME                     "int_driver"

#define DEV_FILE_NAME_MAX_LENGTH    64

struct device_data_t
{
    wait_queue_head_t queue_irq;
    int               irq_sign;
    wait_queue_head_t queue_busy;
    int               busy;
    struct miscdevice dev;
    char              file_name[DEV_FILE_NAME_MAX_LENGTH];
//    int               gpio_pin;
//    int               gpio_active_low;
    struct gpio_desc* gpio;
};


irqreturn_t test_isr(int irq, void* dev_id)
{
    struct device_data_t* device_data=(struct device_data_t*) dev_id;
    printk(KERN_INFO DEVNAME":ISR %s\n",device_data->file_name);
    if(device_data)
    {
      gpiod_set_value_cansleep(device_data->gpio, 1);
      device_data->irq_sign=1;
      wake_up_interruptible(&device_data->queue_irq);
    }
    return IRQ_HANDLED;
}

static int open( struct inode *n, struct file *filp )
{
   return 0;
}

ssize_t read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct device_data_t* device_data;
    device_data=container_of(filp->private_data,struct device_data_t, dev);
    if(device_data)
    {
      if( (filp->f_flags & O_NONBLOCK) && device_data->busy)
      {
        return -EAGAIN;
      }

      if(wait_event_interruptible(device_data->queue_busy,device_data->busy==0))
      {
        return -ERESTARTSYS;
      }
      device_data->busy=1;

      if(wait_event_interruptible(device_data->queue_irq,device_data->irq_sign==1))
      {
        wake_up_interruptible(&device_data->queue_busy);
        device_data->busy=0;
        return -ERESTARTSYS;
      }
      device_data->irq_sign=0;
      gpiod_set_value_cansleep(device_data->gpio, 0);
      wake_up_interruptible(&device_data->queue_busy);
      device_data->busy=0;
    }
    return 0;
}

ssize_t write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    char inner_buff;
    struct device_data_t* device_data;
    int result=count;

    device_data=container_of(filp->private_data,struct device_data_t, dev);
    if(copy_from_user(&inner_buff,buf,sizeof(inner_buff)))
    {
        return -EFAULT;
    }

    if(device_data)
    {
      switch(inner_buff)
      {
        case('0'): gpiod_set_value_cansleep(device_data->gpio, 0); break;
        case('1'): gpiod_set_value_cansleep(device_data->gpio, 1); break;
        default  : result=-EFAULT;                        break;

      }
    }
    return result;
}


static const struct file_operations fops = {
   .owner  = THIS_MODULE,
   .open   = open,
   .read   =  read,
   .write  =  write,
};


static int int_driver_probe(struct platform_device* pdev)
{
    int irq_num, result, idx;
    struct device_data_t* device_data;
    struct resource* dev_resource;
    enum of_gpio_flags of_gpio_flags;
    unsigned long gpio_flags = GPIOF_OUT_INIT_LOW;

    struct gpio_desc* gpio;



    printk(KERN_INFO DEVNAME": start probe\n");

    irq_num=platform_get_irq(pdev,0);

    if(irq_num<0)
    {
      printk(KERN_ERR DEVNAME": can not get irq\n");
      return irq_num;
    }

    dev_resource = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if(!dev_resource)
    {
      printk(KERN_ERR DEVNAME": resource not found\n");
      return -ENXIO;
    }

    device_data = devm_kzalloc(&pdev->dev, sizeof(struct device_data_t),GFP_KERNEL);
    if(!device_data)
    {
       printk(KERN_ERR DEVNAME": can not allocate memory\n");
       return -ENOMEM;
    }

    result=devm_request_irq(&pdev->dev, irq_num, test_isr, 0, DEVNAME, device_data);
    if(result<0)
    {
      printk(KERN_ERR DEVNAME": can not allocate irq\n");
      return result;
    }

    device_data->irq_sign=0;
    init_waitqueue_head(&device_data->queue_irq);

    device_data->busy=0;
    init_waitqueue_head(&device_data->queue_busy);


    idx=0;
    while(dev_resource->name[idx])
    {
      device_data->file_name[idx]=dev_resource->name[idx];
      ++idx;
    }
    device_data->file_name[idx]='\0';


    device_data->gpio=devm_gpiod_get(&pdev->dev, NULL, 0);

    if(!device_data->gpio)
    {
      printk(KERN_ERR DEVNAME": can not allocate gpio\n");
      return -EIO;
    }

    gpiod_direction_output(device_data->gpio,1);


#if 0
    device_data->gpio_pin=of_get_gpio(pdev->dev.of_node, 0);
    if(device_data->gpio_pin<0)
    {
      printk(KERN_ERR DEVNAME": can not allocate gpio\n");
      return result;
    }

    result=of_get_gpio_flags(pdev->dev.of_node, 0, &of_gpio_flags);

    if(result<0)
    {
      printk(KERN_ERR DEVNAME": can not get gpio flags\n");
      return result;
    }


   if (of_gpio_flags & OF_GPIO_ACTIVE_LOW)
   {
       gpio_flags |= GPIOF_ACTIVE_LOW;
       device_data->gpio_active_low=1;
   }
   else
   {
       device_data->gpio_active_low=0;
   }

    result=gpio_request_one(device_data->gpio_pin,gpio_flags,device_data->file_name);

    if(result<0)
    {
      printk(KERN_ERR DEVNAME": can not reques gpio\n");
      return result;
    }

    gpio_direction_output(device_data->gpio_pin, device_data->gpio_active_low);
#endif
    device_data->dev.minor=MISC_DYNAMIC_MINOR;
    device_data->dev.fops=&fops;
    device_data->dev.mode=0666;
    device_data->dev.name=device_data->file_name;
    device_data->dev=device_data->dev;

    result= misc_register(&device_data->dev);

    if(result)
    {
     	printk(KERN_ERR DEVNAME": can not register character file %s\n", dev_resource->name);
        return result;
    }



    platform_set_drvdata(pdev,device_data);
    printk(KERN_INFO DEVNAME": successfully registered %s\n", dev_resource->name);

    return result;
}

static int int_driver_remove(struct platform_device* pdev)
{
    struct device_data_t* device_data;
    device_data=platform_get_drvdata(pdev);
    if(device_data)
    {
      misc_deregister(&device_data->dev);
//      gpio_direction_input(device_data->gpio_pin);
//      gpio_free(device_data->gpio_pin);
    }
    printk(KERN_INFO DEVNAME": %s successfully removed\n", device_data->file_name);
    platform_set_drvdata(pdev,NULL);
    return 0;
}

static const struct of_device_id int_driver_id[]=
{
	{.compatible="alex,myirq"},
	{}
};

static struct platform_driver int_driver=
{
	.driver=
	{
		.name=DEVNAME,
		.owner=THIS_MODULE,
		.of_match_table=of_match_ptr(int_driver_id),
	},
	.probe=int_driver_probe,
	.remove=int_driver_remove
};

module_platform_driver(int_driver);
MODULE_LICENSE("GPL");
