#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/uaccess.h>  
#include <linux/gpio.h>     
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/wait.h>

//#define EN_DEBOUNCE
#ifdef EN_DEBOUNCE
#include <linux/jiffies.h>
extern unsigned long volatile jiffies;
unsigned long old_jiffie = 0;
#endif

typedef struct _user_data_
{
    unsigned int type;
    unsigned int data;
    unsigned int gpio_num;
    unsigned int GPIO_irqNumber;
}UserData;

DECLARE_WAIT_QUEUE_HEAD(WaitQueue_Read);

static unsigned int wake_condition = 0;
static struct cdev intrcdev;
static struct class *dev_class;
dev_t dev = 0;
//This used for storing the IRQ number for the GPIO
//Interrupt handler for GPIO 25. This will be called whenever there is a raising edge detected. 
static irqreturn_t gpio_irq_handler(int irq,void *dev_id) 
{
    UserData* IntrDataPtr = (UserData*)dev_id;

    pr_info("handler : DataPtr->type : 0x%X\n",IntrDataPtr->type);
  
#ifdef EN_DEBOUNCE
    unsigned long diff = jiffies - old_jiffie;
    if (diff < 20)
    {
        return IRQ_HANDLED;
    }
    
    old_jiffie = jiffies;
#endif  
    wake_condition = 1;
    wake_up_interruptible(&WaitQueue_Read);
    return IRQ_HANDLED;
}
 

 
static int __init intr_driver_init(void);
static void __exit intr_driver_exit(void);

static int gpio_intr_probe(struct platform_device *pdev);
static int gpio_intr_remove(struct platform_device *pdev);

static const struct of_device_id gpio_intr_of_matches[] = {
	{.compatible = "jaesun,gpio_intr",},
	{}
};
MODULE_DEVICE_TABLE(of, gpio_intr_of_matches);

static struct platform_driver gpio_intr_driver = {
    .driver = {
      .name	= "gpio_intr",
      .owner = THIS_MODULE,
		  .of_match_table = of_match_ptr(gpio_intr_of_matches),
    },
    .probe        = gpio_intr_probe,
    .remove       = gpio_intr_remove,
};

static int gpio_intr_probe(struct platform_device *pdev)
{
    pr_info("enter gpio_intr_probe[%s]\n",pdev->name);
    UserData* DataPtr;
    DataPtr = (UserData*)kmalloc(sizeof(UserData), GFP_KERNEL);
    int gpio_count;
    enum of_gpio_flags flags;

    platform_set_drvdata(pdev, DataPtr);
    DataPtr->type       = 0x2022;
    gpio_count          = of_gpio_count(pdev->dev.of_node);
    DataPtr->gpio_num   = of_get_gpio_flags(pdev->dev.of_node, gpio_count-1, &flags);

    pr_info("gpio count[%d], gpio[%d], flags[0x%X]\n",gpio_count, DataPtr->gpio_num, flags);

    if(gpio_is_valid(DataPtr->gpio_num) == false)
    {
        pr_err("GPIO %d is not valid\n", DataPtr->gpio_num);
        return -1;
    }
    
    if(gpio_request(DataPtr->gpio_num,"GPIO_INTR_IN") < 0){
        pr_err("ERROR: GPIO %d request\n", DataPtr->gpio_num);
        return -1;
    }
    
    //configure the GPIO as input
    gpio_direction_input(DataPtr->gpio_num);

    DataPtr->GPIO_irqNumber = gpio_to_irq(DataPtr->gpio_num);
    pr_info("GPIO_irqNumber = %d\n", DataPtr->GPIO_irqNumber);

    if (request_irq(DataPtr->GPIO_irqNumber,    //IRQ number
                    (void *)gpio_irq_handler,   //IRQ handler
                    IRQF_TRIGGER_RISING,        //Handler will be called in raising edge
                    "intr_device",              //used to identify the device name using this IRQ
                    (void*)DataPtr)) 
    {                    //device id for shared IRQ
        pr_err("my_device: cannot register IRQ ");
        gpio_free(DataPtr->gpio_num);
        kfree(DataPtr);
    }

    return 0;
}

static int gpio_intr_remove(struct platform_device *pdev)
{
    pr_info("enter gpio_intr_remove\n");
    UserData* DataPtr;
    DataPtr = (UserData*)platform_get_drvdata(pdev);

    pr_info("type : 0x%X, irq_num : %d, gpio_num : %d",DataPtr->type,DataPtr->GPIO_irqNumber,DataPtr->gpio_num);
    
    free_irq(DataPtr->GPIO_irqNumber, DataPtr);
    gpio_free(DataPtr->gpio_num);

    
    kfree(DataPtr);

    return 0;
}
 
/*
** This function will be called when we open the Device file
*/ 
static int myIntrDrv_open(struct inode *inode, struct file *file)
{
    pr_info("Device File Opened...!!!\n");
    return 0;
}
/*
** This function will be called when we close the Device file
*/ 
static int myIntrDrv_release(struct inode *inode, struct file *file)
{
    pr_info("Device File Closed...!!!\n");
    return 0;
}
/*
** This function will be called when we read the Device file
*/ 
static ssize_t myIntrDrv_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{  
  
    return 0;
}
/*
** This function will be called when we write the Device file
*/
static ssize_t myIntrDrv_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{

    return 0;
}

/*
** This function will be called when we poll the Device file
*/
static unsigned int myIntrDrv_poll(struct file *filp, struct poll_table_struct* wait)
{
    unsigned int poll_bit = 0;

    printk("[jaesun] poll_wait WaitQueue\r\n");

    //poll_wait(filp, &WaitQueue_Read, wait);
    wake_condition = 0;
    wait_event_interruptible(WaitQueue_Read,wake_condition);

    printk("[jaesun] poll_wait WaitQueue wake up!!!\r\n");

    poll_bit |= (POLLIN | POLLRDNORM);

    return poll_bit;
}

//File operation structure 
static struct file_operations myIntrDrv_fops =
{
	.owner		    = THIS_MODULE,
	.open		    = myIntrDrv_open,
	.write		    = myIntrDrv_write,
    .read		    = myIntrDrv_read,
	.poll		    = myIntrDrv_poll,
	.release	    = myIntrDrv_release,
};

/*
** Module Init function
*/ 
static int __init intr_driver_init(void)
{
    int ret;
    ret = platform_driver_register(&gpio_intr_driver);

    if(ret != 0)
    {
        pr_err("platform_driver_register err\n");
        goto r_unreg;
    }

    if((alloc_chrdev_region(&dev, 0, 1, "intr_Dev")) <0)
    {
        pr_err("Cannot allocate major number\n");
        goto r_unreg;
    }
    pr_info("Major = %d Minor = %d \n",MAJOR(dev), MINOR(dev));
    
    cdev_init(&intrcdev,&myIntrDrv_fops);
    
    if((cdev_add(&intrcdev,dev,1)) < 0)
    {
        pr_err("Cannot add the device to the system\n");
        goto r_del;
    }
    
    if((dev_class = class_create(THIS_MODULE,"intr_class")) == NULL)
    {
        pr_err("Cannot create the struct class\n");
        goto r_class;
    }
    
    if((device_create(dev_class,NULL,dev,NULL,"intr_device")) == NULL)
    {
        pr_err( "Cannot create the Device \n");
        goto r_device;
    }  
 
    pr_info("Device Driver Insert...Done!!!\n");
    return 0;

r_device:
    device_destroy(dev_class,dev);
r_class:
    class_destroy(dev_class);
r_del:
    cdev_del(&intrcdev);
r_unreg:
    unregister_chrdev_region(dev,1);
  
    return -1;
}
/*
** Module exit function
*/
static void __exit intr_driver_exit(void)
{
    platform_driver_unregister(&gpio_intr_driver);
    device_destroy(dev_class,dev);
    class_destroy(dev_class);
    cdev_del(&intrcdev);
    unregister_chrdev_region(dev, 1);
    pr_info("Device Driver Remove...Done!!\n");
}
 
module_init(intr_driver_init);
module_exit(intr_driver_exit);
 
MODULE_LICENSE("GPL");
MODULE_AUTHOR("djs6421@gmail.com");
MODULE_DESCRIPTION("GPIO INTR Device Driver");
MODULE_VERSION("0.01");