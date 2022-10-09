#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/uaccess.h>  //copy_to/from_user()
#include <linux/gpio.h>     //GPIO

#define MAX_GPIO_NUM  40

typedef struct _gpiostate_
{
  unsigned char assign;
  unsigned char dir;
}GpioState;


unsigned int gNumOfGpio = 0;
GpioState gGpioState[MAX_GPIO_NUM];

dev_t dev = 0;
static struct class *dev_class;
static struct cdev gpio_cdev;
// GPIO_USER_INTERFACE 
#define GPIO_ASSIGN     0x10
#define GPIO_DIR_CONFIG 0x20
#define GPIO_WR_CONFIG  0x30
#define GPIO_RD_CONFIG  0x40

#define GPIO_DIR_IN     0x20
#define GPIO_DIR_OUT    0x21

#define GPIO_VAL_LO    0x00
#define GPIO_VAL_HI    0x01

typedef struct _gpiouserinfo_
{
  unsigned char cmd;
  unsigned char gpioNum;
  unsigned char gpioDir;
  unsigned char gpioValue;
}GpioUserInfo;

unsigned short gSizeOfGpioUserInfo = sizeof(GpioUserInfo);
 //////////////////////
#define IOCTL_MAGIC     'G'
#define IOCTL_READ      _IOR(IOCTL_MAGIC, 0, GpioUserInfo)
#define IOCTL_WRITE     _IOW(IOCTL_MAGIC, 1, GpioUserInfo)
#define IOCTL_STATUS    _IO(IOCTL_MAGIC, 2)
#define IOCTL_RW        _IOWR(IOCTL_MAGIC, 3, GpioUserInfo)
#define IOCTL_NR        4

 static int __init gpio_driver_init(void);
static void __exit gpio_driver_exit(void);
 
/*************** Driver functions **********************/
static int gpio_open(struct inode *inode, struct file *file);
static int gpio_release(struct inode *inode, struct file *file);
static ssize_t gpio_read(struct file *filp, char __user *buf, size_t len,loff_t * off);
static ssize_t gpio_write(struct file *filp, const char *buf, size_t len, loff_t * off);
static long gpio_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
/******************************************************/

int FnProcessGpio(GpioUserInfo* UserPtr);
 
//File operation structure 
static struct file_operations fops =
{
  .owner          = THIS_MODULE,
  .read           = gpio_read,
  .write          = gpio_write,
  .open           = gpio_open,
  .unlocked_ioctl = gpio_ioctl,
  .release        = gpio_release,
};
/*
** This function will be called when we open the Device file
*/ 
static int gpio_open(struct inode *inode, struct file *file)
{
  pr_info("Device File Opened...!!!\n");
  return 0;
}
/*
** This function will be called when we close the Device file
*/
static int gpio_release(struct inode *inode, struct file *file)
{
  pr_info("Device File Closed...!!!\n");
  return 0;
}
/*
** This function will be called when we read the Device file
*/ 
static ssize_t gpio_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
  GpioUserInfo gpioinfo;
  unsigned char rd_value;

  if (copy_from_user((void*)&gpioinfo, (void __user*)buf, gSizeOfGpioUserInfo)) 
  { 
    return -EFAULT; 
  }

  if(gpioinfo.cmd == GPIO_RD_CONFIG && (gGpioState[gpioinfo.gpioNum].assign == 1) && (gGpioState[gpioinfo.gpioNum].dir == GPIO_DIR_IN))
  {
    rd_value = gpio_get_value(gpioinfo.gpioNum);
    return rd_value;
  }
  else
  {
    pr_err("gpio read : %d, %d, %d",gpioinfo.cmd, gGpioState[gpioinfo.gpioNum].assign, gGpioState[gpioinfo.gpioNum].dir);
    return -EFAULT;
  }

  return rd_value;
}
/*
** This function will be called when we write the Device file
*/ 
static ssize_t gpio_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
  GpioUserInfo gpioinfo;

  if (copy_from_user((void*)&gpioinfo, (void __user*)buf, gSizeOfGpioUserInfo)) 
  { 
    return -EFAULT; 
  }
  if(gpioinfo.cmd == GPIO_WR_CONFIG && (gGpioState[gpioinfo.gpioNum].assign == 1) && (gGpioState[gpioinfo.gpioNum].dir == GPIO_DIR_OUT))
  {
    gpio_set_value(gpioinfo.gpioNum, gpioinfo.gpioValue);
  }
  else
  {
    pr_err("gpio read : %d, %d, %d",gpioinfo.cmd, gGpioState[gpioinfo.gpioNum].assign, gGpioState[gpioinfo.gpioNum].dir);
    return -EFAULT;
  }
  
  return len;
}

static long gpio_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    GpioUserInfo gpioinfo;
    
    pr_info("Device IOCTL ...!!!\n");

    switch (cmd)
    {
        case IOCTL_READ:

            break;

        case IOCTL_WRITE:
        			if (copy_from_user((void*)&gpioinfo, (void __user*)arg, gSizeOfGpioUserInfo)) 
              { 
                return -EFAULT; 
              }

              FnProcessGpio(&gpioinfo);

            break;

        case IOCTL_RW:

            break;

        case IOCTL_STATUS:

            break;

        default :

            break;
    }

    return 0;
}

/*
** Module Init function
*/ 
static int __init gpio_driver_init(void)
{
  /*Allocating Major number*/
  if((alloc_chrdev_region(&dev, 0, 1, "gpio_dev")) <0)
  {
    pr_err("Cannot allocate major number\n");
    goto r_unreg;
  }
  pr_info("Major = %d Minor = %d \n",MAJOR(dev), MINOR(dev));
 
  /*Creating cdev structure*/
  cdev_init(&gpio_cdev,&fops);
 
  /*Adding character device to the system*/
  if((cdev_add(&gpio_cdev,dev,1)) < 0)
  {
    pr_err("Cannot add the device to the system\n");
    goto r_del;
  }
 
  /*Creating struct class*/
  if((dev_class = class_create(THIS_MODULE,"gpio_class")) == NULL)
  {
    pr_err("Cannot create the struct class\n");
    goto r_class;
  }
 
  /*Creating device*/
  if((device_create(dev_class,NULL,dev,NULL,"gpio_device")) == NULL)
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
  cdev_del(&gpio_cdev);
r_unreg:
  unregister_chrdev_region(dev,1);
  
  return -1;
}
/*
** Module exit function
*/ 
static void __exit gpio_driver_exit(void)
{
  unsigned char idx = 0;
  for(idx = 0; idx < MAX_GPIO_NUM; idx++)
  {
    if(gGpioState[idx].assign)
      gpio_free(idx);
  }

  device_destroy(dev_class,dev);
  class_destroy(dev_class);
  cdev_del(&gpio_cdev);
  unregister_chrdev_region(dev, 1);
  pr_info("Device Driver Remove...Done!!\n");
}

int FnProcessGpio(GpioUserInfo* UserPtr)
{
  if(UserPtr->cmd == GPIO_ASSIGN)
  {
    if(gGpioState[UserPtr->gpioNum].assign)
      return 1;

//Checking the GPIO is valid or not
    if(gpio_is_valid(UserPtr->gpioNum) == false)
    {
      pr_err("gpio %d is not valid\n", UserPtr->gpioNum);
      return -1;
    }

// //Requesting the GPIO
    if(gpio_request(UserPtr->gpioNum,"jaesun GPIO") < 0)
    {
      pr_err("ERROR: gpio %d request\n", UserPtr->gpioNum);
      return -1;
    }

    gGpioState[UserPtr->gpioNum].assign = 1;
    
  }
  else if(UserPtr->cmd == GPIO_DIR_CONFIG)
  {
    if(gGpioState[UserPtr->gpioNum].assign)
    {
      if(UserPtr->gpioDir == GPIO_DIR_IN)
      {
        gpio_direction_input(UserPtr->gpioNum);
        gGpioState[UserPtr->gpioNum].dir = GPIO_DIR_IN;
      }
      else if(UserPtr->gpioDir == GPIO_DIR_OUT)
      {
        gpio_direction_output(UserPtr->gpioNum,UserPtr->gpioValue);
        gGpioState[UserPtr->gpioNum].dir = GPIO_DIR_OUT;
      }
    }
    else
    {
      pr_err("gpio %d is not assigned\n",UserPtr->gpioNum);
      return -1;
    }
  }

  return 1;
}
 
module_init(gpio_driver_init);
module_exit(gpio_driver_exit);
 
MODULE_LICENSE("GPL");
MODULE_AUTHOR("djs6421@gmail.com");
MODULE_DESCRIPTION("GPIO Device Driver");
MODULE_VERSION("0.01");
