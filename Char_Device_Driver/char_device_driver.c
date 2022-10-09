#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/ioctl.h>

#define DEVICE_NAME "jchardev"
#define USER_DATA_LEN 1500

typedef struct _ioctl_info
{
    unsigned int size;
    unsigned char buff[USER_DATA_LEN];
}IOCTL_INFO;

#define IOCTL_MAGIC     'G'
#define IOCTL_READ      _IOR(IOCTL_MAGIC, 0, IOCTL_INFO)
#define IOCTL_WRITE     _IOW(IOCTL_MAGIC, 1, IOCTL_INFO)
#define IOCTL_STATUS    _IO(IOCTL_MAGIC, 2)
#define IOCTL_RW        _IOWR(IOCTL_MAGIC, 3, IOCTL_INFO)
#define IOCTL_NR        4

DECLARE_WAIT_QUEUE_HEAD(WaitQueue_Read);

static struct cdev jcdev;
static struct class *jcl;
static dev_t dev;

static const unsigned int baseminor     = 0;
static const unsigned int minor_count   = 1;
static unsigned int wake_condition      = 0;

typedef struct _user_data
{
    unsigned char txbuffer[USER_DATA_LEN];
    unsigned char tmpbuffer[USER_DATA_LEN];
    unsigned char rxbuffer[USER_DATA_LEN];
}USER_DATA;

static int myCharDrv_open(struct inode *inode, struct file *file)
{
    printk(KERN_DEBUG"[jaesun] %s: %s\r\n",DEVICE_NAME,__func__);

    USER_DATA *ptr = kmalloc(sizeof(USER_DATA),GFP_KERNEL);

    if(ptr == NULL)
    {
        printk(KERN_ERR"kmalloc return null\r\n");
    }

    file->private_data = ptr;

    return 0;
}

static int myCharDrv_release(struct inode *inode, struct file *file)
{
    printk(KERN_DEBUG"[jaesun] %s: %s\r\n",DEVICE_NAME,__func__);
    if(file->private_data)
    {
        kfree(file->private_data);
        file->private_data = NULL;
    }
    
    return 0;
}

static ssize_t myCharDrv_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    USER_DATA *ptr = filp->private_data;
    int ret;

    ret= copy_from_user(ptr->txbuffer,buf,count);
    memcpy(ptr->tmpbuffer,ptr->txbuffer,count);

    if(ret)
    {
        printk(KERN_ERR"copy_from_user not copied size : %d\r\n",ret);
    }

    return count;
}

static ssize_t myCharDrv_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    USER_DATA *ptr = filp->private_data;
    int ret;

    if(count > USER_DATA_LEN)
    {
        printk(KERN_ERR"[jaesun] 1500 > %d -> read size %d -> %d", count, count, USER_DATA_LEN);
        count = USER_DATA_LEN;
    }

    memcpy(ptr->rxbuffer,ptr->tmpbuffer,count);
    ret = copy_to_user(buf,ptr->rxbuffer,count);

    if(ret)
    {
        printk(KERN_ERR"copy_to_user not copied size : %d\r\n",ret);
    }

    return count;
}

static unsigned int myCharDrv_poll(struct file *filp, struct poll_table_struct* wait)
{
    unsigned int poll_bit;

    printk(KERN_NOTICE"[jaesun] poll_wait WaitQueue\r\n");

    //poll_wait(filp, &WaitQueue_Read, wait);
    wake_condition = 0;
    wait_event_interruptible(WaitQueue_Read,wake_condition);

    printk(KERN_NOTICE"[jaesun] poll_wait WaitQueue wake up!!!\r\n");

    poll_bit |= (POLLIN | POLLRDNORM);

    return poll_bit;
}

//cmd 32bit type(2bit) data size(14bit) magic number(8) identified number(8)

static long myCharDrv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    printk(KERN_NOTICE"[jaesun] %s\r\n",__func__);
    switch (cmd)
    {
        case IOCTL_READ:
    
            break;
    
        case IOCTL_WRITE:
            printk(KERN_NOTICE"[jaesun] wake up WaitQueue\r\n");
            //wake_up(&WaitQueue_Read);
            wake_condition = 1;
            wake_up_interruptible(&WaitQueue_Read);
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

static struct file_operations myCharDrv_fops =
{
	.owner		    = THIS_MODULE,
	.open		    = myCharDrv_open,
	.write		    = myCharDrv_write,
    .read		    = myCharDrv_read,
	.poll		    = myCharDrv_poll,
	.release	    = myCharDrv_release,
    .unlocked_ioctl	= myCharDrv_ioctl,
};

static int __init char_device_driver_module_init(void)
{
	printk(KERN_DEBUG"Init[Jaesun] %s\r\n",__func__);

    int ret;
    struct device *dev_ret;

    ret = alloc_chrdev_region(&dev, baseminor, minor_count, DEVICE_NAME);    //dynamic allocate major, minor number and register chrdev_region

    if(ret < 0)
    {
        printk(KERN_ERR"[jaesun] alloc_chardev_region %d\r\n",ret);
        return ret;
    }

    cdev_init(&jcdev,&myCharDrv_fops);  //reset cdev structure, connect cdev + file operations

    ret = cdev_add(&jcdev, dev, minor_count); //register cdev structure in kernel

    if(ret < 0)
    {
        printk(KERN_ERR"[jaesun] cdev_add %d\r\n",ret);
        unregister_chrdev_region(dev, minor_count);
        return 0;
    }

    jcl = class_create(THIS_MODULE, DEVICE_NAME);   // create /sys/class/DEVICE_NAME

    if(IS_ERR(jcl))
    {
        cdev_del(&jcdev);
        unregister_chrdev_region(dev, minor_count);
        printk(KERN_ERR"[jaesun] class_create\r\n");
        return -1;
    }

    dev_ret = device_create(jcl, NULL, dev, "%s", DEVICE_NAME); // create /dev/DEVICE_NAME

    if(IS_ERR(dev_ret))
    {
        class_destroy(jcl);
        cdev_del(&jcdev);
        unregister_chrdev_region(dev, minor_count);
        printk(KERN_ERR"[jaesun] device_create Err\r\n");

        return PTR_ERR(dev_ret);
    }

 	return 0;
}

static void __exit char_device_driver_module_exit(void)
{
    printk(KERN_DEBUG"Exit[Jaesun] %s\r\n",__func__);
	
    device_destroy(jcl,dev);
    class_destroy(jcl);
    cdev_del(&jcdev);
    unregister_chrdev_region(dev, minor_count);
}

MODULE_AUTHOR("Jaeson  <djs6421@gmail.com>");
MODULE_DESCRIPTION("Char Device Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

module_init(char_device_driver_module_init);
module_exit(char_device_driver_module_exit);