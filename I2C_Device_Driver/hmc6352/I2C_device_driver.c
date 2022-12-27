#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <asm/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#define I2C_BUS_AVAILABLE (1)        // I2C Bus available in our Raspberry Pi
#define SLAVE_DEVICE_NAME ("HMC6352") // Device and Driver Name
#define HMC6352_SLAVE_ADDR (0x21)     // SSD1306 OLED Slave Address
/*Operation Mode Reg Value*/
#define OP_MODE_REG_MEASURE_1HZ     0 << 4
#define OP_MODE_REG_MEASURE_5HZ     1 << 4
#define OP_MODE_REG_MEASURE_10HZ    2 << 4
#define OP_MODE_REG_MEASURE_20HZ    3 << 4
#define OP_MODE_REG_PERIODIC_OFF    0 << 3
#define OP_MODE_REG_PERIODIC_ON     1 << 3
#define OP_MODE_REG_STANBY_MODE     0
#define OP_MODE_REG_QUERY_MODE      1
#define OP_MODE_REG_CONTINUOUS_MODE 2
/*Output Data Mode Reg Value*/
#define OD_MODE_REG_HEADING_MODE    0
#define OD_MODE_REG_RAW_MAG_X_MODE  1
#define OD_MODE_REG_RAW_MAG_Y_MODE  2
#define OD_MODE_REG_MAG_X_MODE      3
#define OD_MODE_REG_MAG_Y_MODE      4

/*Command*/
#define CMD_WRITE_EEPROM         0x77
#define CMD_READ_EEPROM          0x72
#define CMD_WRITE_RAM            0x47
#define CMD_READ_RAM             0x67
#define CMD_SLEEP_MODE           0x53
#define CMD_EXIT_SLEEP_MODE      0x57
#define CMD_UPDATE_BRIDGE_OFFSET 0x4f
#define CMD_ENTER_USER_CAL_MODE  0x43
#define CMD_ENTER_EXIT_CAL_MODE  0x45
#define CMD_SAVE_OP_MODE_EEPROM  0x4c
#define CMD_GET_DATA             0x41

/*RAM ADDRESS*/
#define ADDR_OP_MODE    0x74
#define ADDR_OD_MODE    0x4e

#define EEPROM_ADDR_I2C_SLAVE           0x00
#define EEPROM_ADDR_MAG_X_OFFSET_MSB    0x01
#define EEPROM_ADDR_MAG_X_OFFSET_LSB    0x02
#define EEPROM_ADDR_MAG_Y_OFFSET_MSB    0x03
#define EEPROM_ADDR_MAG_Y_OFFSET_LSB    0x04
#define EEPROM_ADDR_TIME_DELAY          0x05
#define EEPROM_ADDR_NOS_MEASURE         0x06
#define EEPROM_ADDR_SW_VERSION          0x07
#define EEPROM_ADDR_OP_MODE_BYTE        0x08

dev_t dev                                       = 0;
static struct class *dev_class                  = NULL;
static struct cdev hmc6352_cdev;
static struct i2c_adapter *i2c_adapter          = NULL;  // I2C Adapter Structure
static struct i2c_client  *i2c_client_hmc6352   = NULL;  // I2C Cient Structure (In our case it is OLED)

typedef struct _hmc6352info_
{
  unsigned char cmd;
  unsigned char write_val[32];
  unsigned char read_val[32];
}HCM6352Info;

unsigned short gSizeOfHCM6352Info = sizeof(HCM6352Info);
//////////////////////
#define IOCTL_MAGIC     'G'
#define IOCTL_READ      _IOR(IOCTL_MAGIC, 0, HCM6352Info)
#define IOCTL_WRITE     _IOW(IOCTL_MAGIC, 1, HCM6352Info)
#define IOCTL_STATUS    _IO(IOCTL_MAGIC, 2)
#define IOCTL_RW        _IOWR(IOCTL_MAGIC, 3, HCM6352Info)
#define IOCTL_NR        4

/*************** Driver functions **********************/
static int hmc6352_open(struct inode *inode, struct file *file);
static int hmc6352_release(struct inode *inode, struct file *file);
static ssize_t hmc6352_read(struct file *filp, char __user *buf, size_t len,loff_t * off);
static ssize_t hmc6352_write(struct file *filp, const char *buf, size_t len, loff_t * off);
static long hmc6352_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
/******************************************************/
 
//File operation structure 
static struct file_operations fops =
{
  .owner          = THIS_MODULE,
  .read           = hmc6352_read,
  .write          = hmc6352_write,
  .open           = hmc6352_open,
  .unlocked_ioctl = hmc6352_ioctl,
  .release        = hmc6352_release,
};
/*
** This function will be called when we open the Device file
*/ 
static int hmc6352_open(struct inode *inode, struct file *file)
{
  pr_info("Device File Opened...!!!\n");
  return 0;
}
/*
** This function will be called when we close the Device file
*/
static int hmc6352_release(struct inode *inode, struct file *file)
{
  pr_info("Device File Closed...!!!\n");
  return 0;
}
/*
** This function will be called when we read the Device file
*/ 
static ssize_t hmc6352_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
  HCM6352Info gpioinfo;
  unsigned char rd_value;

  if (copy_from_user((void*)&gpioinfo, (void __user*)buf, gSizeOfHCM6352Info)) 
  { 
    return -EFAULT; 
  }


  return rd_value;
}
/*
** This function will be called when we write the Device file
*/ 
static ssize_t hmc6352_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
  HCM6352Info gpioinfo;

  if (copy_from_user((void*)&gpioinfo, (void __user*)buf, gSizeOfHCM6352Info)) 
  { 
    return -EFAULT; 
  }
  
  return len;
}

static long hmc6352_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    HCM6352Info gpioinfo;
    
    pr_info("Device IOCTL ...!!!\n");

    switch (cmd)
    {
        case IOCTL_READ:

            break;

        case IOCTL_WRITE:
        			if (copy_from_user((void*)&gpioinfo, (void __user*)arg, gSizeOfHCM6352Info)) 
              { 
                return -EFAULT; 
              }

              //FnProcessGpio(&gpioinfo);

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

static int WakeUp(void)
{
    unsigned char value = CMD_EXIT_SLEEP_MODE;
    int ret             = i2c_master_send(i2c_client_hmc6352, &value, 1);
    
    return ret;
}

static int I2C_Send(unsigned char cmd, unsigned char* data, unsigned char len)
{
    int ret = 0;
    unsigned char i2c_buff[5] = {0,};
    
    i2c_buff[0] = cmd;
    memcpy(&i2c_buff[1],data,len);
    ret = i2c_master_send(i2c_client_hmc6352, i2c_buff, len+1);

    return ret;
}

static int I2C_Recv(unsigned char* data, unsigned char len)
{
    int ret = 0;

    ret = i2c_master_recv(i2c_client_hmc6352, data, len);

    return ret;
}

void Get_EEPROM_Info(void)
{
    int ret                  = 0;
    unsigned char regval     = {0,};
    unsigned char readregval = {0,};
    unsigned char find_idx   = 0;

    for(find_idx = 0; find_idx <= EEPROM_ADDR_OP_MODE_BYTE; find_idx++)
    {
        regval = find_idx;
        ret = I2C_Send(CMD_READ_EEPROM,&regval,1);
        if(ret < 0)
            dev_err(&i2c_client_hmc6352->dev,"error send read eeprom command\r\n");

        ret = I2C_Recv(&readregval,1);
        if(ret < 0)
            dev_err(&i2c_client_hmc6352->dev,"error recv eeprom address\r\n");

        pr_info("EEPROM ADDR 0x%X : 0x%X\t",find_idx, readregval);            
    }
}

void Set_RAM_REG(void)
{
    int ret                     = 0;
    unsigned char regval[5]     = {0,};
    unsigned char readregval[5] = {0,};

    regval[0] = ADDR_OP_MODE;
    regval[1] = OP_MODE_REG_QUERY_MODE;
    ret = I2C_Send(CMD_WRITE_RAM,regval,2);
    if(ret < 0)
        dev_err(&i2c_client_hmc6352->dev,"error sending write operation mode command\r\n");

    usleep_range(100,120);

    regval[0] = ADDR_OP_MODE;
    ret = I2C_Send(CMD_READ_RAM,regval,1);
    if(ret < 0)
        dev_err(&i2c_client_hmc6352->dev,"error send read ram command\r\n");

    ret = I2C_Recv(readregval,1);
    if(ret < 0)
        dev_err(&i2c_client_hmc6352->dev,"error recv ram address\r\n");

    pr_info("RAM 0x%X : 0x%X\t", regval[0], readregval[0]);   
}

/*
** This function getting called when the slave has been removed
** Note : This will be called only once when we unload the driver.
*/
static int hmc6352_remove(struct i2c_client *client)
{      
    pr_info("hmc6352 Removed!!!\n");
    return 0;
}

/*
** Structure that has slave device id
*/

static const struct i2c_device_id hmc6352_id[] = 
{
        { SLAVE_DEVICE_NAME, 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, hmc6352_id);

static const struct of_device_id hmc6352_of_matches[] = 
{
    {.compatible = "jaesun,hmc6352",},
    {} 
};
MODULE_DEVICE_TABLE(of, hmc6352_of_matches);
/*
** I2C driver Structure that has to be added to linux
*/

static int hmc6352_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret                     = 0;
    unsigned char regval[5]     = {0,};
    unsigned char readregval[5] = {0,};
    i2c_client_hmc6352 = client;

    pr_info("hmc6352 Probed \n");
    pr_info("name           : %s\n",i2c_client_hmc6352->name);
    pr_info("flags          : %X\n",i2c_client_hmc6352->flags);
    pr_info("client addr    : %X\n",i2c_client_hmc6352->addr);

    if( i2c_client_hmc6352 == NULL )
    {
        pr_info("i2c_client_hmc6352 == NULL\n");
        return 1;
    }

    ret = WakeUp();
    if(ret < 0)
        dev_err(&client->dev,"error sending wake command\r\n");

    usleep_range(100,120);

    Get_EEPROM_Info();
    Set_RAM_REG();
/*
    ret = I2C_Recv(readregval,2);
     if(ret < 0)
        dev_err(&client->dev,"error sending write operation mode command\r\n");

    pr_info("0x%X, 0x%X\r\n",readregval[0], readregval[1]);
*/
    return 0;
}

static struct i2c_driver hmc6352_driver = 
{
        .driver = 
        {
            .name   = SLAVE_DEVICE_NAME,
            .owner  = THIS_MODULE,
            .of_match_table = of_match_ptr(hmc6352_of_matches),
        },
        .probe          = hmc6352_probe,
        .remove         = hmc6352_remove,
        .id_table       = hmc6352_id,
};

/*
** Module Init function
*/
static int __init hmc6352_driver_init(void)
{
    int ret = -1;
    /*Allocating Major number*/
    if((alloc_chrdev_region(&dev, 0, 1, "hmc6352_dev")) <0)
    {
        pr_err("Cannot allocate major number\n");
        goto r_unreg;
    }
    pr_info("Major = %d Minor = %d \n",MAJOR(dev), MINOR(dev));
    
    /*Creating cdev structure*/
    cdev_init(&hmc6352_cdev,&fops);
    
    /*Adding character device to the system*/
    if((cdev_add(&hmc6352_cdev,dev,1)) < 0)
    {
        pr_err("Cannot add the device to the system\n");
        goto r_del;
    }
    
    /*Creating struct class*/
    if((dev_class = class_create(THIS_MODULE,"hmc6352_class")) == NULL)
    {
        pr_err("Cannot create the struct class\n");
        goto r_class;
    }
    
    /*Creating device*/
    if((device_create(dev_class,NULL,dev,NULL,"hmc6352_device")) == NULL)
    {
        pr_err( "Cannot create the Device \n");
        goto r_device;
    }

    i2c_adapter     = i2c_get_adapter(I2C_BUS_AVAILABLE);
    
    if( i2c_adapter != NULL )
    {
        ret = 0;
        i2c_put_adapter(i2c_adapter);
        i2c_add_driver(&hmc6352_driver);

        pr_info("Device Driver Insert...Done!!!\n");
        return 0;
    }

    pr_info("Driver Added!!! [%d]\n",ret);

    r_device:
    device_destroy(dev_class,dev);
    r_class:
    class_destroy(dev_class);
    r_del:
    cdev_del(&hmc6352_cdev);
    r_unreg:
    unregister_chrdev_region(dev,1);

    return ret;
}
/*
** Module Exit function
*/
static void __exit hmc6352_driver_exit(void)
{
    device_destroy(dev_class,dev);
    class_destroy(dev_class);
    cdev_del(&hmc6352_cdev);
    unregister_chrdev_region(dev, 1);
    i2c_del_driver(&hmc6352_driver);
    pr_info("Driver Removed!!!\n");
}

module_init(hmc6352_driver_init);
module_exit(hmc6352_driver_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("<djs6421@gmail.com>");
MODULE_DESCRIPTION("HMC6352 Device Driver");
MODULE_VERSION("0.01");