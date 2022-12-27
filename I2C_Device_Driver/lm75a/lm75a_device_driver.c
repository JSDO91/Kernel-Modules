#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <asm/delay.h>
#include <linux/platform_device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/sysfs.h>

#define I2C_BUS_AVAILABLE (1)        // I2C Bus available in our Raspberry Pi
#define SLAVE_DEVICE_NAME ("LM75A") // Device and Driver Name
#define lm75a_SLAVE_ADDR (0x48)     // SSD1306 OLED Slave Address

#define TEMP_REG   0x00
#define CONFG_REG  0x01
#define THYST_REG  0x02
#define TOS_REG    0x03

static struct i2c_adapter *i2c_adapter        = NULL;  // I2C Adapter Structure
static struct i2c_client  *i2c_client_lm75a   = NULL;  // I2C Cient Structure
struct lm75a_info
{
    struct i2c_client* lm75a_info; 
    //const struct attribute_group* lm75a_group;
    unsigned char read_val[4];
    unsigned char write_val[4];
    unsigned char preset_reg;
};
unsigned int gSizeOflm75aInfo = sizeof(struct lm75a_info);

int lm75a_reg_read(struct lm75a_info* sensor_data, unsigned char reg_type);
int lm75a_reg_write(struct lm75a_info* sensor_data, unsigned char reg_type);
/*************************************attribute********************************************************/
static ssize_t lm75a_hwmon_show_temp(struct device *dev, struct device_attribute *dev_attr, char *buf)
{
    struct lm75a_info* sensor_info = dev_get_drvdata(dev);
    int temp = 0;
    int ret = 0;
    
    if(sensor_info == NULL)
		return -EIO;
	else
    {
        ret = lm75a_reg_read(sensor_info,TEMP_REG);
        if(ret > 0)
        {
            if(sensor_info->read_val[0]&0x80)
            {
                temp = 0xFFFFF800;
            }
            temp |= sensor_info->read_val[0]<<3|sensor_info->read_val[1]>>5;
        }

        return sprintf(buf, "%d\n", temp * 125);
    }
}
static SENSOR_DEVICE_ATTR(temp1_input, 0444, lm75a_hwmon_show_temp, NULL, 1);

static ssize_t lm75a_hwmon_show_hyst(struct device *dev, struct device_attribute *dev_attr, char *buf)
{
    int hyst                        = 0;
    int ret                         = 0;
    struct lm75a_info* sensor_info  = dev_get_drvdata(dev);

    if(sensor_info == NULL)
		return -EIO;
	else
    {
        ret = lm75a_reg_read(sensor_info,THYST_REG);
        if(ret > 0)
        {
            if(sensor_info->read_val[0] & 0x80)
            {
                hyst = 0xFFFFFE00;
            }   
            hyst |= sensor_info->read_val[0]<<1|sensor_info->read_val[1]>>7;
        }
        
        return sprintf(buf, "%d\n", hyst * 50);
    }
}

static ssize_t lm75a_hwmon_store_hyst(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct lm75a_info* sensor_info = dev_get_drvdata(dev);
    int hyst    = 0;
    int ret     = 0;
    int val     = 0;

   if(sensor_info == NULL)
		return -EIO;
	else
    {
        ret = kstrtoint(buf,10,&val);
        if(ret != 0)
            return -EIO;
    
        val = val/50;
        pr_info("ret : %d, val : %d, %x\n",ret, val, val);
        sensor_info->write_val[1] = (unsigned char)((val&0x1FE) >> 1);
        sensor_info->write_val[2] = (unsigned char)((val&0x0001));

        ret = lm75a_reg_write(sensor_info,THYST_REG);
        if(ret < 0)
            return -EIO;

        return count;
    }
}
static SENSOR_DEVICE_ATTR(temp1_max_hyst, 0644, lm75a_hwmon_show_hyst, lm75a_hwmon_store_hyst, 1);

static struct attribute *lm75a_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
    &sensor_dev_attr_temp1_max_hyst.dev_attr.attr,
	NULL
};

ATTRIBUTE_GROUPS(lm75a);
/******************************************************************************************************/
/*
** Structure that has slave device id
*/

static const struct i2c_device_id lm75a_id[] = 
{
        { SLAVE_DEVICE_NAME, 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, lm75a_id);

static const struct of_device_id lm75a_of_matches[] = 
{
    //{.compatible = "jaesun,lm75a",},
    {.compatible = "jaesun,temp_sensor",},
    {}  
};
MODULE_DEVICE_TABLE(of, lm75a_of_matches);

static int lm75a_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret          = 0;
    i2c_client_lm75a = client;
    struct lm75a_info* sensor_info;
    struct device *hwmon_dev;

    sensor_info = devm_kzalloc(&client->dev,gSizeOflm75aInfo,GFP_KERNEL);
    if(!sensor_info)
        return -ENOMEM;

    sensor_info->preset_reg = 0xFF;

    i2c_set_clientdata(client, sensor_info);
    sensor_info->lm75a_info = client;

    pr_info("lm75a Probed \n");
    pr_info("name           : %s\n",sensor_info->lm75a_info->name);
    pr_info("flags          : %X\n",sensor_info->lm75a_info->flags);
    pr_info("client addr    : %X\n",sensor_info->lm75a_info->addr);

    if( sensor_info->lm75a_info == NULL )
    {
        pr_info("i2c_client_lm75a == NULL\n");
        return 1;
    }

    ret = lm75a_reg_read(sensor_info,TEMP_REG);
    if(ret > 0)
        pr_info("temp reg  : 0x%x, 0x%x\r\n",sensor_info->read_val[0],sensor_info->read_val[1]);
    else
        pr_info("temp reg read error : %d",ret);

    ret = lm75a_reg_read(sensor_info,CONFG_REG);
    if(ret > 0)
        pr_info("config reg : 0x%x\r\n",sensor_info->read_val[0]);
    else
        pr_info("config reg read error : %d",ret);
    
    ret = lm75a_reg_read(sensor_info,THYST_REG);
    if(ret > 0)
        pr_info("thyst reg  : 0x%x, 0x%x\r\n",sensor_info->read_val[0],sensor_info->read_val[1]);
    else
        pr_info("thyst reg read error : %d",ret);

    ret = lm75a_reg_read(sensor_info,TOS_REG);
    if(ret > 0)
        pr_info("tos reg    : 0x%x, 0x%x\r\n",sensor_info->read_val[0],sensor_info->read_val[1]);
    else
        pr_info("tos reg read error : %d",ret);

    hwmon_dev = devm_hwmon_device_register_with_groups(&sensor_info->lm75a_info->dev,sensor_info->lm75a_info->name,sensor_info,lm75a_groups);

    return PTR_ERR_OR_ZERO(hwmon_dev);
}

static int lm75a_remove(struct i2c_client *client)
{   
    pr_info("lm75a Removed!!!\n");

    return 0;
}

static struct i2c_driver lm75a_driver = 
{
        .driver = 
        {
            .name   = SLAVE_DEVICE_NAME,
            .owner  = THIS_MODULE,
            .of_match_table = of_match_ptr(lm75a_of_matches),
        },
        .probe          = lm75a_probe,
        .remove         = lm75a_remove,
        .id_table       = lm75a_id,
};

/*
** Module Init function
*/
static int __init lm75a_driver_init(void)
{
    int ret = -1;
    i2c_adapter     = i2c_get_adapter(I2C_BUS_AVAILABLE);
    
    if( i2c_adapter != NULL )
    {
        ret = 0;
        i2c_put_adapter(i2c_adapter);
        i2c_add_driver(&lm75a_driver);

        pr_info("Device Driver Insert...Done!!!\n");
        return 0;
    }

    pr_info("Driver Added!!! [%d]\n",ret);
    return ret;
}
/*
** Module Exit function
*/
static void __exit lm75a_driver_exit(void)
{
    i2c_del_driver(&lm75a_driver);
    pr_info("Driver Removed!!!\n");
}

int reg_read_with_pointer_byte(struct lm75a_info* sensor_data, unsigned char reg_type)
{
    int ret                 = 0;
    struct i2c_msg msg[2]   = {0,}; 

    msg[0].addr         = sensor_data->lm75a_info->addr;
    msg[0].buf          = &reg_type;
    msg[0].len          = 1;

    if(reg_type == CONFG_REG)
    {
        msg[1].addr         = sensor_data->lm75a_info->addr;
        msg[1].flags        = I2C_M_RD;
        msg[1].len          = 1;
        msg[1].buf          = sensor_data->read_val;
    }
    else if(reg_type == TEMP_REG || reg_type == TOS_REG || reg_type == THYST_REG)
    {
        msg[1].addr         = sensor_data->lm75a_info->addr;
        msg[1].flags        = I2C_M_RD;
        msg[1].len          = 2;
        msg[1].buf          = sensor_data->read_val;
    }
    else
    {
        return -1;
    }
        
    ret = i2c_transfer(sensor_data->lm75a_info->adapter,&msg[0],2);

    return ret;
}

int reg_read_preset(struct lm75a_info* sensor_data, unsigned char reg_type)
{
    int ret                 = 0;
    struct i2c_msg msg[2]   = {0,}; 

    if(reg_type == CONFG_REG)
    {
        msg[0].addr         = sensor_data->lm75a_info->addr;
        msg[0].flags        = I2C_M_RD;
        msg[0].len          = 1;
        msg[0].buf          = sensor_data->read_val;
    }
    else if(reg_type == TEMP_REG || reg_type == TOS_REG || reg_type == THYST_REG)
    {
        msg[0].addr         = sensor_data->lm75a_info->addr;
        msg[0].flags        = I2C_M_RD;
        msg[0].len          = 2;
        msg[0].buf          = sensor_data->read_val;
    }
    else
    {
        return -1;
    }

    ret = i2c_transfer(sensor_data->lm75a_info->adapter,&msg[0],1);

    return ret;
}   

int lm75a_reg_read(struct lm75a_info* sensor_data, unsigned char reg_type)
{
    int ret = 0;

    if(sensor_data->preset_reg != reg_type)
    {
        ret = reg_read_with_pointer_byte(sensor_data, reg_type);
    }
    else
    {
        ret = reg_read_preset(sensor_data, reg_type);
    }

    if(ret > 0)
        sensor_data->preset_reg = reg_type;

    return ret;
}

int lm75a_reg_write(struct lm75a_info* sensor_data, unsigned char reg_type)
{
    int ret                 = 0;
    struct i2c_msg msg[2]   = {0,};

    sensor_data->write_val[0] = reg_type;
    msg[0].addr         = sensor_data->lm75a_info->addr;
    msg[0].buf          = sensor_data->write_val;

    if(reg_type == CONFG_REG)
    {
        msg[0].len          = 2;

    }
    else if(reg_type == TEMP_REG || reg_type == TOS_REG || reg_type == THYST_REG)
    {
        msg[0].len          = 3;

    }
    else
    {
        return -1;
    }
    
    ret = i2c_transfer(sensor_data->lm75a_info->adapter,&msg[0],1);

    return ret;
}

module_init(lm75a_driver_init);
module_exit(lm75a_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("<djs6421@gmail.com>");
MODULE_DESCRIPTION("lm75a Device Driver");
MODULE_VERSION("0.01");