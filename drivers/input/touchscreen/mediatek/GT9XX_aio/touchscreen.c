#include <linux/module.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include "touchscreen.h"

touchscreen_ops_tpye *touchscreen_ops[2];

#define TOUCH_IN_ACTIVE(num) (touchscreen_ops[num] && touchscreen_ops[num]->active && touchscreen_ops[num]->active())
static DEFINE_MUTEX(touchscreen_mutex);

int touchscreen_set_ops(touchscreen_ops_tpye *ops)
{
    if(ops==NULL || ops->touch_id>1 )
    {
        printk("BJ_BSP_Driver:CP_Touchscreen:ops error!\n");
        return -EBUSY;
    }
    mutex_lock(&touchscreen_mutex);
    if(touchscreen_ops[ops->touch_id]!=NULL)
    {
        printk("BJ_BSP_Driver:CP_Touchscreen:ops has been used!\n");
        mutex_unlock(&touchscreen_mutex);
        return -EBUSY;
    }
       touchscreen_ops[ops->touch_id] = ops;
    mutex_unlock(&touchscreen_mutex);
    printk("BJ_BSP_Driver:CP_Touchscreen:ops add success!\n");
    return 0;
}

static ssize_t  touchscreen_active_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret=0;
    int ret1=0;

    if(buf==NULL)
    {
        printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
        return -ENOMEM;
    }

    mutex_lock(&touchscreen_mutex);
    if (touchscreen_ops[0] && touchscreen_ops[0]->active)
    {
        ret = touchscreen_ops[0]->active();
    }

    if (touchscreen_ops[1] && touchscreen_ops[1]->active)
    {
        ret1 = touchscreen_ops[1]->active();
    }
    mutex_unlock(&touchscreen_mutex);

    printk("BJ_BSP_Driver:CP_Touchscreen:%d,%d in %s\n",ret,ret1,__FUNCTION__);
    return sprintf(buf, "%d,%d\n",ret,ret1);
}

static ssize_t  touchscreen_gesture_wakeup_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    char gesture[64]={0};

    if(buf==NULL)
    {
        printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
        return -ENOMEM;
    }

    mutex_lock(&touchscreen_mutex);
    if(TOUCH_IN_ACTIVE(0))
    {
        if(touchscreen_ops[0]->get_wakeup_gesture)
            touchscreen_ops[0]->get_wakeup_gesture(gesture);
    }
    else if(TOUCH_IN_ACTIVE(1))
    {
        if(touchscreen_ops[1]->get_wakeup_gesture)
            touchscreen_ops[1]->get_wakeup_gesture(gesture);
    }
   mutex_unlock(&touchscreen_mutex);

    return sprintf(buf, "%s\n",gesture);
}

static ssize_t  touchscreen_gesture_ctrl_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    char gesture_ctrl[64]={0};

    if(buf==NULL)
    {
        printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
        return -ENOMEM;
    }

    mutex_lock(&touchscreen_mutex);
    if(TOUCH_IN_ACTIVE(0))
    {
        if(touchscreen_ops[0]->get_gesture_ctrl)
            touchscreen_ops[0]->get_gesture_ctrl(gesture_ctrl);
    }
    else if(TOUCH_IN_ACTIVE(1))
    {
        if(touchscreen_ops[1]->get_gesture_ctrl)
            touchscreen_ops[1]->get_gesture_ctrl(gesture_ctrl);
    }
    mutex_unlock(&touchscreen_mutex);

    return sprintf(buf, "%s\n",gesture_ctrl);
}

static ssize_t  touchscreen_gesture_ctrl_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
    int ret=0;
    if(buf==NULL)
    {
        printk("BJ_BSP_Driver:CP_Touchscreen:buf is NULL!\n");
        return -ENOMEM;
    }

    printk("%s: count = %d.\n", __func__, (int)count);
    mutex_lock(&touchscreen_mutex);
    if(TOUCH_IN_ACTIVE(0))
    {
        if(touchscreen_ops[0]->gesture_ctrl)
            ret = touchscreen_ops[0]->gesture_ctrl(buf);
    }
    else if(TOUCH_IN_ACTIVE(1))
    {
        if(touchscreen_ops[1]->gesture_ctrl)
            ret = touchscreen_ops[1]->gesture_ctrl(buf);
    }
    mutex_unlock(&touchscreen_mutex);
    return count;
}

static DEVICE_ATTR(active, 0666, touchscreen_active_show, NULL);
static DEVICE_ATTR(gesture_wakeup, 0666, touchscreen_gesture_wakeup_show, NULL);//444
static DEVICE_ATTR(gesture_ctrl, 0666, touchscreen_gesture_ctrl_show, touchscreen_gesture_ctrl_store);//222


static const struct attribute *touchscreen_attrs[] = {
    &dev_attr_active.attr,
    &dev_attr_gesture_wakeup.attr,
    &dev_attr_gesture_ctrl.attr,
    NULL,
};

static const struct attribute_group touchscreen_attr_group = {
    .attrs = (struct attribute **) touchscreen_attrs,
};

static ssize_t export_store(struct class *class, struct class_attribute *attr, const char *buf, size_t len)
{
    return 1;
}

static ssize_t unexport_store(struct class *class, struct class_attribute *attr,const char *buf, size_t len)
{
    return 1;
}

static struct class_attribute uart_class_attrs[] = {
    __ATTR(export, 0200, NULL, export_store),
    __ATTR(unexport, 0200, NULL, unexport_store),
    __ATTR_NULL,
};

static struct class touchscreen_class = {
    .name =     "touchscreen",
    .owner =    THIS_MODULE,

    .class_attrs =  uart_class_attrs,
};

static struct device *touchscreen_dev;
struct device *touchscreen_get_dev(void)
{
    return touchscreen_dev;
}
EXPORT_SYMBOL(touchscreen_get_dev);



static bool current_status = 1;

struct device *dev = NULL;

void touch_toggle(bool t)
{
    int rc = 0;
    if (t && (current_status == 0))
    {
        current_status = 1;
        rc = sysfs_create_group(&dev->kobj, &touchscreen_attr_group);
    }
    else if ((!t) && (current_status == 1))
    {
        current_status = 0;
        sysfs_remove_group(&dev->kobj, &touchscreen_attr_group);      
    }
}




static int touchscreen_export(void)
{
    int status = 0;
    //struct device   *dev = NULL;

    dev = device_create(&touchscreen_class, NULL, MKDEV(0, 0), NULL, "touchscreen_dev");
    if (dev)
    {
        status = sysfs_create_group(&dev->kobj, &touchscreen_attr_group);
        touchscreen_dev = dev;
    }
    else
    {
        printk(KERN_ERR"BJ_BSP_Driver:CP_Touchscreen:uart switch sysfs_create_group fail\r\n");
        status = -ENODEV;
    }

    return status;
}

static int __init touchscreen_sysfs_init(void)
{
    int status = 0;
    touchscreen_ops[0]=NULL;
    touchscreen_ops[1]=NULL;
    status = class_register(&touchscreen_class);
    if (status < 0)
    {
        printk(KERN_ERR"BJ_BSP_Driver:CP_Touchscreen:uart switch class_register fail\r\n");
        return status;
    }

    status = touchscreen_export();

    return status;
}

arch_initcall(touchscreen_sysfs_init);

