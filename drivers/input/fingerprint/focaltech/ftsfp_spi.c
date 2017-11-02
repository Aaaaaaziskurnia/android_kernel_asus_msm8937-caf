/*Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *     Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
#include <linux/sched.h>

#include "ftsfp_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

#define FTSFP_SPIDEV_NAME       "focal,fingerprint"
/*device name after register in charater*/
#define FTSFP_DEV_NAME          "focal_fp"
#define FTSFP_INPUT_NAME        "qwerty"

#define CHRD_DRIVER_NAME        "focal_fp_spi"
#define CLASS_NAME              "focal_fp"
#define SPIDEV_MAJOR            154 /* assigned */
#define N_SPI_MINORS            32  /* ... up to 256 */

static int fp_get_io_resource(void);
static int boost_flag;

struct ftsfp_key_map_ft key_map_ft[] =
{
    {  "POWER",  KEY_POWER  },
    {  "HOME" ,  KEY_HOME   },
    {  "MENU" ,  KEY_MENU   },
    {  "BACK" ,  KEY_BACK   },
    {  "UP"   ,  KEY_UP     },
    {  "DOWN" ,  KEY_DOWN   },
    {  "LEFT" ,  KEY_LEFT   },
    {  "RIGHT",  KEY_RIGHT  },
    {  "FORCE",  KEY_F9     },
    {  "CLICK",  KEY_F19    },
};

/*Global variables*/
/*static MODE g_mode = FTSFP_IMAGE_MODE;*/
static DECLARE_BITMAP (minors, N_SPI_MINORS);
static LIST_HEAD (device_list);
static DEFINE_MUTEX (device_list_lock);
static struct ftsfp_dev g_ftsfp_dev;

static void ftsfp_enable_irq (struct ftsfp_dev *ftsfp_dev)
{
    if (ftsfp_dev->irq_enabled) {
        ftsfp_dbg ("IRQ has been enabled.\n");
    }
    else {
        enable_irq(ftsfp_dev->irq);
        ftsfp_dev->irq_enabled = 1;
	ftsfp_dbg ("IRQ enabled.\n");
    }
}

static void ftsfp_disable_irq (struct ftsfp_dev *ftsfp_dev)
{
    if (ftsfp_dev->irq_enabled) {
        ftsfp_dev->irq_enabled = 0;
        disable_irq(ftsfp_dev->irq);
    }
    else {
        ftsfp_dbg ("IRQ has been disabled.\n");
    }
}

#ifdef AP_CONTROL_CLK
static long spi_clk_max_rate (struct clk *clk, unsigned long rate)
{
    long lowest_available, nearest_low, step_size, cur;
    long step_direction = -1;
    long guess = rate;
    int max_steps = 10;

    cur = clk_round_rate (clk, rate);

    if (cur == rate)
    { return rate; }

    /* if we got here then: cur > rate */
    lowest_available = clk_round_rate (clk, 0);

    if (lowest_available > rate)
    { return -EINVAL; }

    step_size = (rate - lowest_available) >> 1;
    nearest_low = lowest_available;

    while (max_steps-- && step_size)
    {
        guess += step_size * step_direction;
        cur = clk_round_rate (clk, guess);

        if ((cur < rate) && (cur > nearest_low))
        { nearest_low = cur; }

        /*
         * if we stepped too far, then start stepping in the other
         * direction with half the step size
         */
        if (((cur > rate) && (step_direction > 0))
                || ((cur < rate) && (step_direction < 0)))
        {
            step_direction = -step_direction;
            step_size >>= 1;
        }
    }

    return nearest_low;
}

static void spi_clock_set (struct ftsfp_dev *ftsfp_dev, int speed)
{
    long rate;
    int rc;

    rate = spi_clk_max_rate (ftsfp_dev->core_clk, speed);

    if (rate < 0)
    {
        ftsfp_dbg ("%s: no match found for requested clock frequency:%d",
                 __func__, speed);
        return;
    }

    rc = clk_set_rate (ftsfp_dev->core_clk, rate);
}

static int ftsfp_spi_clk_init (struct ftsfp_dev *data)
{
    ftsfp_dbg ("%s: enter\n", __func__);

    data->clk_enabled = 0;
    data->core_clk = clk_get (&data->spi->dev, "core_clk");

    if (IS_ERR_OR_NULL (data->core_clk))
    {
        ftsfp_dbg ("%s: fail to get core_clk\n", __func__);
        return -1;
    }

    data->iface_clk = clk_get (&data->spi->dev, "iface_clk");

    if (IS_ERR_OR_NULL (data->iface_clk))
    {
        ftsfp_dbg ("%s: fail to get iface_clk\n", __func__);
        clk_put (data->core_clk);
        data->core_clk = NULL;
        return -2;
    }

    return 0;
}

static int ftsfp_spi_clk_enable (struct ftsfp_dev *data)
{
    int err;

    ftsfp_dbg ("%s: enter\n", __func__);

    if (data->clk_enabled)
    { return 0; }

    err = clk_prepare_enable (data->core_clk);

    if (err)
    {
        ftsfp_dbg ("%s: fail to enable core_clk\n", __func__);
        return -1;
    }

    err = clk_prepare_enable (data->iface_clk);

    if (err)
    {
        ftsfp_dbg ("%s: fail to enable iface_clk\n", __func__);
        clk_disable_unprepare (data->core_clk);
        return -2;
    }

    data->clk_enabled = 1;

    return 0;
}

static int ftsfp_spi_clk_disable (struct ftsfp_dev *data)
{
    ftsfp_dbg ("%s: enter\n", __func__);

    if (!data->clk_enabled)
    { return 0; }

    clk_disable_unprepare (data->core_clk);
    clk_disable_unprepare (data->iface_clk);
    data->clk_enabled = 0;

    return 0;
}

static int ftsfp_spi_clk_exit (struct ftsfp_dev *data)
{
    ftsfp_dbg ("%s: enter\n", __func__);

    if (data->clk_enabled)
    { ftsfp_spi_clk_disable (data); }

    if (!IS_ERR_OR_NULL (data->core_clk))
    {
        clk_put (data->core_clk);
        data->core_clk = NULL;
    }

    if (!IS_ERR_OR_NULL (data->iface_clk))
    {
        clk_put (data->iface_clk);
        data->iface_clk = NULL;
    }

    return 0;
}
#endif

static long ftsfp_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct ftsfp_dev *ftsfp_dev = &g_ftsfp_dev;
    struct ftsfp_key ftsfp_key = { 0 };
    int retval = 0;
    int i;
#ifdef AP_CONTROL_CLK
    unsigned int speed = 0;
#endif
    FUNC_ENTRY();

    if (_IOC_TYPE (cmd) != FTSFP_IOC_MAGIC)
    { return -ENODEV; }

    if (_IOC_DIR (cmd) & _IOC_READ)
        retval =
            !access_ok (VERIFY_WRITE, (void __user *)arg, _IOC_SIZE (cmd));

    if ((retval == 0) && (_IOC_DIR (cmd) & _IOC_WRITE))
        retval =
            !access_ok (VERIFY_READ, (void __user *)arg, _IOC_SIZE (cmd));

    if (retval)
    { return -EFAULT; }

    if (ftsfp_dev->device_available == 0)
    {
        if ((cmd == FTSFP_IOC_POWER_ON) || (cmd == FTSFP_IOC_POWER_OFF))
        {
            ftsfp_dbg ("power cmd\n");
        }
        else
        {
            ftsfp_dbg ("Sensor is power off currently. \n");
            return -ENODEV;
        }
    }

    switch (cmd)
    {
        case FTSFP_IOC_DISABLE_IRQ:
            ftsfp_disable_irq (ftsfp_dev);
            break;

        case FTSFP_IOC_ENABLE_IRQ:
            ftsfp_enable_irq (ftsfp_dev);
            break;

        case FTSFP_IOC_SETSPEED:
#ifdef AP_CONTROL_CLK
            retval = __get_user (speed, (u32 __user *) arg);

            if (retval == 0)
            {
                if (speed > 8 * 1000 * 1000)
                {
                    ftsfp_dbg ("Set speed:%d is larger than 8Mbps.\n", speed);
                }
                else
                {
                    spi_clock_set (ftsfp_dev, speed);
                }
            }
            else
            {
                ftsfp_dbg ("Failed to get speed from user. retval = %d\n",  retval);
            }

#else
            ftsfp_dbg ("This kernel doesn't support control clk in AP\n");
#endif
            break;

        case FTSFP_IOC_RESET:
            ftsfp_hw_reset (ftsfp_dev, 70);
            break;

        case FTSFP_IOC_COOLBOOT:
            ftsfp_power_off (ftsfp_dev);
            mdelay (5);
            ftsfp_power_on (ftsfp_dev);
            break;

        case FTSFP_IOC_SENDKEY:
			if (copy_from_user(&ftsfp_key, (struct ftsfp_key *)arg, sizeof (struct ftsfp_key)))
			{
				ftsfp_dbg ("Failed to copy data from user space.\n");
				retval = -EFAULT;
				break;
			}
			ftsfp_dbg ("key step 0!!\n");

			for (i = 0; i < ARRAY_SIZE (key_map_ft); i++)
			{  
				ftsfp_dbg ("key step 1!!\n");
				if (key_map_ft[i].val == ftsfp_key.key)
				{
					ftsfp_dbg ("key step ||| key : %d  || value : %d \n", ftsfp_key.key, ftsfp_key.value);
					input_report_key (ftsfp_dev->input, ftsfp_key.key, ftsfp_key.value);
					input_sync (ftsfp_dev->input);
					break;
				}
			}

			if (i == ARRAY_SIZE (key_map_ft))
			{
				ftsfp_dbg ("key %d not support yet \n", ftsfp_key.key);
				retval = -EFAULT;
			}
			break;

        case FTSFP_IOC_CLK_READY:
#ifdef AP_CONTROL_CLK
            ftsfp_spi_clk_enable (ftsfp_dev);
#else
            ftsfp_dbg ("Doesn't support control clock.\n");
#endif
            break;

        case FTSFP_IOC_CLK_UNREADY:
#ifdef AP_CONTROL_CLK
            ftsfp_spi_clk_disable (ftsfp_dev);
#else
            ftsfp_dbg ("Doesn't support control clock.\n");
#endif
            break;

        case FTSFP_IOC_PM_FBCABCK:
            __put_user (ftsfp_dev->fb_black, (u8 __user *) arg);
            break;

        case FTSFP_IOC_POWER_ON:
            if (ftsfp_dev->device_available == 1)
            { ftsfp_dbg ("Sensor has already powered-on.\n"); }
            else
            { ftsfp_power_on (ftsfp_dev); }

            ftsfp_dev->device_available = 1;
            break;

        case FTSFP_IOC_POWER_OFF:
            if (ftsfp_dev->device_available == 0)
            { ftsfp_dbg ("Sensor has already powered-off.\n"); }
            else
            { ftsfp_power_off (ftsfp_dev); }

            ftsfp_dev->device_available = 0;
            break;

        case FTSFP_IOC_GET_MCU_STATUS:
            retval = __put_user(__gpio_get_value(ftsfp_dev->irq_gpio), (u8 __user *)arg);
            break;

	case FTSFP_IOC_GET_IO_RESOURCE:
		#if 0
		retval = gpio_request(ftsfp_dev->reset_gpio, "focal_reset");
		if(retval) {
			ftsfp_dbg("Failed to request RESET GPIO. retval = %d\n", retval);
			retval = __put_user(-1, (__u8 __user *)arg);
		}
		gpio_direction_output(ftsfp_dev->reset_gpio, 1);

		retval = gpio_request(ftsfp_dev->irq_gpio, "focal_irq");
		if(retval) {
			dev_err(&ftsfp_dev->spi->dev, "Failed to request IRQ GPIO. rc = %d\n", retval);
			retval = __put_user(-1, (__u8 __user *)arg);
		}
		gpio_direction_input(ftsfp_dev->irq_gpio);
		#endif
		fp_get_io_resource();
		break;

        default:
            ftsfp_dbg ("Unsupport cmd:0x%x\n", cmd);
            break;
    }

    FUNC_EXIT();
    return retval;
}

#ifdef CONFIG_COMPAT
static long ftsfp_compat_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
    return ftsfp_ioctl (filp, cmd, (unsigned long)compat_ptr (arg));
}
#endif /*CONFIG_COMPAT*/

static irqreturn_t ftsfp_irq (int irq, void *handle)
{
    struct ftsfp_dev *ftsfp_dev = &g_ftsfp_dev;
#if 0//zengfl    
#ifdef FTSFP_FASYNC

    if (ftsfp_dev->async)
    { kill_fasync (&ftsfp_dev->async, SIGIO, POLL_IN); }

#endif
#else
    char *envp[2];

    envp[0] = "FOCAL=fingeron";     
    envp[1] = NULL;

	ftsfp_dbg("enter irq %s,boost_flag = %d\n",__func__,boost_flag);
	if(boost_flag == 1)
	{
		ftsfp_dbg("sched_set_boost value :1\n");
		sched_set_boost(1);
	}

	wake_lock_timeout(&ftsfp_dev->fts_wlock, msecs_to_jiffies(1000));
    kobject_uevent_env(&ftsfp_dev->spi->dev.kobj, KOBJ_CHANGE, envp);
#endif

    return IRQ_HANDLED;
}

static void ftsfp_screen_detect(bool screenon)
{
    struct ftsfp_dev *ftsfp_dev = &g_ftsfp_dev;
    char *envp[2];

	if(screenon)
	{
	    envp[0] = "FOCAL=screenon";
	}
	else
	{
	    envp[0] = "FOCAL=screenoff";
	}
	
    envp[1] = NULL;

    kobject_uevent_env(&ftsfp_dev->spi->dev.kobj, KOBJ_CHANGE, envp);
}

static int ftsfp_open (struct inode *inode, struct file *filp)
{
    struct ftsfp_dev *ftsfp_dev;
    int status = -ENXIO;

    FUNC_ENTRY();
    mutex_lock (&device_list_lock);

    list_for_each_entry (ftsfp_dev, &device_list, device_entry)
    {
        if (ftsfp_dev->devt == inode->i_rdev)
        {
            ftsfp_dbg ("Device Found\n");
            status = 0;
            break;
        }
    }

    if (status == 0)
    {
        if (status == 0)
        {
            ftsfp_dev->users++;
            filp->private_data = ftsfp_dev;
            nonseekable_open (inode, filp);
            ftsfp_dbg ("Succeed to open device. irq = %d\n", ftsfp_dev->irq);
#if 0
            if (ftsfp_dev->users == 1)
            { ftsfp_enable_irq (ftsfp_dev); }
#endif
            /*power the sensor*/
            //ftsfp_power_on (ftsfp_dev);
            //ftsfp_hw_reset (ftsfp_dev, 360);
            ftsfp_dev->device_available = 1;
        }
    }
    else
    {
        ftsfp_dbg ("No device for minor %d\n", iminor (inode));
    }

    mutex_unlock (&device_list_lock);
    FUNC_EXIT();
    return status;
}

static int fp_get_io_resource(void)
{
	int ret;

	struct ftsfp_dev *dev;
	dev = &g_ftsfp_dev;
	if (ftsfp_parse_dts (&g_ftsfp_dev))
	{
		ftsfp_dbg("ftsfp_parse_dts error...\n");
	       return -1;
	}

	g_ftsfp_dev.irq = ftsfp_irq_num (&g_ftsfp_dev);
	ret = request_threaded_irq (g_ftsfp_dev.irq, NULL, ftsfp_irq,
	                            IRQF_TRIGGER_RISING | IRQF_ONESHOT,
	                            "ftsfp", &g_ftsfp_dev);
    ftsfp_dbg("ftsfp_probe: g_ftsfp_dev.irq_enabled = %d\n", g_ftsfp_dev.irq_enabled);

	if (!ret)
	{
		g_ftsfp_dev.irq_enabled = 1;
        enable_irq_wake (g_ftsfp_dev.irq);
        ftsfp_disable_irq (&g_ftsfp_dev);
	    ftsfp_dbg("ftsfp request_threaded_irq success!\n");
	}
	wake_lock_init(&dev->fts_wlock, WAKE_LOCK_SUSPEND, "focalfp_irq_wakelock");
	ftsfp_dbg("ftsfp fp_get_io_resource  end__ !\n");
	return 0;
}

#if 0//zengfl
#ifdef FTSFP_FASYNC
static int ftsfp_fasync (int fd, struct file *filp, int mode)
{
    struct ftsfp_dev *ftsfp_dev = filp->private_data;
    int ret;

    FUNC_ENTRY();
    ret = fasync_helper (fd, filp, mode, &ftsfp_dev->async);
    FUNC_EXIT();
    ftsfp_dbg ("ret = %d\n", ret);
    return ret;
}
#endif
#endif

static int ftsfp_release (struct inode *inode, struct file *filp)
{
    struct ftsfp_dev *ftsfp_dev;
    int status = 0;

    FUNC_ENTRY();
    mutex_lock (&device_list_lock);
    ftsfp_dev = filp->private_data;
    filp->private_data = NULL;

    /*last close?? */
    ftsfp_dev->users--;

    if (!ftsfp_dev->users)
    {
        ftsfp_dbg ("disble_irq. irq = %d\n", ftsfp_dev->irq);
        ftsfp_disable_irq (ftsfp_dev);
        /*power off the sensor*/
        ftsfp_dev->device_available = 0;
        ftsfp_power_off (ftsfp_dev);
    }

    mutex_unlock (&device_list_lock);
    FUNC_EXIT();
    return status;
}

static const struct file_operations ftsfp_fops =
{
    .owner = THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
    */
    .unlocked_ioctl = ftsfp_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = ftsfp_compat_ioctl,
#endif /*CONFIG_COMPAT*/
    .open = ftsfp_open,
    .release = ftsfp_release,
#if 0//zengfl    
#ifdef FTSFP_FASYNC
    .fasync = ftsfp_fasync,
#endif
#endif
};

#if 1//zengfl
static int ftsfp_fb_state_chg_callback (struct notifier_block *nb,
        unsigned long val, void *data)
{
    struct ftsfp_dev *ftsfp_dev;
    struct fb_event *evdata = data;
    unsigned int blank;

    if (val != FB_EARLY_EVENT_BLANK)
    { return 0; }

    ftsfp_dbg ("[info] %s go to the ftsfp_fb_state_chg_callback value = %d\n",
             __func__, (int)val);
    ftsfp_dev = container_of (nb, struct ftsfp_dev, notifier);

    if (evdata && evdata->data && val == FB_EARLY_EVENT_BLANK && ftsfp_dev)
    {
        blank = * (int *) (evdata->data);

        switch (blank)
        {
            case FB_BLANK_POWERDOWN:
                ftsfp_dbg ("%s : FB_BLANK_POWERDOWN\n", __func__);

				boost_flag = 1;

				if (ftsfp_dev->device_available == 1)
                {
						ftsfp_dev->fb_black = 1;
						ftsfp_screen_detect(0);
#ifdef FTSFP_FASYNC

                    if (ftsfp_dev->async)
                    {
                        kill_fasync (&ftsfp_dev->async, SIGIO, POLL_IN);
                    }

#endif
                    /*device unavailable */
                    //ftsfp_dev->device_available = 0;
                }

                break;

            case FB_BLANK_UNBLANK:
                ftsfp_dbg ("%s : FB_BLANK_UNBLANK\n", __func__);

				if(boost_flag == 1)
				{
					sched_set_boost(0);
					boost_flag = 0;
					ftsfp_dbg("sched_set_boost value 0\n");
				}

                if (ftsfp_dev->device_available == 1)
                {
                    ftsfp_dev->fb_black = 0;
					ftsfp_screen_detect(1);
#ifdef FTSFP_FASYNC

                    if (ftsfp_dev->async)
                    {
                        kill_fasync (&ftsfp_dev->async, SIGIO, POLL_IN);
                    }

#endif
                    /*device available */
                    //ftsfp_dev->device_available = 1;
                }

                break;

            default:
                ftsfp_dbg ("%s defalut\n", __func__);
                break;
        }
    }

    return NOTIFY_OK;
}

static struct notifier_block ftsfp_noti_block =
{
    .notifier_call = ftsfp_fb_state_chg_callback,
};
#endif

static void ftsfp_reg_key_kernel (struct ftsfp_dev *ftsfp_dev)
{
    int i;

    set_bit (EV_KEY, ftsfp_dev->input->evbit); //tell the kernel is key event

    for (i = 0; i < ARRAY_SIZE (key_map_ft); i++)
    {
        set_bit (key_map_ft[i].val, ftsfp_dev->input->keybit);
    }

    ftsfp_dev->input->name = FTSFP_INPUT_NAME;

    if (input_register_device (ftsfp_dev->input))
    { ftsfp_dbg ("Failed to register ftsFP as input device.\n"); }
}

static struct class *ftsfp_class;
static int ftsfp_probe (
#if defined(USE_SPI_BUS)
    struct spi_device *spi
#elif defined(USE_PLATFORM_BUS)
    struct platform_device *pdev
#endif
)
{
    struct ftsfp_dev *ftsfp_dev = &g_ftsfp_dev;
    int status = -EINVAL;
    unsigned long minor;
    //int ret;
//    struct regulator *vreg;
    FUNC_ENTRY();

    /* Initialize the driver data */
    INIT_LIST_HEAD (&ftsfp_dev->device_entry);
#if defined(USE_SPI_BUS)
    ftsfp_dev->spi = spi;
#elif defined(USE_PLATFORM_BUS)
    ftsfp_dev->spi = pdev;
#endif
    ftsfp_dev->irq_gpio = -EINVAL;
    ftsfp_dev->reset_gpio = -EINVAL;
    ftsfp_dev->pwr_gpio = -EINVAL;
    ftsfp_dev->device_available = 0;
    ftsfp_dev->fb_black = 0;
    ftsfp_dev->users = 0;

#if 0
    if (ftsfp_parse_dts (ftsfp_dev))
    {
        ftsfp_dbg("ftsfp_parse_dts error...\n");
    	goto error;
    }
#endif
    /*
        if (ftsfp_power_on(ftsfp_dev))
            goto error;
        ftsfp_dev->device_available = 1;
    */
/*
    vreg = regulator_get (&spi->dev, "vdd_ana");

    if (!vreg)
    {
        dev_err (&ftsfp_dev->spi->dev, "Unable to get vdd_ana\n");
        goto error;
    }

    if (regulator_count_voltages (vreg) > 0)
    {
        ret = regulator_set_voltage (vreg, 2800000, 2800000);

        if (ret)
        {
            dev_err (&ftsfp_dev->spi->dev, "Unable to set voltage on vdd_ana");
            goto error;
        }
    }

    ret = regulator_enable (vreg);

    if (ret)
    {
        dev_err (&ftsfp_dev->spi->dev, "error enabling vdd_ana %d\n", ret);
        regulator_put (vreg);
        vreg = NULL;
        goto error;
    }

    dev_info ( && ftsfp_dev->spi->dev, "Set voltage on vdd_ana for focaltech fingerprint");
*/

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
    */
    mutex_lock (&device_list_lock);
    minor = find_first_zero_bit (minors, N_SPI_MINORS);

    if (minor < N_SPI_MINORS)
    {
        struct device *dev;
        ftsfp_dev->devt = MKDEV (SPIDEV_MAJOR, minor);
        dev = device_create (ftsfp_class, &ftsfp_dev->spi->dev, ftsfp_dev->devt,
                             ftsfp_dev, FTSFP_DEV_NAME);
        status = IS_ERR (dev) ? PTR_ERR (dev) : 0;
    }
    else
    {
        ftsfp_dbg("no minor number available!\n");
        status = -ENODEV;
    }

    if (status == 0)
    {
        set_bit (minor, minors);
        list_add (&ftsfp_dev->device_entry, &device_list);
    }
    else
    {
        ftsfp_dbg("device_create error...\n");
        ftsfp_dev->devt = 0;
    }

    mutex_unlock (&device_list_lock);

    if (status == 0)
    {
        /*input device subsystem */
        ftsfp_dev->input = input_allocate_device();

        if (ftsfp_dev->input == NULL)
        {
            ftsfp_dbg("Faile to allocate input device.\n");
            status = -ENOMEM;
        }

#ifdef AP_CONTROL_CLK
        dev_info (&ftsfp_dev->spi->dev, "Get the clk resource.\n");

        /* Enable spi clock */
        if (ftsfp_spi_clk_init (ftsfp_dev))
        { //goto ftsfp_probe_spi_clk_init_failed;
		#ifdef AP_CONTROL_CLK 
		ftsfp_spi_clk_exit (ftsfp_dev);
		#endif
	}

        if (ftsfp_spi_clk_enable (ftsfp_dev))
        { //goto ftsfp_probe_spi_clk_enable_failed; 
	}

        spi_clock_set (ftsfp_dev, 1000000);
#endif

#if 1
        ftsfp_dev->notifier = ftsfp_noti_block;
        fb_register_client (&ftsfp_dev->notifier);
#endif
        ftsfp_reg_key_kernel (ftsfp_dev);
    	input_set_capability(ftsfp_dev->input, EV_KEY, 102);
#if 0
        ftsfp_dev->irq = ftsfp_irq_num (ftsfp_dev);
#if 1
        ret = request_threaded_irq (ftsfp_dev->irq, NULL, ftsfp_irq,
                                    IRQF_TRIGGER_RISING | IRQF_ONESHOT,
                                    "ftsfp", ftsfp_dev);
#else
        ret = request_irq (ftsfp_dev->irq, ftsfp_irq,
                           IRQ_TYPE_EDGE_RISING, /*IRQ_TYPE_LEVEL_HIGH,*/
                           "ftsfp", ftsfp_dev);
#endif

        if (!ret)
        {
            enable_irq_wake (ftsfp_dev->irq);
            ftsfp_disable_irq (ftsfp_dev);
        }
#endif

    }

    return status;

#if 0
error:
    ftsfp_cleanup (ftsfp_dev);
    ftsfp_dev->device_available = 0;

    if (ftsfp_dev->devt != 0)
    {
        dev_info (&ftsfp_dev->spi->dev, "Err: status = %d\n", status);
        mutex_lock (&device_list_lock);
        list_del (&ftsfp_dev->device_entry);
        device_destroy (ftsfp_class, ftsfp_dev->devt);
        clear_bit (MINOR (ftsfp_dev->devt), minors);
        mutex_unlock (&device_list_lock);

#ifdef AP_CONTROL_CLK
ftsfp_probe_spi_clk_enable_failed:
        ftsfp_spi_clk_exit (ftsfp_dev);
ftsfp_probe_spi_clk_init_failed:
#endif

        if (ftsfp_dev->input != NULL)
        { input_unregister_device (ftsfp_dev->input); }
    }
#endif
    FUNC_EXIT();
    return status;
}

/*static int __devexit ftsfp_remove(struct spi_device *spi)*/
static int ftsfp_remove (
#if defined(USE_SPI_BUS)
    struct spi_device *spi
#elif defined(USE_PLATFORM_BUS)
    struct platform_device *pdev
#endif
)
{
    struct ftsfp_dev *ftsfp_dev = &g_ftsfp_dev;
    FUNC_ENTRY();

    /* make sure ops on existing fds can abort cleanly */
    if (ftsfp_dev->irq)
    { free_irq (ftsfp_dev->irq, ftsfp_dev); }

    if (ftsfp_dev->input != NULL)
    { input_unregister_device (ftsfp_dev->input); }

    input_free_device (ftsfp_dev->input);

	wake_lock_destroy(&ftsfp_dev->fts_wlock);

    /* prevent new opens */
    mutex_lock (&device_list_lock);
    list_del (&ftsfp_dev->device_entry);
    device_destroy (ftsfp_class, ftsfp_dev->devt);
    clear_bit (MINOR (ftsfp_dev->devt), minors);

    if (ftsfp_dev->users == 0)
    { kfree (ftsfp_dev); }

    mutex_unlock (&device_list_lock);

    FUNC_EXIT();
    return 0;
}

static int ftsfp_suspend (
#if defined(USE_SPI_BUS)
    struct spi_device *spi, pm_message_t mesg
#elif defined(USE_PLATFORM_BUS)
    struct platform_device *pdev, pm_message_t state
#endif
)
{
#if 0//defined(USE_SPI_BUS)
    struct ftsfp_dev *ftsfp_dev;

    ftsfp_dbg ("%s: enter\n", __func__);

    ftsfp_dev = spi_get_drvdata (spi);
    ftsfp_spi_clk_disable (ftsfp_dev);
#endif
    ftsfp_dbg ("ftsfp_suspend_test.\n");
    return 0;
}

static int ftsfp_resume (
#if defined(USE_SPI_BUS)
    struct spi_device *spi
#elif defined(USE_PLATFORM_BUS)
    struct platform_device *pdev
#endif
)
{
#if 0//defined(USE_SPI_BUS)
    struct ftsfp_dev *ftsfp_dev;

    ftsfp_dbg ("%s: enter\n", __func__);

    ftsfp_dev = spi_get_drvdata (spi);
    ftsfp_spi_clk_enable (ftsfp_dev);
#endif
    ftsfp_dbg ("ftsfp_resume_test.\n");
    return 0;
}

static struct of_device_id ftsfp_of_match[] =
{
    {.compatible = FTSFP_SPIDEV_NAME,},
    {},
};

#if defined(USE_SPI_BUS)
static struct spi_driver ftsfp_drv =
#elif defined(USE_PLATFORM_BUS)
static struct platform_driver ftsfp_drv =
#endif
{

    .driver =
    {
        .name = FTSFP_DEV_NAME,
        .owner = THIS_MODULE,
#if defined(USE_SPI_BUS)
        //  .bus = &spi_bus_type,
#endif
        .of_match_table = ftsfp_of_match,
    },
    .probe   = ftsfp_probe,
    .remove  = ftsfp_remove,
    .suspend = ftsfp_suspend,
    .resume  = ftsfp_resume,
};

static int __init ftsfp_init (void)
{
    int status;
    FUNC_ENTRY();

    /* Claim our 256 reserved device numbers.  Then register a class
     * that will key udev/mdev to add/remove /dev nodes.  Last, register
     * the driver which manages those device numbers.
    */
    BUILD_BUG_ON (N_SPI_MINORS > 256);
    status = register_chrdev (SPIDEV_MAJOR, CHRD_DRIVER_NAME, &ftsfp_fops);

    if (status < 0) {
        ftsfp_dbg("Failed to register char device!\n");
        FUNC_EXIT();
        return status;
    }

    ftsfp_class = class_create (THIS_MODULE, CLASS_NAME);

    if (IS_ERR (ftsfp_class)) {
        unregister_chrdev (SPIDEV_MAJOR, ftsfp_drv.driver.name);
        ftsfp_dbg ("Failed to create class.\n");
        FUNC_EXIT();
        return PTR_ERR (ftsfp_class);
    }

#if defined(USE_PLATFORM_BUS)
    status = platform_driver_register (&ftsfp_drv);
#elif defined(USE_SPI_BUS)
    status = spi_register_driver (&ftsfp_drv);
#endif

    if (status < 0) {
        class_destroy (ftsfp_class);
        unregister_chrdev (SPIDEV_MAJOR, ftsfp_drv.driver.name);
        ftsfp_dbg ("Failed to register SPI driver.\n");
    }

    ftsfp_dbg (" status = 0x%x\n", status);
    FUNC_EXIT();
    return 0;       //status;
}

static void __exit ftsfp_exit (void)
{
    FUNC_ENTRY();
#if defined(USE_PLATFORM_BUS)
    platform_driver_unregister (&ftsfp_drv);
#elif defined(USE_SPI_BUS)
    spi_unregister_driver (&ftsfp_drv);
#endif
    class_destroy (ftsfp_class);
    unregister_chrdev (SPIDEV_MAJOR, ftsfp_drv.driver.name);
    FUNC_EXIT();
}
module_init (ftsfp_init);
module_exit (ftsfp_exit);

MODULE_AUTHOR ("focaltech");
MODULE_DESCRIPTION ("Focaltech Fingerprint chip FT9338/FT9336 TEE driver");
MODULE_LICENSE ("GPL");
MODULE_ALIAS ("spi:ftsFP_spi");
