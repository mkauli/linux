/*
 * pust.c
 *
 * Establish event trigger between PRU and host user-space application.
 *
 *  Created on: 01.02.2018
 *      Author: Martin Kaul <martin@familie-kaul.de>
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/slab.h>

//== trigger handling ==============================================================================
static int export_var = 0;
static int unexport_var = 0;
static int trigger_var = 0;

#define NUMBER_OF_TRIGGER 4

/*
 * Defines the attribute structure for pust trigger handling
 */
struct pust_attribute
{
	struct kobj_attribute kobj_attr;
	ssize_t number;
	struct kobject *kobj;
};
static struct pust_attribute * pust_trigger_attrs[NUMBER_OF_TRIGGER];

static ssize_t trigger_show( struct kobject *kobj, struct kobj_attribute *attr, char *buf );
static ssize_t trigger_store( struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count );

static struct kobject *pust_module;
static struct platform_device *pust_pdev = NULL;

/*
 * interrupt function triggered the pru triggers an host sys-event
 */
static irqreturn_t triggerISR( int irq, void *data )
{
	struct pust_attribute *pust_trigger_attr;
	struct kobject * kobj;

	if( data == NULL )
	{
		return IRQ_HANDLED;
	}

	pust_trigger_attr = (struct pust_attribute *)data;
	kobj = pust_trigger_attr->kobj;

	if( kobj == NULL )
	{
		printk(KERN_ERR "pust: cannot notify user-space - no kernel-object initialized. Dummy Read/write trigger device to initialize kernel-object.\n");
		return IRQ_HANDLED;
	}

	sysfs_notify( kobj, NULL, pust_trigger_attr->kobj_attr.attr.name );

	return IRQ_HANDLED;
}

/*
 * Releases all resources created while initialize trigger sysfs file.
 */
static void releaseTriggerSysFS( int trigger )
{
	struct pust_attribute * pust_attr;
	if( trigger >= NUMBER_OF_TRIGGER )
	{
		return;
	}
	pust_attr = pust_trigger_attrs[trigger];
	if( pust_attr == NULL )
	{
		return;
	}

	pust_trigger_attrs[trigger] = NULL;
	kfree( pust_attr->kobj_attr.attr.name );
	kfree( pust_attr );
}

/*
 * Create the trigger corresponding sysfs file.
 */
static ssize_t createTriggerSysFS( int trigger )
{
	int error;
	struct pust_attribute * pust_attr;

	if( trigger >= NUMBER_OF_TRIGGER )
	{
		return -EPERM;
	}
	if( pust_trigger_attrs[trigger] != NULL )
	{
		printk( KERN_ERR "pust: trigger already registered\n" );
		return -EPERM;
	}

	// create memory for sysfs file attribute
	pust_attr = kmalloc( sizeof(struct kobj_attribute), GFP_ATOMIC );
	if( pust_attr == NULL )
	{
		return -ENOMEM;
	}
	pust_trigger_attrs[trigger] = pust_attr;

	memset( pust_attr, 0, sizeof(struct kobj_attribute) );
	pust_attr->kobj_attr.attr.name = kmalloc( 9, GFP_ATOMIC );

	// initialize kernel attributes for new trigger sysfs file
	pust_attr->kobj_attr.attr.mode = 0644;
	sprintf( (char*) pust_attr->kobj_attr.attr.name, "trigger%d", trigger );
	pust_attr->kobj_attr.show = trigger_show;
	pust_attr->kobj_attr.store = trigger_store;
	pust_attr->number = trigger;
	pust_attr->kobj = NULL;

	// create sysfs irq file
	error = sysfs_create_file( pust_module, &pust_attr->kobj_attr.attr );
	if( error )
	{
		printk( KERN_ERR "pust: failed to create the trigger file in /sys/kernel/pust\n" );
		releaseTriggerSysFS( trigger );
		return -EPERM;
	}

	return 0;
}

/*
 * destroys the irq corresponding sysfs file
 */
static ssize_t destroyTriggerSysFS( int trigger )
{
	struct pust_attribute * pust_attr;
	if( trigger >= NUMBER_OF_TRIGGER )
	{
		return -EPERM;
	}
	pust_attr = pust_trigger_attrs[trigger];
	if( pust_attr == NULL )
	{
		printk( KERN_ERR "pust: trigger not registered\n" );
		return -EPERM;
	}

	// remove sysfs trigger file
	sysfs_remove_file( pust_module, &pust_attr->kobj_attr.attr );
	releaseTriggerSysFS( trigger );

	return 0;
}

/*
 * Removes the given trigger.
 */
static ssize_t removeTrigger( int trigger )
{
	int irq;
	struct pust_attribute * pust_attr;
	if( trigger >= NUMBER_OF_TRIGGER )
	{
		return -EPERM;
	}
	pust_attr = pust_trigger_attrs[trigger];
	if( pust_attr == NULL )
	{
		return -EPERM;
	}

	/* unregister interrupt */
    irq = platform_get_irq_byname(pust_pdev, pust_attr->kobj_attr.attr.name);
	free_irq( irq, pust_attr );

	/* delete the corresponding sysfs file. */
	return destroyTriggerSysFS(trigger);
}

static ssize_t export_show( struct kobject *kobj, struct kobj_attribute *attr, char *buf )
{
	// outputs the list of already registered interrupts
	int size = 0;
	int count = 0;
	int i;
	for( i = 0; i < NUMBER_OF_TRIGGER; i++ )
	{
		if( pust_trigger_attrs[i] != NULL )
		{
			size += sprintf( buf, "%d;", pust_trigger_attrs[i]->number );
			count += size;
			buf += size;
		}
	}

	size += sprintf( buf, "\n" );
	count += size;

	return count;
}

static ssize_t export_store( struct kobject *kobj, struct kobj_attribute *attr, char *buf, size_t count )
{
	int error = 0;
	int irq;
	struct pust_attribute * pust_attr;
	int irqflags = 0;

	if( pust_pdev == NULL )
	{
		printk(KERN_ERR "pust: driver not already initialized\n");
		return -EIO;
	}

	/* get trigger number from sysfs file */
	sscanf( buf, "%du", &export_var );

	/* create the corresponding sysfs file. */
	error = createTriggerSysFS( export_var );
	if( error != 0 )
	{
		return error;
	}
	pust_attr = pust_trigger_attrs[export_var];

	/* initialize interrupt flags */
	irqflags |= IRQF_TRIGGER_RISING;
	irqflags |= IRQF_ONESHOT;
	irqflags |= IRQF_SHARED;

	/* register interrupt */
    irq = platform_get_irq_byname(pust_pdev, pust_attr->kobj_attr.attr.name);
	error = request_threaded_irq(irq, NULL, triggerISR, irqflags, dev_name(&pust_pdev->dev), pust_attr);
	if( error != 0 )
	{
		printk(KERN_ERR "pust: cannot register trigger IRQ %d (error=%d)\n", export_var, error);
		destroyTriggerSysFS( export_var );
		return -EIO;
	}

	return count;
}

static ssize_t unexport_show( struct kobject *kobj, struct kobj_attribute *attr, char *buf )
{
	// outputs the list of already registered interrupts
	int size = 0;
	int count = 0;
	int i;
	for( i = 0; i < NUMBER_OF_TRIGGER; i++ )
	{
		if( pust_trigger_attrs[i] != NULL )
		{
			size += sprintf( buf, "%d;", pust_trigger_attrs[i]->number );
			count += size;
			buf += size;
		}
	}

	size += sprintf( buf, "\n" );
	count += size;

	return count;
}

static ssize_t unexport_store( struct kobject *kobj, struct kobj_attribute *attr, char *buf, size_t count )
{
	int error;

	if( pust_pdev == NULL )
	{
		printk(KERN_ERR "pust: driver not already initialized\n");
		return -EIO;
	}

	/* get irq number from sysfs file */
	sscanf( buf, "%du", &unexport_var );

	error = removeTrigger( export_var );
	if( error != 0 )
	{
		return error;
	}
	return count;
}

static ssize_t trigger_show( struct kobject *kobj, struct kobj_attribute *attr, char *buf )
{
	struct pust_attribute * pust_attr = (struct pust_attribute *) attr;
	if( pust_attr == NULL )
	{
		return -EPERM;
	}
	pust_attr->kobj = kobj;

	return sprintf( buf, "%d\n", pust_attr->number );
}

static ssize_t trigger_store( struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count )
{
	struct pust_attribute * pust_attr = (struct pust_attribute *) attr;
	if( pust_attr == NULL )
	{
		return -EPERM;
	}
	pust_attr->kobj = kobj;

	sscanf( buf, "%du", &trigger_var );
	return count;
}

static struct kobj_attribute pust_attrs[] =
{
  __ATTR(export, 0644, export_show, (void*)export_store),
  __ATTR(unexport, 0644, unexport_show, (void*)unexport_store),
  __ATTR_NULL
};

static int pust_init (void)
{
    int error = 0;

    printk( KERN_INFO "pust: initialised\n" );
    pust_module = kobject_create_and_add("pust", kernel_kobj);
    if (pust_module == NULL)
    {
    	return -ENOMEM;
    }

    error = sysfs_create_file(pust_module, &pust_attrs[0].attr);
    if (error)
    {
    	printk( KERN_ERR "pust: failed to create the export file in /sys/kernel/pust\n" );
    }
    error = sysfs_create_file(pust_module, &pust_attrs[1].attr);
    if (error)
    {
    	printk( KERN_ERR "pust: failed to create the unexport file in /sys/kernel/ssp_pru_host\n" );
    }

    // initialize all irq sysfs file attributes
    memset( &pust_trigger_attrs[0], 0, sizeof(pust_trigger_attrs) );

    return error;
}

static void pust_exit (void)
{
	int i;

	printk( KERN_INFO "pust: Exit success\n" );

    // de-initialize all irq sysfs file attributes
    for( i=0; i<NUMBER_OF_TRIGGER; i++ )
    {
    	if( pust_trigger_attrs[i] != NULL )
    	{
    		removeTrigger( i );
    	}
    }

    kobject_put(pust_module);
}

//== Driver handling ==============================================================================

/*
 * Will be called when the driver is installed.
 *
 * Checks if the PRU host interrupts are available and stores the corresponding
 * interrupt numbers.
 */
static int pust_probe(struct platform_device *pdev)
{
	printk( KERN_INFO "pust: pust_probe()\n" );

	pust_pdev = pdev;
	pust_init();

	return 0;
}

/*
 * Will be called when the driver is removed.
 *
 * It frees eventual registered interrupts.
 */
static int pust_remove(struct platform_device *pdev)
{
	if( pust_pdev == NULL )
	{
		return 0;
	}

	printk( KERN_INFO "pust: pust_remove()\n" );

	pust_exit();
	pust_pdev = NULL;
	return 0;
}

static const struct of_device_id pust_match_table[] = {
	{
		.compatible = "pust",
		.data = NULL,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, pust_match_table);

static struct platform_driver pust_platform_driver = {
	.driver = {
		.name = "pust",
		.of_match_table = pust_match_table,
	},
	.probe  = pust_probe,
	.remove = pust_remove,
};
module_platform_driver(pust_platform_driver);

MODULE_AUTHOR("Martin Kaul <martin@familie-kaul.de>");
MODULE_DESCRIPTION("PRU-Userspace-Trigger");
MODULE_LICENSE("GPL v2");
