#ifndef SG_DEBUG_H
#define SG_DEBUG_H

#ifndef DEFAULT_LEV 
#	define DEFAULT_LEV 0
#endif

#define PDEBUG(lev,fmt,args...)
#ifdef DEBUG_ON
#       undef PDEBUG
#       define PDEBUG(lev,fmt,args...) \
		if( lev<=DEFAULT_LEV ) \
			printk(KERN_NOTICE "sg16lan: %s " fmt " \n",__FUNCTION__, ## args  )
static int xmit_debug=10;
static int recv_debug=10;
static int irq_debug=10;
static int debug_init=10;
static int debug_ioctl=10;
static int debug_hw=10;
static int debug_procfs=10;


#endif
#endif		    
