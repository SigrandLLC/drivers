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
#endif
static int xmit_debug=0;
static int recv_debug=10;
static int irq_debug=0;
#endif		    
