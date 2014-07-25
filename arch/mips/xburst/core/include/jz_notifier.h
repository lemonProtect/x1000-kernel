#ifndef _JZ_NOTIFIER_H_
#define _JZ_NOTIFIER_H_

#include <linux/notifier.h>

/* Hibernation and suspend events */
#define JZ_HIBERNATION_PREPARE	0x0001 /* Going to hibernate */
#define JZ_POST_HIBERNATION	0x0002 /* Hibernation finished */
#define JZ_SUSPEND_PREPARE	0x0003 /* Going to suspend the system */
#define JZ_POST_SUSPEND		0x0004 /* Suspend finished */
#define JZ_RESTORE_PREPARE	0x0005 /* Going to restore a saved image */
#define JZ_POST_RESTORE		0x0006 /* Restore failed */

/**
 *	reset_notifier_client_register - register a client notifier
 *	@nb: notifier block to callback on events
 */
int reset_notifier_client_register(struct notifier_block *nb);

/**
 *	reset_notifier_client_unregister - unregister a client notifier
 *	@nb: notifier block to callback on events
 */
int reset_notifier_client_unregister(struct notifier_block *nb);

/**
 * reset_notifier_call_chain - notify clients of reset_events
 *
 */
int reset_notifier_call_chain(unsigned long val, void *v);

#endif /* _JZ_NOTIFIER_H_ */
