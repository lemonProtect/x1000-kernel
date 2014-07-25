#include <jz_notifier.h>

static BLOCKING_NOTIFIER_HEAD(reset_notifier_chain);

/**
 *	reset_notifier_client_register - register a client notifier
 *	@nb: notifier block to callback on events
 */
int reset_notifier_client_register(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&reset_notifier_chain, nb);
}

/**
 *	reset_notifier_client_unregister - unregister a client notifier
 *	@nb: notifier block to callback on events
 */
int reset_notifier_client_unregister(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&reset_notifier_chain, nb);
}

/**
 * reset_notifier_call_chain - notify clients of reset_events
 *
 */
int reset_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&reset_notifier_chain, val, v);
}
