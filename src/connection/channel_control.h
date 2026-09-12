#ifndef SLIMENRF_CHANNEL_CONTROL_H
#define SLIMENRF_CHANNEL_CONTROL_H

/* Thread context only: serializes storage and radio reinitialization.
 * Set accepts user channels 0..100; reset selects the configured default.
 * Returns 0 on success, -EINVAL for an invalid channel, -ENODEV without
 * retained storage, or the storage/radio errno. RAM is updated even when
 * persistence fails, and radio reinitialization is still attempted; when
 * both fail, the storage error takes precedence. No rollback is implied.
 */
int channel_control_set(int channel);
int channel_control_reset(void);

#endif
