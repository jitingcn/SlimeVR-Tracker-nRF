# Peer relay

The receiver boots in `relay auto` mode and uses direct TDMA while links are healthy. Sustained loss or an outage can trigger relay, and stable direct recovery stages a return to direct TDMA. `relay force` requests relay whenever the roster, layout and radio state allow it, regardless of link quality. `relay off` requests direct TDMA. On exit, control frames continue until activation so trackers retain their direct clock anchors.

Validated power-down notices provide bounded intent, not proof that a tracker is asleep. The receiver excludes that intent from automatic relay decisions and special query-recovery leases, but keeps ordinary RF membership and inactivity expiry. Cancellation, wake, session replacement and pairing invalidation clear the intent. Unannounced silence still follows the normal outage policy.

An engaged receiver broadcasts control feedback, assigns helper watch windows, and grants repair transmissions. Helpers cache peer packets and transmit only granted repairs. Static cache ages remain 0 through 2; demand scheduling retains its bounded five-frame cache. Helper qualification remains 950 permille.

Each primary or repeat control transmission has a four-RTC-tick arming guard after the preceding direct or repair area. Direct capture cutoffs and the receipt freeze relative to that area are unchanged. Control acquisition, frame periods, repair starts and repeat anchors include the guards. Native profile 0 is unchanged.

Delivery feedback is qualified by the active session and layout epoch. A received control covers its own membership and announces the next two frames, so their later receipt masks remain usable even when those controls were missed. Feedback outside observed or announced coverage is not delivery evidence, and a session, layout, clock-acquisition or profile reset fences old confirmations.

PONG trailers coordinate layout activation and passive clock presence. Profile 0 means direct TDMA, and profile 1 means engaged relay. Both firmware repositories carry byte-identical `src/connection/relay_protocol.h` headers. Keep pre-v5 relay firmware out of a v5 relay fleet because it uses different control timing. V5 trackers derive activation from the delta transmitted in the PONG, so they accept a v5 receiver's updated announcement lead.

The receiver stores the selected mode in `relay_user_mode`. `relay_coord.enabled` controls effective permission, `auto_engaged` records automatic demand, and `relay_profile` records the applied profile. Use `relay` for status and `relay layout` for settings. `relay on` is an alias for `relay auto`. Diagnostic block and pool commands remain available.

Receiver counters use the same definitions in both profiles. `arrivals` counts validated pose records before deduplication; `duplicate_stale` counts duplicate or stale records rejected at publication; `direct_accepted` and `repair_accepted` count unique accepted records by path. Total TPS counts accepted unique records, including info and status records. Malformed packets and injected direct drops do not enter arrivals. Host `total_received`, gap and status counters retain publication accounting.

Host tests cover normal links, sustained/intermittent loss, outages, helper selection, bounded caches, repair grants, recovery hold, permission off, staged exit, native sequence re-anchor, clock handoff, control windows, delivery confirmation history and maintenance starvation. Firmware builds include nRF52840, nRF52833 and relay disabled. RF loss, physical clock drift and power savings still require hardware validation.
