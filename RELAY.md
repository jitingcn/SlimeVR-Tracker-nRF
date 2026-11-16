# Peer relay

The receiver uses normal direct TDMA while links are healthy. `relay on` permits automatic relay entry after sustained loss or an outage. It does not start control broadcasts or helper listening by itself. Stable direct recovery stages a coordinated return to direct TDMA. `relay off` withdraws permission and stages the same safe exit; control frames continue until activation so trackers retain their direct clock anchors.

An engaged receiver broadcasts control feedback, assigns helper watch windows, and grants repair transmissions. Helpers cache peer packets and transmit only granted repairs. Static cache ages remain 0 through 2; demand scheduling retains its bounded five-frame cache. Helper qualification remains 950 permille. RX lead, 8/10/12 tick reserves, clock guards, freeze/launch boundaries and R0+3 are unchanged.

PONG trailers coordinate layout activation and passive clock presence. Profile 0 means normal direct TDMA; profile 1 means engaged relay. Protocol version 4 validates wire format. Both firmware repositories carry the same `src/connection/relay_protocol.h`.

Permission is `relay_user_enabled`, effective permission is `relay_coord.enabled`, automatic demand is `auto_engaged`, and the applied state is `relay_profile`. Commands are `relay on`, `relay off`, `relay` for status, and `relay layout` for settings. Diagnostic block and pool commands remain available. There is no second protocol switch or command alias.

Receiver counters use the same definitions in both profiles. `arrivals` counts validated pose records before deduplication; `duplicate_stale` counts duplicate or stale records rejected at publication; `direct_accepted` and `repair_accepted` count unique accepted records by path. Total TPS counts accepted unique records, including info and status records. Malformed packets and injected direct drops do not enter arrivals. Host `total_received`, gap and status counters retain publication accounting.

Host tests cover normal links, sustained/intermittent loss, outages, helper selection, bounded caches, repair grants, recovery hold, permission off, staged exit, native sequence re-anchor, clock handoff, control windows, delivery confirmation history and maintenance starvation. Firmware builds include nRF52840, nRF52833 and relay disabled. RF loss, physical clock drift and power savings still require hardware validation.
