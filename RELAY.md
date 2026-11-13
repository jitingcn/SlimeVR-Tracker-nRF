# Peer relay

Mirrored in the tracker and receiver repositories. The wire format lives in
`src/connection/relay_protocol.h`, which is byte-identical in both trees
(`tests/host/relay/check_protocol_sync.py` checks this).

## What it does

When a tracker's uplink to the receiver fails, for example because a body is
in the way, a nearby tracker with a good link forwards its pose frames. Nothing
changes on the SlimeVR side: relayed frames produce exactly the same HID
packets as direct delivery, so an unmodified SlimeVR server works.
nRF52840 and nRF54L15 trackers run the same protocol and can relay for each
other.

## Topology and roles

The receiver coordinates a single relay hop. It appoints up to 3 **heads**, each
with up to 3 **children** (`RELAY_MAX_HEADS`, `RELAY_MAX_CHILDREN`).

- **Child.** Behaves exactly as before. It keeps sending its ordinary frames in
  its own TDMA slot and does nothing relay-specific on air.
- **Head.** Listens during each child's slot (raw RADIO RX while ESB is idle,
  see `relay_radio.c`). It forwards what it captures in one of two ways:
  - inside its own data frame, as a *bundle*, when there is room (no extra
    airtime); or
  - in a **forwarding slot** that the receiver appends to the frame for that
    head, when one bundle per frame cannot carry the children. Forwarding
    capacity is granted on demand: a head asks with
    `RELAY_CAP_FORWARD_REQUEST`, or the receiver sees overflowing bundles.
- **No back channel.** The receiver's downlink (about +22 dBm through its FEM) is far
  stronger than any tracker uplink (+8 dBm). A child therefore keeps its time
  sync, layout and delivery confirmation from PONGs it overhears; heads never
  forward receiver traffic to children.

All decisions are made at the receiver and change dynamically: who is weak,
who becomes head, child assignments, and how many forwarding slots exist. The
user can only pin a preferred head (`relay pin <id>`) or turn relaying off
(`relay off`).

## Discovery and capability

A tracker advertises `RELAY_INFO_CAPS` (helper, presence, protocol version 2)
in the reserved byte of its info frame. The receiver zeroes that byte before
HID output. Extended PONGs, which carry the relay trailer, go only to
trackers that advertised a matching version. A tracker that never advertises
is never made a head, but it can still be a child: a child needs no relay
logic, because heads listen in its ordinary slot.

## Coordinator (receiver, `relay_coord.c`)

All checks run per 500 ms window (`RC_WINDOW_MS`) and use direct delivery
ratios. The weak and release verdicts need at least 20 expected frames
(`RC_MIN_EXPECTED`). An idle tracker sends 10-20 frames a second, so sparse
windows are carried into the next judgement (at most 8 windows,
`RC_JUDGE_MAX_WINDOWS`) rather than skipped. Skipping them left a verdict
reached while moving stuck once the tracker went still: OTA stayed refused and
idle children were never released.

| Decision | Rule |
|---|---|
| Weak (needs a relay) | below 80 % for 2 judgements (the farthest tracker of an 11-tracker fleet dips to 82–92 % at idle and stays direct); or direct silence once it has missed about 20 frames at its own recent rate (at least 500 ms: 0.5 s at full rate, about 2 s idle) while its last direct frame is under 12 s old. An empty window alone is not loss: still trackers, trackers throttled by another tracker's OTA, and reboots produce them |
| Head eligible | 95 % or better and relay-capable; the strongest RSSI or a pinned tracker wins |
| Head dropped | below 85 % |
| Child released | 90 % or better for 6 judgements, held at least 5 s (children keep sending directly, so direct is preferred and an unneeded relay only costs head RX); a tracker whose head was lost clears its relay need on one good judgement |
| Failed pair (head delivers too little within 3 s) | rotate to another head; ban the pair for 20 s (2 s if the child is simply silent) |
| Silent child | retried with other heads within 15 s; after that the receiver's 15 s membership eviction applies |
| Forwarding slot | granted after 2 overflowing windows; withdrawn after 20 idle windows, 4 s after revocation (longer than the head lease plus one PONG period) |

## Scheduling

- Forwarding slots follow the direct members' slots and count toward the
  per-tracker rate cap. Members plus forwarding slots never exceed 16.
- A head opens its receiver `RELAY_RX_LEAD_TICKS` (4) ticks before the child's
  slot and keeps it open for `RELAY_RX_TAIL_US` (400 µs) after the slot ends.
- A captured record is forwarded only while more than
  `RELAY_FORWARD_GUARD_TICKS` (11) ticks remain before the child's next
  preamble. A relayed copy therefore always arrives before the child's next
  frame, so the receiver's sequence check never sees frames reorder.
- **Presence windows.** A degraded tracker listens for about 1.6 ms every
  300 ms, during other trackers' PING exchanges, to overhear PONGs.

## Supervision and recovery (tracker, `link_supervision.h`)

This replaces the old "3 missed PINGs or 4.5 s without a PONG means the
receiver moved" rule:

- **HEALTHY.** The tracker's own PING/PONG exchange works.
- **DEGRADED.** The own exchange fails, but either the tracker overheard a
  PONG within 4.5 s or the outage is younger than 8 s (the uplink holdover).
  The tracker keeps its slot, sync and transmissions, so a head can relay it.
  A lost own PING does not escalate toward shutdown while a relayed-mask
  confirmation is younger than 2 s.
- **LOST.** Neither condition holds. Only then does the tracker start a
  channel search, and every third probe revisits the home channel
  (`LINK_SEARCH_HOME_EVERY`).

Head and child failure:

- **Head lease.** A head forwards for at most 2.5 s after its last PONG naming
  its children. A head that stops hearing the receiver therefore stops using
  the forwarding slot before the receiver can reassign it.
- **Lost head.** The receiver sees the child's delivery stop and rotates it to
  another head (see the coordinator table).

### Staged layout activation

Layout changes (membership, slot width, forwarding slots) switch for every
tracker at the same receiver tick. Previously each tracker switched whenever
its next PONG arrived, so trackers on the old and new layouts could transmit
in the same slot.

1. The receiver stages the new layout and announces it in every NORMAL PONG
   for 3.5 s (`RELAY_LAYOUT_ACTIVATION_MS`). The announcement is a 17-byte
   trailer, so the PONG is 30 bytes. It carries the pending epoch, total
   slots, slot width, member mask, and the activation time relative to the
   PONG's own timestamp. Until activation, the legacy tuple keeps describing
   the current layout. Transitions are serialized: one is pending at a time.
2. A tracker stages the pending layout from its own PONG, or from an
   overheard one when its receiver clock agrees. It switches at the
   activation tick. No data or PING window may overlap ±4 ticks of that tick
   (`TDMA_ACTIVATION_QUIET_TICKS`), and a slot wait that crosses the tick
   aborts and retries in the new layout.
3. **Layout grant.** A tracker may use its current layout only for 3 s
   (`RELAY_LAYOUT_GRANT_MS`) after the last trailer-bearing PONG confirming
   it lists the tracker under the current epoch. Any PONG sent after staging
   also carries the announcement. A tracker that missed every announcement
   therefore falls silent at least 500 ms before the old layout ends. Hearing
   any later valid PONG recovers it.
4. A tracker removed by the new layout is withdrawn at activation and stays
   silent until it is granted a slot again. A joining tracker gets its first
   slot through the pending block and starts data after its first PONG in the
   new epoch.

Staging is used only if every affected tracker is relay-capable at protocol
v2 and the old layout is not empty. Otherwise the change is published
immediately, as before (legacy immediate mode). Administrative changes forced
from the console (remove/forget) also publish immediately.

## Packet formats

All formats are documented in the header comment of `relay_protocol.h`:

- **Bundle**, type 0xE6: the head's own frame plus records of
  `[len][rssi][frame]`.
- **Extended PONG trailer**: 10 bytes (magic 0x5A), or 17 bytes with a pending
  layout (magic 0x5B), each protected by its own CRC-8.
- **Diagnostic frame**, type 0xE7, 44 bytes: sent about once a second by a
  tracker that holds an assignment or runs presence windows, and only to a
  relay-aware receiver. It appears in the receiver's `relay` output and is
  never sent to HID.

OTA is direct only. The receiver refuses OTA BEGIN for a relay child, for a
tracker without a direct frame in the last `RC_OTA_DIRECT_FRESH_MS`, and for
one the coordinator judges weak (`RC_WEAK_WINDOWS` lossy judgements) or that
still needs a relay. An earlier 95 %-per-window gate refused healthy idle
trackers on hardware, because one lost frame in a 10-frame window is 90 %.

## Timing budget

`tests/host/relay/budget.py` reads the production constants from both trees
and checks every hop at the narrowest slot (16 ticks = 488 µs, ESB at 2 Mbps):

| Hop | Worst case | Limit |
|---|---|---|
| PING plus 30-byte PONG, two attempts | receiver RX ready at 1048 µs | next owner's preamble at 1078 µs (**30 µs of slack**) |
| Child's 17-byte frame | ends at 414 µs | slot end at 488 µs |
| Largest child composite | overruns by 7 µs | head RX tail of 400 µs |
| Head RX settled | −32 µs | child's earliest preamble at +40 µs (2 ticks of sync error) |
| 72-byte bundle, admitted last | ends at 512 µs | next owner's preamble at 589 µs |
| Bundle vs the child's next frame | 390 µs | guard of 437 µs |
| Activation margin | 500 ms | sync error + one 72-byte frame + the widest frame (125 ms) |

Latency added by relaying and the head's RX duty per child:

| trackers | slot (ticks) | frame (ms) | worst added (ms) | head RX typ | head RX max |
|---:|---:|---:|---:|---:|---:|
| 4 | 36 | 4.39 | 5.49 | 7.6 % | 27.8 % |
| 6 | 24 | 4.39 | 5.13 | 7.6 % | 19.4 % |
| 8 | 18 | 4.39 | 4.94 | 7.6 % | 15.3 % |
| 11 | 16 | 5.37 | 5.86 | 6.2 % | 11.4 % |
| 16 | 16 | 7.81 | 8.30 | 4.3 % | 7.8 % |

The airtime model's software allowances (call latency 20 µs, turnaround
45 µs, RX settle 40 µs) are assumptions, not measurements.

## Trade-offs

- **Latency.** A relayed frame arrives at most one frame plus one slot later,
  about 5–8 ms. Direct frames are unaffected.
- **Battery.**
  - Only heads pay a significant cost: RX for each child slot (see the table),
    plus HFCLK and CPU time.
  - Children pay nothing extra while relayed.
  - A degraded tracker pays about 1.6 ms of RX plus a 2 ms HFCLK lead every
    300 ms for presence windows.
  - Heads are picked by link quality, not battery level.
- **Capacity.** A forwarding slot widens the frame for everyone, so it is
  granted only when a head's own frame cannot carry its children.
- **Layout changes** take 3.5 s to apply when staged, instead of applying at
  each tracker's next PONG.

## Known limitations

- **Rediscovery after eviction.** A child whose direct uplink fails
  completely, and that no head hears, is evicted by the receiver after 15 s.
  After that it has no slot for a later head to listen to, and it sends no
  direct PING to be re-admitted. Recovery then requires its direct link to
  return. Fixing this needs a receiver-scheduled discovery exchange, which is
  not implemented.
- **Receiver reboot vs epoch wrap.** The 8-bit epoch cannot distinguish a
  receiver reboot from a wrap. Trackers detect a reboot by a backward PONG
  timestamp instead (`RECEIVER_RESTART_BACKWARD_TICKS`).
- **Immediate publication.** Mixed fleets with legacy tracker firmware, and
  forced administrative changes, keep immediate layout publication. The
  collision-free guarantee holds only for all-v2 fleets.
- **Tight PING allowance.** The 30-byte PONG leaves 30 µs of modelled slack in
  the PING window. Measure it on both MCUs under IRQ load.

## Verification: what is established and what is not

Host tests (`make check` in `tests/host/relay` in both trees, plus the updated
fixtures) run the production functions, not models of them. They establish:

- Wire formats and CRC rejection, including the pending trailer.
- Coordinator decisions: thresholds, rotation, bans, release, forwarding-slot
  lifecycle, and head loss.
- TDMA forwarding admission and capture windows.
- Staged activation:
  - promotion at the activation tick;
  - the quiet zone;
  - withdrawal and recovery;
  - a sweep over 288 activation phases showing that no admission ever mixes
    layouts (the review's counterexample, which now holds only for a tracker
    that is also out of its grant);
  - the layout grant on own and overheard PONGs.
- Receiver staged publication: announce, serialize, activate, legacy fallback
  and forced changes.
- Passive PONG parsing and the reboot fence.
- Relay submission gates.
- HID parity of relayed frames.
- The timing budget above.

Seen on hardware (two RevF trackers side by side, HolyIOT receiver):

- Real relayed delivery through a forced head (`relay block <id> 100`).
- Presence: a blocked child overheard 14 of 15 PONGs.
- Recovery about 6 s after unblocking.
- Protocol v2 capability negotiation.
- Staged activation applied exactly 3.5 s after announcement in both
  directions (slot 64↔16 ticks), with a single tracker at 93–99 TPS and no
  data gap.

Not yet established:

- Head capture quality. Early runs missed about 70 % of child frames, and 85 %
  of heard frames failed CRC. The RX lead was raised from 1 to 4 ticks and the
  record expiry fixed since then; this has not been re-measured.
- Collision freedom of staged activation with several trackers on air.
- Mixed nRF52840/nRF54L15 relaying.
- Battery cost of the head role.
- Behaviour on a body.

## V3 (in progress)

Design: fast peer repair at 120 Hz (`relay-v2-design.md` in the workspace).
Done so far: the v3 control, repair, peer-report and layout codecs in
`relay_protocol.h`; the profile-1 clock (4096 ticks per 15 frames: 11 direct
slots, a receiver broadcast slot, 5 repair slots); direct payload caps (60
bytes, 32 in the last direct slot). Nothing runs on air yet: trackers still
advertise v2, and `relay v3 on|off` (default off) only records the request
that the eligibility check needs, along with all members advertising v3 and at
most 11 members.

### Phase 3 direct stream

The receiver console `relay on` then `relay v3 on` requests profile 1. The
switch defaults off. Every active member must advertise wire v3 and there may
be at most eleven members. Entry uses the existing 3.5 s staged activation,
with the 0x5C trailer. No repair or maintenance grants are issued yet.

Profile 1 sends one pose every rational 120 Hz frame and listens for B on
paired pipe 1. The connection owner prepares before its direct deadline and
waits outside the ESB lifecycle lock. B uses hardware END capture and renews
clock and presence only after epoch, membership, session and innovation
validation. Three missed frames exhaust direct holdover. A changed session
fences TX; phase 3 has no maintenance invitation for automatic re-entry.

For a first bench test, use only the receiver and two upgraded paired trackers.
Check `relay` on the receiver for both v3 capabilities, request v3, and wait
at least four seconds. Receiver `relay` reports profile, B_sent and B_failed;
tracker `relay` reports profile, B_received and B_missed. Compare counter
changes over ten seconds with 1,200 broadcasts and direct poses per tracker.
Receiver `stats 10` and tracker `tdma stats` expose the existing delivery and
admission counters. Hardware timing, FEM gain, B reception and HID-drop
acceptance still require measurement.

Keep membership, channel, OTA and raw collection fixed during this canary.
To return to v2, issue receiver `relay v3 off`, wait four seconds, then restart
the trackers. Maintenance and live layout/session recovery remain later work;
loss of B or a receiver restart suppresses v3 TX rather than launching legacy
recovery traffic into the protected slots.

### Phase 4: granted repairs

Profile 1 now assigns primary and backup watchers by direct RSSI, excluding
adjacent direct slots and limiting each helper to three watched children. This
is a placeholder for phase 5's peer evidence. B advertises the next frame's
watch plan; grants use the plan that actually applied to each cached frame.
The coordinator grants oldest unresolved frames to primaries before spending
remaining slots on backups, grouping up to three records per helper grant.

The tracker extends the relay cache to three frames, confirms records from B,
and retains records across failed queue submissions. The connection owner sends
repair envelopes through the existing per-slot admission and final recheck.
Missed B never reuses a repair grant. Peer capture uses the existing raw RX and
hardware END capture, with D10 completion clipped before B ownership.

The receiver validates session, epoch, physical repair slot, owner, child and
capture age before unpacking records through the normal/composite HID paths.
Frame-indexed delivery suppresses exact copies and historical poses, independent
of native sequence wrap or restart heuristics. Direct receipts remain separate
from any-path recovery. B watch selection is prepared early; missing masks and
grants freeze after D10. The shared CRC uses the same polynomial with nibble
lookups to keep final B encoding bounded.

Bench: enable `relay v3 on` for an all-v3 fleet and wait for profile 1 activation.
Use at least five trackers for non-adjacent primary/backup coverage, then
`relay block <child-id> 100`. Confirm captures and accepted repair records rise
while direct receipts stop. Keep the child blocked and physically obstruct the primary's uplink to exercise
backup recovery. The console holds one block ID, so another block command replaces it. Clear each block with `relay block <id> 0`; disable with `relay v3 off`.
Forced receiver drops test recovery behavior; peer RF coverage and D10/B/R0
turnaround still require measurements on both tracker boards.
