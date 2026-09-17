/* Verbatim parser body from SlimeVR-Tracker-nRF-Receiver/src/connection/esb.c
 * commit a8e489028313ea4112988759e66a164c035309f5.
 * SlimeVR Contributors, MIT license (repository LICENSE).
 * Preserved to test forward compatibility after receiver parser changes.
 */
				int pos = 3;                     /* skip header: type, id, sub_count */
				int end = rx_payload.length - 1; /* exclude sequence byte */

				for (int i = 0; i < sub_count && pos < end; i++) {
					uint8_t sub_type = rx_payload.data[pos++];
					int sub_len;

					/* Determine sub-packet data length */
					switch (sub_type) {
					case 0:
						sub_len = 13;
						break; /* info */
					case 1:
						sub_len = 14;
						break; /* quat+accel */
					case 2:
						sub_len = 13;
						break; /* compact quat */
					case 3:
						sub_len = 2;
						break; /* status */
					case 4:
						sub_len = 14;
						break; /* quat+mag */
					case 5:
						sub_len = 8;
						break; /* runtime */
					default:
						LOG_ERR("Unknown composite sub-type: %d", sub_type);
						sub_len = -1;
						break;
					}

					if (sub_len < 0 || pos + sub_len > end) {
						break;
					}

					/* Reconstruct a standard 16-byte packet */
					uint8_t pkt[16] = {0};
					pkt[0] = sub_type;
					pkt[1] = tracker_id;
					memcpy(&pkt[2], &rx_payload.data[pos], MIN(sub_len, 14));

					/* For status packets (type 3), fill packet loss stats */
					if (sub_type == 3) {
						struct packet_stats *stats = &tracker_stats[tracker_id];
						pkt[4] = stats->status_received;
						pkt[5] = stats->status_lost;
						pkt[6] = 0;
						pkt[7] = 0;
						stats->status_received = 0;
						stats->status_lost = 0;
					}

					hid_write_packet_n(pkt, rx_payload.rssi);
					pos += sub_len;
				}
