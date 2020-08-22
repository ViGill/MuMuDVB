/*
 * MuMuDVB - Stream a DVB transport stream.
 * PMT rewrite by Danijel Tudek, Dec 2016.
 *
 * (C) 2008-2013 Brice DUBOST <mumudvb@braice.net>
 *
 * Parts of this code come from libdvb, modified for mumudvb
 * by Brice DUBOST
 * Libdvb part : Copyright (C) 2000 Klaus Schmidinger
 *
 * The latest version can be found at http://mumudvb.net
 *
 * Copyright notice:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/** @file
 *  @brief This file contains code for PMT rewrite
 *  PMT must be rewritten if we don't stream all PIDs, otherwise some players
 *  may get confused.
 *  (See "Problem with streaming individual PIDs", Nov 2016 on mailing list.)
 *
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "mumudvb.h"
#include "ts.h"
#include "rewrite.h"
#include "log.h"
#include <stdint.h>

extern uint32_t crc32_table[256];
static char *log_module = "PMT rewrite: ";

/**
 * @brief Check the streamed PMT version and compare to the stored PMT.
 * @param ts_packet
 * @param channel
 * @return return 1 if update is needed, 2 if multi-part PMT is being assembled
 */
int pmt_need_update(unsigned char *ts_packet, mumudvb_channel_t *channel) {
	pmt_t *pmt = (pmt_t *) (ts_packet + TS_HEADER_LEN);


	if (pmt->table_id == 0x02) {
		if (channel->original_pmt_ready) {
			if (pmt->version_number != channel->generated_pmt_version) {
				log_message(log_module, MSG_DEBUG, "PMT changed, old version: %d, new version: %d, rewriting...", channel->generated_pmt_version, pmt->version_number);
				channel->original_pmt_ready = 0;
			} else {
				return 1;
			}
		}
		if (!channel->original_pmt_ready) {
			int pmt_length = HILO(pmt->section_length) + 3;
			channel->pmt_part_num =	((pmt_length + TS_PACKET_SIZE) % TS_PACKET_SIZE) ? (pmt_length / TS_PACKET_SIZE) : ((pmt_length / TS_PACKET_SIZE) - 1); /* exact ceil() simulation */
			channel->pmt_part_count = 0;
			memcpy(channel->original_pmt + (TS_PACKET_SIZE*channel->pmt_part_count), ts_packet, TS_PACKET_SIZE);
			if (channel->pmt_part_num == 0) {
				channel->original_pmt_ready = 1;
				return 1;
			} else {
				return 2;
			}
		}
	} else {
		if (!channel->original_pmt_ready && channel->pmt_part_num > 0) {
			channel->pmt_part_count++;
			log_message(log_module, MSG_DETAIL, "Got part %i of PMT", channel->pmt_part_count + 1);
			memcpy(channel->original_pmt + (TS_PACKET_SIZE*channel->pmt_part_count), ts_packet + 4, TS_PACKET_SIZE);
			if (channel->pmt_part_count == channel->pmt_part_num) {
				channel->original_pmt_ready = 1;
				return 1;
			} else {
				return 2;
			}
		} else if (!channel->original_pmt_ready && channel->pmt_part_num == 0) {
			log_message(log_module, MSG_DETAIL, "We didn't get the good PMT yet (wrong table ID 0x%02X), search for a new one", pmt->table_id);
		}
		return 0;
	}
	return 0;
}

/** @brief Main function for PMT rewrite.
 * The goal of this function is to make a new PMT with only the announcement for the streamed PIDs for each channel.
 * By default it contains all PIDs which confuses players if we don't actually stream all of them.
 * The PMT is read and the list of PIDs is compared to user-specified PID list for the channel.
 * If there is a match, PID is copied to generated PMT.
 * @param channel
 * @return 0
 */
int pmt_channel_rewrite(mumudvb_channel_t *channel) {
	unsigned char *ts_packet = channel->original_pmt;
	ts_header_t *ts_header = (ts_header_t *) ts_packet;
	pmt_t *pmt = (pmt_t *) (ts_packet + TS_HEADER_LEN);

	if (pmt->table_id != 0x02) {
		log_message(log_module, MSG_DETAIL, "We didn't get the good PMT (wrong table ID 0x%02X), ts_packet[0]=0x%x search for a new one", pmt->table_id, ts_packet[0]);
		return 0;
	}

	if (channel->forced_service_id)
	{
		pmt->program_number_hi = (channel->forced_service_id>>8)&0xFF;
		pmt->program_number_lo = channel->forced_service_id&0xFF;
	}
	if (channel->forced_pmt_pid)
	{
		ts_header->pid_hi = (channel->forced_pmt_pid>>8)&0xFF;
		ts_header->pid_lo = channel->forced_pmt_pid&0xFF;
        }
	memcpy(channel->generated_pmt, channel->original_pmt, TS_PACKET_SIZE);

	int section_length = 0;
	section_length = HILO(pmt->section_length);
	//CRC32 calculation inspired by the xine project
	//Now we must adjust the CRC32
	//we compute the CRC32
	unsigned long crc32, i;
	crc32 = 0xffffffff;
	for (i = 0; i < section_length - 1; i++) {
		crc32 = (crc32 << 8) ^ crc32_table[((crc32 >> 24) ^ channel->generated_pmt[i + TS_HEADER_LEN]) & 0xff];
	}
	int buf_dest_pos = section_length+4;

	//We write the CRC32 to the buffer
	channel->generated_pmt[buf_dest_pos] = (crc32 >> 24) & 0xff;
	buf_dest_pos += 1;
	channel->generated_pmt[buf_dest_pos] = (crc32 >> 16) & 0xff;
	buf_dest_pos += 1;
	channel->generated_pmt[buf_dest_pos] = (crc32 >> 8) & 0xff;
	buf_dest_pos += 1;
	channel->generated_pmt[buf_dest_pos] = crc32 & 0xff;
	buf_dest_pos += 1;


	return 1;
}

int pmt_send_packet(unsigned char *pmt_ts_packet, mumudvb_channel_t *channel) {
	//Everything is good, send the generated packet
	channel->pmt_continuity_counter++;
	channel->pmt_continuity_counter = channel->pmt_continuity_counter % 32;
	memcpy(pmt_ts_packet, channel->generated_pmt, TS_PACKET_SIZE);
        pmt_t *pmt = (pmt_t *) (pmt_ts_packet + TS_HEADER_LEN);
	set_continuity_counter(pmt_ts_packet, channel->pmt_continuity_counter);
	return 1;
}

int pmt_rewrite_new_channel_packet(unsigned char *ts_packet, unsigned char *pmt_ts_packet, mumudvb_channel_t *channel, int curr_channel) {
	if (channel->channel_ready >= READY) {
		int need_update = pmt_need_update(ts_packet, channel);
		if (need_update == 0) {
			return pmt_send_packet(pmt_ts_packet, channel);
		} else if (need_update == 1) {
			//Needs update, call rewrite function
			if (!pmt_channel_rewrite(channel)) {
				log_message(log_module, MSG_DEBUG, "Cannot rewrite (for the moment) the PMT for the channel %d : \"%s\"\n", curr_channel, channel->name);
				//In case of an error, send the previously generated PMT, if any, or skip sending
				if (channel->generated_pmt_version) {
					return pmt_send_packet(pmt_ts_packet, channel);
				} else {
					return 0;
				}
			} else {
				return pmt_send_packet(pmt_ts_packet, channel);
			}
		} else if (need_update == 2) {
			//Multi-part PMT, we have to assemble it before processing
			log_message(log_module, MSG_DEBUG, "Assembling multi-part PMT for the channel %d : \"%s\"...\n", curr_channel, channel->name);
			return 0;
		}
	}
	return 0;
}
