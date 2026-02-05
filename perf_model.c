// SPDX-License-Identifier: GPL-2.0-only

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include "perf_model.h"

#define NVMEV_PERF_ACT_INVALID (-1)

enum nvmev_perf_chan_state {
	NVMEV_CHAN_IDLE = 0,
	NVMEV_CHAN_CMD,
	NVMEV_CHAN_DATA,
};

enum nvmev_perf_slot_state {
	NVMEV_SLOT_EMPTY = 0,
	NVMEV_SLOT_WAIT_DATA,
	NVMEV_SLOT_WAIT_PROG,
};

struct nvmev_perf_list {
	int head;
	int tail;
	int act;
	bool empty;
	enum nvmev_perf_slot_state state;
	u64 ready_time;
	int *queue;
};

struct nvmev_perf_chan {
	enum nvmev_perf_chan_state state;
	int act;
	u32 list_info;
	u64 time;
};

struct nvmev_perf_cfg {
	u32 qd;
	u32 chan_num;
	u32 die_num;
	u32 planes;
	u32 iwl_slot;
	u32 element;
	u32 ecc_parity_bytes;
	u32 page_bytes;
	u32 chan_speed_mt;
	u64 cmd_overhead_ns;
	u64 tr_ns;
	u64 tprog_ns;
};

struct nvmev_perf_run {
	u64 iops;
	u64 duration_ns;
	u64 completed;
};

static struct nvmev_perf_cfg nvmev_perf_default_cfg(void)
{
	struct nvmev_perf_cfg cfg = {
		.qd = NVMEV_PERF_QD,
		.chan_num = NVMEV_PERF_CHAN_NUM,
		.die_num = NVMEV_PERF_DIE_NUM,
		.planes = NVMEV_PERF_PLANES,
		.iwl_slot = NVMEV_PERF_IWL_SLOT,
		.element = NVMEV_PERF_ELEMENT,
		.ecc_parity_bytes = NVMEV_PERF_ECC_PARITY_BYTES,
		.page_bytes = NVMEV_PERF_PAGE_BYTES,
		.chan_speed_mt = NVMEV_PERF_CHAN_SPEED_MT,
		.cmd_overhead_ns = NVMEV_PERF_CMD_OVERHEAD_NS,
		.tr_ns = NVMEV_PERF_TR_NS,
		.tprog_ns = NVMEV_PERF_TPROG_NS,
	};

	return cfg;
}

static inline u32 nvmev_perf_list_index(u32 chan, u32 die_in_chan, u32 plane_idx,
					u32 die_per_chan, u32 plane_groups)
{
	return (chan * die_per_chan + die_in_chan) * plane_groups + plane_idx;
}

static inline void nvmev_perf_queue_push(struct nvmev_perf_list *list, u32 depth, int act)
{
	list->queue[list->head] = act;
	list->head++;
	if (list->head == depth)
		list->head = 0;
	list->empty = false;
}

static inline int nvmev_perf_queue_pop(struct nvmev_perf_list *list, u32 depth)
{
	int act = list->queue[list->tail];

	list->tail++;
	if (list->tail == depth)
		list->tail = 0;
	if (list->tail == list->head)
		list->empty = true;
	return act;
}

static inline u64 nvmev_perf_data_time_ns(const struct nvmev_perf_cfg *cfg)
{
	u64 bytes = (u64)cfg->page_bytes + cfg->ecc_parity_bytes;

	/*
	 * MT/s to ns:
	 * bytes / (MT/s) yields microseconds, then scale to ns by 1000.
	 */
	return div64_u64(bytes * 1000ULL, cfg->chan_speed_mt);
}

static int nvmev_perf_simulate(const struct nvmev_perf_cfg *cfg, bool is_write,
			       struct nvmev_perf_run *run)
{
	struct nvmev_perf_list *lists = NULL;
	struct nvmev_perf_chan *chans = NULL;
	int *queue_mem = NULL;
	u8 *act_map = NULL;
	u64 sim_time = 0;
	u64 start_time = 0;
	u64 end_time = 0;
	u64 total_cmd = 0;
	u32 inflight = 0;
	u32 tmp_cmd_cnt;
	u32 die_per_chan;
	u32 plane_share = 1;
	u32 plane_groups;
	u32 list_count;
	u64 data_time_ns;
	u32 xor_ratio;
	int ret = 0;
	u32 i;

	if (cfg->chan_num == 0 || cfg->die_num == 0 || cfg->planes == 0 || cfg->iwl_slot == 0)
		return -EINVAL;
	if (cfg->chan_speed_mt == 0 || cfg->qd == 0 || cfg->element == 0)
		return -EINVAL;
	if (cfg->die_num % cfg->chan_num)
		return -EINVAL;

	die_per_chan = cfg->die_num / cfg->chan_num;
	if (cfg->planes * cfg->die_num > cfg->iwl_slot) {
		plane_share = (cfg->planes * cfg->die_num) / cfg->iwl_slot;
		if (plane_share != 2 && plane_share != 4)
			return -EINVAL;
	}
	if (cfg->planes % plane_share)
		return -EINVAL;
	plane_groups = cfg->planes / plane_share;
	if (plane_groups == 0)
		return -EINVAL;

	list_count = cfg->chan_num * die_per_chan * plane_groups;
	data_time_ns = nvmev_perf_data_time_ns(cfg);
	xor_ratio = (cfg->die_num > 64) ? 64 : cfg->die_num;

	lists = kvcalloc(list_count, sizeof(*lists), GFP_KERNEL);
	if (!lists)
		return -ENOMEM;
	chans = kcalloc(cfg->chan_num, sizeof(*chans), GFP_KERNEL);
	if (!chans) {
		ret = -ENOMEM;
		goto out;
	}
	queue_mem = kvcalloc((size_t)list_count * cfg->iwl_slot, sizeof(*queue_mem), GFP_KERNEL);
	if (!queue_mem) {
		ret = -ENOMEM;
		goto out;
	}
	act_map = kcalloc(cfg->qd, sizeof(*act_map), GFP_KERNEL);
	if (!act_map) {
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < list_count; i++) {
		lists[i].head = 0;
		lists[i].tail = 0;
		lists[i].act = NVMEV_PERF_ACT_INVALID;
		lists[i].empty = true;
		lists[i].state = NVMEV_SLOT_EMPTY;
		lists[i].ready_time = 0;
		lists[i].queue = queue_mem + (size_t)i * cfg->iwl_slot;
	}
	for (i = 0; i < cfg->chan_num; i++)
		chans[i].state = NVMEV_CHAN_IDLE;

	if (cfg->qd >= 512)
		tmp_cmd_cnt = (cfg->qd * 3) / 4;
	else
		tmp_cmd_cnt = (cfg->qd * 4) / 5;

	while (1) {
		u64 next_time = ~0ULL;
		bool progress = false;

		if (total_cmd > cfg->element)
			break;

		if (total_cmd > tmp_cmd_cnt && start_time == 0)
			start_time = sim_time;

		/* Issue new commands up to tmp_cmd_cnt inflight. */
		while (inflight < tmp_cmd_cnt) {
			u32 plane;
			u32 die;
			u32 chan;
			u32 die_in_chan;
			u32 list_dest;
			u32 list_idx;
			int act = NVMEV_PERF_ACT_INVALID;
			u32 search;

			for (search = 0; search < cfg->qd; search++) {
				if (act_map[search] == 0) {
					act = (int)search;
					break;
				}
			}
			if (act == NVMEV_PERF_ACT_INVALID)
				break;

			plane = prandom_u32_max(cfg->planes * cfg->die_num);
			die = plane / cfg->planes;
			chan = die % cfg->chan_num;
			die_in_chan = die / cfg->chan_num;
			list_dest = (plane % cfg->planes) / plane_share;
			list_idx = nvmev_perf_list_index(chan, die_in_chan, list_dest,
							 die_per_chan, plane_groups);
			nvmev_perf_queue_push(&lists[list_idx], cfg->iwl_slot, act);
			act_map[act] = 1;
			inflight++;
			progress = true;
		}

		/* Process channel CMD/DATA completion. */
		for (i = 0; i < cfg->chan_num; i++) {
			struct nvmev_perf_chan *chan = &chans[i];
			u32 die_in_chan;
			u32 list_dest;
			u32 list_idx;
			struct nvmev_perf_list *list;

			if (chan->state == NVMEV_CHAN_CMD && chan->time <= sim_time) {
				die_in_chan = chan->list_info >> 16;
				list_dest = chan->list_info & 0xFFFF;
				list_idx = nvmev_perf_list_index(i, die_in_chan, list_dest,
								 die_per_chan, plane_groups);
				list = &lists[list_idx];
				list->act = chan->act;
				list->state = NVMEV_SLOT_WAIT_DATA;
				list->ready_time = sim_time + (is_write ? 0 : cfg->tr_ns);
				chan->state = NVMEV_CHAN_IDLE;
				chan->act = NVMEV_PERF_ACT_INVALID;
				chan->list_info = 0xFFFFFFFF;
				progress = true;
			}

			if (chan->state == NVMEV_CHAN_DATA && chan->time <= sim_time) {
				die_in_chan = chan->list_info >> 16;
				list_dest = chan->list_info & 0xFFFF;
				list_idx = nvmev_perf_list_index(i, die_in_chan, list_dest,
								 die_per_chan, plane_groups);
				list = &lists[list_idx];
				if (is_write) {
					list->state = NVMEV_SLOT_WAIT_PROG;
					list->ready_time = sim_time + cfg->tprog_ns;
				} else {
					act_map[chan->act] = 0;
					inflight--;
					total_cmd++;
					list->act = NVMEV_PERF_ACT_INVALID;
					list->state = NVMEV_SLOT_EMPTY;
				}
				chan->state = NVMEV_CHAN_IDLE;
				chan->act = NVMEV_PERF_ACT_INVALID;
				chan->list_info = 0xFFFFFFFF;
				progress = true;
			}
		}

		/* Program completion for writes (no channel usage). */
		if (is_write) {
			u32 idx;

			for (idx = 0; idx < list_count; idx++) {
				if (lists[idx].act != NVMEV_PERF_ACT_INVALID &&
				    lists[idx].state == NVMEV_SLOT_WAIT_PROG &&
				    lists[idx].ready_time <= sim_time) {
					act_map[lists[idx].act] = 0;
					inflight--;
					total_cmd++;
					lists[idx].act = NVMEV_PERF_ACT_INVALID;
					lists[idx].state = NVMEV_SLOT_EMPTY;
					progress = true;
				}
			}
		}

		/* Schedule new CMD/DATA on idle channels. */
		for (i = 0; i < cfg->chan_num; i++) {
			struct nvmev_perf_chan *chan = &chans[i];
			u32 j, k;
			bool scheduled = false;

			if (chan->state != NVMEV_CHAN_IDLE)
				continue;

			for (j = 0; j < die_per_chan && !scheduled; j++) {
				for (k = 0; k < plane_groups; k++) {
					u32 idx = nvmev_perf_list_index(i, j, k,
									die_per_chan, plane_groups);
					struct nvmev_perf_list *list = &lists[idx];

					if (!list->empty && list->act == NVMEV_PERF_ACT_INVALID) {
						chan->state = NVMEV_CHAN_CMD;
						chan->time = sim_time + cfg->cmd_overhead_ns;
						chan->act = nvmev_perf_queue_pop(list, cfg->iwl_slot);
						chan->list_info = (j << 16) | k;
						scheduled = true;
						progress = true;
						break;
					}
				}
			}

			if (scheduled)
				continue;

			for (j = 0; j < die_per_chan && !scheduled; j++) {
				for (k = 0; k < plane_groups; k++) {
					u32 idx = nvmev_perf_list_index(i, j, k,
									die_per_chan, plane_groups);
					struct nvmev_perf_list *list = &lists[idx];

					if (list->act != NVMEV_PERF_ACT_INVALID &&
					    list->state == NVMEV_SLOT_WAIT_DATA &&
					    list->ready_time <= sim_time) {
						chan->state = NVMEV_CHAN_DATA;
						chan->time = sim_time + data_time_ns;
						chan->act = list->act;
						chan->list_info = (j << 16) | k;
						scheduled = true;
						progress = true;
						break;
					}
				}
			}
		}

		if (progress)
			continue;

		/* Advance simulated time to the next event. */
		for (i = 0; i < cfg->chan_num; i++) {
			if (chans[i].state != NVMEV_CHAN_IDLE && chans[i].time > sim_time)
				next_time = min(next_time, chans[i].time);
		}

		for (i = 0; i < list_count; i++) {
			if (lists[i].act == NVMEV_PERF_ACT_INVALID)
				continue;
			if ((lists[i].state == NVMEV_SLOT_WAIT_DATA ||
			     lists[i].state == NVMEV_SLOT_WAIT_PROG) &&
			    lists[i].ready_time > sim_time) {
				next_time = min(next_time, lists[i].ready_time);
			}
		}

		if (next_time == ~0ULL)
			break;

		sim_time = next_time;
		cond_resched();
	}

	end_time = sim_time;

	if (start_time == 0 || end_time <= start_time) {
		run->iops = 0;
		run->duration_ns = 0;
		run->completed = total_cmd;
		goto out;
	}

	run->duration_ns = end_time - start_time;
	if (total_cmd > cfg->qd)
		run->completed = total_cmd - cfg->qd;
	else
		run->completed = 0;

	run->iops = div64_u64(run->completed * 1000000000ULL, run->duration_ns);
	if (xor_ratio > 0)
		run->iops = div64_u64(run->iops * (xor_ratio - 1), xor_ratio);

out:
	kvfree(act_map);
	kvfree(queue_mem);
	kfree(chans);
	kvfree(lists);
	return ret;
}

int nvmev_perf_run(struct nvmev_perf_result *result)
{
	struct nvmev_perf_cfg cfg = nvmev_perf_default_cfg();
	struct nvmev_perf_run read_run = {};
	struct nvmev_perf_run write_run = {};
	int ret;

	if (!result)
		return -EINVAL;

	ret = nvmev_perf_simulate(&cfg, false, &read_run);
	if (ret)
		return ret;

	ret = nvmev_perf_simulate(&cfg, true, &write_run);
	if (ret)
		return ret;

	result->read_iops = read_run.iops;
	result->write_iops = write_run.iops;
	result->read_duration_ns = read_run.duration_ns;
	result->write_duration_ns = write_run.duration_ns;
	result->read_completed = read_run.completed;
	result->write_completed = write_run.completed;

	return 0;
}
