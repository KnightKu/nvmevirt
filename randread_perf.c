#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <limits.h>

/***************************** Usage ***************************/
// 1. Modify parameters in the following part.
// 2. gcc -O2 randread_perf.c -o randread_perf
// 3. ./randread_perf (fast, event-driven)
// This model includes host/PCIe/internal-bus limits.
// It still focuses on random read; no writes or GC modeled.
/***************************************************************/

/******************************* Parameters ***********************************/
#define CMD_OVERHEAD_US   (1.7)   /* cmd issue overhead in controller */
#define CHAN_SPEED_MT     (2400)  /* MT/s */
#define ECC_PARITY_BYTES  (600)   /* parity bytes per 4KiB */
#define tR_US             (40)    /* NAND tR */
#define QD                (512)   /* host queue depth */
#define CHAN_NUM          (16)
#define DIE_NUM           (128)
#define PLANE             (4)
#define IWL_SLOT          (256)

#define IO_SIZE_BYTES     (4096)

/* Host + PCIe + controller */
#define HOST_SUBMIT_US    (0.6)
#define HOST_COMPLETE_US  (1.0)
#define CTRL_PROC_US      (0.8)

#define PCIE_GBPS         (16.0)  /* one direction bandwidth */
#define PCIE_LAT_US       (1.0)
#define CMD_BYTES         (64)
#define CQE_BYTES         (16)

/* Internal controller resources */
#define INT_BUS_GBPS      (50.0)
#define INT_BUS_LAT_US    (0.2)
#define ECC_DECODE_US     (2.0)

#define DMA_ENGINES       (4)
#define DMA_SETUP_US      (0.3)

/* Simulation control */
#define TARGET_CMDS       (1 * 32 * 1024)
/*****************************************************************************/

/* No need to change in general */
#define TIME_SCALE        (1000ULL)
#define US_TO_TIME(us)    ((uint64_t)((us) * TIME_SCALE + 0.5))

#define CMD_TIME          (US_TO_TIME(CMD_OVERHEAD_US))
#define TREAD_TIME        (US_TO_TIME(tR_US))
#define NAND_BYTES        (IO_SIZE_BYTES + ECC_PARITY_BYTES)
#define NAND_DATA_TIME    ((uint64_t)(((double)NAND_BYTES * TIME_SCALE) / (double)CHAN_SPEED_MT))

#define SLOT              (IWL_SLOT)
#define CMD_CNT           (QD)
#define DIE_PER_CHAN      (DIE_NUM / CHAN_NUM)

enum CHAN_STATE {
	CHAN_IDLE,
	CHAN_CMD,
	CHAN_DATA,
};

typedef struct list_s {
	int head;
	int tail;
	int act;
	int empty;
	int list[SLOT];
	uint64_t time;
} list_t;

typedef struct chan_s {
	int state;
	int act;
	uint32_t list_info;
	uint64_t time;
} chan_t;

typedef struct pending_cmd_s {
	int used;
	int act;
	int plane;
	uint64_t ready_time;
} pending_cmd_t;

typedef struct pending_comp_s {
	int used;
	int act;
	uint64_t ready_time;
} pending_comp_t;

static int map[CMD_CNT];
static uint64_t issue_time[CMD_CNT];
static list_t list_slot[CHAN_NUM][DIE_PER_CHAN][PLANE];
static chan_t chan[CHAN_NUM];
static pending_cmd_t pending_cmds[CMD_CNT];
static pending_comp_t pending_comps[CMD_CNT];

static uint64_t pcie_tx_avail;
static uint64_t pcie_rx_avail;
static uint64_t int_bus_avail;
static uint64_t dma_avail[DMA_ENGINES];

static uint64_t pcie_tx_busy;
static uint64_t pcie_rx_busy;
static uint64_t int_bus_busy;
static uint64_t dma_busy;
static uint64_t chan_cmd_busy;
static uint64_t chan_data_busy;

static uint64_t pcie_tx_busy_start;
static uint64_t pcie_rx_busy_start;
static uint64_t int_bus_busy_start;
static uint64_t dma_busy_start;
static uint64_t chan_cmd_busy_start;
static uint64_t chan_data_busy_start;

static inline uint64_t max_u64(uint64_t a, uint64_t b)
{
	return (a > b) ? a : b;
}

static inline uint64_t time_for_transfer(uint64_t bytes, double gbps)
{
	double us = (double)bytes / (gbps * 1000.0);
	return US_TO_TIME(us);
}

static inline uint64_t schedule_pcie_tx(uint64_t ready_time, uint64_t bytes)
{
	uint64_t start = max_u64(ready_time, pcie_tx_avail);
	uint64_t duration = US_TO_TIME(PCIE_LAT_US) + time_for_transfer(bytes, PCIE_GBPS);
	uint64_t end = start + duration;

	pcie_tx_avail = end;
	pcie_tx_busy += duration;
	return end;
}

static inline uint64_t schedule_pcie_rx(uint64_t ready_time, uint64_t bytes)
{
	uint64_t start = max_u64(ready_time, pcie_rx_avail);
	uint64_t duration = US_TO_TIME(PCIE_LAT_US) + time_for_transfer(bytes, PCIE_GBPS);
	uint64_t end = start + duration;

	pcie_rx_avail = end;
	pcie_rx_busy += duration;
	return end;
}

static inline uint64_t schedule_internal_bus(uint64_t ready_time, uint64_t bytes)
{
	uint64_t start = max_u64(ready_time, int_bus_avail);
	uint64_t duration = US_TO_TIME(INT_BUS_LAT_US) + time_for_transfer(bytes, INT_BUS_GBPS);
	uint64_t end = start + duration;

	int_bus_avail = end;
	int_bus_busy += duration;
	return end;
}

static inline uint64_t schedule_dma_pcie_rx(uint64_t ready_time, uint64_t bytes)
{
	int i;
	int idx = 0;
	uint64_t earliest = dma_avail[0];
	uint64_t pcie_duration = US_TO_TIME(PCIE_LAT_US) + time_for_transfer(bytes, PCIE_GBPS);

	for (i = 1; i < DMA_ENGINES; i++) {
		if (dma_avail[i] < earliest) {
			earliest = dma_avail[i];
			idx = i;
		}
	}

	uint64_t dma_start = max_u64(ready_time, earliest);
	uint64_t dma_setup_end = dma_start + US_TO_TIME(DMA_SETUP_US);
	uint64_t pcie_start = max_u64(dma_setup_end, pcie_rx_avail);
	uint64_t pcie_end = pcie_start + pcie_duration;

	dma_avail[idx] = pcie_end;
	pcie_rx_avail = pcie_end;

	dma_busy += (pcie_end - dma_start);
	pcie_rx_busy += pcie_duration;
	return pcie_end;
}

static inline void update_list_head(int plane, int plane_share, int act)
{
	int die = plane / PLANE;
	int chan_id = die % CHAN_NUM;
	int die_in_chan = die / CHAN_NUM;
	int list_dest = (plane % PLANE) / plane_share;
	list_t *list = &(list_slot[chan_id][die_in_chan][list_dest]);

	list->list[list->head] = act;
	list->head++;
	if (list->head == SLOT) {
		list->head = 0;
	}
	if (list->head != list->tail) {
		list->empty = 0;
	}
}

static inline void update_list_time(uint64_t time, uint64_t add_time, list_t *list)
{
	list->time = time + add_time;
}

static inline void update_list_tail(list_t *list)
{
	list->act = list->list[list->tail];
	list->tail++;
	if (list->tail == SLOT) {
		list->tail = 0;
	}
	if (list->head == list->tail) {
		list->empty = 1;
	}
}

static void pending_cmd_add(int act, int plane, uint64_t ready_time)
{
	int i;
	for (i = 0; i < CMD_CNT; i++) {
		if (!pending_cmds[i].used) {
			pending_cmds[i].used = 1;
			pending_cmds[i].act = act;
			pending_cmds[i].plane = plane;
			pending_cmds[i].ready_time = ready_time;
			return;
		}
	}
	printf("pending_cmds full\n");
	exit(1);
}

static int pending_cmd_pop_ready(uint64_t now, int *act, int *plane, uint64_t *ready_time)
{
	int i;
	int idx = -1;
	uint64_t earliest = 0;

	for (i = 0; i < CMD_CNT; i++) {
		if (!pending_cmds[i].used) {
			continue;
		}
		if (pending_cmds[i].ready_time > now) {
			continue;
		}
		if (idx < 0 || pending_cmds[i].ready_time < earliest) {
			idx = i;
			earliest = pending_cmds[i].ready_time;
		}
	}

	if (idx < 0) {
		return 0;
	}

	*act = pending_cmds[idx].act;
	*plane = pending_cmds[idx].plane;
	*ready_time = pending_cmds[idx].ready_time;
	pending_cmds[idx].used = 0;
	return 1;
}

static void pending_comp_add(int act, uint64_t ready_time)
{
	int i;
	for (i = 0; i < CMD_CNT; i++) {
		if (!pending_comps[i].used) {
			pending_comps[i].used = 1;
			pending_comps[i].act = act;
			pending_comps[i].ready_time = ready_time;
			return;
		}
	}
	printf("pending_comps full\n");
	exit(1);
}

static int pending_comp_pop_ready(uint64_t now, int *act, uint64_t *ready_time)
{
	int i;
	int idx = -1;
	uint64_t earliest = 0;

	for (i = 0; i < CMD_CNT; i++) {
		if (!pending_comps[i].used) {
			continue;
		}
		if (pending_comps[i].ready_time > now) {
			continue;
		}
		if (idx < 0 || pending_comps[i].ready_time < earliest) {
			idx = i;
			earliest = pending_comps[i].ready_time;
		}
	}

	if (idx < 0) {
		return 0;
	}

	*act = pending_comps[idx].act;
	*ready_time = pending_comps[idx].ready_time;
	pending_comps[idx].used = 0;
	return 1;
}

static uint64_t next_event_time(uint64_t now, uint64_t host_next_issue, int outstanding)
{
	int i, j, k;
	uint64_t next = UINT64_MAX;

	if (outstanding < CMD_CNT && host_next_issue > now) {
		next = host_next_issue;
	}

	for (i = 0; i < CMD_CNT; i++) {
		if (pending_cmds[i].used && pending_cmds[i].ready_time > now &&
		    pending_cmds[i].ready_time < next) {
			next = pending_cmds[i].ready_time;
		}
		if (pending_comps[i].used && pending_comps[i].ready_time > now &&
		    pending_comps[i].ready_time < next) {
			next = pending_comps[i].ready_time;
		}
	}

	for (i = 0; i < CHAN_NUM; i++) {
		if (chan[i].state != CHAN_IDLE && chan[i].time > now &&
		    chan[i].time < next) {
			next = chan[i].time;
		}
	}

	for (i = 0; i < CHAN_NUM; i++) {
		for (j = 0; j < DIE_PER_CHAN; j++) {
			for (k = 0; k < PLANE; k++) {
				if (list_slot[i][j][k].act == 0xFFF) {
					continue;
				}
				if (list_slot[i][j][k].time > now &&
				    list_slot[i][j][k].time < next) {
					next = list_slot[i][j][k].time;
				}
			}
		}
	}

	if (next == UINT64_MAX) {
		return now + 1;
	}
	return next;
}

static int find_free_act(void)
{
	int i;
	for (i = 0; i < CMD_CNT; i++) {
		if (map[i] == 0) {
			return i;
		}
	}
	return -1;
}

int main(void)
{
	int i, j, k;
	uint64_t total_cmd = 0;
	uint64_t measured_cmds = 0;
	uint64_t sim_time = 0;
	uint64_t start_time = 0;
	uint64_t end_time = 0;
	uint64_t latency_sum = 0;
	uint64_t warmup_cmds;
	uint64_t host_next_issue = 0;
	int plane_share = 1;
	int outstanding = 0;
	int xor_ratio;

	if (DIE_NUM % CHAN_NUM != 0) {
		printf("Error: DIE_NUM must be divisible by CHAN_NUM\n");
		return 1;
	}

	if (PLANE * DIE_NUM > SLOT) {
		if ((PLANE * DIE_NUM) % SLOT != 0) {
			printf("Error: SLOT must divide PLANE * DIE_NUM\n");
			return 1;
		}
		plane_share = (PLANE * DIE_NUM) / SLOT;
		if (plane_share != 2 && plane_share != 4) {
			printf("Error planes! plane_share = %d\n", plane_share);
			return 1;
		}
		if (PLANE % plane_share != 0) {
			printf("Error: plane_share must divide PLANE\n");
			return 1;
		}
	}

	for (i = 0; i < CHAN_NUM; i++) {
		for (j = 0; j < DIE_PER_CHAN; j++) {
			for (k = 0; k < PLANE; k++) {
				list_slot[i][j][k].head = 0;
				list_slot[i][j][k].tail = 0;
				list_slot[i][j][k].act = 0xFFF;
				list_slot[i][j][k].empty = 1;
				list_slot[i][j][k].time = 0;
			}
		}
	}

	for (i = 0; i < CHAN_NUM; i++) {
		chan[i].state = CHAN_IDLE;
		chan[i].act = 0xFFF;
		chan[i].list_info = 0xFFFFFFFF;
		chan[i].time = 0;
	}

	for (i = 0; i < CMD_CNT; i++) {
		map[i] = 0;
		pending_cmds[i].used = 0;
		pending_comps[i].used = 0;
	}

	for (i = 0; i < DMA_ENGINES; i++) {
		dma_avail[i] = 0;
	}

	srand((unsigned)time(NULL));

	warmup_cmds = (CMD_CNT >= 512) ? (CMD_CNT * 3 / 4) : (CMD_CNT * 4 / 5);

	while (total_cmd < TARGET_CMDS) {
		int progress = 0;
		int act, plane;
		uint64_t ready_time;

		while (outstanding < CMD_CNT && host_next_issue <= sim_time) {
			act = find_free_act();
			if (act < 0) {
				break;
			}
			plane = rand() % (PLANE * DIE_NUM);

			uint64_t issue_done = host_next_issue + US_TO_TIME(HOST_SUBMIT_US);
			uint64_t cmd_end = schedule_pcie_tx(issue_done, CMD_BYTES);
			uint64_t ctrl_end = cmd_end + US_TO_TIME(CTRL_PROC_US);

			issue_time[act] = host_next_issue;
			host_next_issue = issue_done;

			pending_cmd_add(act, plane, ctrl_end);
			map[act] = 1;
			outstanding++;
			progress = 1;
		}

		while (pending_cmd_pop_ready(sim_time, &act, &plane, &ready_time)) {
			update_list_head(plane, plane_share, act);
			progress = 1;
		}

		for (i = 0; i < CHAN_NUM; i++) {
			uint64_t cur_time = sim_time;
			int can_break = 0;
			switch (chan[i].state) {
			case CHAN_IDLE:
				for (j = 0; j < DIE_PER_CHAN; j++) {
					for (k = 0; k < PLANE / plane_share; k++) {
						if (list_slot[i][j][k].empty == 0 &&
						    list_slot[i][j][k].act == 0xFFF) {
							chan[i].state = CHAN_CMD;
							chan[i].time = cur_time + CMD_TIME;
							chan_cmd_busy += CMD_TIME;
							chan[i].act = list_slot[i][j][k].list[list_slot[i][j][k].tail];
							chan[i].list_info = (j << 16) | k;
							update_list_tail(&list_slot[i][j][k]);
							can_break = 1;
							progress = 1;
							break;
						}
					}
					if (can_break) {
						break;
					}
				}

				if (can_break == 0) {
					for (j = 0; j < DIE_PER_CHAN; j++) {
						for (k = 0; k < PLANE / plane_share; k++) {
							if (list_slot[i][j][k].act != 0xFFF &&
							    cur_time >= list_slot[i][j][k].time) {
								chan[i].state = CHAN_DATA;
								chan[i].time = cur_time + NAND_DATA_TIME;
								chan_data_busy += NAND_DATA_TIME;
								chan[i].act = list_slot[i][j][k].act;
								chan[i].list_info = (j << 16) | k;
								can_break = 1;
								progress = 1;
								break;
							}
						}
						if (can_break) {
							break;
						}
					}
				}
				break;
			case CHAN_CMD:
				if (cur_time >= chan[i].time) {
					uint32_t list_info = chan[i].list_info;
					chan[i].state = CHAN_IDLE;
					list_slot[i][list_info >> 16][list_info & 0xFFFF].act = chan[i].act;
					chan[i].act = 0xFFF;
					chan[i].list_info = 0xFFFFFFFF;
					update_list_time(cur_time, TREAD_TIME,
							&list_slot[i][list_info >> 16][list_info & 0xFFFF]);
					progress = 1;
				}
				break;
			case CHAN_DATA:
				if (cur_time >= chan[i].time) {
					uint32_t list_info = chan[i].list_info;
					uint64_t bus_end = schedule_internal_bus(cur_time, NAND_BYTES);
					uint64_t ecc_end = bus_end + US_TO_TIME(ECC_DECODE_US);
					uint64_t dma_end = schedule_dma_pcie_rx(ecc_end, IO_SIZE_BYTES);
					uint64_t cqe_end = schedule_pcie_rx(dma_end, CQE_BYTES);
					uint64_t comp_time = cqe_end + US_TO_TIME(HOST_COMPLETE_US);

					pending_comp_add(chan[i].act, comp_time);

					list_slot[i][list_info >> 16][list_info & 0xFFFF].act = 0xFFF;
					chan[i].act = 0xFFF;
					chan[i].list_info = 0xFFFFFFFF;
					chan[i].state = CHAN_IDLE;
					progress = 1;
				}
				break;
			default:
				printf("Should not be here for chan state!\n");
				return 1;
			}
		}

		while (pending_comp_pop_ready(sim_time, &act, &ready_time)) {
			map[act] = 0;
			outstanding--;
			total_cmd++;

			if (start_time == 0 && total_cmd >= warmup_cmds) {
				start_time = ready_time;
				pcie_tx_busy_start = pcie_tx_busy;
				pcie_rx_busy_start = pcie_rx_busy;
				int_bus_busy_start = int_bus_busy;
				dma_busy_start = dma_busy;
				chan_cmd_busy_start = chan_cmd_busy;
				chan_data_busy_start = chan_data_busy;
			} else if (start_time != 0) {
				latency_sum += (ready_time - issue_time[act]);
				measured_cmds++;
			}
			progress = 1;
		}

		if (!progress) {
			sim_time = next_event_time(sim_time, host_next_issue, outstanding);
		}
	}

	end_time = sim_time;

	xor_ratio = (DIE_NUM > 64) ? 64 : DIE_NUM;
	double xor_penalty = (xor_ratio > 1) ?
		(double)(xor_ratio - 1) / (double)xor_ratio : 1.0;
	double elapsed_us = (double)(end_time - start_time) / (double)TIME_SCALE;

	if (elapsed_us <= 0 || measured_cmds == 0) {
		printf("Not enough samples to report performance\n");
		return 0;
	}

	double iops = (double)measured_cmds * 1000000.0 / elapsed_us;
	double avg_lat_us = (double)latency_sum / (double)measured_cmds / (double)TIME_SCALE;

	uint64_t pcie_tx_busy_meas = pcie_tx_busy - pcie_tx_busy_start;
	uint64_t pcie_rx_busy_meas = pcie_rx_busy - pcie_rx_busy_start;
	uint64_t int_bus_busy_meas = int_bus_busy - int_bus_busy_start;
	uint64_t dma_busy_meas = dma_busy - dma_busy_start;
	uint64_t chan_cmd_busy_meas = chan_cmd_busy - chan_cmd_busy_start;
	uint64_t chan_data_busy_meas = chan_data_busy - chan_data_busy_start;

	double pcie_tx_util = (double)pcie_tx_busy_meas / (double)(end_time - start_time);
	double pcie_rx_util = (double)pcie_rx_busy_meas / (double)(end_time - start_time);
	double int_bus_util = (double)int_bus_busy_meas / (double)(end_time - start_time);
	double dma_util = (double)dma_busy_meas / (double)((end_time - start_time) * DMA_ENGINES);
	double chan_util = (double)(chan_cmd_busy_meas + chan_data_busy_meas) /
			   (double)((end_time - start_time) * CHAN_NUM);

	printf("Performance = %.2f IOPS\n", iops * xor_penalty);
	printf("Avg latency = %.2f us\n", avg_lat_us);
	printf("PCIe TX util = %.1f%%, PCIe RX util = %.1f%%\n", pcie_tx_util * 100.0, pcie_rx_util * 100.0);
	printf("Internal bus util = %.1f%%\n", int_bus_util * 100.0);
	printf("DMA util = %.1f%% (engines=%d)\n", dma_util * 100.0, DMA_ENGINES);
	printf("Channel util = %.1f%%\n", chan_util * 100.0);

	return 0;
}
