#ifndef _NVMEVIRT_PERF_MODEL_H
#define _NVMEVIRT_PERF_MODEL_H

#include <linux/types.h>

/*
 * Random read/write performance simulator for channel + NAND side.
 * All timing values are in nanoseconds unless stated otherwise.
 *
 * These defaults mirror the standalone sample logic (read-only) and are
 * extended for writes. Override by defining macros before including this
 * header if needed.
 */
#ifndef NVMEV_PERF_CMD_OVERHEAD_NS
#define NVMEV_PERF_CMD_OVERHEAD_NS (1700ULL) /* 1.7 us */
#endif
#ifndef NVMEV_PERF_TR_NS
#define NVMEV_PERF_TR_NS (40ULL * 1000ULL) /* 40 us */
#endif
#ifndef NVMEV_PERF_TPROG_NS
#define NVMEV_PERF_TPROG_NS (200ULL * 1000ULL) /* 200 us */
#endif
#ifndef NVMEV_PERF_CHAN_SPEED_MT
#define NVMEV_PERF_CHAN_SPEED_MT (2400U) /* MT/s */
#endif
#ifndef NVMEV_PERF_ECC_PARITY_BYTES
#define NVMEV_PERF_ECC_PARITY_BYTES (600U)
#endif
#ifndef NVMEV_PERF_QD
#define NVMEV_PERF_QD (512U)
#endif
#ifndef NVMEV_PERF_CHAN_NUM
#define NVMEV_PERF_CHAN_NUM (16U)
#endif
#ifndef NVMEV_PERF_DIE_NUM
#define NVMEV_PERF_DIE_NUM (128U)
#endif
#ifndef NVMEV_PERF_PLANES
#define NVMEV_PERF_PLANES (4U)
#endif
#ifndef NVMEV_PERF_IWL_SLOT
#define NVMEV_PERF_IWL_SLOT (256U)
#endif
#ifndef NVMEV_PERF_ELEMENT
#define NVMEV_PERF_ELEMENT (1U * 32U * 1024U)
#endif
#ifndef NVMEV_PERF_PAGE_BYTES
#define NVMEV_PERF_PAGE_BYTES (4096U)
#endif

struct nvmev_perf_result {
	u64 read_iops;
	u64 write_iops;
	u64 read_duration_ns;
	u64 write_duration_ns;
	u64 read_completed;
	u64 write_completed;
};

int nvmev_perf_run(struct nvmev_perf_result *result);

#endif /* _NVMEVIRT_PERF_MODEL_H */
