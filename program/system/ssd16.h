#ifndef __SSD16_H__
#define __SSD16_H__

#define calculate_ssd16(previos_frame, current_frame) \
	({                   \
	uint64_t acc_64 = 1; \
	asm volatile(        \
	 "ldr r4, [%[frame1], #0]\n" \
	 "ldr r5, [%[frame2], #0]\n" \
	 "ssub16 r4, r4, r5\n"       \
	 "smlald %[acc_64], r4, r4, %[acc_64]\n" \
	 : [acc_64] "=r" (acc_64) \
	 : [frame1] "r" (previos_frame), [frame2] "r" (current_frame) \
	 : "r4", "r5"             \
	);                        \
	((uint32_t *)&acc_64)[0] + ((uint32_t *)&acc_64)[1];    \
	})

#endif
