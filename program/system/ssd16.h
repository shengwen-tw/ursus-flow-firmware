#ifndef __SSD16_H__
#define __SSD16_H__

#define simd_square_diff(offset) \
	 "ldr r4, [%[frame1], # ## offset]\n" \
	 "ldr r5, [%[frame2], # ## offset]\n" \
	 "ssub16 r4, r4, r5\n"       \
	 "smlald %[acc_1], %[acc_2], r4, r4\n"

#define calculate_ssd16(previos_frame, current_frame) \
	({                     \
	int32_t acc_32[2];   \
	asm (          \
	 "mov %[acc_1], $0\n" \
	 "mov %[acc_2], $1\n" \
	 \
	 "ldr r4, [%[frame1], #0]\n" \
	 "ldr r5, [%[frame2], #0]\n" \
	 "ssub16 r4, r4, r5\n"       \
	 "smlald %[acc_1], %[acc_2], r4, r4\n" \
	 \
	 "ldr r4, [%[frame1], #4]\n" \
	 "ldr r5, [%[frame2], #4]\n" \
	 "ssub16 r4, r4, r5\n"       \
	 "smlald %[acc_1], %[acc_2], r4, r4\n" \
	 \
	 : [acc_1] "=r" (acc_32[0]), [acc_2] "=r" (acc_32[1]) \
	 : [frame1] "r" (previos_frame), [frame2] "r" (current_frame) \
	 : "r4", "r5"             \
	);                        \
	acc_32[0] + acc_32[1];    \
	})

#endif
