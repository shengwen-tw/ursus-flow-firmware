#ifndef __SSD16_H__
#define __SSD16_H__

#define simd_square_diff(offset) \
	 "ldr r4, [%[frame1], " #offset "]\n" \
	 "ldr r5, [%[frame2], " #offset "]\n" \
	 "ssub16 r4, r4, r5\n"       \
	 "smlald %[acc_1], %[acc_2], r4, r4\n"

#define calculate_ssd16_full(template_image, search_image) \
	({                     \
	int32_t acc_32[2];   \
	asm (          \
	 "mov %[acc_1], $0\n" \
	 "mov %[acc_2], $1\n" \
	 \
	 simd_square_diff(0) \
	 simd_square_diff(2) \
	 simd_square_diff(4) \
	 simd_square_diff(6) \
	 \
	 simd_square_diff(40) \
	 simd_square_diff(42) \
	 simd_square_diff(44) \
	 simd_square_diff(46) \
	 \
	 simd_square_diff(80) \
	 simd_square_diff(82) \
	 simd_square_diff(84) \
	 simd_square_diff(86) \
	 \
	 simd_square_diff(120) \
	 simd_square_diff(122) \
	 simd_square_diff(124) \
	 simd_square_diff(126) \
	 \
	 simd_square_diff(160) \
	 simd_square_diff(162) \
	 simd_square_diff(164) \
	 simd_square_diff(166) \
	 \
	 simd_square_diff(200) \
	 simd_square_diff(202) \
	 simd_square_diff(204) \
	 simd_square_diff(206) \
	 \
	 simd_square_diff(240) \
	 simd_square_diff(242) \
	 simd_square_diff(244) \
	 simd_square_diff(246) \
	 \
	 simd_square_diff(280) \
	 simd_square_diff(282) \
	 simd_square_diff(284) \
	 simd_square_diff(286) \
	 : [acc_1] "=r" (acc_32[0]), [acc_2] "=r" (acc_32[1]) \
	 : [frame1] "r" (template_image), [frame2] "r" (search_image) \
	 : "r4", "r5"             \
	);                        \
	acc_32[0] + acc_32[1];    \
	})

#endif
