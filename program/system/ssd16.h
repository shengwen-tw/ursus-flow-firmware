#ifndef __SSD16_H__
#define __SSD16_H__

/* convet image (r, c) position to image array offset in byte
 * input = row, column
 * output = (row * sizeof(uint16_t) * image_size) + (column * sizeof(uint16_t)) */
#define img_offset_calc(r, c) \
	"((" #r "* 2) * 40) + (" #c "* 2)" //((r * 2) * 40) + (c * 2)

#define simd_square_diff(r, c) \
	 "ldr r4, [%[frame1], " img_offset_calc(r, c) "]\n" \
	 "ldr r5, [%[frame2], " img_offset_calc(r, c) "]\n" \
	 "ssub16 r4, r4, r5\n"       \
	 "smlald r6, r7, r4, r4\n"

#define simd_calculate_ssd16_full(template_image, search_image) \
	({ \
	uint32_t acc_32; \
	asm ( \
	 "mov r6, $0\n"  \
	 "mov r7, $1\n"  \
	 \
	 simd_square_diff(0, 0) \
	 simd_square_diff(0, 2) \
	 simd_square_diff(0, 4) \
	 simd_square_diff(0, 8) \
	 \
	 simd_square_diff(1, 0) \
	 simd_square_diff(1, 2) \
	 simd_square_diff(1, 4) \
	 simd_square_diff(1, 8) \
	 \
	 simd_square_diff(2, 0) \
	 simd_square_diff(2, 2) \
	 simd_square_diff(2, 4) \
	 simd_square_diff(2, 8) \
	 \
	 simd_square_diff(3, 0) \
	 simd_square_diff(3, 2) \
	 simd_square_diff(3, 4) \
	 simd_square_diff(3, 8) \
	 \
	 simd_square_diff(4, 0) \
	 simd_square_diff(4, 2) \
	 simd_square_diff(4, 4) \
	 simd_square_diff(4, 8) \
	 \
	 simd_square_diff(5, 0) \
	 simd_square_diff(5, 2) \
	 simd_square_diff(5, 4) \
	 simd_square_diff(5, 8) \
	 \
	 simd_square_diff(6, 0) \
	 simd_square_diff(6, 2) \
	 simd_square_diff(6, 4) \
	 simd_square_diff(6, 8) \
	 \
	 simd_square_diff(7, 0) \
	 simd_square_diff(7, 2) \
	 simd_square_diff(7, 4) \
	 simd_square_diff(7, 8) \
	 \
	 "add r6, r6, r7\n"    \
	 "str r6, %[acc]\n"    \
	 : [acc] "=m" (acc_32) \
	 : [frame1] "r" (template_image), [frame2] "r" (search_image) \
	 : "r4", "r5", "r6", "r7" \
	);                        \
	acc_32; \
	})

#define simd_calculate_ssd16_row(template_image, search_image, row_offset) \
	({ \
	uint32_t acc_32; \
        uint16_t *_template = template_image + (row_offset * FLOW_IMG_SIZE); \
        uint16_t *_search = search_image + (row_offset * FLOW_IMG_SIZE);   \
	asm ( \
	 "mov r6, $0\n"  \
	 "mov r7, $1\n"  \
	 \
	 simd_square_diff(0) \
	 simd_square_diff(2) \
	 simd_square_diff(4) \
	 simd_square_diff(6) \
	 \
	 "add r6, r6, r7\n"    \
	 "str r6, %[acc]\n"    \
	 : [acc] "=m" (acc_32) \
	 : [frame1] "r" (_template), [frame2] "r" (_search) \
	 : "r4", "r5", "r6", "r7" \
	);                        \
	acc_32; \
	})

#endif
