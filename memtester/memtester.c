#include "memtester.h"
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#include "types.h"
#include "tests.h"
#include "sizes.h"

static const struct test tests[] = {
    { "Random Value", test_random_value },
    { "Compare XOR", test_xor_comparison },
    { "Compare SUB", test_sub_comparison },
    { "Compare MUL", test_mul_comparison },
    { "Compare DIV",test_div_comparison },
    { "Compare OR", test_or_comparison },
    { "Compare AND", test_and_comparison },
    { "Sequential Increment", test_seqinc_comparison },
    { "Solid Bits", test_solidbits_comparison },
    { "Block Sequential", test_blockseq_comparison },
    { "Checkerboard", test_checkerboard_comparison },
    { "Bit Spread", test_bitspread_comparison },
    { "Bit Flip", test_bitflip_comparison },
    { "Walking Ones", test_walkbits1_comparison },
    { "Walking Zeroes", test_walkbits0_comparison },
#ifdef TEST_NARROW_WRITES
    { "8-bit Writes", test_8bit_wide_random },
    { "16-bit Writes", test_16bit_wide_random },
#endif
    { NULL, NULL }
};

unsigned int memtester_stm32(void* ram, size_t bufsize, size_t loops) {
    size_t halflen = bufsize / 2;
    size_t count = halflen / sizeof(ul);
    ulv *bufa = (ulv *) ram;
    ulv *bufb = (ulv *) ((size_t) ram + halflen);
	size_t loop;
	unsigned int exit_code = 0;
	size_t i;

    for(loop=1; ((!loops) || loop <= loops); loop++) {
        if (test_stuck_address(ram, bufsize / sizeof(ul))) {
            exit_code |= 1;
        }
        for (i=0;;i++) {
            if (!tests[i].name) break;
            /* If using a custom testmask, only run this test if the
               bit corresponding to this test was set by the user.
             */
            // if (testmask && (!((1 << i) & testmask))) {
                // continue;
            // }
            if (tests[i].fp(bufa, bufb, count)) {
                exit_code |= 1 << (i+1);
            }
            /* clear buffer */
            memset((void *) ram, 255, bufsize);
        }
    }

	return exit_code;
}

unsigned long long rand_state;
unsigned int rand32() {
	rand_state = rand_state * 6364136223846793005LL + 1;
	return (unsigned int)((rand_state >> 32) & RAND_MAX);
}