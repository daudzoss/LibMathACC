/* sqrtable.c
 *
 * command-line utility to generate the N*2048-entry table of retlw 0x??
 *  instructions used by the square-root macros in sqrt_acc.inc
 *
 * the index into the lookup table is the argument to the square root function,
 *  divided by 16, unsigned of course
 *
 * sqrt16u() requires a 2048-entry table to return square roots in the domain
 *  0 to 32767, so this program should be invoked as: sqrtable 2048 > tbl.inc
 *
 * sqrt16f() requires a 4096-entry table to return square roots in the domain
 *  0 to 65535, so this program should be invoked as: sqrtable 4096 > tbl.inc
 *
 * in the PIC source code the file must be included at a 2048-word alignment:
 *
 *  org 0x800 ; or 0x1000, or 0x1800 etc.
 *  include tbl.inc
 *
 * although the first entry, i.e. sqrt(x<16) < 4, is never used due to code
 *  that must figure out whether the root in that range is 0, 1, 2 or 3 so in
 *  reality the required table length is N*2048-1
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main (int argc, char** argv)
{
  unsigned entry, N, root;

  if (argc < 2) {
    fprintf(stderr, "usage: \"%s N\" where N is the size of the table (multiple of 2048)\n", argv[0]);
    exit(0);
  } else
    N = atoi(argv[1]);

  for (entry = 0; entry < N; entry++) {
    double value;

    value = (entry << 4) | 0xf;
    root = (unsigned) sqrt(value);
    printf("\tretlw\t0x%x\t; %u: floor(sqrt(%g)) = %u\n", root, entry, value, root);
  }

  exit((unsigned)root & 0xff);
}
