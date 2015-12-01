/* atan_tbl.c
 *
 * command-line utility to generate the N*4-entry table (default 256-entry)
 *  of retlw 0x?? instructions that performs the inverse tangent operation
 *
 * the index into the lookup table is the pair of N/16-bit signed side
 *  lengths (default 4-bit) expressed as a ratio with the "opposite" side in
 *  the top four bits and the "adjacent" side in the bottom four bits as
 *  performed by normal(macro) in atan_acc.inc
 *
 * this program should be invoked as: atan_tbl 256 > tbl.inc
 *
 * in the PIC source code the file must be included at a 256-word alignment:
 *
 *  org 0x100 ; or 0x200, or 0x300 etc.
 *  include tbl.inc
 */

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

int main(int argc, char** argv) {
  const double deg_per_rad = 360.0/(2*acos(-1.0));

  uint8_t deg2[65536];
  int elements = 256, element = 0, x, y, min, max;

  if (argc > 1)
    if ((elements = atoi(argv[1])) > 65536)
      elements = 256;

  max = (int) (sqrt(elements)/2);
  min = -max;

  for (y = min; y < max; y++)
    for (x = min; x < max; x++) {
      int deg;

      if (x)
	deg = (int) ( deg_per_rad * atan((double)y/(double)x) );
      else if (y)
	deg = (y > 0) ? +90 : -90;
      else
	deg = 0;


      if (x < 0)
	deg += 180;
      else if (y < 0)
	deg += 360;
      if (deg < 0) exit(deg); // assert: deg >= 0

      element = ((y & 0x0f) << 4) | (x & 0x0f);
      deg2[element] = (uint8_t) ( deg/2 );
    }

  for (element = 0; element < elements; element++)
    printf("\tretlw\t0x%x\t; %d: atan(0x%X/0x%X)=%u degrees\n", deg2[element],
	   element,
           y = (int8_t) ((element & 0xf0)>>4), x = (int8_t) (element & 0x0f),
           deg2[element]*2);
  exit(0);
}
