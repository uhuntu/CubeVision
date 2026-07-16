#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_MOVES	30

extern char U[3][3], D[3][3], L[3][3], R[3][3], F[3][3], B[3][3];
extern char cube[3][3][3][2];
extern char tcube[3][3][3];
extern char moves[MAX_MOVES];
extern int cur_move, phase1_cur, max_sol, do_max20;
