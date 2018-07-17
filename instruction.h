#ifndef INSTRUCTION_H
#define INSTRUCTION_H

struct shm_instr_t; // forward decl
struct shm_live_t; // forward decl

//#include "struct_shm.h"


void new_instruction(shm_instr_t* sh_instr_memory, shm_live_t* sh_live_memory);
void new_trajectory(shm_instr_t* sh_instr_memory, shm_live_t* sh_live_memory);

#endif
