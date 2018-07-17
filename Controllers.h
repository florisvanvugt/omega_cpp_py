
struct shm_instr_t; // forward decl
struct shm_live_t; // forward decl

struct setforce_t {
  // The forces to be applied in the x,y,z direction
  double x;
  double y;
  double z;

  setforce_t() { // Initialise
    x=0;y=0;z=0;
  }
};




setforce_t* forceFromController(shm_live_t* shm);
bool protection (shm_live_t* shm);
