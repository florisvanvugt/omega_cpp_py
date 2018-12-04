#include "Controllers.h"
#include <math.h>       /* pow, exp, min */
#include "struct_shm.h"
#include <iostream>
#include <stdlib.h> /* abs */
#include <stdio.h>


/*
   Compute the target position corresponding to the current time
   for a minimum-jerk trajectory.

   Arguments
   movt : the current time in the movement (t=0 is move start)
   movdur : the desired duration of the current movement
*/
void compute_minjerk_target (double movt,double movdur,shm_live_t* shm) {
/*
  source for minimum jerk computation:
  Flash, Tamar, and Neville Hogan. "The coordination of arm
  movements: an experimentally confirmed mathematical model." The
  journal of Neuroscience 5, no. 7 (1985): 1688-1703.
*/
  double tau = movt/movdur; // the proportion of the movement completed
  double position = 10*pow(tau,3)-15*pow(tau,4)+6*pow(tau,5); // minimum jerk position
  shm->desired_x = (shm->start_x) + (shm->target_x - shm->start_x)*position;
  shm->desired_y = (shm->start_y) + (shm->target_y - shm->start_y)*position;
  shm->desired_z = (shm->start_z) + (shm->target_z - shm->start_z)*position;

  // Compute the desired velocity
  double velocity = -30*pow(movt,4)/pow(movdur,5) + 60*pow(movt,3)/pow(movdur,4) - 30*pow(movt,2)/pow(movdur,3);
  shm->desired_vel_x = (shm->start_x - shm->target_x)*velocity;
  shm->desired_vel_y = (shm->start_y - shm->target_y)*velocity;
  shm->desired_vel_z = (shm->start_z - shm->target_z)*velocity;

}




/*
   Compute forces as defined by a P controller (position-only)
   This reads the current position from shared memory
   and compares it against the target position (shm->target_{x,y,z})
*/
setforce_t* compute_p_forces (shm_live_t* shm )
{
  double stiffness = shm->stiffness;

  // Protection to avoid too large movements in 1ms
  //if (!protection (shm)) return (new setforce_t);

  // compute reaction force
  setforce_t *f = new setforce_t;
  f->x = -stiffness*(shm->x - shm->desired_x);
  f->y = -stiffness*(shm->y - shm->desired_y);
  f->z = -stiffness*(shm->z - shm->desired_z);
  return(f);

}




/*
   Compute forces as defined by a PD controller (position-and-derivative).
   This reads the current position from shared memory
   and compares it against the target position (shm->target_{x,y,z})
*/
setforce_t *compute_pd_forces (shm_live_t* shm)
{
  double stiffness = shm->stiffness;
  double damping   = shm->damping;

  // Protection to avoid too large movements in 1ms
  //if (!protection (shm)) return (new setforce_t);

  // compute reaction force
  setforce_t *f = new setforce_t;
  f->x = -stiffness*(shm->x - shm->desired_x) - damping*(shm->vel_x - shm->desired_vel_x);
  f->y = -stiffness*(shm->y - shm->desired_y) - damping*(shm->vel_y - shm->desired_vel_y);
  f->z = -stiffness*(shm->z - shm->desired_z) - damping*(shm->vel_z - shm->desired_vel_z);

  return(f);

}


bool protection (shm_live_t* shm) {
  double delta_x = shm->x - shm->desired_x;
  double delta_y = shm->y - shm->desired_y;
  double delta_z = shm->z - shm->desired_z;
  if (sqrt(pow(delta_x,2)+pow(delta_y,2)+pow(delta_z,2))>shm->distance_max){
    cout << "CAUTION : Desired_position is too far from current_position" << endl;
    shm->move_done=1;
    shm->controller=0;
    return (false);
  }
  else return true;
}


setforce_t* ControllerNull() {
  return (new setforce_t);
}



setforce_t* ControllerMoveToPoint(shm_live_t* shm) {
  double movt   = shm->move_iterator*shm->main_loop_time; // how long we have been moving
  double movdur = shm->movement_duration;      // how long this move should take
  // If the movement is completed, switch back to an other controller
  if (movt >= movdur) {
    shm->controller = 2;
    shm->move_done = 1;
    return (new setforce_t);
  }

  else {
    // Compute where I should be at the current time
    compute_minjerk_target(movt,movdur,shm);
    (shm->move_iterator)++;
    return compute_pd_forces(shm);

  }
}

setforce_t* ControllerPlayMovement (shm_live_t* shm) {
   double movt = shm->move_iterator;
   double movdur = shm->trajectory_size;
   if (movt <= movdur) {
     shm->desired_x = (shm->trajectory_x)[shm->move_iterator];
     shm->desired_y = (shm->trajectory_y)[shm->move_iterator];
     shm->desired_z = (shm->trajectory_z)[shm->move_iterator];
     if (movt==0) {
       shm->desired_vel_x =0;
       shm->desired_vel_y =0;
       shm->desired_vel_z =0;
     }
     else {
       shm->desired_vel_x = (shm->desired_x -(shm->trajectory_x)[shm->move_iterator -1])/shm->main_loop_time;
       shm->desired_vel_y =(shm->desired_y -(shm->trajectory_y)[shm->move_iterator -1])/shm->main_loop_time;
       shm->desired_vel_z = (shm->desired_z -(shm->trajectory_z)[shm->move_iterator -1])/shm->main_loop_time;
     }
     (shm->move_iterator)++;
     return compute_pd_forces(shm);
   }
   else {
     shm->controller = 0;
     shm->move_done = 1;
     return new setforce_t; // empty forces
   }
 }


setforce_t* ControllerViscousForceField(shm_live_t* shm) {
  // A simple viscous force field controller.
  setforce_t *f = new setforce_t;
  f->x = -shm->viscosity * shm->vel_x;
  f->y = -shm->viscosity * shm->vel_y;
  f->z = -shm->viscosity * shm->vel_z;
  return(f);
}



setforce_t* ControllerHoldAtPoint(shm_live_t* shm) {
  //shm_t * shm = getShm();
  //computedPos.set(shm->target_x, shm->target_y, shm->target_z);
  shm->desired_x = shm->target_x;
  shm->desired_y = shm->target_y;
  shm->desired_z = shm->target_z;
  shm->desired_vel_x = 0;
  shm->desired_vel_y = 0;
  shm->desired_vel_z = 0;
  shm->gap_iterator ++;
  return compute_pd_forces(shm);
}


setforce_t* ControllerActiveToNull(shm_live_t* shm) {
  double movt = (shm->move_iterator);
  double movdur = (shm->active_to_null);  // from active to null in 0.5 second
  if (movt<=movdur){
    setforce_t *f = compute_pd_forces(shm);
    double a = exp(-movt/(movdur/5));
    (f->x)*=a;
    (f->y)*=a;
    (f->z)*=a;
    (shm->move_iterator)++;
    return f;
   }
   else {
     shm->controller = 0;
     shm->move_done = 1;
     return new setforce_t; // empty forces
   }
  }

setforce_t* ControllerNullHoldX (shm_live_t* shm) {
  double stiffness = shm->stiffness;
  double damping   = shm->damping;
  shm->desired_x = shm->target_x;
  shm->desired_vel_x = 0;
  setforce_t *f = new setforce_t;
  f->x = -stiffness*(shm->x - shm->desired_x) - damping*(shm->vel_x - shm->desired_vel_x);
  return (f);
}




struct arcdist_t {
  double y,z; // The closest point on the arc
  double dist; // The corresponding distance
};





arcdist_t* dist_to_arc(double y,double z,double arcy,double arcz,double arcrad, bool upper) {
  /* 
     Return the distance from the position (y,z) to an arc, centered at (arcy,arcz), and with
     radius arcrad.
     If upper is true, then the arc extends to zs that are greater than arcz, 
     and if it's false, then the arc extends to zs that are smaller than arcz (that sounds
     cryptical, sorry, I can't find a better way to say that).

     Also returns the (y,z) position of the closest point on that arc.
  */

  arcdist_t* arcd = new arcdist_t;
  
  /* First, let's determine whether we are on the "right" side of the arc,
     i.e. if the arc extends upwards (in the z dimension), are we above it as well? */
  if ( (upper && z>arcz) || ((!upper) && z<arcz)) {
    /* We are on the "correct" side of the arc, yay! */

    // Compute the vector V from the center of the arc to the point (y,z)
    double vY = y-arcy;
    double vZ = z-arcz;

    // Compute the magnitude of the V vector, i.e. pointing to the center
    double magV = sqrt(pow(vY,2) + pow(vZ,2));

    arcd->dist = fabs(magV-arcrad); // the distance to the arc

    // Turn the v vector into a unit vector (v/|v|) and multiply it by the radius, that should get us
    // where we want to be.
    arcd->y    = arcy + (vY/magV) * arcrad;
    arcd->z    = arcz + (vZ/magV) * arcrad;

  } else {

    /* Okay, this is if we are on the "wrong" side of the arc.
       In this case, the closest point to the arc is always one of the edges 
       (Emily proved this last summer). */
    double dleftedge  = sqrt(pow(y-(arcy-arcrad),2) + pow(z-arcz,2));
    double drightedge = sqrt(pow(y-(arcy+arcrad),2) + pow(z-arcz,2));

    if (dleftedge<drightedge) {
      /* We're closest to the left edge, so return that */
      arcd->dist   = dleftedge;
      arcd->y      = arcy-arcrad; 
    } else {
      arcd->dist   = drightedge;
      arcd->y      = arcy+arcrad;
    }
    arcd->z = arcz;
    
  }
  return arcd;
  
}






setforce_t* ControllerArcChannel(shm_live_t* shm) {
  /* This is a controller that is basically a force channel,
     but in the shape of an arc. */

  /* 
     Ok, so the idea is that we will, based on the subject's current position, we will compute
     the closest point in the arc, i.e. where they "should" be. We put that in 
     shm->desired_{x,y,z}
     and then compute_pd_forces will take care of computing the appropriate forces towards that.
  */

  /* Compute shm->target */
  arcdist_t* leftdist  = dist_to_arc(shm->y,shm->z, shm->arc_base_y-shm->arc_radius, shm->arc_base_z, shm->arc_radius, true);
  //printf("Doing right\n");
  arcdist_t* rightdist = dist_to_arc(shm->y,shm->z, shm->arc_base_y+shm->arc_radius, shm->arc_base_z, shm->arc_radius, false);
  
  if (leftdist->dist < rightdist->dist) { /* If the left arc is the closest to the current position, use that as the attractor */
    shm->desired_y = leftdist->y;  shm->desired_z = leftdist->z;  shm->arc_dist = leftdist->dist;
  } else { /* Else, use the right arc as attractor */
    shm->desired_y = rightdist->y; shm->desired_z = rightdist->z; shm->arc_dist = rightdist->dist;
  }
  shm->desired_x = shm->arc_x_plane;
  // So now we have set shm->desired_{x,y,z}, time to continue!

  /* Get the corresponding PD forces for shm->target */
  return compute_pd_forces(shm);
}






setforce_t* forceFromController(shm_live_t* shm) {

  // Execute the controller that is currently selected,
  // which will compute the forces fx,fy,fz that we want to apply
  switch (shm->controller) {
  case 0:   /* null field (no forces) */
    return ControllerNull(); break;
  case 1:   /*movement to point*/
    return ControllerMoveToPoint(shm); break;
  case 2:   /*hold at point*/
    return ControllerHoldAtPoint(shm); break;
  case 3:   /*viscous force*/
    return ControllerViscousForceField(shm); break;
  case 4:
    return ControllerPlayMovement (shm); break;
  case 5:
    return ControllerActiveToNull (shm); break;
  case 6:
    return ControllerNullHoldX (shm); break;
  case 7:
    return ControllerArcChannel (shm); break;
  };

  return ControllerNull(); // Default: if an invalid controller is specified

};
