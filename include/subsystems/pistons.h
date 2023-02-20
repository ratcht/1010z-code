#ifndef PISTONS_H
#define PISTONS_H
#include "vex.h"

inline bool intakeUp = false;
inline bool blooperToggle= false;

inline void EndgameFire() { 
  EndgamePiston.set(true); 
} 

inline void ShootEngaged() { 
  MagazinePiston.set(false); 
} 

inline void IntakeEngaged() { 
  MagazinePiston.set(true); 
} 


inline void BlooperFire() { 
  
} 

inline void BlooperToggle(){
  blooperToggle = !blooperToggle;
  BlooperPiston.set(blooperToggle); 
}


#endif