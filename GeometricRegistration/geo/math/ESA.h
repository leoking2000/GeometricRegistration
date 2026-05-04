#ifndef _ESA_
#define _ESA_

#include <float.h>

#define frand()   ((float)rand()/(float)RAND_MAX)

//mersenne twister
// much more improved results/alignment in Tombstone
//#define frand()   ((double)randomMT()/(double)(UINT_MAX))

#ifndef mymin
#define mymin(a,b) (((a)<=(b))?(a):(b))
#define mymax(a,b) (((a)>=(b))?(a):(b))
#endif

static int ESA_Initialized = 0;

void Partitioning(long n, long p, long *p_index ,long *p_accum);



// The Enhanced Simulated Annealing method as presented by P. Siarry, G. Berthiau,
// F. Durbin and J. Haussy in "Enhanced Simulated Annealing for Globally Minimizing
// Functions of Many-Continuous Variables", ACM Trans on Mathematical Software, 23(2),
// June 1997, pp.209-228.  
//
// ARGUMENTS: 
//
// dim		         : the dimensionality of the search space. 
// subdim		     : the dimensionality of the partitioning subspace (subdim <= dim).
// x_init            : an optional initial starting point. x_init is randomly selected if
//				       this argument is NULL. 
// x_best            : the returned variable vector.
// x_min             : the minimum x values vector.
// x_max             : the maximum x values vector. 
// step_fraction     : a vector defining the maximum ( normalized ) jump for each variable.
// wraparound        : a flag vector (of size dim) specifying if the i-th variable values 
//				       must wrap around the limits ( 1=wrap, 0=truncate )  
// e(x)              : pointer the cost function to be minimized.
// monitor(x,e)      : pointer a monitoring function.
// iterations_max    : maximum itaerations.
float EnhancedSimulatedAnnealing( long dim, long subdim, float *x_init, float *x_best,
								  float *x_min, float *x_max, float *step_fraction,
								  long *wraparound, float (*e)( float*), 
								  void (*monitor)(float*,float), long iterations_max );

float EnhancedSimulatedAnnealingPlus
( long dim, long subdim, float *x_init, float *x_best,
  float *x_min, float *x_max, float *step_fraction,
  long *wraparound, float (*e)( float*), 
  void (*monitor)(float*,float), long iterations_max  );


#endif 