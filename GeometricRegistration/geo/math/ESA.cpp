#include <float.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <memory.h>
#include "ESA.h"

#pragma warning( disable : 6993 4244 4305)

#define mymin(a,b) (((a)<=(b))?(a):(b))
#define mymax(a,b) (((a)>=(b))?(a):(b))

#define frand() rng.Float(0.0f, 1.0f)

// Partitioning procedure for the ESA algorithm
static void Partitioning(long n, long p, long* p_index, long* p_accum, geo::Random& rng)
{
	long i;
	long p_accum_min;
	long elements_set = 0;

	// Reset p_index
	if (n == p)
	{
		// no need to select some, select all of 'em
		for (i = 0; i < n; i++)
		{
			p_index[i] = 1;
			p_accum[i]++;
		}
		return;
	}
	else
		memset(p_index, 0, n * sizeof(long));

	while (elements_set < p)
	{
		// find the smallest element of p_accum
		p_accum_min = p_accum[0];
		for (i = 1; i < n; i++)
			p_accum_min = mymin(p_accum_min, p_accum[i]);
		// select the variables which are selected the least times
		for (i = 0; (i < n) && (elements_set < p); i++)
			if ((abs(p_accum_min - p_accum[i]) < 2) &&
				(p_index[i] == 0) &&
				(frand() > 0.5)
				)
			{
				p_index[i]++;
				p_accum[i] += p_index[i];
				elements_set++;
			}
	}
}

float EnhancedSimulatedAnnealingPlus
(long dim, long subdim, float* x_init, float* x_best,
	float* x_min, float* x_max, float* step_fraction,
	long* wraparound, std::function<float(float*)> e,
	void (*monitor)(float*, float), long iterations_max,
	geo::Random& rng)
{
	long no_x_init = 0,			// flag: 1 if x_init==NULL
		vsize,					// vector size in bytes
		i, j,					// temporary counter	
		up_hill_counter,		// up-hill movement counter
		e_num,				    // number of errors calculated at one temperature 
		e_num_uphill,			// number of up-hill moves
		e_num_accepted,			// number of accepted moves
		iterations,				// total number of iterations (ie error estimations)
		no_downhill_loops,		// # of loops where no downhill movement took place
		no_stagnant_best,       // # of loops where the best value did not change much. 
		step_too_small,			// flag: 1 if there exists a step(k) too small to go on
		equilibrium_reached = 0;  // flag: 1 if a thermodynamical equilibrium condition is
	// satisfied
	long* p,						// partition flag vector: during a temperature cycle,
		// subdim variable flags are set to 1, indicating that
		// the i-th variable is part of the current annealing
		* p_accum,				// partition flag vector: each cell indicates the
		// number of times the corrsponding variable has been
		// selected.
		* moves_accepted;		// vector holding the accepted # of moves for each var
	float* step_init,			// initial step
		* step,				// current step	
		* x,					// the current x vector
		* x_old;				// the previous x vector
	float e_best = FLT_MAX,		// best error
		e_best_old,           // previous best error
		e_var = 0.0,			// error variance
		e_avg = 0.0,			// error average in one temperature cycle
		e_min,				// minimum error encountered in one temperature cycle
		e_old,				// previous error
		e_new,				// current error 
		uphill_distance,      // uphill distance so far at current temperature 
		P_init = 0.5,			// initial desired uphill movement propability
		T_init,				// initial temperature	
		T_stop,				// final (lower) temperature 
		T,					// current temperature
		a_min = 0.6,			// minimum temperature decrease rate
		a_max = 0.9,			// maximum temperature decrease rate
		a,					// temperature decrease rate
		val,					// a scalar temporary value
		sum1,					// eq/um condition variable				
		stepval,				// temporary step value
		N1 = 8,//12				// eq/um condition threshold				
		N2 = 80,//100				// eq/um condition threshold				
		rate_max = 0.19,//0.2			// max acceptance rate for each variable
		rate_min = 0.05,		// min acceptance rate for each variable
		step_mult = 2.0,		// step multiplier for step increase
		step_div = 0.50,		// step multiplier for step decrease
		acc_best = 10e-4,       // best error improvement accuracy limit
		acc_rel = 10e-6,		// relative accuracy acceptance limit 
		acc_abs = 10e-8;		// absolute accuracy acceptance limit


	if ((x_min == NULL) || (x_max == NULL) || (step_fraction == NULL) ||
		(wraparound == NULL) || (e == NULL))
		return FLT_MAX;

	vsize = dim * sizeof(float);
	step_init = (float*)malloc(vsize);
	step = (float*)malloc(vsize);
	x = (float*)malloc(vsize);
	x_old = (float*)malloc(vsize);
	if (x_init == NULL)
	{
		x_init = (float*)malloc(vsize);
		no_x_init = 1;
	}
	if (x_best == NULL) x_best = (float*)malloc(vsize);
	if (subdim > dim) subdim = dim;
	p = (long*)malloc(dim * sizeof(long));
	moves_accepted = (long*)malloc(dim * sizeof(long));
	p_accum = (long*)malloc(dim * sizeof(long));


	// jump step initialization
	for (i = 0; i < dim; i++)
		step_init[i] = (x_max[i] - x_min[i]) * step_fraction[i];
	memcpy(step, step_init, vsize);

	// x vector initialization
	if (no_x_init)
		for (i = 0; i < dim; i++)
			x_init[i] = (x_max[i] - x_min[i]) * frand();

	// best x vector initialization
	memcpy(x_best, x_init, vsize);

	// estimate uphill movement error variance
	for (j = 0; j < 50; j++)
	{
		float e1, e2;
		// supposed low value
		for (i = 0; i < dim; i++)
			x[i] = x_min[i] + (x_max[i] - x_min[i]) * frand();
		e1 = e(x);
		if (e_best > e1)
		{
			e_best = e1;
			memcpy(x_best, x, vsize);
		}
		// supposed high value
		for (i = 0; i < dim; i++)
		{
			stepval = step_init[i] * (2 * frand() - 1.0); // current step
			if ((x[i] + stepval > x_max[i]) || (x[i] + stepval < x_min[i])) // range check 
				x[i] = x[i] - stepval; // range fix
			else
				x[i] = x[i] + stepval;
		}
		e2 = e(x);
		if (e_best > e2)
		{
			e_best = e2;
			memcpy(x_best, x, vsize);
		}
		e_var += (float)fabs((double)(e2 - e1));
	}
	e_var /= 50.0;
	//if (no_x_init)
	memcpy(x, x_best, vsize);
	//else 
	//	memcpy(x,x_init,vsize);

	// estimate initial and final temperatures
	T_init = -e_var / log(P_init);
	T_stop = -(acc_rel * e_var + acc_abs) / log(acc_rel * P_init + acc_abs);
	T = T_init;

	// calculate initial error
	e_new = e(x);
	e_old = e_new;
	e_best = e_new;
	e_best_old = 2 * e_best;
	iterations = 0;
	step_too_small = 0;
	no_downhill_loops = 0;
	no_stagnant_best = 0;

	if (monitor != NULL) (*monitor)(x, e_new);
	// cooling loop
	while ((T > T_stop) &&
		(iterations < iterations_max) &&
		(!step_too_small) &&
		//   (no_stagnant_best<4)&&
		(no_downhill_loops < 3))

	{
		e_avg = 0.0;
		e_num = 0;
		e_num_uphill = 0;
		e_num_accepted = 0;
		e_min = e_old;
		uphill_distance = 0.0;
		memset(p_accum, 0, dim * sizeof(long));
		memset(moves_accepted, 0, dim * sizeof(long));
		equilibrium_reached = 0;
		no_stagnant_best = 0;

		// stabilizing loop
		while (!equilibrium_reached)
		{
			Partitioning(dim, subdim, p, p_accum, rng);
			memcpy(x_old, x, vsize);
			iterations++;

			// perturbate x vector
			for (i = 0; i < dim; i++)
			{
				stepval = p[i] * step[i] * (2 * frand() - 1.0);
				if (wraparound[i])
				{
					if (x[i] + stepval > x_max[i])
						x[i] = x[i] + stepval - x_max[i] + x_min[i];
					else if (x[i] + stepval < x_min[i])
						x[i] = x[i] + stepval + x_max[i] - x_min[i];
					else
						x[i] = x[i] + stepval;
				}
				else
				{
					if ((x[i] + stepval > x_max[i]) || (x[i] + stepval < x_min[i]))
						x[i] = x[i] - stepval;
					else
						x[i] = x[i] + stepval;
				}
			}

			e_old = e_new;
			e_new = e(x);
			e_avg += e_new;
			e_num++;


			// Metropolis criterion

			if (e_new - e_old <= 0)
			{
				// down-hill movement
				if (e_best > e_new)
				{
					e_best = e_new;
					memcpy(x_best, x, vsize);


				}
				if (e_min > e_new) e_min = e_new;
				for (i = 0; i < dim; i++)
					moves_accepted[i] += p[i];
				e_num_accepted += subdim;
				if (monitor != NULL) (*monitor)(x, e_new);
			}
			else
			{
				//up-hill movement
				if (exp(-(e_new - e_old) / T) > frand())
				{
					// Accepted
					e_num_accepted += subdim;
					e_num_uphill++;
					uphill_distance += e_new - e_old;
					for (i = 0; i < dim; i++)
						moves_accepted[i] += p[i];
					if (monitor != NULL) (*monitor)(x, e_new);
				}
				else
				{	// Rejected
					memcpy(x, x_old, vsize);
					e_new = e_old;
				}

			}

			// check for deadend situation
			if (e_best_old - e_best < acc_best * e_best_old)
			{
				no_stagnant_best++;
				if (no_stagnant_best > N2 / 4)
					equilibrium_reached = 1;
			}
			else
				no_stagnant_best = 0;
			e_best_old = e_best;
			//equilibrium condition
			sum1 = 0;
			for (i = 0; i < dim; i++)
				sum1 += p_accum[i];
			if ((e_num_accepted >= N1 * dim) || (sum1 >= N2 * dim))
				equilibrium_reached = 1;



		} // stabilizing loop


		if (e_num_accepted == 0) no_downhill_loops++;

		// adjust temperature and temperature decrease rate
		e_avg /= e_num;
		a = mymax(mymin(e_min / e_avg, a_max), a_min);
		T *= a;

		//check for a sub accuracy small step
		for (i = 0; i < dim; i++)
			if (step[i] < 10 * acc_rel * step_init[i] + 100 * acc_abs)
				step_too_small = 1;

		// adjust the current step
		for (i = 0; i < dim; i++)
			if (moves_accepted[i] / (float)p_accum[i] > rate_max)
			{
				if (step[i] * step_mult <= step_init[i])
					step[i] *= step_mult;
			}
			else if (moves_accepted[i] / (float)p_accum[i] < rate_min)
			{
				if (step[i] >= step_init[i] / 20.0)
					step[i] *= step_div;
			}

	} // cooling loop	 

	if (monitor != NULL) (*monitor)(x, e_best);

	// clean-up
	free(moves_accepted);
	free(x);
	free(x_old);
	free(step_init);
	free(step);
	free(p);
	free(p_accum);
	if (no_x_init) free(x_init);

	return e_best;
}


namespace geo
{
    RigidTransform ConvertToRigidTransform(const ESARigidTransform& x)
    {
        const glm::vec3 t(x[0], x[1], x[2]);
        const glm::vec3 r(x[3], x[4], x[5]); // radians

        glm::mat4 Rx = glm::rotate(glm::mat4(1.0f), r.x, glm::vec3(1, 0, 0));
        glm::mat4 Ry = glm::rotate(glm::mat4(1.0f), r.y, glm::vec3(0, 1, 0));
        glm::mat4 Rz = glm::rotate(glm::mat4(1.0f), r.z, glm::vec3(0, 0, 1));

        glm::mat4 R = Ry * Rx * Rz;

        return { glm::mat3(R), t };
    }

	ESAResult RunESA(const ESAParameters& params, std::function<float(float*)> costfn)
    {
		ESAResult result{};

		TimePoint start = Clock::now();

		// 1. Build ESA Variables

		geo::Random rng(params.seed);

		float x_init[6] = { 
			params.initialTransform[0], 
			params.initialTransform[1], 
			params.initialTransform[2], 
			params.initialTransform[3], 
			params.initialTransform[4],
			params.initialTransform[5]};

		float x_min[6];
		float x_max[6];
		float step_fraction[6];
		long  wraparound[6];

		// --- translation ---
		x_min[0] = params.searchSpace.translationMin.x; x_max[0] = params.searchSpace.translationMax.x;
		x_min[1] = params.searchSpace.translationMin.y; x_max[1] = params.searchSpace.translationMax.y;
		x_min[2] = params.searchSpace.translationMin.z; x_max[2] = params.searchSpace.translationMax.z;
	   
		// --- rotation ---
		x_min[3] = params.searchSpace.rotationMin.x; x_max[3] = params.searchSpace.rotationMax.x;
		x_min[4] = params.searchSpace.rotationMin.y; x_max[4] = params.searchSpace.rotationMax.y;
		x_min[5] = params.searchSpace.rotationMin.z; x_max[5] = params.searchSpace.rotationMax.z;

		// --- step size ---
		step_fraction[0] = params.searchSpace.translationStep; 
		step_fraction[1] = params.searchSpace.translationStep; 
		step_fraction[2] = params.searchSpace.translationStep;

		step_fraction[3] = params.searchSpace.rotationStep;
		step_fraction[4] = params.searchSpace.rotationStep;
		step_fraction[5] = params.searchSpace.rotationStep;

		// --- wrap ---
		wraparound[0] = 0; wraparound[1] = 0; wraparound[2] = 0;
		wraparound[3] = 1; wraparound[4] = 1; wraparound[5] = 0;

		// 2. Run ESA → get best x (6D vector)

		ESARigidTransform x_best{};
		float cost = EnhancedSimulatedAnnealingPlus(
			6,                    // the dimensionality of the search space. 
			params.subDim,        // the dimensionality of the partitioning subspace (subdim <= dim).
			x_init,               // x_init, random initial starting point.
			x_best.data(),        // the returned variable vector.
			x_min,                // the minimum x values vector.
			x_max,                // the maximum x values vector. 
			step_fraction,        // a vector defining the maximum ( normalized ) jump for each variable.
			wraparound,           // a flag vector, ( 1=wrap, 0=truncate )
			// pointer the cost function to be minimized.
			costfn,              
			// pointer a monitoring function.
			NULL,
			params.maxIterations, // maximum itaerations.
			rng
		);

		// 3. Return Result
		result.transform = ConvertToRigidTransform(x_best);
		result.rmse = glm::sqrt(cost);
		result.totalTime = TimeDifferenceMs(Clock::now(), start);

        return result;
    }

	ESAResult MultiConfigESA(const std::vector<ESAParameters>& configs, std::function<float(float*)> costfn)
	{
		TimePoint start = Clock::now();
		std::vector<ESAResult> results(configs.size(), {});

		#pragma omp parallel for schedule(dynamic)
		for (int i = 0; i < (int)configs.size(); i++)
		{
			results[i] = RunESA(configs[i], costfn);
		}

		auto best = std::min_element(results.begin(), results.end(),
			[](const ESAResult& a, const ESAResult& b) { return a.rmse < b.rmse; });

		ESAResult winner = *best;
		winner.totalTime = TimeDifferenceMs(Clock::now(), start);

		return winner;
	}

	ESAResult MultiStartESA(const ESAParameters& params, std::function<float(float*)> costfn, u32 numRuns)
	{
		std::vector<ESAParameters> configs(numRuns, {});
		for (u32 i = 0; i < numRuns; i++)
		{
			configs[i] = params;
			configs[i].seed = params.seed ^ (i * 2654435761u);
		}
		return MultiConfigESA(configs, costfn);
	}
}

