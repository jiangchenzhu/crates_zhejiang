// For base noise type
#include "Ornstein.h"

// Used to tokenize the parameters
#include <boost/tokenizer.hpp>

using namespace gazebo;

// Configure using the given SDF
Ornstein::Ornstein(double beta, double sigma) : Noise(), _beta(beta), _sigma(sigma)
{
	Reset();
}

// Destructor
Ornstein::~Ornstein()
{
	// Do nothing
}

void Ornstein::Reset()
{
	// Rest time
	tim = 0;	

	// Initialise parameters
	for (int i = 0; i < MAX_VARS; i++)
		vars[i] = math::Rand::GetDblNormal(0,_sigma);
}

void Ornstein::Sample(double t)
{
	// Discrete time
	double dt = t - tim;

	// Get parameter k, acknowledging that values outside (0,1] may cause issues
	// with exp(.) precision. So, set to 0 to avoid NaNs and Infs in state.
	double k  = (dt <= 0.0 || dt > 1.0  ? 0 : exp(-_beta*dt));

    // Sample!
    for (int i = 0; i < MAX_VARS; i++)
    	vars[i] = vars[i] * k + math::Rand::GetDblNormal(0,_sigma);

    // Time lag
    tim = t;
}
