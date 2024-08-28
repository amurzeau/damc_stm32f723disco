#include "BiquadFilter.h"

#include <math.h>
#include <string.h>

void BiquadFilter::init(const float a_coefs[], const float b_coefs[]) {
	update(a_coefs, b_coefs);
}

void BiquadFilter::update(const float a_coefs[], const float b_coefs[]) {
	this->coefs[0] = b_coefs[0] / a_coefs[0];
	this->coefs[1] = b_coefs[1] / a_coefs[0];
	this->coefs[2] = b_coefs[2] / a_coefs[0];
	this->coefs[3] = -a_coefs[1] / a_coefs[0];
	this->coefs[4] = -a_coefs[2] / a_coefs[0];

	arm_biquad_cascade_df2T_init_f32(&S, 1, coefs, state);
}

void BiquadFilter::processFilter(float* __restrict samples, size_t count) {
	// Previous custom biquad algorithm cpu usage: 4.48%
	// With df1 CMSIS algorithm: 2.78%
	// With df2T CMSIS algorithm: 2.52%
	arm_biquad_cascade_df2T_f32(&S, samples, samples, count);
}

float BiquadFilter::put(float input) {
	const float* const b = &coefs[0];
	const float* const a = &coefs[3];

	// Direct form 1 implementation, leads to better performance than Transposed direct form 2. (18% cpu in audio
	// processing vs 20% for 10 EqFilters (percentages without other processing))
	float y = b[0] * input + b[1] * input1 + b[2] * input2 + a[0] * output1 + a[1] * output2;

	input2 = input1;
	input1 = input;
	output2 = output1;
	output1 = y;

	return y;
}

void BiquadFilter::computeFilter(
    bool enabled, FilterType filterType, float f0, float fs, float gain, float Q, float a_coefs[3], float b_coefs[3]) {
	if(!enabled || Q == 0 || fs == 0) {
		b_coefs[0] = 1;
		b_coefs[1] = 0;
		b_coefs[2] = 0;
		a_coefs[0] = 1;
		a_coefs[1] = 0;
		a_coefs[2] = 0;
	} else {
		double A = pow(10.0, gain / 40.0);
		double w0 = 2.0 * M_PI * f0 / fs;
		double alpha = sin(w0) / (2.0 * Q);

		switch(filterType) {
			default:
			case FilterType::None:
				b_coefs[0] = 1;
				b_coefs[1] = 0;
				b_coefs[2] = 0;
				a_coefs[0] = 1;
				a_coefs[1] = 0;
				a_coefs[2] = 0;
				break;
			case FilterType::LowPass:
				b_coefs[0] = (1 - cos(w0)) / 2;
				b_coefs[1] = 1 - cos(w0);
				b_coefs[2] = (1 - cos(w0)) / 2;
				a_coefs[0] = 1 + alpha;
				a_coefs[1] = -2 * cos(w0);
				a_coefs[2] = 1 - alpha;
				break;
			case FilterType::HighPass:
				b_coefs[0] = (1 + cos(w0)) / 2;
				b_coefs[1] = -(1 + cos(w0));
				b_coefs[2] = (1 + cos(w0)) / 2;
				a_coefs[0] = 1 + alpha;
				a_coefs[1] = -2 * cos(w0);
				a_coefs[2] = 1 - alpha;
				break;
			case FilterType::BandPassConstantSkirt:
				b_coefs[0] = Q * alpha;
				b_coefs[1] = 0;
				b_coefs[2] = -Q * alpha;
				a_coefs[0] = 1 + alpha;
				a_coefs[1] = -2 * cos(w0);
				a_coefs[2] = 1 - alpha;
				break;
			case FilterType::BandPassConstantPeak:
				b_coefs[0] = alpha;
				b_coefs[1] = 0;
				b_coefs[2] = -alpha;
				a_coefs[0] = 1 + alpha;
				a_coefs[1] = -2 * cos(w0);
				a_coefs[2] = 1 - alpha;
				break;
			case FilterType::Notch:
				b_coefs[0] = 1;
				b_coefs[1] = -2 * cos(w0);
				b_coefs[2] = 1;
				a_coefs[0] = 1 + alpha;
				a_coefs[1] = -2 * cos(w0);
				a_coefs[2] = 1 - alpha;
				break;
			case FilterType::AllPass:
				b_coefs[0] = 1 - alpha;
				b_coefs[1] = -2 * cos(w0);
				b_coefs[2] = 1 + alpha;
				a_coefs[0] = 1 + alpha;
				a_coefs[1] = -2 * cos(w0);
				a_coefs[2] = 1 - alpha;
				break;
			case FilterType::Peak:
				b_coefs[0] = 1 + alpha * A;
				b_coefs[1] = -2 * cos(w0);
				b_coefs[2] = 1 - alpha * A;
				a_coefs[0] = 1 + alpha / A;
				a_coefs[1] = -2 * cos(w0);
				a_coefs[2] = 1 - alpha / A;
				break;
			case FilterType::LowShelf:
				b_coefs[0] = A * ((A + 1) - (A - 1) * cos(w0) + 2 * sqrt(A) * alpha);
				b_coefs[1] = 2 * A * ((A - 1) - (A + 1) * cos(w0));
				b_coefs[2] = A * ((A + 1) - (A - 1) * cos(w0) - 2 * sqrt(A) * alpha);
				a_coefs[0] = (A + 1) + (A - 1) * cos(w0) + 2 * sqrt(A) * alpha;
				a_coefs[1] = -2 * ((A - 1) + (A + 1) * cos(w0));
				a_coefs[2] = (A + 1) + (A - 1) * cos(w0) - 2 * sqrt(A) * alpha;
				break;
			case FilterType::HighShelf:
				b_coefs[0] = A * ((A + 1) + (A - 1) * cos(w0) + 2 * sqrt(A) * alpha);
				b_coefs[1] = -2 * A * ((A - 1) + (A + 1) * cos(w0));
				b_coefs[2] = A * ((A + 1) + (A - 1) * cos(w0) - 2 * sqrt(A) * alpha);
				a_coefs[0] = (A + 1) - (A - 1) * cos(w0) + 2 * sqrt(A) * alpha;
				a_coefs[1] = 2 * ((A - 1) - (A + 1) * cos(w0));
				a_coefs[2] = (A + 1) - (A - 1) * cos(w0) - 2 * sqrt(A) * alpha;
				break;
		}
	}
}

void BiquadFilter::computeFilter(bool enabled, FilterType filterType, float f0, float fs, float gain, float Q) {
	float a_coefs[3];
	float b_coefs[3];

	BiquadFilter::computeFilter(enabled, filterType, f0, fs, gain, Q, a_coefs, b_coefs);
	update(a_coefs, b_coefs);
}
