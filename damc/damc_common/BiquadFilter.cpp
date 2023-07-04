#include "BiquadFilter.h"

#include <math.h>
#include <string.h>

void BiquadFilter::init(const float a_coefs[], const float b_coefs[]) {
	update(a_coefs, b_coefs);
}

void BiquadFilter::update(const float a_coefs[], const float b_coefs[]) {
	this->b_coefs[0] = b_coefs[0] / a_coefs[0];
	this->b_coefs[1] = b_coefs[1] / a_coefs[0];
	this->b_coefs[2] = b_coefs[2] / a_coefs[0];
	this->a_coefs[0] = a_coefs[1] / a_coefs[0];
	this->a_coefs[1] = a_coefs[2] / a_coefs[0];
}

float BiquadFilter::put(float input) {
	const float* const a = a_coefs;
	const float* const b = b_coefs;

	float y = b[0] * input + s1 - 0.5;
	s1 = s2 + b[1] * input - a[0] * y;
	s2 = b[2] * input - a[1] * y + 0.5;

	return y;
}

std::complex<float> BiquadFilter::getResponse(float f0, float fs) {
	const float* const a = a_coefs;
	const float* const b = b_coefs;

	float w = 2 * M_PI * f0 / fs;
	std::complex<float> z_1 = std::exp(std::complex<float>(0, -1 * w));
	std::complex<float> z_2 = std::exp(std::complex<float>(0, -2 * w));

	return (b[0] + b[1] * z_1 + b[2] * z_2) / (std::complex<float>(1) + a[0] * z_1 + a[1] * z_2);
}

void BiquadFilter::computeFilter(bool enabled,
                                 FilterType filterType,
                                 float f0,
                                 float fs,
                                 float gain,
                                 float Q,
                                 float a_coefs[3],
                                 float b_coefs[3]) {
	if(!enabled || Q == 0 || fs == 0) {
		b_coefs[0] = 1;
		b_coefs[1] = 0;
		b_coefs[2] = 0;
		a_coefs[0] = 1;
		a_coefs[1] = 0;
		a_coefs[2] = 0;
	} else {
		float A = pow(10, gain / 40);
		float w0 = 2 * M_PI * f0 / fs;
		float alpha = sin(w0) / (2 * Q);

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
