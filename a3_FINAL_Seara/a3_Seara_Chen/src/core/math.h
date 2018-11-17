/*
    This file is part of TinyRender, an educative rendering system.

    Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
    Derek Nowrouzezahrai, McGill University.
*/

#pragma once

TR_NAMESPACE_BEGIN

/**
 * Computes barycentric coordinates.
 */
    template<class T>
    inline T barycentric(const T& a, const T& b, const T& c, const float u, const float v) {
        return a * (1 - u - v) + b * u + c * v;
    }

/**
 * Restricts a value to a given interval.
 */
    template<class T>
    inline T clamp(T v, T min, T max) {
        return std::min(std::max(v, min), max);
    }

/**
 * Checks if vector is zero.
 */
    inline bool isZero(const v3f v) {
        return glm::dot(v, v) < Epsilon;
    }

/**
 * Generates coordinate system.
 */
    inline void coordinateSystem(const v3f& a, v3f& b, v3f& c) {
        if (std::abs(a.x) > std::abs(a.y)) {
            float invLen = 1.f / std::sqrt(a.x * a.x + a.z * a.z);
            c = v3f(a.z * invLen, 0.f, -a.x * invLen);
        } else {
            float invLen = 1.f / std::sqrt(a.y * a.y + a.z * a.z);
            c = v3f(0.f, a.z * invLen, -a.y * invLen);
        }
        b = glm::cross(c, a);
    }

/**
 * Converts RGB value to luminance.
 */
    inline float getLuminance(const v3f& rgb) {
        return glm::dot(rgb, v3f(0.212671f, 0.715160f, 0.072169f));
    }

/**
 * Pseudo-random sampler (Mersenne Twister 19937) structure.
 */
    struct Sampler {
        std::mt19937 g;
        std::uniform_real_distribution<float> d;
        explicit Sampler(int seed) {
            g = std::mt19937(seed);
            d = std::uniform_real_distribution<float>(0.f, 1.f);
        }
        float next() { return d(g); }
        p2f next2D() { return {d(g), d(g)}; }
        void setSeed(int seed) {
            g.seed(seed);
            d.reset();
        }
    };

/**
 * 1D discrete distribution.
 */
    struct Distribution1D {
        std::vector<float> cdf{0};
        bool isNormalized = false;

        inline void add(float pdfVal) {
            cdf.push_back(cdf.back() + pdfVal);
        }

        size_t size() {
            return cdf.size() - 1;
        }

        float normalize() {
            float sum = cdf.back();
            for (float& v : cdf) {
                v /= sum;
            }
            isNormalized = true;
            return sum;
        }

        inline float pdf(size_t i) const {
            assert(isNormalized);
            return cdf[i + 1] - cdf[i];
        }

        int sample(float sample) const {
            assert(isNormalized);
            const auto it = std::upper_bound(cdf.begin(), cdf.end(), sample);
            return clamp(int(distance(cdf.begin(), it)) - 1, 0, int(cdf.size()) - 2);
        }
    };


/**
 * Warping functions.
 */

# include "platform.h"

    namespace Warp {

        inline v3f squareToUniformSphere(const p2f& sample) {
            v3f v(0.f);
            double phi = 2 * M_PI * sample.x;
            double theta = acos(1 - 2 * sample.y);
            v.x = sin(theta)*cos(phi);
            v.y = sin(theta)*sin(phi);
            v.z = cos(theta);

            return normalize(v);
        }

        inline float squareToUniformSpherePdf() {

            float pdf = 1/(4*M_PI);
            return pdf;
        }

// first two are constant
        inline v3f squareToUniformHemisphere(const p2f& sample) {
            v3f v(0.f);

            double phi = 2 * M_PI * sample.x;
            double theta = acos(1 - sample.y);
            v.x = sin(theta)*cos(phi);
            v.y = sin(theta)*sin(phi);
            v.z = cos(theta);
            return normalize(v);
        }

        inline float squareToUniformHemispherePdf(const v3f& v) {

            float pdf = 1/(2*M_PI);
            return pdf;
        }



        inline v3f squareToCosineHemisphere(const p2f& sample) {
            v3f v(0.f);

            float theta = acos(sqrt(sample.x));
            float phi = 2*M_PI *sample.y;

            v.x = sin(theta)*cos(phi);
            v.y = sin(theta)*sin(phi);
            v.z = cos(theta);

            return normalize(v);
        }

        inline float squareToCosineHemispherePdf(const v3f& v) {
            float pdf = 0.f;
            float theta = acos(v.z);
            pdf = cos(theta) / M_PI;
            return pdf;
        }

        inline v3f squareToPhongLobe(const p2f& sample, double expo) {
            v3f v(0.f);

            double xi1 = sample.x;
            double xi2 = sample.y;
            float theta = acos(pow((1-xi1),1.0f/(expo+2)));

            double phi = 2*M_PI *xi2;

            v = v3f(sin(theta)*cos(phi),sin(theta)*sin(phi),cos(theta));

            return v;
        }

        inline float squareToPhongLobePdf(const v3f& v, double expo) {

            return ((expo+2)*INV_TWOPI*pow(v.z,expo));
        }

    }

TR_NAMESPACE_END