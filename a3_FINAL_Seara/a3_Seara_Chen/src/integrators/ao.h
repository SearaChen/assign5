/*
    This file is part of TinyRender, an educative rendering system.

    Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
    Derek Nowrouzezahrai, McGill University.
*/

#pragma once

//#include <core/math.h>

TR_NAMESPACE_BEGIN
//#include "core/math.h"

/**
 * Ambient occlusion integrator
 */
	struct AOIntegrator : Integrator {
		explicit AOIntegrator(const Scene& scene) : Integrator(scene) { }

		v3f render(const Ray& ray, Sampler& sampler) const override {
			v3f Li(0.f);

			SurfaceInteraction info;

			if (scene.bvh -> intersect(ray,info))
			{
				int sampleTimes = 16;
				v3f x = info.p;

				double totalSum = 0;

				for (int i = 0; i < sampleTimes; i++) {
					p2f samplePoints(sampler.next(), sampler.next());
					v3f sampledDirection = Warp::squareToUniformSphere(samplePoints);
					float sampleP = Warp::squareToUniformSpherePdf();

					v3f sampledLightDirection = info.frameNs.toWorld(sampledDirection); // default

					// check visibility
					SurfaceInteraction shadowInfo;
					Ray shadowRay = Ray(x, sampledLightDirection); // epsilon
					shadowRay.max_t = glm::length(scene.aabb.getBSphere().radius/2);
					int isVisible = 1; // is visible
					if (scene.bvh->intersect(shadowRay, shadowInfo)) {
						isVisible = 0;
					}

					// cos factor
					float cosFactor = Frame::cosTheta(sampledDirection);
					if (cosFactor < 0)   // clip to 0
					{
						cosFactor = 0;
					}

					float sampleResult = isVisible * cosFactor / sampleP;
					totalSum += sampleResult;
				}
				Li = v3f(totalSum / sampleTimes / M_PI);
			}
			return Li;
		}
	};

TR_NAMESPACE_END