/*
    This file is part of TinyRender, an educative rendering system.

    Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
    Derek Nowrouzezahrai, McGill University.
*/

#pragma once
#include <random>

TR_NAMESPACE_BEGIN

/**
 * Reflection occlusion integrator
 */
	struct ROIntegrator : Integrator {

		float m_exponent;

		explicit ROIntegrator(const Scene& scene) : Integrator(scene) {
			m_exponent = scene.config.integratorSettings.ro.exponent;
		}

		inline v3f reflect(const v3f& d) const {
			return v3f(-d.x, -d.y, d.z);
		}

		v3f render(const Ray& ray, Sampler& sampler) const override {
			v3f Li(0.f);

			SurfaceInteraction info;

			if (scene.bvh -> intersect(ray,info))
			{
				int sampleTimes = 16;
				v3f x = info.p;
				double totalSum = 0;

				for (int i = 0; i < sampleTimes; i++) {

					p2f p(sampler.next2D());;
					v3f wi = Warp::squareToPhongLobe(p,m_exponent);
					v3f wr = reflect(info.wo);
					v3f wrw = glm::normalize(info.frameNs.toWorld(wr));

					Frame rFrame = Frame(wrw);
					v3f wiw = normalize(rFrame.toWorld(wi)); // default

					// check visibility
					SurfaceInteraction shadowInfo;
					Ray shadowRay = Ray(x , wiw); // epsilon
					int isVisible = 1; // is visible
					if (scene.bvh->intersect(shadowRay, shadowInfo)) {
						isVisible = 0;
					}

					//reflective phong BRDF
					float dotProduct = wi.z;
					if (dotProduct < 0)
					{
						dotProduct = 0;
					}
					float cosExponentResult = pow(dotProduct,m_exponent);

					float phongBRDF = (m_exponent+2)*INV_TWOPI * cosExponentResult;

					float cosFactor = glm::dot(info.frameNs.n,wiw);
					float sampleP = Warp ::squareToPhongLobePdf(wi,m_exponent);
					float sampleResult = phongBRDF*isVisible * cosFactor / (sampleP*16);
					totalSum += sampleResult;
				}
				Li = v3f(totalSum);
				return Li;
			}
		}
	};

TR_NAMESPACE_END