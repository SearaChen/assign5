/*
    This file is part of TinyRender, an educative rendering system.

    Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
    Derek Nowrouzezahrai, McGill University.
*/

#pragma once

TR_NAMESPACE_BEGIN

/**
 * Path tracer integrator
 */
struct PathTracerIntegrator : Integrator {
    explicit PathTracerIntegrator(const Scene& scene) : Integrator(scene) {
        m_isExplicit = scene.config.integratorSettings.pt.isExplicit;
        m_maxDepth = scene.config.integratorSettings.pt.maxDepth;
        m_rrDepth = scene.config.integratorSettings.pt.rrDepth;
        m_rrProb = scene.config.integratorSettings.pt.rrProb;
    }


    v3f renderImplicit(const Ray& ray, Sampler& sampler, SurfaceInteraction& hit) const {

        v3f Lr(0.f);
        if (getEmission (hit) != v3f(0.f)) // if it is a light
        {
            size_t emitterID = getEmitterIDByShapeID(hit.shapeID);
            Emitter em = getEmitterByID(emitterID);
            Lr = em.getRadiance();
            return Lr;
        }
        else
        {
            SurfaceInteraction info = hit;
            v3f totalBSDF(1.f);
            for (int t =0; t < m_maxDepth ; t++)
            {
                float pdf= 0;
                v3f bsdf = getBSDF(info)->sample(info,sampler.next2D(),&pdf);

                SurfaceInteraction shadowInfo;
                Ray shadowRay = Ray(info.p + Epsilon, info.frameNs.toWorld(info.wi), Epsilon); // epsilon wi represent the sampled light direction
                v3f underEmission (0.f);   // if no hit, return zero
                if (scene.bvh->intersect(shadowRay, shadowInfo)) {      // hit something
                    underEmission = getEmission(shadowInfo);

                    totalBSDF = totalBSDF * bsdf;
                    if (underEmission == v3f(0.f))  // hitting surface
                    {
                        info = shadowInfo; // update for nextround
                        continue;
                    }
                    else if (dot(shadowInfo.frameNs.n, -info.frameNs.toWorld(info.wi) ) > 0)                            // hitting light
                    {
                        // if it is hitting light
                        Lr = totalBSDF * underEmission;
                        return Lr;
                    }
                }
                else
                {
                    return v3f(0.f);
                }
            }

        }
        return Lr;
    }

//    v3f renderExplicit(const Ray& ray, Sampler& sampler, SurfaceInteraction& hit) const {
//        v3f Li(0.f);
//
//        SurfaceInteraction sInfo;
//
//        if (length(getEmission(hit))!=0)
//        {
//            return getEmission(hit);
//        }
//
//        int n_recursions = 0;
//        while (n_recursions < m_maxDepth || m_maxDepth == -1) {
//
//            n_recursions ++;
//
//            if (n_recursions >= m_rrDepth) {
//                if (sampler.next() > m_rrProb) {
//                    return v3f(0.f);
//                }
//            }
//
//            float emPdf;
//            const Emitter &emitter = getEmitterByID(selectEmitter(sampler.next(), emPdf));
//
//            float pdf;
//            v3f emPos, emNormal;
//            p2f sigmas = sampler.next2D();
//            sampleEmitterPosition(sampler, emitter, emNormal, emPos, pdf);
//            v3f wiW = normalize(emPos - hit.p);
//
//            hit.wi = normalize(hit.frameNs.toLocal(wiW));
//
//            Ray sRay = Ray(hit.p, wiW, Epsilon);
//            if (scene.bvh->intersect(sRay, sInfo)) {
//                v3f emission = getEmission(sInfo);
//                if (length(emission) != 0) {
//                    v3f BRDF = getBSDF(hit)->eval(hit);
//
//                    float cosTheta0 = glm::dot(-wiW, emNormal);
//                    cosTheta0 < 0 ? cosTheta0 = 0 : cosTheta0 < cosTheta0;
//                    float distance2 = glm::length2(emPos - hit.p);
//                    float jacob = cosTheta0 / distance2;
//                    Li += emission * BRDF * jacob / (pdf * emPdf);
//                } else {
//                    Li += v3f(0.f);
//                }
//            }
//
//            // INDIRECT ILLUMINATION
//            pdf = 0;
//            v3f BRDF = getBSDF(hit) -> sample(hit,sampler.next2D(),&pdf);
//            wiW = glm::normalize(hit.frameNs.toWorld(hit.wi));
//            Ray nextRay = Ray(hit.p, wiW, Epsilon);
//            SurfaceInteraction nextHit;
//
//            if(scene.bvh ->intersect(nextRay,nextHit))
//            {
//                v3f nextLr = getEmission(nextHit);
//                if(glm::length(nextLr)!=0)
//                {
//                    return Li;
//                } else
//                {
//                    continue;
//                }
//            }
//        }
//
//        return Li;
//    }


    v3f renderExplicit(const Ray& ray, Sampler& sampler, SurfaceInteraction& hit) const {
        v3f Li(0.f);

        SurfaceInteraction sInfo;

        if (length(getEmission(hit))!=0)
        {
            return getEmission(hit);
        }

        int n_recursions = 0;
        v3f totalBRDF(1.f);
        while (n_recursions < m_maxDepth || m_maxDepth == -1) {

            n_recursions ++;

            if (n_recursions >= m_rrDepth && m_maxDepth == -1) { // if it is russian roulette
                if (sampler.next() > m_rrProb) {
                    return Li; // was zero
                }
            }

            float emPdf;
            const Emitter &emitter = getEmitterByID(selectEmitter(sampler.next(), emPdf));

            float pdf;
            v3f emPos, emNormal;
            p2f sigmas = sampler.next2D();
            sampleEmitterPosition(sampler, emitter, emNormal, emPos, pdf);
            v3f wiW = normalize(emPos - hit.p);

            hit.wi = normalize(hit.frameNs.toLocal(wiW));

            Ray sRay = Ray(hit.p, wiW, Epsilon);
            if (scene.bvh->intersect(sRay, sInfo)) {
                v3f emission = getEmission(sInfo);
                if (length(emission) != 0) {
                    v3f BRDF = getBSDF(hit)->eval(hit);

                    float cosTheta0 = glm::dot(-wiW, emNormal);
                    cosTheta0 < 0 ? cosTheta0 = 0 : cosTheta0 < cosTheta0;
                    float distance2 = glm::length2(emPos - hit.p);
                    float jacob = cosTheta0 / distance2;
                    Li += emission * BRDF* totalBRDF * jacob / (pdf * emPdf);
                } else {
                    Li += v3f(0.f);
                }
            }

            // INDIRECT ILLUMINATION
            pdf = 0;
            v3f BRDF = getBSDF(hit) -> sample(hit,sampler.next2D(),&pdf);
            wiW = glm::normalize(hit.frameNs.toWorld(hit.wi));
            Ray nextRay = Ray(hit.p, wiW, Epsilon);
            SurfaceInteraction nextHit;

            if(scene.bvh ->intersect(nextRay,nextHit))
            {
                v3f nextLr = getEmission(nextHit);
                if(glm::length(nextLr)!=0)
                {
                    return Li;
                } else
                {
                    totalBRDF = totalBRDF*BRDF;
                    hit=nextHit;
                    continue;
                }
            }
        }

        return Li;
    }


    v3f render(const Ray& ray, Sampler& sampler) const override {
        Ray r = ray;
        SurfaceInteraction hit;

        if (scene.bvh->intersect(r, hit)) {
            if (m_isExplicit)
                return this->renderExplicit(ray, sampler, hit);
            else
                return this->renderImplicit(ray, sampler, hit);
        }
        return v3f(0.0);
    }

    int m_maxDepth;     // Maximum number of bounces
    int m_rrDepth;      // When to start Russian roulette
    float m_rrProb;     // Russian roulette probability
    bool m_isExplicit;  // Implicit or explicit
};

TR_NAMESPACE_END
