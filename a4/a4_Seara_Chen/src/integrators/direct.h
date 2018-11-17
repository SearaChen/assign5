/*
    This file is part of TinyRender, an educative rendering system.

    Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
    Derek Nowrouzezahrai, McGill University.
*/

#pragma once

#include "core/math.h"
TR_NAMESPACE_BEGIN

/**
 * Direct illumination integrator with MIS
 */
struct DirectIntegrator : Integrator {
    explicit DirectIntegrator(const Scene& scene) : Integrator(scene) {
        m_emitterSamples = scene.config.integratorSettings.di.emitterSamples;
        m_bsdfSamples = scene.config.integratorSettings.di.bsdfSamples;
        m_samplingStrategy = scene.config.integratorSettings.di.samplingStrategy;
    }

    static inline float balanceHeuristic(float nf, float fPdf, float ng, float gPdf) {
        float f = nf * fPdf, g = ng * gPdf;
        return f / (f + g);
    }

    void sampleSphereByCosineHemisphere(const p2f& sample,
                                        const v3f& n,
                                        const p3f& pShading,
                                        const v3f& emitterCenter,
                                        float emitterRadius,
                                        v3f& wiW,
                                        float& pdf) const {
        // TODO: Implement this
    }

    void sampleSphereByArea(const p2f& sample,
                            const p3f& pShading,  // point of shading
                            const v3f& emitterCenter,
                            float emitterRadius,
                            v3f& pos,
                            v3f& ne,
                            v3f& wiW,
                            float& pdf) const {

        v3f sampledUnitDir = Warp::squareToUniformSphere(sample);
        pos = sampledUnitDir * emitterRadius + emitterCenter;
        wiW = normalize(pos - pShading);
        ne =  normalize(sampledUnitDir);
        pdf = Warp::squareToUniformSpherePdf()/ pow(emitterRadius,2);
    }

    void sampleSphereBySolidAngle(const p2f& sample,
                                  const p3f& pShading,
                                  const v3f& emitterCenter,
                                  float emitterRadius,
                                  v3f& wi,
                                  float& pdf) const {
        float xi1 = sample.x;
        float xi2 = sample.y;

        // compute coord system for sphere sampling
        v3f spec = glm::normalize(emitterCenter - pShading); // shadingPoint-emitterCenter vector in world coords
        Frame specFrame = Frame(spec);

        // sample sphere uniformly inside subtended cone
        // compute theta and phi values for sample in cone
        float specDistance = glm::distance(emitterCenter, pShading); // distance between shadingPoint and emitterCenter
        float sinThetaMaxSquared = pow(emitterRadius / specDistance, 2);
        float cosThetaMax = std::sqrt(std::max(0.f, 1 - sinThetaMaxSquared));
        float cosTheta = (1 - xi1) + xi1 * cosThetaMax;
        float sinTheta = std::sqrt(std::max(0.f, 1 - pow(cosTheta, 2)));
        float phi = xi2 * 2 * M_PI;

        // compute angle alpha from center of sphere to sampled point on surface
        float ds = specDistance * cosTheta
                   - std::sqrt(std::max(0.f, pow(emitterRadius, 2) - pow(specDistance * sinTheta, 2))); // distance between shading point and sampled point on sphere
        float cosAlpha = (pow(specDistance, 2) + pow(emitterRadius, 2) - pow(ds, 2)) / (2 * specDistance*emitterRadius);
        float sinAlpha = std::sqrt(std::max(0.f, 1 - pow(cosAlpha, 2)));

        // compute surface normal and sapmled point on sphere
        v3f sampledPointNormal = v3f(sinAlpha * std::cos(phi), sinAlpha * std::sin(phi), cosAlpha); // normals local to wcFrame
        v3f sampledPoint = emitterRadius * sampledPointNormal; // sampled points local to wcFrame
        v3f sampledPointWorld = specFrame.toWorld(sampledPoint); // sampled points in world coords

        wiW = glm::normalize(specFrame.toWorld(v3f(std::cos(phi) * sinTheta, std::sin(phi)*sinTheta, cosTheta)));

        // pdf
        pdf = 1/(2*M_PI*(1-(cosThetaMax)));
    }

    v3f renderArea(const Ray& ray, Sampler& sampler) const {
        v3f Lr(0.f);
        SurfaceInteraction info;
        if (scene.bvh -> intersect(ray,info))
        {
            if (getEmission (info) != v3f(0.f)) // if it is a light
            {
                size_t emitterID = getEmitterIDByShapeID(info.shapeID);
                Emitter em = getEmitterByID(emitterID);
                Lr = em.getRadiance();
            }
            else
            {
                int sampleSize = m_emitterSamples;
                v3f x = info.p;
                v3f totalSum (0);
                for (int i = 0; i < sampleSize; i++)
                {
                    // sample emitter
                    float emPdf;
                    size_t id = selectEmitter(sampler.next(),emPdf);
                    const Emitter& em = getEmitterByID(id);

                    // calculate emitter information
                    float pdf = 0.f;
                    v3f pos(0.f);  // output
                    v3f ne(0.f);   // output
                    v3f wiW(0.f);  // output
                    v3f emCenter = scene.getShapeCenter(em.shapeID);
                    float emRadius = scene.getShapeRadius(em.shapeID);
                    sampleSphereByArea (sampler.next2D(),x,emCenter,emRadius,pos,ne,wiW,pdf);

                    v3f wi = info.frameNs.toLocal(wiW);
                    info.wi = normalize(wi);
                    v3f bsdf = getBSDF(info)->eval(info);

                    // check visibility under the emission light
                    SurfaceInteraction shadowInfo;
                    Ray shadowRay = Ray(x, info.frameNs.toWorld(info.wi)); // epsilon wi represent the sampled light direction
                    v3f underEmission (0.f);
                    if (scene.bvh->intersect(shadowRay, shadowInfo)) {
                        underEmission = getEmission(shadowInfo);
                    }

                    // calculating Jacobian
                    double cosTheta0 = max(0.f,glm::dot(wiW,ne)); // had a negative
                    double jacob = cosTheta0 / pow(glm::length(pos-x),2);


                    v3f sampleResult = v3f(underEmission * bsdf*jacob) / (emPdf*pdf);  // do we need pi ?? divide by pdf emitter and


                    totalSum += sampleResult;
                }

                Lr = v3f(totalSum/sampleSize) ;
            }
        }
        return Lr;
    }

    v3f renderCosineHemisphere(const Ray& ray, Sampler& sampler) const {
        v3f Lr(0.f);

        SurfaceInteraction info;

        if (scene.bvh -> intersect(ray,info))
        {
            if (getEmission (info) != v3f(0.f)) // if it is a light
            {
                Lr = getEmission (info);
            }
            else
            {
                int sampleSize = m_emitterSamples ;
                v3f x = info.p;
                v3f totalSum (0);
                for (int i = 0; i < sampleSize; i++)
                {
                    v3f sampledDirection = Warp::squareToCosineHemisphere(sampler.next2D());
                    float sampleP = Warp::squareToCosineHemispherePdf(sampledDirection);
                    info.wi = sampledDirection;
                    v3f sampledLightDirection = info.frameNs.toWorld(sampledDirection); // default


                    // check visibility under the emission light
                    SurfaceInteraction shadowInfo;
                    Ray shadowRay = Ray(x, sampledLightDirection); // epsilon
                    v3f underEmission (0.f);
                    v3f rho (0.f);
                    if (scene.bvh->intersect(shadowRay, shadowInfo)) {
                        underEmission = getEmission(shadowInfo);
                        rho = getBSDF(info)->eval(info);
                    }

                    v3f sampleResult = v3f( underEmission *rho * INV_PI / sampleP);
                    totalSum += sampleResult;
                }

                Lr = v3f(totalSum/sampleSize) ;

            }
        }
        return Lr;
    }

    v3f renderBSDF(const Ray& ray, Sampler& sampler) const {
        v3f Lr(0.f);

        SurfaceInteraction info;

        if (scene.bvh -> intersect(ray,info))
        {
            // if it is a light
            if (getEmission (info) != v3f(0.f))
            {
                size_t emitterID = getEmitterIDByShapeID(info.shapeID);
                Emitter em = getEmitterByID(emitterID);
                Lr = em.getRadiance(); // first part of 2.1
            }
            else
            {
                int sampleSize = m_emitterSamples;
                v3f x = info.p;

                v3f totalSum (0);
                for (int i = 0; i < sampleSize; i++)
                {
                    v3f bsdf = getBSDF(info)->sample(info,sampler.next2D(),0); // already divided by pdf

                    // check visibility under the emission light
                    SurfaceInteraction shadowInfo;
                    Ray shadowRay = Ray(x + Epsilon, info.frameNs.toWorld(info.wi), Epsilon); // epsilon wi represent the sampled light direction
                    v3f underEmission (0.f);
                    if (scene.bvh->intersect(shadowRay, shadowInfo)) {
                        underEmission = getEmission(shadowInfo);
                    }

                    v3f sampleResult = v3f(underEmission * bsdf);  // TODO : maybe add back pi
                    totalSum += sampleResult;
                }
                Lr = v3f(totalSum/sampleSize) ;
            }
        }

        return Lr;
    }

    v3f renderSolidAngle(const Ray& ray, Sampler& sampler) const {
        v3f Lr(0.f);
        SurfaceInteraction info;
        if (scene.bvh -> intersect(ray,info)) {
            if (getEmission(info) != v3f(0.f)) // if it is a light
            {
                size_t emitterID = getEmitterIDByShapeID(info.shapeID);
                Emitter em = getEmitterByID(emitterID);
                Lr = em.getRadiance();
            } else {

                v3f x = info.p;
                v3f totalSum(0);
                for (int i = 0; i < m_emitterSamples; i++) {
                    // sample emitter
                    float emPdf;
                    size_t id = selectEmitter(sampler.next(), emPdf);
                    const Emitter &em = getEmitterByID(id);

                    // calculate emitter information
                    float pdf = 0.f;
                    v3f wi(0.f);  //  in the coordinate of the distance???
                    v3f emCenter = scene.getShapeCenter(em.shapeID);
                    float emRadius = scene.getShapeRadius(em.shapeID);
                    sampleSphereBySolidAngle(sampler.next2D(), x, emCenter, emRadius, wi, pdf);

                    // change coordinate
                    Frame disFrame = new Frame(glm::normalize(emCenter-x));
                    v3f wiW = disFrame.toWorld(wi);
                    info.wi = normalize(info.frameNs.toLocal(normalize(wiW)));

                    v3f bsdf = getBSDF(info)->eval(info);

                    // check visibility under the emission light
                    SurfaceInteraction shadowInfo;
                    Ray shadowRay = Ray(x + Epsilon, info.frameNs.toWorld(info.wi)); // epsilon wi represent the sampled light direction
                    v3f underEmission(0.f);
                    if (scene.bvh->intersect(shadowRay, shadowInfo))
                    {
                        underEmission = getEmission(shadowInfo);
                    }

                    v3f sampleResult = v3f(underEmission * bsdf) / (pdf*emPdf);  // do we need pi ?? divide by pdf emitter and
                    totalSum += sampleResult;
                }

                Lr = v3f(totalSum / m_emitterSamples);
            }

        }
        return Lr;
    }

    v3f renderMIS(const Ray& ray, Sampler& sampler) const {

        v3f Lr = v3f(0.f);
        SurfaceInteraction info;
        if (scene.bvh -> intersect(ray,info)) {
            if (getEmission(info) != v3f(0.f)) // if it is a light
            {
                size_t emitterID = getEmitterIDByShapeID(info.shapeID);
                Emitter em = getEmitterByID(emitterID);
                Lr = em.getRadiance();
            }
            else {
                // ----------------------------------  SOLID ANGLE  ---------------------------------
                v3f saTotalSum (0);
                int sampleSize = m_emitterSamples;

                for (int i = 0; i < sampleSize; i++)
                {
                    // sample emitter
                    float emPdf;
                    size_t id = selectEmitter(sampler.next(), emPdf);
                    const Emitter &em = getEmitterByID(id);
                    v3f emCenter = scene.getShapeCenter(em.shapeID);
                    float emRadius = scene.getShapeRadius(em.shapeID);

                    float pdf = 0.f;
                    v3f wi(0.f);  //  in the coordinate of the distance???
                    sampleSphereBySolidAngle(sampler.next2D(), info.p, emCenter, emRadius, wi, pdf);   // todo :: change it to word
                    Frame* disFrame = new Frame(glm::normalize(emCenter-info.p));
                    v3f wiW = disFrame->toWorld(wi);
                    info.wi = normalize(info.frameNs.toLocal(normalize(wiW)));

                    // check visibility under the emission light
                    SurfaceInteraction shadowInfo;
                    Ray shadowRay = Ray(info.p + Epsilon, info.frameNs.toWorld(info.wi), Epsilon); // epsilon wi represent the sampled light direction
                    v3f underEmission(0.f);
                    if (scene.bvh->intersect(shadowRay, shadowInfo)) {
                        underEmission = getEmission(shadowInfo);
                    }

                    v3f sampleResult = v3f(0.f);
                    if (underEmission == v3f(0.f))
                    {
                        sampleResult = v3f(0.f);
                    }
                    else
                    {
                        v3f bsdf = getBSDF(info)->eval(info);
                        float BRDFpdf = getBSDF(info)->pdf(info); // TODO: should clip ?
                        float weight = balanceHeuristic( m_emitterSamples, pdf * emPdf,m_bsdfSamples, BRDFpdf);

                        sampleResult = v3f(underEmission * bsdf *  weight) / (pdf*emPdf);  // do we need pi ?? divide by pdf emitter and
                    }
                    saTotalSum += sampleResult;
                }
                v3f LrSA(0.f);
                if (sampleSize !=0){ LrSA = v3f(saTotalSum/sampleSize); }

                // ----------------------------------  brdf ---------------------------------
                v3f brdfTotalSum(0.f);
                for (int i = 0; i < m_bsdfSamples; i++)
                {
                    v3f bsdf = getBSDF(info)->sample(info,sampler.next2D(),0); // already have everything wrapped
                    float BRDFpdf = getBSDF(info)->pdf(info); // pdf

                    SurfaceInteraction shadowInfo;
                    Ray shadowRay = Ray(info.p + Epsilon, info.frameNs.toWorld(info.wi), Epsilon); // epsilon wi represent the sampled light direction
                    v3f underEmission (0.f);
                    if (scene.bvh->intersect(shadowRay, shadowInfo)) {
                        underEmission = getEmission(shadowInfo);
                    }

                    v3f sampleResult = v3f(0.f);
                    if (underEmission == v3f(0.f))
                    {
                        sampleResult = underEmission;
                    }
                    else
                    {
                        int emID = getEmitterIDByShapeID(shadowInfo.shapeID);
                        const Emitter& em = getEmitterByID(emID);
                        v3f emCenter = scene.getShapeCenter(em.shapeID); // TODO :: verify is this is true
                        float emRadius = scene.getShapeRadius(em.shapeID);
                        float distanceToCenter = glm :: length(emCenter - info.p);
                        float sinThetaMax2 = (emRadius / distanceToCenter) * (emRadius / distanceToCenter);
                        float cosThetaMax = sqrt(max(1.0f - sinThetaMax2,0.f));
                        float SApdf = 1/(2*M_PI*(1-(cosThetaMax)));
                        float emPdf = 1.f / scene.emitters.size(); // get pdf function

                        float weight= balanceHeuristic(m_bsdfSamples, BRDFpdf, m_emitterSamples, SApdf * emPdf);

                        sampleResult = v3f(underEmission * bsdf * weight);
                    }
                    brdfTotalSum += sampleResult;
                }
                v3f LrBrdf(0.f);
                if (m_bsdfSamples != 0 ) { LrBrdf = v3f(brdfTotalSum/m_bsdfSamples) ; }


                // FINAL AGGRAGATION
                Lr = LrBrdf + LrSA;
             }
        }

        return Lr;
        /* Probable to do
         * clip
         * */
    }

    v3f render(const Ray& ray, Sampler& sampler) const override {
        if (m_samplingStrategy == "mis")
            return this->renderMIS(ray, sampler);
        else if (m_samplingStrategy == "area")
            return this->renderArea(ray, sampler);
        else if (m_samplingStrategy == "solidAngle")
            return this->renderSolidAngle(ray, sampler);
        else if (m_samplingStrategy == "cosineHemisphere")
            return this->renderCosineHemisphere(ray, sampler);
        else if (m_samplingStrategy == "bsdf")
            return this->renderBSDF(ray, sampler);
        std::cout << "Error: wrong strategy" << std::endl;
        exit(EXIT_FAILURE);
    }

    size_t m_emitterSamples;     // Number of emitter samples
    size_t m_bsdfSamples;        // Number of BSDF samples
    string m_samplingStrategy;   // Sampling strategy to use
};

TR_NAMESPACE_END