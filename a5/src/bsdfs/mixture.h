/*
    This file is part of TinyRender, an educative rendering system.

    Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
    Derek Nowrouzezahrai, McGill University.
*/

#pragma once

#include "core/core.h"
#include <random>

TR_NAMESPACE_BEGIN

/**
 * Modified Phong reflectance model + Diffuse
 */
struct MixtureBSDF : BSDF {
    std::unique_ptr<Texture < v3f>> specularReflectance;
    std::unique_ptr<Texture < v3f>> diffuseReflectance;
    std::unique_ptr<Texture < float>> exponent;
    float specularSamplingWeight;
    float scale;

    MixtureBSDF(const WorldData& scene, const Config& config, const size_t& matID) : BSDF(scene, config, matID) {
        const tinyobj::material_t& mat = scene.materials[matID];

        if (mat.specular_texname.empty())
            specularReflectance = std::unique_ptr<Texture<v3f>>(new ConstantTexture3f(glm::make_vec3(mat.specular)));
        else
            specularReflectance = std::unique_ptr<Texture<v3f>>(new BitmapTexture3f(config, mat.specular_texname));

        if (mat.diffuse_texname.empty())
            diffuseReflectance = std::unique_ptr<Texture<v3f>>(new ConstantTexture3f(glm::make_vec3(mat.diffuse)));
        else
            diffuseReflectance = std::unique_ptr<Texture<v3f>>(new BitmapTexture3f(config, mat.diffuse_texname));

        exponent = std::unique_ptr<Texture<float>>(new ConstantTexture1f(mat.shininess));

        //get scale value to ensure energy conservation
        v3f maxValue = specularReflectance->getMax() + diffuseReflectance->getMax();
        float actualMax = max(max(maxValue.x, maxValue.y), maxValue.z);
        scale = actualMax > 1.0f ? 0.99f * (1.0f / actualMax) : 1.0f;

        float dAvg = getLuminance(diffuseReflectance->getAverage() * scale);
        float sAvg = getLuminance(specularReflectance->getAverage() * scale);
        specularSamplingWeight = sAvg / (dAvg + sAvg);

        components.push_back(EGlossyReflection);
        components.push_back(EDiffuseReflection);

        combinedType = 0;
        for (unsigned int component : components)
            combinedType |= component;
    }

    inline v3f reflect(const v3f& d) const {
        return v3f(-d.x, -d.y, d.z);
    }

    v3f eval(const SurfaceInteraction& i) const override {
        v3f val(0.f);
        if (Frame :: cosTheta(i.wi) >=0 && Frame:: cosTheta(i.wi) <=1 && Frame::cosTheta(i.wo) >=0 && Frame:: cosTheta(i.wo) <=1)
        {
            v3f wr = normalize(reflect(i.wi));  // was wo
            float expo = exponent->eval(worldData, i);
            v3f rho_s = specularReflectance->eval(worldData, i);
            v3f rho_d = diffuseReflectance->eval(worldData, i);
            float cos_alpha = dot(wr, normalize(i.wo)); // was wi
            cos_alpha < 0 ? cos_alpha = 0 : cos_alpha = cos_alpha;

            v3f specularVal = rho_s * (expo + 2) * INV_TWOPI * pow(cos_alpha, expo);
            v3f diffuseVal = rho_d * INV_PI;

            val = specularVal + diffuseVal;

            // multiply by cosFactor
            float cosFactor = Frame::cosTheta(i.wi);
            //val = val * glm::max(cosFactor,0.f);
            val = val * cosFactor * scale;

            if (val.x > 1 && val.y > 1 && val.z > 1) {
                int b = 0;
            }
        }
        return val;
    }

    float pdf(const SurfaceInteraction& i) const override {
        float pdf = 0.f;

        int expo = exponent->eval(worldData, i);

        v3f wrW = normalize(i.frameNs.toWorld(reflect(i.wo)));
        Frame wrFrame = Frame(wrW);
        v3f wiR =  wrFrame.toLocal(i.frameNs.toWorld(i.wi));
        float specularPdf = max(Warp :: squareToPhongLobePdf(wiR, expo),0.f);  // TODO: put this back to reflected frame
        //float specularPdf = max(Warp :: squareToPhongLobePdf(i.wi, expo),0.f);  // TODO: put this back to reflected frame
        float diffusePdf = max(Warp::squareToCosineHemispherePdf(i.wi),0.f);
        pdf = specularPdf * specularSamplingWeight + diffusePdf *(1-specularSamplingWeight);
        return pdf;
    }

    v3f sample(SurfaceInteraction& i, const v2f& sample, float *pdf) const override {
        v3f val(0.f);

        // retrieving the weight
        float expo =exponent->eval(worldData,i);

        //Russian Roulette sampling
        double seed = sample.x; // check up on: sample.x
        if (seed < specularSamplingWeight)
        {
            // SPECULAR
            float rescaling = (seed) / (specularSamplingWeight) ; // not the best renaming
            v2f newSample = v2f(rescaling , sample.y);

            v3f wiR = Warp::squareToPhongLobe(newSample,expo);
            v3f wr = glm::normalize(reflect(i.wo));
            v3f wrW =normalize(i.frameNs.toWorld(wr)); // this is what was missing
            Frame* wrFrame = new Frame(wrW);
            v3f wiW = normalize(wrFrame->toWorld(wiR));
            i.wi = normalize(i.frameNs.toLocal(wiW)); // assigned the sample direction

            // assign probability
            *pdf = this->pdf(i);

            // brdf
            v3f brdf = this->eval(i);

            // putting this together
            *pdf <= 0? val = v3f(0.f) : val = brdf/ *pdf;

            if (brdf.x > 1 && brdf.y >1 && brdf.z > 1)
            {
                int a = 0;
            }

        }
        else
        {
            // DIFFUSE
            float rescaling = (seed-specularSamplingWeight) / (1-specularSamplingWeight);
            v2f newSample = v2f(rescaling, sample.y);

            // direction sample
            i.wi = Warp::squareToCosineHemisphere(newSample);

            // assign probability
            *pdf = this->pdf(i);

            // brdf
            v3f brdf = this->eval(i);

            // putting it together
            *pdf <= 0? val = v3f(0.f) : val = brdf/ *pdf ;

            if (brdf.x > 1 && brdf.y >1 && brdf.z > 1)
            {
                int a = 0;
            }
        }
        return val;
    }

    std::string toString() const override { return "Mixture"; }
};

TR_NAMESPACE_END