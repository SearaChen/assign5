/*
    This file is part of TinyRender, an educative rendering system.

    Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
    Derek Nowrouzezahrai, McGill University.
*/

#pragma once

#include "core/core.h"

TR_NAMESPACE_BEGIN

/**
 * Modified Phong reflectance model
 */
struct PhongBSDF : BSDF {

    std::unique_ptr<Texture < v3f>> specularReflectance;
    std::unique_ptr<Texture < v3f>> diffuseReflectance;
    std::unique_ptr<Texture < float>> exponent;
    float specularSamplingWeight;
    float scale;

    PhongBSDF(const WorldData& scene, const Config& config, const size_t& matID) : BSDF(scene, config, matID) {
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

        v3f wr = glm::normalize(PhongBSDF::reflect(i.wo));

        float dotProduct = glm::dot(wr, glm::normalize(i.wi));
        dotProduct < 0 ? dotProduct = 0 : dotProduct = dotProduct;
        float expo =exponent->eval(worldData,i);

        v3f specularVal = specularReflectance->eval(worldData,i) * (expo + 2 ) * INV_TWOPI * pow(dotProduct,expo);

        val = specularVal;

        // multiply by cosFactor
        float cosFactor = Frame:: cosTheta(i.wi);
        val = val*cosFactor*scale; //TODO : should the scale be here??

        return val;
    }

//    float pdf(const SurfaceInteraction& i) const override {
//
//        //v is just i.wi
//        float expo = exponent->eval(worldData, i);
//        int phongExponent = exponent->eval(worldData, i);
//        v3f wr = reflect(i.wo);
//        float cosAlpha = glm::dot(normalize(i.wi), normalize(wr));
//        float pdf = (phongExponent + 2) * INV_TWOPI * pow(cosAlpha, phongExponent);
//
//
//        return pdf;
//    }

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

    v3f sample(SurfaceInteraction& i, const v2f& sample, float* pdf) const override {
        v3f val(0.f);

        float expo = exponent->eval(worldData,i);

        double xi1 = sample.x;
        double xi2 = sample.y;
        float theta = acos(pow((1-xi1),1.0f/(expo+2)));

        double phi = 2*M_PI *xi2;

        // storing the r
        v3f wr = glm::normalize(reflect(i.wo));
        v3f wrWorld = i.frameNs.toWorld(wr); // this is what was missing
        Frame* wrFrame = new Frame(wrWorld);

        v3f sampleD = Warp::squareToPhongLobe(sample,expo);
        v3f sampleDw = wrFrame->toWorld(sampleD);
        i.wi = i.frameNs.toLocal(sampleDw); // assigned the sample direction

        float pdfi = Warp::squareToPhongLobePdf(sampleD,expo); // obtain the pdf
        *pdf = pdfi;

        // calculate brdf
        v3f brdf = this->eval(i);
        if (pdfi == 0)
        {
            return v3f(0.f);
        }

        return brdf/pdfi;
    }

    std::string toString() const override { return "Phong"; }
};

TR_NAMESPACE_END