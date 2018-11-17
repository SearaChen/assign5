/*
    This file is part of TinyRender, an educative rendering system.

    Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
    Derek Nowrouzezahrai, McGill University.
*/

#pragma once

#include "core/core.h"

TR_NAMESPACE_BEGIN

/**
 * Perfectly diffuse, Lambertian reflectance model
 */
struct DiffuseBSDF : BSDF {
    std::unique_ptr<Texture < v3f>> albedo;

    DiffuseBSDF(const WorldData& scene, const Config& config, const size_t& matID) : BSDF(scene, config, matID) {
        const tinyobj::material_t& mat = scene.materials[matID];

        if (mat.diffuse_texname.empty())
            albedo = std::unique_ptr<Texture<v3f>>(new ConstantTexture3f(glm::make_vec3(mat.diffuse)));
        else
            albedo = std::unique_ptr<Texture<v3f>>(new BitmapTexture3f(config, mat.diffuse_texname));

        components.push_back(EDiffuseReflection);

        combinedType = 0;
        for (size_t i = 0; i < components.size(); ++i)
            combinedType |= components[i];
    }

    v3f eval(const SurfaceInteraction& i) const override {
        v3f val(0.f);

        if (Frame :: cosTheta(i.wi) >=0 && Frame:: cosTheta(i.wi) <=1 && Frame::cosTheta(i.wo) >=0 && Frame:: cosTheta(i.wo) <=1) {
            val = albedo->eval(worldData, i) * max(0.f, Frame::cosTheta(i.wi)) * INV_PI;
        }
        return val;
    }

    float pdf(const SurfaceInteraction& i) const override {
        float pdf = max(i.wi.z/ M_PI , 0.f); // times the cos factor
        return pdf;
    }

    v3f sample(SurfaceInteraction& i, const v2f& sample, float* pdf) const override {

        // never used

        v3f v(0.0);
        i.wi = Warp::squareToCosineHemisphere(sample);

        // calculate brdf
        v3f brdf = this->eval(i);
        *pdf = this->pdf(i);   // TODO: change pdf

        if (*pdf == 0)
        {
            return v3f(0.f);
        }
        return brdf/(*pdf);
    }

    std::string toString() const override { return "Diffuse"; }
};

TR_NAMESPACE_END