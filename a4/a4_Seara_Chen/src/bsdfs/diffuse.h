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

    v3f eval(const SurfaceInteraction& i) const override { // this is right I THINK
        v3f val(0.f);
        val = albedo->eval(worldData,i)* max(0.f,Frame :: cosTheta(i.wi)) * INV_PI;
        return val;
    }

    float pdf(const SurfaceInteraction& i) const override {

        float pdf = max(i.wi.z/ M_PI , 0.f); // times the cos factor
        return pdf;
    }


    v3f sample(SurfaceInteraction& i, const v2f& sample, float* pdf) const override {
        // i and pdf not used
        v3f v(0.0);
        float theta = acos(sqrt(sample.x));
        float phi = 2*M_PI *sample.y;

        v.x = sin(theta)*cos(phi);
        v.y = sin(theta)*sin(phi);
        v.z = cos(theta);

        i.wi = v;

        float ipdf = this->pdf(i);
        v3f brdf = this->eval(i);

        //check and return result
        if (ipdf == 0 )
        {
            return v3f(0.f);
        }
        return brdf / ipdf;
    }

    std::string toString() const override { return "Diffuse"; }
};

TR_NAMESPACE_END