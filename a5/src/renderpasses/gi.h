/*
    This file is part of TinyRender, an educative rendering system.

    Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
    Derek Nowrouzezahrai, McGill University.
*/

#pragma once

#include <core/core.h>
#include "core/renderpass.h"
#include "tiny_obj_loader.h"
#include "integrators/path.h"

TR_NAMESPACE_BEGIN

/**
 * Global Illumination baking renderpass.
 */
struct GIPass : RenderPass {
    GLuint shader{0};

    GLuint modelMatUniform{0};
    GLuint viewMatUniform{0};
    GLuint projectionMatUniform{0};

    int m_samplePerVertex;

    std::unique_ptr<PathTracerIntegrator> m_ptIntegrator;

    explicit GIPass(const Scene& scene) : RenderPass(scene) {
        m_ptIntegrator = std::unique_ptr<PathTracerIntegrator>(new PathTracerIntegrator(scene));
        m_ptIntegrator->m_maxDepth = scene.config.integratorSettings.gi.maxDepth;
        m_ptIntegrator->m_rrProb = scene.config.integratorSettings.gi.rrProb;
        m_ptIntegrator->m_rrDepth = scene.config.integratorSettings.gi.rrDepth;
        m_samplePerVertex = scene.config.integratorSettings.gi.samplesByVertex;
    }

    virtual void buildVBO(size_t objectIdx) override {
        //TODO: SAMPLE PER VERTEX
        GLObject& obj = objects[objectIdx];
        obj.nVerts = scene.getObjectNbVertices(objectIdx);
        obj.vertices.resize(obj.nVerts*6);

        int i = 0;
        Sampler sampler = Sampler(260670714);
        for (size_t index = 0;
             index < scene.getObjectNbVertices(objectIdx); index++) // TODO :: is this the way to retrieve index?
        {
            SurfaceInteraction info;
            v3f pos = scene.getObjectVertexPosition(objectIdx, index);
            v3f vertexN = scene.getObjectVertexNormal(objectIdx, index);


            info.p = pos + Epsilon * normalize(vertexN);
            info.primID = scene.getPrimitiveID(index); // primID
            info.matID = scene.getMaterialID(objectIdx, info.primID); //matID
            info.shapeID = objectIdx;
            info.frameNs = Frame(vertexN);
            info.frameNg = Frame(vertexN);

            obj.vertices[i + 0] = pos.x;
            obj.vertices[i + 1] = pos.y;
            obj.vertices[i + 2] = pos.z;

            Ray ray = Ray(info.p, vertexN); // any ray should do
            v3f RGB = v3f(0);

            for (int j = 0; j < m_samplePerVertex; j++)
            {
                info.wo = Warp::squareToUniformSphere(sampler.next2D());
                v3f onebounce= m_ptIntegrator->renderExplicit(ray, sampler, info);
                RGB+= onebounce;
            }
            RGB /= m_samplePerVertex;

            obj.vertices[i+3] =RGB.x;
            obj.vertices[i+4] =RGB.y;
            obj.vertices[i+5] =RGB.z;

            i+= N_ATTR_PER_VERT;
        }


        // VBO
        glGenVertexArrays(1, &obj.vao);
        glBindVertexArray(obj.vao);

        glGenBuffers(1, &obj.vbo);
        glBindBuffer(GL_ARRAY_BUFFER, obj.vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(GLfloat) * obj.nVerts * N_ATTR_PER_VERT,
                     (GLvoid*) (&obj.vertices[0]),
                     GL_STATIC_DRAW);
    }

    bool init(const Config& config) override {
        RenderPass::init(config);

        // Create shader
        GLuint vs = compileShader("gi.vs", GL_VERTEX_SHADER);
        GLuint fs = compileShader("gi.fs", GL_FRAGMENT_SHADER);
        shader = compileProgram(vs, fs);
        glDeleteShader(vs);
        glDeleteShader(fs);

        // Create uniforms
        modelMatUniform = GLuint(glGetUniformLocation(shader, "model"));
        viewMatUniform = GLuint(glGetUniformLocation(shader, "view"));
        projectionMatUniform = GLuint(glGetUniformLocation(shader, "projection"));

        // Create vertex buffers
        objects.resize(scene.worldData.shapes.size());
        for (size_t i = 0; i < objects.size(); i++) {
            buildVBO(i);
            buildVAO(i);
        }

        return true;
    }

    void cleanUp() override {
        // Delete vertex buffers
        for (size_t i = 0; i < objects.size(); i++) {
            glDeleteBuffers(1, &objects[i].vbo);
            glDeleteVertexArrays(1, &objects[i].vao);
        }

        RenderPass::cleanUp();
    }

    void render() override {
        glBindFramebuffer(GL_FRAMEBUFFER, postprocess_fboScreen);
        glClearColor(0.f, 0.f, 0.f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);

        // Define shader to use
        glUseProgram(shader);

        // Update camera
        glm::mat4 model, view, projection;
        camera.Update();
        camera.GetMatricies(projection, view, model);

        // Pass uniforms
        glUniformMatrix4fv(modelMatUniform, 1, GL_FALSE, &(modelMat[0][0]));
        glUniformMatrix4fv(viewMatUniform, 1, GL_FALSE, &(view[0][0]));
        glUniformMatrix4fv(projectionMatUniform, 1, GL_FALSE, &(projection[0][0]));

        // Draw
        for (auto& object : objects) {
            glBindVertexArray(object.vao); // original vao
            glDrawArrays(GL_TRIANGLES, 0, object.vertices.size()); // possibility number 1
            glBindVertexArray(0);
        }

        RenderPass::render();
    }

};

TR_NAMESPACE_END
