#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class NprIntegrator : public Integrator
{
public:
    // Constructor
    NprIntegrator(const PropertyList& props) {
    }

    // Required method to hookup the class to nori
    virtual std::string toString() const {
        return tfm::format(
            "NprIntegrator[\n"
            "  \n"
            "]");
    }

    // Core integrator function
    virtual Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        return 0.f;
    }

    // Integrator function that uses ray differentials
    virtual Color3f Li(const Scene *scene, Sampler *sampler, const RayDifferential& ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return Color3f(1.f);
        }
        else {
            const BSDF* currBSDF = its.mesh->getBSDF();
            if (currBSDF != nullptr) {
                // create the emitter query record and sample a light source
                // assuming just one light source for now.
                EmitterQueryRecord eRec;
                const Emitter* light = scene->getLights()[0];
                if (light != nullptr) {
                    // Sample the light source
                    eRec.ref = its.p;
                    Color3f Li = light->sample(eRec, sampler->next2D());

                    // compute the bsdf contribution
                    const Vector3f wo = its.shFrame.toLocal(-ray.d.normalized());
                    const Vector3f wi = its.shFrame.toLocal(eRec.wi);
                    BSDFQueryRecord bRec(wo, wi, ESolidAngle);
                    const Color3f f = currBSDF->eval(bRec);

                    // Compute visibility;
                    Ray3f shadowRay(its.p, eRec.wi, Epsilon, (1.0f - Epsilon) * eRec.dist);
                    const float vis = scene->rayIntersect(shadowRay) ? 0.f : 1.f;

                    // compute other terms and the final color
                    const float cosTheta = std::abs(Frame::cosTheta(wi));
                    Color3f L = Li * f * cosTheta * vis;
                    
                    // Compute the edge color
                    std::unique_ptr<Intersection> stencilHits(new Intersection[ray.m_totalStencilRays]);
                    for (int rd = 0; rd < ray.m_totalStencilRays; rd++) {
                        scene->rayIntersect(ray.getStencilRay(rd), stencilHits.get()[rd]);
                    }

                    // count m
                    int m = 0;
                    const Mesh* gS = its.mesh;
                    for (int i = 0; i < ray.m_totalStencilRays; i++) {
                        const Mesh* gR = stencilHits.get()[i].mesh;
                        if (gR != gS)
                            m++;
                    }

                    // compute the edge metric
                    float constant = 0.5f * ray.m_totalStencilRays;
                    float edgeStrength = 1.0f - std::abs(m - constant) / constant;

                    // lerp between L and edge strength
                    return (1.0f - edgeStrength) * L + edgeStrength * Color3f(0.f);
                }
            }
            return Color3f(1.f);
        }
    }
};

NORI_REGISTER_CLASS(NprIntegrator, "npr")
NORI_NAMESPACE_END