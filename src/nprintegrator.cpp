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
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return Color3f(0.f);
        } else {
            const BSDF* currBSDF = its.mesh->getBSDF();
            if (currBSDF != nullptr) {
                // create the emitter query record and sample a light source
                // assuming just one light source for now.
                EmitterQueryRecord eRec;
                const Emitter* light = scene->getLights()[0];
                if (light != nullptr) {
                    eRec.ref = its.p;
                    Color3f Li = light->sample(eRec, sampler->next2D());

                    // compute the contribution
                    BSDFQueryRecord bRec(its.shFrame.toLocal(eRec.wi.normalized()), 
                        its.shFrame.toLocal(-ray.d.normalized()), ESolidAngle);

                    // Compute visibility;
                    Ray3f shadowRay(its.p, eRec.wi, Epsilon, (1.0f - Epsilon) * eRec.dist);
                    const float vis = scene->rayIntersect(shadowRay) ? 0.f : 1.f;
                    
                    // compute other terms
                    const float cosTheta = std::max(Frame::cosTheta(its.shFrame.toLocal(eRec.wi.normalized())), 0.f);
                    Color3f L = vis *  currBSDF->eval(bRec) * Li * cosTheta ;
                    return L;
                }
            }
        }
        return 0.f;
    }

    // Integrator function that uses ray differentials
    virtual Color3f Li(const Scene *scene, Sampler *sampler, const RayDifferential& rayDifferential) const {
        return Color3f(0.f);
    }
};

NORI_REGISTER_CLASS(NprIntegrator, "npr")
NORI_NAMESPACE_END