#include "NewtOgre.h"

#include <OgreNode.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h>
#include <OgreSubMesh.h>

#include <Poco/NObserver.h>

#include "dMath/dMatrix.h"
#include "CustomHinge.h"
#include "CustomUpVector.h"
#include "CustomBallAndSocket.h"
#include "CustomUniversal.h"
#include "Custom6DOF.h"

#include "BaseFrameListener.h"

//............

unsigned RayFilterObj::check(const NewtonBody *body)
{
    unsigned ret = 0;
    if(caster != body)
    {
        int mat = NewtonBodyGetMaterialGroupID(body);
        if(mat != PhysEngine::self()->m_ghostMat && mat != PhysEngine::self()->m_emptyMat && mat != PhysEngine::self()->m_pointMat)
            ret = 1;
    }

    return ret;
}

float RayGrabObj::finded(const NewtonBody *body, const Ogre::Vector3 &point, float intersectParam)
{
    obj.obj = body;
    obj.point = point;
    obj.intersectParam = intersectParam;

    return 0.0;
}

float RayGrabManyObj::finded(const NewtonBody *body, const Ogre::Vector3 &point, float intersectParam)
{
    objs.push_back(RayCastObj(body, point, intersectParam));

    return 1.0;
}

const RayCastObj* RayGrabManyObj::getFinded()
{
    RayCastObj* rco = 0;
    if(curObj < objs.size())
        rco = &objs[curObj++];

    return rco;
}

//.......................
