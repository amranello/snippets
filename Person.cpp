#include "Person.h"

void PersonAutomat::setState(PersonState *ns)
{
    if(ns != curState)
    {
        prevState = curState;
        curState = ns;
        curState->enter();
        stateChanged();
    }
}

CastPoint::CastPoint(size_t _memPoint):memPoint(0)
{
    memPoints.resize(_memPoint);

    rfilter = new RayGrabManyObj();
}

unsigned CastPoint::check(const NewtonBody *body)
{
    unsigned ret = 0;
    if(NewtonBodyGetMaterialGroupID(body) == PhysEngine::self()->m_pointMat)
        ret = 1;

    return ret;
}

float CastPoint::finded(const NewtonBody *body, const Ogre::Vector3 &point, float intersectParam)
{
    if(point == Ogre::Vector3::ZERO)
    {
        if(memPoint >= memPoints.size())
            memPoint = 0;

        memPoints[memPoint++] = body;

        return 0.0;
    }

    //пропускаем уже найденные
    bool skip = false;
    std::vector<const NewtonBody*>::iterator it = memPoints.begin();
    for(; it != memPoints.end(); it++)
        if((*it) == body)
        {
            skip = true;
            break;
        }

    //иначе сохраням в список найденных
    if(!skip)
        rfilter->finded(body, point, intersectParam);

    return 1.0;
}

PersonFindPoint::PersonFindPoint(float _findDist, float _distToPoint, float _speed)
    :findDist(_findDist), distToPoint(_distToPoint), speed(_speed)
{
    castPoint = new CastPoint(3);
    pointCol = PhysEngine::self()->createCollision("MCPointCast", "box", Vector3(3.0, 1.0, 0.1));
}

const NewtonBody* PersonFindPoint::find(PersonState *st, const NewtonBody *mbody, RayFilterObj *filter)
{
    const NewtonBody *fb = 0;
    Ogre::Vector3 dest;
    st->getDest(dest);
    //если не задано направление ищем ближайшие объекты и при нахождение установливаем его целью
    if(dest == Ogre::Vector3::ZERO)
    {
        PhysEngineFun::convexRayCast(mbody, Vector3(0, 0, -findDist), pointCol, (filter != 0 ) ? filter : castPoint);

        const RayCastObj *robj;
        dMatrix mat;
        NewtonBodyGetMatrix(mbody, &mat[0][0]);
        Ogre::Vector3 pos(PhysEngineFun::from(mat.m_posit));
        float mdiff = 0, diff;

        do
        {
            robj = castPoint->getFinded();
            if(robj && robj->obj)
            {
                diff = (robj->point - pos).length();
                if(diff < mdiff || mdiff == 0)
                {
                    mdiff = diff;
                    fb = robj->obj;
                }
            }
        }
        while(robj);

        if(fb)
        {
            castPoint->finded(fb, Ogre::Vector3::ZERO, 0);

            dMatrix cObjMat;
            NewtonBodyGetMatrix(fb, &cObjMat[0][0]);
            st->setDest(PhysEngineFun::from(cObjMat.m_posit), distToPoint, speed);
        }
    }

    return fb;
}

void PersonFind::exec()
{
    const NewtonBody *fb = atm->getFindStrategy()->find(this, atm->getBody());

    //найден объект переходим в состояние ходьбы иначе поворачиваемся
    if(fb)
        atm->setState(atm->getWalk());
    else
    {
        Quaternion q;
        NewtonBodyGetRotation(atm->getBody(), &q.w);
        atm->rotate(q * Quaternion(Radian(Degree(10).valueRadians()), Vector3(0, 1, 0)));
    }
}

void PersonWalk::exec()
{
    dMatrix mat;
    NewtonBodyGetMatrix(atm->getBody(), &mat[0][0]);
    Ogre::Vector3 diff(destPoint - PhysEngineFun::from(mat.m_posit));
    if(diff.length() < distToObj)
    {
        //подошли на нужное растояние переход в дейсвие
        destPoint = Ogre::Vector3(0, 0, 0);
        atm->setState(atm->getAction());
    }
    else
    {
        //поворачиваемся в направлении объекта, идем и продолжаем поиск объектов
        Ogre::Quaternion rot(Movable::rotateTo(atm->getBody(), destPoint));
        atm->rotate(rot);
        atm->walk(-speed);

        atm->getFindStrategy()->find(this, atm->getBody());
    }
}

void PersonWalk::setDest(const Ogre::Vector3 &dest, float _distToObj, float _speed)
{
    destPoint = dest;
    speed = _speed;
    distToObj = _distToObj;
}
