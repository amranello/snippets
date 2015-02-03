#ifndef NEWTOGRE_H_INCLUDED
#define NEWTOGRE_H_INCLUDED

#include <map>

#include <OgreManualObject.h>
#include <OgreNode.h>

#include <Poco/NotificationCenter.h>
#include <Poco/SharedPtr.h>

#include <OgreOggSound/OgreOggISound.h>

#include "Newton.h"
#include "dMath/dVector.h"
#include "dMath/dQuaternion.h"

#include "NotificationClases.h"

using namespace std;
using namespace Poco;

//.......

class RayCastObj
{
public:
    const NewtonBody *obj;
    Ogre::Vector3 point;
    float intersectParam;

    RayCastObj():obj(0), point(0, 0, 0), intersectParam(0) {}
    RayCastObj(const NewtonBody *_obj, const Ogre::Vector3 &_point, float _intersectParam)
        :obj(_obj), point(_point), intersectParam(_intersectParam) {}
};

//интерфейс для фильтра который передаеться в функцию поиска объектов
class RayFilterObj
{
public:
    virtual ~RayFilterObj() {}

    void setCaster(const NewtonBody *_caster) { caster = _caster; }

    //функция для фильтра найденых объектов
    virtual unsigned check(const NewtonBody *body);
    //функция для сохранения подходящего пойманого объекта
    virtual float finded(const NewtonBody *body, const Ogre::Vector3 &point, float intersectParam) = 0;

    virtual const RayCastObj* getFinded() = 0;
    virtual void clear() = 0;

protected:
    //объект который буде исключен со скана
    const NewtonBody *caster;
};

//класс для сохранения одного объекта с отменой дальнейшего поиска
class RayGrabObj : public RayFilterObj
{
public:
    virtual float finded(const NewtonBody *body, const Ogre::Vector3 &point, float intersectParam);

    const RayCastObj* getFinded() { return &obj; }
    void clear() { obj.obj = 0; }

protected:
    RayCastObj obj;
};

//класс для сохранени всех объектов на пути следования луча
class RayGrabManyObj : public RayFilterObj
{
public:
    RayGrabManyObj():curObj(0) {}

    virtual float finded(const NewtonBody *body, const Ogre::Vector3 &point, float intersectParam);

    const RayCastObj* getFinded();
    void clear() { curObj = 0; objs.clear(); }

private:
    size_t curObj;
    std::vector<RayCastObj> objs;
};

//..............................

#endif // NEWTOGRE_H_INCLUDED
