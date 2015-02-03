#ifndef PERSON_H_INCLUDED
#define PERSON_H_INCLUDED

#include "NewtOgre.h"
#include "Movable.h"

class PersonState
{
public:
    virtual ~PersonState() {}
    virtual void enter() {}
    virtual void exec() = 0;
    virtual void setDest(const Ogre::Vector3 &dest, float _distToObj, float _speed) {}
    virtual void getDest(Ogre::Vector3 &dest) {}
    virtual void setFindObj(const NewtonBody *body) {}
};

class PersonFindStrategy
{
public:
    virtual ~PersonFindStrategy() {}

    virtual const NewtonBody* find(PersonState *st, const NewtonBody *mbody, RayFilterObj *filter = 0) = 0;
    virtual RayFilterObj* getFilter() { return 0; }
};

class PersonAutomat
{
public:
    PersonAutomat():curState(0), prevState(0), findState(0), walkState(0), actState(0) {}
    virtual ~PersonAutomat() {}

    inline PersonFindStrategy* getFindStrategy() { return fst; }

    PersonState* getFind() { return findState; }
    PersonState* getWalk() { return walkState; }
    PersonState* getAction() { return actState; }

    inline void setFind(PersonState *ps) { findState = ps; }

    void setState(PersonState *ns);
    PersonState* getState() { return curState; }
    PersonState* getPrevState() { return prevState; }
    virtual void stateChanged() = 0;

    inline const NewtonBody* getBody() { return body->body; }
    inline void walk(const float speed, const float strafe = 0) { body->walk(speed, strafe); }
    inline void rotate(const Ogre::Quaternion &rot) { body->rotate(rot); }

protected:
    PersonFindStrategy *fst;

    PersonState *curState, *prevState;
    PersonState *findState;
    PersonState *walkState;
    PersonState *actState;

    Poco::SharedPtr<Movable> body;
};

//класс для фильтра найденых объектов с сохранием для последуещего пропуска при повторном скане
class CastPoint : public RayFilterObj
{
public:
    CastPoint(size_t _memPoint);
    ~CastPoint() { delete rfilter; }

    unsigned check(const NewtonBody *body);
    //установив аргумент point нулевым объект будет добавлен в список уже обноруженых
    float finded(const NewtonBody *body, const Ogre::Vector3 &point, float intersectParam);

    inline const RayCastObj* getFinded() { return rfilter->getFinded(); }
    inline void clear() { rfilter->clear(); }

private:
    RayFilterObj *rfilter;
    size_t memPoint;
    std::vector<const NewtonBody*> memPoints;
};

//базовая стратегия для поиска объектов
class PersonFindPoint : public PersonFindStrategy
{
public:
    PersonFindPoint(float _findDist, float _distToPoint, float _speed);
    ~PersonFindPoint() { delete castPoint; }

    const NewtonBody* find(PersonState *st, const NewtonBody *mbody, RayFilterObj *filter = 0);
    inline RayFilterObj* getFilter() { return castPoint; }

private:
    RayFilterObj *castPoint;
    NewtonCollision *pointCol;
    float findDist;
    float distToPoint;
    float speed;
};

//состояние реализующее поиск объектов
class PersonFind : public PersonState
{
public:
    PersonFind(PersonAutomat *_atm):atm(_atm) {}

    inline void setDest(const Ogre::Vector3 &dest, float distToObj, float speed) { atm->getWalk()->setDest(dest, distToObj, speed); }
    inline void getDest(Ogre::Vector3 &dest) { atm->getWalk()->getDest(dest); }

    void exec();

private:
    PersonAutomat *atm;
};

//стратегия анализа найденых объектов
class PersonWalkDest : public PersonFindStrategy
{
public:
    PersonWalkDest(float _findDist, float _distToPoint, float _speed);
    ~PersonWalkDest() { delete castPoint; }

    const NewtonBody* find(PersonState *st, const NewtonBody *mbody, RayFilterObj *filter = 0);

private:
    RayFilterObj *castPoint;
    NewtonCollision *pointCol;
    float findDist;
    float distToPoint;
    float speed;
};

//состояние реализующее ходьбу объекта
class PersonWalk : public PersonState
{
public:
    PersonWalk(PersonAutomat *_atm)
        :atm(_atm), distToObj(0), lastDist(0), speed(0), destPoint(0, 0, 0) {}

    void exec();

    void getDest(Ogre::Vector3 &dest) { dest = destPoint; }
    void setDest(const Ogre::Vector3 &dest, float _distToObj, float _speed);

private:
    PersonAutomat *atm;
    float distToObj, lastDist;
    float speed;
    Ogre::Vector3 destPoint;
};

#endif // PERSON_H_INCLUDED
