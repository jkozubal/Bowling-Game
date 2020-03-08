#pragma once

#include "PxPhysicsAPI.h"
using namespace physx;

class Physics
{
public:
    Physics(float gravity);
    virtual ~Physics();
    PxCooking* cooking;
    PxPhysics*              physics = nullptr;
    PxScene*				scene = nullptr;

    void step(float dt);

private:
    PxDefaultAllocator		allocator;
    PxDefaultErrorCallback	errorCallback;
    PxFoundation*			foundation = nullptr;
    PxDefaultCpuDispatcher*	dispatcher = nullptr;
};