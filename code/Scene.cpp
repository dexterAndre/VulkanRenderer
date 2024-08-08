//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	Body body;
	body.m_position = Vec3( 0, 0, 0.5f );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_invMass = 1.0f;
	body.m_shape = new ShapeSphere( 1.0f );
	m_bodies.push_back( body );

	Body globe;
	globe.m_position = Vec3( 0, 0, -101 );
	globe.m_orientation = Quat( 0, 0, 0, 1 );
	globe.m_invMass = 0.0f;
	globe.m_shape = new ShapeSphere( 100.0f );
	m_bodies.push_back( globe );

	// TODO: Add code
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec ) {
	for (int i = 0; i < m_bodies.size(); i++)
	{
		// Acceleration due to gravity
		Body& body = m_bodies[i];

		// Gravity needs to be an impulse
		// I = dp, F = dp/dt => dp = F * dt => I = F * dt
		// F = mgs
		float mass = 1.0f / body.m_invMass;
		Vec3 impulseGravity = Vec3(0, 0, -9.81f) * mass * dt_sec;
		body.ApplyPulseLinear(impulseGravity);
	}

	// Check for collisions with other bodies
	for (int i = 0; i < m_bodies.size(); i++)
	{
		for (int j = i + 1; j < m_bodies.size(); j++)
		{
			Body* bodyA = &m_bodies[i];
			Body* bodyB = &m_bodies[j];

			if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass)
			{
				continue;
			}

			contact_t contact;
			if (Intersect(bodyA, bodyB, contact))
			{
				ResolveContact(contact);
			}
		}
	}

	for (int i = 0; i < m_bodies.size(); i++)
	{
		// Positional update
		Body& body = m_bodies[i];
		body.m_position += body.m_linearVelocity * dt_sec;
	}
}

/*
====================================================
Scene::Intersect
====================================================
*/
bool Scene::Intersect(Body* bodyA, Body* bodyB, contact_t& contact)
{
	contact.bodyA = bodyA;
	contact.bodyB = bodyB;

	const Vec3 ab = bodyA->m_position - bodyB->m_position;
	contact.normal = ab;
	contact.normal.Normalize();

	const ShapeSphere* sphereA = (const ShapeSphere*)bodyA->m_shape;
	const ShapeSphere* sphereB = (const ShapeSphere*)bodyB->m_shape;

	contact.ptOnA_WorldSpace = bodyA->m_position + contact.normal * sphereA->m_radius;
	contact.ptOnB_WorldSpace = bodyB->m_position + contact.normal * sphereB->m_radius;

	const float radiusAB = sphereA->m_radius + sphereB->m_radius;
	const float lengthSqr = ab.GetLengthSqr();

	if (lengthSqr <= (radiusAB * radiusAB))
	{
		return true;
	}

	return false;
}

/*
====================================================
Scene::ResolveContact
====================================================
*/
void Scene::ResolveContact(contact_t& contact)
{
	Body* bodyA = contact.bodyA;
	Body* bodyB = contact.bodyB;

	bodyA->m_linearVelocity.Zero();
	bodyB->m_linearVelocity.Zero();
}
