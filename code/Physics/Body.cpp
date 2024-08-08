//
//  Body.cpp
//
#include "Body.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body() :
	m_position( 0.0f ),
	m_orientation( 0.0f, 0.0f, 0.0f, 1.0f ),
	m_linearVelocity( 0.0f, 0.0f, 0.0f ),
	m_invMass( 0.0f ),
	m_shape( NULL ) {
}

Vec3 Body::GetCenterOfMassWorldSpace() const
{
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	const Vec3 pos = m_position + m_orientation.RotatePoint(centerOfMass);
	return pos;
}

Vec3 Body::GetCenterOfMassModelSpace() const
{
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	return centerOfMass;
}

Vec3 Body::WorldSpaceToBodySpace(const Vec3& worldPt) const
{
	Vec3 temp = worldPt - GetCenterOfMassWorldSpace();
	Quat inverseOrient = m_orientation.Inverse();
	Vec3 bodySpace = inverseOrient.RotatePoint(temp);
	return bodySpace;
}

Vec3 Body::BodyspaceToWorldSpace(const Vec3& worldPt) const
{
	Vec3 worldSpace = GetCenterOfMassWorldSpace() + m_orientation.RotatePoint(worldPt);
	return worldSpace;
}

void Body::ApplyPulseLinear(const Vec3& impulse)
{
	if (0.0f == m_invMass)
	{
		return;
	}

	// p = mv
	// dp = m dv = J
	// => dv = J / m
	m_linearVelocity += impulse * m_invMass;
}
