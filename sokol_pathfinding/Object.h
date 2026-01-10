#pragma once
#ifndef SHAPE_STRUCT_H
#define SHAPE_STRUCT_H

#include "mesh.h"

#include "linemesh.h"

#include "math/mat4.h"

enum objectType
{
	OBJECT,
	BILLBOARD,
	TERRAIN,
	OUTLINES
};

struct Object {
	Mesh mesh;
	objectType objtype;
	LineMesh linemesh;
	AABB3 aabb;

	sg_view tex{};
	
	bool draggable = false;
	bool isbillboard = false;

	cmn::vf3d translation, rotation, scale{ 1, 1, 1 };
	mat4 model = mat4::makeIdentity();
	int num_x = 0, num_y = 0;
	int num_ttl = 0;

	float anim_timer = 0;
	int anim = 0;

	Object() {}

	Object(const Mesh& m, sg_view t) {
		mesh = m;
		linemesh = LineMesh::makeFromMesh(m);
		linemesh.randomizeColors();
		linemesh.updateVertexBuffer();
		tex = t;
	}

	void updateMatrixes() {
		//xyz euler angles?
		mat4 rot_x = mat4::makeRotX(rotation.x);
		mat4 rot_y = mat4::makeRotY(rotation.y);
		mat4 rot_z = mat4::makeRotZ(rotation.z);
		mat4 rot = mat4::mul(rot_z, mat4::mul(rot_y, rot_x));

		mat4 scl = mat4::makeScale(scale);

		mat4 trans = mat4::makeTranslation(translation);

		//combine & invert
		model = mat4::mul(trans, mat4::mul(rot, scl));
	}

	//aabb stuff
	AABB3 getAABB() const
	{
		AABB3 box;
		float w = 1.0f;
		for (const auto& v : mesh.verts)
		{
			box.fitToEnclose(matMulVec(model,v.pos, w));
		}
		return box;
	}

	float random() const
	{
		static const float rand_max = RAND_MAX;
		return rand() / rand_max;
	}

	bool contains(const cmn::vf3d& pt) 
	{
		cmn::vf3d dir = cmn::vf3d(
			0.5f - random(),
			0.5f - random(),
			0.5f - random()
		).norm();

		int num = 0;
		for (const auto& t : mesh.tris)
		{
			float dist = intersectRay(pt, pt - dir);
			if (dist > 0) num++;

		}
		return num & 2;
	}

	float intersectRay(const cmn::vf3d& orig, const cmn::vf3d& dir)
	{
		float record = -1;
		for (const auto& t : mesh.tris)
		{
			float dist = mesh.rayIntersectTri(
				                         orig,
				                         dir,
				                         mesh.verts[t.a].pos,
				                         mesh.verts[t.b].pos,
				                         mesh.verts[t.c].pos);

			if (dist > 0)
			{
				if (record < 0 || dist < record)
				{
					record = dist;
				}
			}
		}

		return record;
	}
};
#endif
