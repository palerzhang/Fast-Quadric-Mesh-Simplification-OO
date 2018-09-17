#pragma once
#pragma once
/////////////////////////////////////////////
// Original project license
/////////////////////////////////////////////
// Mesh Simplification Tutorial
//
// (C) by Sven Forstmann in 2014
//
// License : MIT
// http://opensource.org/licenses/MIT
//
// https://github.com/sp4cerat/Fast-Quadric-Mesh-Simplification
//
// 5/2016: Chris Rorden created minimal version for OSX/Linux/Windows compile
//
/////////////////////////////////////////////
// This project is an OOP version of Fast-Quadric-Mesh-Simplification
// 
// https://github.com/palerzhang/Fast-Quadric-Mesh-Simplification-OO
//
/////////////////////////////////////////////
//
// This is the plus edition of Fast-Quadric-Mesh-Simplification-OO,
// under namespace MeshSimplifierPlusSpace
// It supports multiple attributs vertices such as uv and normal
// 
/////////////////////////////////////////////

#include <math.h>
#include <vector>
#include <string>

#include "SymetricMatrix.hpp"
#include "Vector3.hpp"

using namespace MeshSimplifierMath;

namespace MeshSimplifierPlusSpace
{
	enum Attributes
	{
		POSITION,
		NORMAL = 2,
		TEXCOORD = 4,
		COLOR = 8
	};
	struct Triangle
	{
		int v[3];
		double err[4];
		int deleted, dirty;
		//MsVec3f n;
		//std::vector<MsVec3f> ns; // same size as  num_fields
		MsVec3f ns[3];
		//MsVec3f uvs[3];
		int material;
	};
	struct Vertex
	{
		std::vector<MsVec3f> attrs;
		SymetricMatrix qs[3];
		int tstart, tcount;
		//SymetricMatrix q; // position quadric
		//SymetricMatrix qu; // uv quadric
		//SymetricMatrix qn; // normal quadric

		int border;
	};

	struct Ref
	{
		int tid, tvertex;
	};

	// MeshSimplifier
	class MeshSimplifier
	{
	public:
		std::vector<Triangle> triangles;
		std::vector<Vertex> vertices;
		std::vector<Ref> refs;
		std::string mtllib;
		std::vector<std::string> materials;

		int attribute_fields;
		int num_fields;
		std::vector<float> coefs;

	public:
		MeshSimplifier() {}

	public:
		static MsVec3f Barycentric(const MsVec3f &p, const MsVec3f &a, const MsVec3f &b, const MsVec3f &c);
		static MsVec3f Interpolate(const MsVec3f &p, const MsVec3f &a, const MsVec3f &b, const MsVec3f &c, const MsVec3f attrs[3]);
		
	protected:
		// Error between vertex and Quadric
		double VertexError(const SymetricMatrix & q, double x, double y, double z);
		// Error for one edge
		double CalculateError(int id_v1, int id_v2, MsVec3f *p_result);
		// Check if a triangle flips when this edge is removed
		bool Flipped(const MsVec3f & p, int i0, int i1, Vertex &v0, Vertex &v1, std::vector<int> &deleted);
		// Update uvs
		void UpdateUVs(int i0, const Vertex &v, const MsVec3f &p, std::vector<int> &deleted);
		// Update triangle connections and edge error after a edge is collapsed
		void UpdateTriangles(int i0, Vertex &v, std::vector<int> &deleted, int &deleted_triangles);
		// Compact triangles, compute edge error and build reference list
		void UpdateMesh(int iteration);
		// Finally compact mesh before exiting
		void CompactMesh();

	public:

		void OptimizeVertices();
		//
		// Main simplification function
		//
		// target_count  : target nr. of triangles
		// agressiveness : sharpness to increase the threashold.
		//                 5..8 are good numbers
		//                 more iterations yield higher quality
		//
		void SimplifyMesh(int target_count, double agressiveness = 7, bool verbose = false);
		// Lossless mode
		void SimplifyMeshLossless(bool verbose = false);
	};

} // end mesh simplifier