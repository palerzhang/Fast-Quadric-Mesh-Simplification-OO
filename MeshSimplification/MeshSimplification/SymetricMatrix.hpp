#pragma once

#include "Vector3.hpp"

namespace MeshSimplifierMath
{
#define LOOP(_iter_, s, e) for (int _iter_ = s; _iter_ < e; _iter_++)

	// SymetricMatrix
	class SymetricMatrix
	{
		/*
			0 1 2 3
			1 4 5 6
			2 5 7 8
			3 6 8 9
		*/
	public:
		SymetricMatrix(double c = 0) { LOOP(i, 0, 10) m[i] = c; }

		SymetricMatrix(double m11, double m12, double m13, double m14,
								   double m22, double m23, double m24,
											   double m33, double m34,
														   double m44) 
		{
			m[0] = m11; m[1] = m12; m[2] = m13; m[3] = m14;
						m[4] = m22; m[5] = m23; m[6] = m24;
									m[7] = m33; m[8] = m34;
												m[9] = m44;
		}

		// Make plane

		SymetricMatrix(double a, double b, double c, double d)
		{
			m[0] = a * a; m[1] = a * b; m[2] = a * c; m[3] = a * d;
						  m[4] = b * b; m[5] = b * c; m[6] = b * d;
										m[7] = c * c; m[8] = c * d;
													  m[9] = d * d;
		}

		double operator[](int c) const { return m[c]; }

		// Determinant

		double det()
		{
			double A00 = det(4, 5, 6, 5, 7, 8, 6, 8, 9);
			double A01 = -det(1, 5, 6, 2, 7, 8, 3, 8, 9);
			double A02 = det(1, 4, 6, 2, 5, 8, 3, 6, 9);
			double A03 = -det(1, 4, 5, 2, 5, 7, 3, 6, 8);
			return A00 + A01 + A02 + A03;
		}

		double det(	int a11, int a12, int a13,
					int a21, int a22, int a23,
					int a31, int a32, int a33)
		{
			double det = m[a11] * m[a22] * m[a33] + m[a13] * m[a21] * m[a32] + m[a12] * m[a23] * m[a31]
				- m[a13] * m[a22] * m[a31] - m[a11] * m[a23] * m[a32] - m[a12] * m[a21] * m[a33];
			return det;
		}

		double det(	int a11, int a12,
					int a21, int a22)
		{
			return m[a11] * m[a22] - m[a12] * m[a21];
		}
		const SymetricMatrix operator+(const SymetricMatrix& n) const
		{
			return SymetricMatrix(m[0] + n[0], m[1] + n[1], m[2] + n[2], m[3] + n[3],
				m[4] + n[4], m[5] + n[5], m[6] + n[6],
				m[7] + n[7], m[8] + n[8],
				m[9] + n[9]);
		}

		SymetricMatrix& operator+=(const SymetricMatrix& n)
		{
			m[0] += n[0];   m[1] += n[1];   m[2] += n[2];   m[3] += n[3];
			m[4] += n[4];   m[5] += n[5];   m[6] += n[6];   m[7] += n[7];
			m[8] += n[8];   m[9] += n[9];
			return *this;
		}

		const MsVec3f operator* (const MsVec3f & v) const
		{
			return MsVec3f( m[0] * v.x + m[1] * v.y + m[2] * v.z,
							m[1] * v.x + m[4] * v.y + m[5] * v.z,
							m[2] * v.x + m[5] * v.y + m[7] * v.z);
		}

		double m[10];
	};

	/*
		vall = [vx,vy,vz, nx,ny,nz, u,v] (8)

		Q = M = M1 0 0	1 for position
				0 M2 0	2 for normal
				0 0 M3	3 for uv

		Qi = Mi = Ai bi		Ai : 3x3 for postion and normal
				  bi ci			 2x2 for uv

		Aall = A1 0 0	Aall-1 = A1-1 0 0
			   0 A2 0			 0 A2-1 0
			   0 0 A3			 0 0 A3-1

		Expand:
			Aex1 = A1 0 0	Aex2 = 0 0 0	Aex3 = 0 0 0
					0 0 0		   0 A2 0		   0 0 0
					0 0 0		   0 0 0		   0 0 A3

			bex1 = b1		bex2 = 0		bex2 = 0
					0			   b2			   0
					0			   0			   b3

		ball = [b1, b2, b3]T
		
		Qall(v) = Q1 + Q2 + Q3
				= sum(viT*Ai*vi + 2biT*vi + ci)
				(with Aexi and bexi)
				= vallT*Aall*vall + 2ballT*vall + sum(ci)

		d(Qall)/d(v) = 2Aall*vall + 2ball
		v = -Aall-1 * ball
	*/
	class SymetricMatrix3
	{
	public:
		SymetricMatrix ms[3];

		SymetricMatrix3(double c = 0)
		{
			ms[0] = SymetricMatrix(c);
			ms[1] = SymetricMatrix(c);
			ms[2] = SymetricMatrix(c);
		}

		SymetricMatrix3(SymetricMatrix s1, SymetricMatrix s2, SymetricMatrix s3)
		{
			ms[0] = s1;
			ms[1] = s2;
			ms[2] = s3;
		}

		double detAall()
		{
			return ms[0].det(0, 1, 2, 1, 4, 5, 2, 5, 7) 
				* ms[1].det(0, 1, 2, 1, 4, 5, 2, 5, 7) 
				* ms[2].det(0, 1, 1, 4);
		}

		const SymetricMatrix3 operator+(const SymetricMatrix3& n) const
		{
			return SymetricMatrix3(ms[0] + n.ms[0], ms[1] + n.ms[1], ms[2] + n.ms[2]);
		}

		SymetricMatrix3& operator+=(const SymetricMatrix3& n)
		{
			ms[0] += n.ms[0];
			ms[1] += n.ms[1];
			ms[2] += n.ms[2];
			return *this;
		}

		void InverseOfAllA(SymetricMatrix3 & inv)
		{
			for (int i = 0; i < 2; i++)
			{
				double det_a_i = ms[i].det(0, 1, 2, 1, 4, 5, 2, 5, 7);
				inv.ms[i].m[0] = - ms[i].det(4, 5, 5, 7) / det_a_i;
				inv.ms[i].m[1] = ms[i].det(1, 5, 2, 7) / det_a_i;
				inv.ms[i].m[2] = - ms[i].det(1, 4, 2, 5) / det_a_i;
				inv.ms[i].m[4] = - ms[i].det(0, 2, 2, 7) / det_a_i;
				inv.ms[i].m[5] = ms[i].det(0, 1, 2, 5) / det_a_i;
				inv.ms[i].m[7] = -ms[i].det(0, 1, 1, 5) / det_a_i;
			}
		}

		// find minimum of Qall
		double OptimalVectorsAndUpdateErrors(MsVec3f * v1, MsVec3f * v2, MsVec3f * vecs, bool border)
		{
			double det_all = detAall();
			if (det_all != 0 && !border)
			{
				
			}
		}
	};
}