#pragma once

#include <math.h>

namespace MeshSimplifierMath
{
	// vector3
	struct MsVector3
	{
		double x, y, z;
	};

	// vec3f
	struct MsVec3f
	{
		double x, y, z;

		inline MsVec3f(void) { x = 0; y = 0; z = 0; }

		inline MsVec3f(MsVector3 a)
		{
			x = a.x; y = a.y; z = a.z;
		}

		inline MsVec3f(const double X, const double Y, const double Z)
		{
			x = X; y = Y; z = Z;
		}

		inline MsVec3f operator + (const MsVec3f& a) const
		{
			return MsVec3f(x + a.x, y + a.y, z + a.z);
		}

		inline MsVec3f operator += (const MsVec3f& a) const
		{
			return MsVec3f(x + a.x, y + a.y, z + a.z);
		}

		inline MsVec3f operator * (const double a) const
		{
			return MsVec3f(x * a, y * a, z * a);
		}

		inline MsVec3f operator * (const MsVec3f a) const
		{
			return MsVec3f(x * a.x, y * a.y, z * a.z);
		}

		inline MsVec3f v3() const
		{
			return MsVec3f(x, y, z);
		}

		inline MsVec3f operator = (const MsVector3 a)
		{
			x = a.x; y = a.y; z = a.z; return *this;
		}

		inline MsVec3f operator = (const MsVec3f a)
		{
			x = a.x; y = a.y; z = a.z; return *this;
		}

		inline MsVec3f operator / (const MsVec3f a) const
		{
			return MsVec3f(x / a.x, y / a.y, z / a.z);
		}

		inline bool operator == (const MsVec3f a) const
		{
			return (x == a.x && y == a.y && z == a.z);
		}

		inline MsVec3f operator - (const MsVec3f& a) const
		{
			return MsVec3f(x - a.x, y - a.y, z - a.z);
		}

		inline MsVec3f operator / (const double a) const
		{
			return MsVec3f(x / a, y / a, z / a);
		}

		inline double dot(const MsVec3f& a) const
		{
			return a.x*x + a.y*y + a.z*z;
		}

		inline MsVec3f cross(const MsVec3f& a, const MsVec3f& b)
		{
			x = a.y * b.z - a.z * b.y;
			y = a.z * b.x - a.x * b.z;
			z = a.x * b.y - a.y * b.x;
			return *this;
		}

		inline double angle(const MsVec3f& v)
		{
			MsVec3f a = v, b = *this;
			double dot = v.x*x + v.y*y + v.z*z;
			double len = a.length() * b.length();
			if (len == 0)len = 0.00001f;
			double input = dot / len;
			if (input < -1) input = -1;
			if (input > 1) input = 1;
			return (double)acos(input);
		}

		inline double angle2(const MsVec3f& v, const MsVec3f& w)
		{
			MsVec3f a = v, b = *this;
			double dot = a.x*b.x + a.y*b.y + a.z*b.z;
			double len = a.length() * b.length();
			if (len == 0)len = 1;

			MsVec3f plane; plane.cross(b, w);

			if (plane.x * a.x + plane.y * a.y + plane.z * a.z > 0)
				return (double)-acos(dot / len);

			return (double)acos(dot / len);
		}

		inline MsVec3f rot_x(double a)
		{
			double yy = cos(a) * y + sin(a) * z;
			double zz = cos(a) * z - sin(a) * y;
			y = yy; z = zz;
			return *this;
		}
		inline MsVec3f rot_y(double a)
		{
			double xx = cos(-a) * x + sin(-a) * z;
			double zz = cos(-a) * z - sin(-a) * x;
			x = xx; z = zz;
			return *this;
		}
		inline void clamp(double min, double max)
		{
			if (x < min) x = min;
			if (y < min) y = min;
			if (z < min) z = min;
			if (x > max) x = max;
			if (y > max) y = max;
			if (z > max) z = max;
		}
		inline MsVec3f rot_z(double a)
		{
			double yy = cos(a) * y + sin(a) * x;
			double xx = cos(a) * x - sin(a) * y;
			y = yy; x = xx;
			return *this;
		}
		inline MsVec3f invert()
		{
			x = -x; y = -y; z = -z; return *this;
		}
		inline MsVec3f frac()
		{
			return MsVec3f(
				x - double(int(x)),
				y - double(int(y)),
				z - double(int(z))
			);
		}

		inline MsVec3f integer()
		{
			return MsVec3f(
				double(int(x)),
				double(int(y)),
				double(int(z))
			);
		}

		inline double length() const
		{
			return (double)sqrtf(x*x + y * y + z * z);
		}

		inline MsVec3f normalize(double desired_length = 1)
		{
			double square = sqrtf(x*x + y * y + z * z);
			x /= square; y /= square; z /= square;

			return *this;
		}

		static int random_number;

		double random_float_01(double a)
		{
			double rnf = a * 14.434252 + a * 364.2343 + a * 4213.45352 + a * 2341.43255 + a * 254341.43535 + a * 223454341.3523534245 + 23453.423412;
			int rni = ((int)rnf) % 100000;
			return double(rni) / (100000.0 - 1.0);
		}

		MsVec3f random01_fxyz()
		{
			x = (double)random_float_01(x);
			y = (double)random_float_01(y);
			z = (double)random_float_01(z);
			return *this;
		}
	};
}