#include "MeshSimplifier.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <iostream>

using namespace MeshSimplifierSpace;
using namespace MeshSimplifierMath;

char *trimwhitespace(char *str)
{
	char *end;

	// Trim leading space
	while (isspace((unsigned char)*str)) str++;

	if (*str == 0)  // All spaces?
		return str;

	// Trim trailing space
	end = str + strlen(str) - 1;
	while (end > str && isspace((unsigned char)*end)) end--;

	// Write new null terminator
	*(end + 1) = 0;

	return str;
}

//Option : Load OBJ
void load_obj(MeshSimplifier * ms, const char* filename, bool process_uv = false) 
{
	ms->vertices.clear();
	ms->triangles.clear();
	//printf ( "Loading Objects %s ... \n",filename);
	FILE* fn;
	if (filename == NULL)		return;
	if ((char)filename[0] == 0)	return;
	if ((fn = fopen(filename, "rb")) == NULL)
	{
		printf("File %s not found!\n", filename);
		return;
	}
	char line[1000];
	memset(line, 0, 1000);
	int vertex_cnt = 0;
	int material = -1;
	std::map<std::string, int> material_map;
	std::vector<MsVec3f> uvs;
	// uvMap[i][j] means i-th triangle's j-th vertex's uv index
	std::vector<std::vector<int> > uvMap; 

	while (fgets(line, 1000, fn) != NULL)
	{
		Vertex v;
		MsVec3f uv;

		if (strncmp(line, "mtllib", 6) == 0)
		{
			ms->mtllib = trimwhitespace(&line[7]);
		}
		if (strncmp(line, "usemtl", 6) == 0)
		{
			std::string usemtl = trimwhitespace(&line[7]);
			if (material_map.find(usemtl) == material_map.end())
			{
				material_map[usemtl] = ms->materials.size();
				ms->materials.push_back(usemtl);
			}
			material = material_map[usemtl];
		}

		if (line[0] == 'v' && line[1] == 't')
		{
			if (line[2] == ' ')
				if (sscanf(line, "vt %lf %lf",
					&uv.x, &uv.y) == 2)
				{
					uv.z = 0;
					uvs.push_back(uv);
				}
				else
					if (sscanf(line, "vt %lf %lf %lf",
						&uv.x, &uv.y, &uv.z) == 3)
					{
						uvs.push_back(uv);
					}
		}
		else if (line[0] == 'v')
		{
			if (line[1] == ' ')
				if (sscanf(line, "v %lf %lf %lf",
					&v.p.x, &v.p.y, &v.p.z) == 3)
				{
					ms->vertices.push_back(v);
				}
		}
		int integers[9];
		if (line[0] == 'f')
		{
			Triangle t;
			bool tri_ok = false;
			bool has_uv = false;

			if (sscanf(line, "f %d %d %d",
				&integers[0], &integers[1], &integers[2]) == 3)
			{
				tri_ok = true;
			}
			else
				if (sscanf(line, "f %d// %d// %d//",
					&integers[0], &integers[1], &integers[2]) == 3)
				{
					tri_ok = true;
				}
				else
					if (sscanf(line, "f %d//%d %d//%d %d//%d",
						&integers[0], &integers[3],
						&integers[1], &integers[4],
						&integers[2], &integers[5]) == 6)
					{
						tri_ok = true;
					}
					else
						if (sscanf(line, "f %d/%d/%d %d/%d/%d %d/%d/%d",
							&integers[0], &integers[6], &integers[3],
							&integers[1], &integers[7], &integers[4],
							&integers[2], &integers[8], &integers[5]) == 9)
						{
							tri_ok = true;
							has_uv = true;
						}
						else
						{
							printf("unrecognized sequence\n");
							printf("%s\n", line);
							while (1);
						}
			if (tri_ok)
			{
				t.v[0] = integers[0] - 1 - vertex_cnt;
				t.v[1] = integers[1] - 1 - vertex_cnt;
				t.v[2] = integers[2] - 1 - vertex_cnt;
				t.attr = 0;

				if (process_uv && has_uv)
				{
					std::vector<int> indices;
					indices.push_back(integers[6] - 1 - vertex_cnt);
					indices.push_back(integers[7] - 1 - vertex_cnt);
					indices.push_back(integers[8] - 1 - vertex_cnt);
					uvMap.push_back(indices);
					t.attr |= TEXCOORD;
				}

				t.material = material;
				//geo.triangles.push_back ( tri );
				ms->triangles.push_back(t);
				//state_before = state;
				//state ='f';
			}
		}
	}

	if (process_uv && uvs.size())
	{
		LOOP(i, 0, ms->triangles.size())
		{
			LOOP(j, 0, 3)
				ms->triangles[i].uvs[j] = uvs[uvMap[i][j]];
		}
	}

	fclose(fn);

	//printf("load_obj: vertices = %lu, triangles = %lu, uvs = %lu\n", vertices.size(), triangles.size(), uvs.size() );
} // load_obj()

// Optional : Store as OBJ

void write_obj(MeshSimplifier * ms, const char* filename)
{
	FILE *file = fopen(filename, "w");
	int cur_material = -1;
	bool has_uv = (ms->triangles.size() && (ms->triangles[0].attr & TEXCOORD) == TEXCOORD);

	if (!file)
	{
		printf("write_obj: can't write data file \"%s\".\n", filename);
		exit(0);
	}
	if (!ms->mtllib.empty())
	{
		fprintf(file, "mtllib %s\n", ms->mtllib.c_str());
	}
	LOOP(i, 0, ms->vertices.size())
	{
		//fprintf(file, "v %lf %lf %lf\n", vertices[i].p.x,vertices[i].p.y,vertices[i].p.z);
		fprintf(file, "v %g %g %g\n", ms->vertices[i].p.x, ms->vertices[i].p.y, ms->vertices[i].p.z); //more compact: remove trailing zeros
	}
	if (has_uv)
	{
		LOOP(i, 0, ms->triangles.size()) if (!ms->triangles[i].deleted)
		{
			fprintf(file, "vt %g %g\n", ms->triangles[i].uvs[0].x, ms->triangles[i].uvs[0].y);
			fprintf(file, "vt %g %g\n", ms->triangles[i].uvs[1].x, ms->triangles[i].uvs[1].y);
			fprintf(file, "vt %g %g\n", ms->triangles[i].uvs[2].x, ms->triangles[i].uvs[2].y);
		}
	}
	int uv = 1;
	LOOP(i, 0, ms->triangles.size()) if (!ms->triangles[i].deleted)
	{
		if (ms->triangles[i].material != cur_material)
		{
			cur_material = ms->triangles[i].material;
			fprintf(file, "usemtl %s\n", ms->materials[ms->triangles[i].material].c_str());
		}
		if (has_uv)
		{
			fprintf(file, "f %d/%d %d/%d %d/%d\n", ms->triangles[i].v[0] + 1, uv, ms->triangles[i].v[1] + 1, uv + 1, ms->triangles[i].v[2] + 1, uv + 2);
			uv += 3;
		}
		else
		{
			fprintf(file, "f %d %d %d\n", ms->triangles[i].v[0] + 1, ms->triangles[i].v[1] + 1, ms->triangles[i].v[2] + 1);
		}
		//fprintf(file, "f %d// %d// %d//\n", triangles[i].v[0]+1, triangles[i].v[1]+1, triangles[i].v[2]+1); //more compact: remove trailing zeros
	}
	fclose(file);
}

int main()
{
	std::string s = "D:/download/new_cube.obj";
	//std::cin >> s;
	MeshSimplifier * ms = new MeshSimplifier();
	//std::string outs = "D:/download/bunny_sim3.obj";

	load_obj(ms, s.c_str(), false);
	ms->OptimizeVertices();
	ms->SimplifyMeshLossless();
	//ms->SimplifyMesh(1500);
	write_obj(ms, "D:/download/cube_sim2.obj");
	//ms->SimplifyMesh(200);
	//write_obj(ms, "D:/download/bunny_sim200.obj");
}