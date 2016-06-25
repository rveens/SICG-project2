#include "RigidBodySquare.h"
#include <GL/glut.h>

#include "Eigen/Dense"

#include <iostream>

#define IX(i,j) ((i)+(N+2)*(j))

RigidBodySquare::RigidBodySquare(const Vector2d & ConstructPos, Vector2d & size, int mass,
		Matrix2d & rotation) : RigidBody(ConstructPos, mass, rotation),
					m_Size(size)
{
	// calc Ibody
	m_Ibody(0, 0) = m_Size[0];
	m_Ibody(1, 1) = m_Size[1];
	m_Ibody *= 1.0/12.0;

	// calc IbodyInv
	m_IbodyInv = m_Ibody.inverse();

	std::cout << "ibody:" << std::endl;
	std::cout << m_Ibody << std::endl;

	std::cout << "ibodyInv:" << std::endl;
	std::cout << m_IbodyInv << std::endl;
}

RigidBodySquare::~RigidBodySquare(void)
{

}

void RigidBodySquare::draw()
{
	/* printf("1: (%f, %f)\n", m_Position[0], m_Position[1]); */
	/* printf("2: (%f, %f)\n", m_Position[0]+m_Size[0], m_Position[1]); */
	/* printf("3: (%f, %f)\n", m_Position[0]+m_Size[0], m_Position[1]+m_Size[1]); */
	/* printf("4: (%f, %f)\n", m_Position[0], m_Position[1]+m_Size[1]); */

	// m_Size[0] means x
	// m_Size[1] means y
	Vector2d bl = Vector2d(-m_Size[0]/2, -m_Size[1]/2);
	Vector2d br = Vector2d(+m_Size[0]/2, -m_Size[1]/2);
	Vector2d tr = Vector2d(+m_Size[0]/2, +m_Size[1]/2);
	Vector2d tl = Vector2d(-m_Size[0]/2, +m_Size[1]/2);

	Vector2d bl_rot = m_Rotation * bl;
	Vector2d br_rot = m_Rotation * br;
	Vector2d tr_rot = m_Rotation * tr;
	Vector2d tl_rot = m_Rotation * tl;

	glColor3f(1.f, 1.f, 1.f);
	glBegin(GL_QUADS);
	glVertex2f(m_Position[0] + bl_rot[0], m_Position[1] + bl_rot[1]);
	glVertex2f(m_Position[0] + br_rot[0], m_Position[1] + br_rot[1]);
	glVertex2f(m_Position[0] + tr_rot[0], m_Position[1] + tr_rot[1]);
	glVertex2f(m_Position[0] + tl_rot[0], m_Position[1] + tl_rot[1]);
	glEnd();

	// draw AABB
	if (m_Drawbb) {
		std::vector<double> coords = computeAABB();
		glColor3f(1.f, 0.f, 0.f);
		glBegin(GL_LINE_STRIP);
		glVertex2f(coords[0], coords[1]);
		glVertex2f(coords[2], coords[1]);
		glVertex2f(coords[2], coords[3]);
		glVertex2f(coords[0], coords[3]);
		glVertex2f(coords[0], coords[1]);
		glEnd();
	}

	// draw AABB cell aligned
	if (m_DrawbbCells) {
		std::vector<int> coordsAligned = computeAABBcellAligned(64);
		// convert back to doubles for drawing (divide by 64)
		glColor3f(0.f, 1.f, 0.f);
		glBegin(GL_LINE_STRIP);
		glVertex2f(((double) coordsAligned[0])/ 64.0, ((double)coordsAligned[1]) / 64.0);
		glVertex2f(((double)coordsAligned[2]) / 64.0, ((double)coordsAligned[1]) / 64.0);
		glVertex2f(((double)coordsAligned[2]) / 64.0, ((double)coordsAligned[3]) / 64.0);
		glVertex2f(((double)coordsAligned[0]) / 64.0, ((double)coordsAligned[3]) / 64.0);
		glVertex2f(((double)coordsAligned[0]) / 64.0, ((double)coordsAligned[1]) / 64.0);
		glEnd();
	}
}

std::vector<double> RigidBodySquare::computeAABB()
{
	Vector2d bl = Vector2d(-m_Size[0]/2, -m_Size[1]/2);
	Vector2d br = Vector2d(+m_Size[0]/2, -m_Size[1]/2);
	Vector2d tr = Vector2d(+m_Size[0]/2, +m_Size[1]/2);
	Vector2d tl = Vector2d(-m_Size[0]/2, +m_Size[1]/2);

	Vector2d bl_rot = m_Rotation * bl;
	Vector2d br_rot = m_Rotation * br;
	Vector2d tr_rot = m_Rotation * tr;
	Vector2d tl_rot = m_Rotation * tl;
	bl_rot += m_Position;
	br_rot += m_Position;
	tr_rot += m_Position;
	tl_rot += m_Position;


	std::vector<double> xcoords({bl_rot[0], br_rot[0], tr_rot[0], tl_rot[0]});
	std::vector<double> ycoords({bl_rot[1], br_rot[1], tr_rot[1], tl_rot[1]});

	std::max_element(ycoords.cbegin(), ycoords.cend());

	std::vector<double> coords;

	coords.push_back( std::min_element(xcoords.cbegin(), xcoords.cend())[0] );
	coords.push_back(std::min_element(ycoords.cbegin(), ycoords.cend())[0] );
	coords.push_back(std::max_element(xcoords.cbegin(), xcoords.cend())[0] );
	coords.push_back(std::max_element(ycoords.cbegin(), ycoords.cend())[0] );

	return coords;
}

std::vector<int> RigidBodySquare::computeAABBcellAligned(int N)
{
	std::vector<double> coords;		// contains positions in 2D space
	std::vector<int> cellCoords;	// contains fluid-grid indexes

	// first compute the bounding box coordinates just like in the computeAABB
	// function.
	coords = computeAABB();
	double x1 = coords[0];
	double y1 = coords[1];
	double x2 = coords[2];
	double y2 = coords[3];

	// then adjust them to closest position of the grid indexes
	//
	// idea: we have N grid cells, and coordinates ranging from 0.0-1.0.
	//	so: multiply by 64 and convert to integer.
	cellCoords.push_back((int) (x1 * N));
	cellCoords.push_back((int) (y1 * N));
	cellCoords.push_back((int) (x2 * N));
	cellCoords.push_back((int) (y2 * N));

	return cellCoords;
}


std::vector<Vector2d> RigidBodySquare::getVertices()
{
	Vector2d bl(-m_Size[0]/2, -m_Size[1]/2);
	Vector2d br(+m_Size[0]/2, -m_Size[1]/2);
	Vector2d tr(+m_Size[0]/2, +m_Size[1]/2);
	Vector2d tl(-m_Size[0]/2, +m_Size[1]/2);

	Vector2d bl_rot = m_Rotation * bl;
	Vector2d br_rot = m_Rotation * br;
	Vector2d tr_rot = m_Rotation * tr;
	Vector2d tl_rot = m_Rotation * tl;
	bl_rot += m_Position;
	br_rot += m_Position;
	tr_rot += m_Position;
	tl_rot += m_Position;

	return std::vector<Vector2d>({bl_rot, br_rot, tr_rot, tl_rot});
}

std::vector<std::tuple<Vector2d, Vector2d>> RigidBodySquare::getEdges()
{
	auto vertices = getVertices();
	Vector2d edgeblbr = vertices[1] - vertices[0];
	Vector2d edgebrtr = vertices[2] - vertices[1];
	Vector2d edgetrtl = vertices[3] - vertices[2];
	Vector2d edgetlbl = vertices[0] - vertices[3];
	
	std::vector<std::tuple<Vector2d, Vector2d>> edges;
	edges.push_back(std::make_tuple(vertices[0], edgeblbr));
	edges.push_back(std::make_tuple(vertices[1], edgebrtr));
	edges.push_back(std::make_tuple(vertices[2], edgetrtl));
	edges.push_back(std::make_tuple(vertices[3], edgetlbl));

	return edges;
}

std::vector<Vector2d> RigidBodySquare::getEdgeNormals()
{
	auto vert = getVertices();

	std::vector<Vector2d> edgeNormals;

	double dxblbr = vert[1][0] - vert[0][0];
	double dyblbr = vert[1][1] - vert[0][1];
	Vector2d eNorm0(dyblbr, -dxblbr);
	eNorm0.normalize();
	edgeNormals.push_back(eNorm0);
	
	double dxbrtr = vert[2][0] - vert[1][0];
	double dybrtr = vert[2][1] - vert[1][1];
	Vector2d eNorm1(dybrtr, -dxbrtr);
	eNorm1.normalize();
	edgeNormals.push_back(eNorm1);

	double dxtrtl = vert[3][0] - vert[2][0];
	double dytrtl = vert[3][1] - vert[2][1];
	Vector2d eNorm2(dytrtl, -dxtrtl); 
	eNorm2.normalize();
	edgeNormals.push_back(eNorm2);

	double dxtlbl = vert[0][0] - vert[3][0];
	double dytlbl = vert[0][1] - vert[3][1];
	Vector2d eNorm3(dytlbl, -dxtlbl);
	eNorm3.normalize();
	edgeNormals.push_back(eNorm3);

	return edgeNormals;
}
