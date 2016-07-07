#include "RigidBodyRectangle.h"
#include <GL/glut.h>

#include "Eigen/Dense"

#include <iostream>

#include "FluidSolverDefines.h"


RigidBodyRectangle::RigidBodyRectangle(const Vector2d & ConstructPos, Vector2d & size, int mass,
		Matrix2d & rotation) : RigidBody(ConstructPos, mass, rotation),
					m_Size(size)
{
	// calc Ibody
	double x = m_Size[0];
	double y = m_Size[1];
	m_Ibody = m_Mass*(x*x + y*y) / 12.0;

	// calc IbodyInv
	m_IbodyInv = 1/m_Ibody;

	//std::cout << "ibody:" << std::endl;
	//std::cout << m_Ibody << std::endl;

	//std::cout << "ibodyInv:" << std::endl;
	//std::cout << m_IbodyInv << std::endl;
}

RigidBodyRectangle::~RigidBodyRectangle(void)
{

}

void RigidBodyRectangle::draw(int N)
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
	glLineWidth(1);
	glBegin(GL_QUADS);
	glVertex2f(m_Position[0] + bl_rot[0], m_Position[1] + bl_rot[1]);
	glVertex2f(m_Position[0] + br_rot[0], m_Position[1] + br_rot[1]);
	glVertex2f(m_Position[0] + tr_rot[0], m_Position[1] + tr_rot[1]);
	glVertex2f(m_Position[0] + tl_rot[0], m_Position[1] + tl_rot[1]);
	glEnd();
}

void RigidBodyRectangle::drawbb()
{
	std::vector<double> coords = computeAABB();
	glColor3f(1.f, 0.f, 0.f);
	glLineWidth(1);
	glBegin(GL_LINE_STRIP);
	glVertex2f(coords[0], coords[1]);
	glVertex2f(coords[2], coords[1]);
	glVertex2f(coords[2], coords[3]);
	glVertex2f(coords[0], coords[3]);
	glVertex2f(coords[0], coords[1]);
	glEnd();
}

void RigidBodyRectangle::drawbbCells(int N)
{
	std::vector<int> coordsAligned = computeAABBcellAligned(64);
	// convert back to doubles for drawing (divide by 64)
	glColor3f(0.f, 1.f, 0.f);
	glLineWidth(1);
	glBegin(GL_LINE_STRIP);
	glVertex2f(((double)coordsAligned[0]) / N, ((double)coordsAligned[1]) / N);
	glVertex2f(((double)coordsAligned[2]) / N, ((double)coordsAligned[1]) / N);
	glVertex2f(((double)coordsAligned[2]) / N, ((double)coordsAligned[3]) / N);
	glVertex2f(((double)coordsAligned[0]) / N, ((double)coordsAligned[3]) / N);
	glVertex2f(((double)coordsAligned[0]) / N, ((double)coordsAligned[1]) / N);
	glEnd();
}

void RigidBodyRectangle::drawbbCellsOccupied(int N)
{
	for (Vector2i cellIndex : gridIndicesOccupied) {
		// compute the world space coordinate
		Vector2d bl = Vector2d(((double)cellIndex[0]) / N, ((double)cellIndex[1]) / N);
		Vector2d br = bl + Vector2d(1.0 / N, 0.0);
		Vector2d tl = bl + Vector2d(0.0, 1.0 / N);
		Vector2d tr = bl + Vector2d(1.0 / N, 1.0 / N);

		// draw the cells
		glColor3f(0.f, 0.f, 1.f);
		glLineWidth(1);
		glBegin(GL_LINE_STRIP);
		glVertex2f(bl[0], bl[1]);
		glVertex2f(br[0], br[1]);
		glVertex2f(tr[0], tr[1]);
		glVertex2f(tl[0], tl[1]);
		glVertex2f(bl[0], bl[1]);
		glEnd();
	}
}

void RigidBodyRectangle::drawBoundaryCells(int N, int *solid)
{
	for (Vector2i cellIndex : getBoundaryCells(N, solid)) {
		// compute the world space coordinate
		Vector2d bl = Vector2d(((double)cellIndex[0]) / N, ((double)cellIndex[1]) / N);
		Vector2d br = bl + Vector2d(1.0 / N, 0.0);
		Vector2d tl = bl + Vector2d(0.0, 1.0 / N);
		Vector2d tr = bl + Vector2d(1.0 / N, 1.0 / N);

		// draw the cells
		glColor3f(1.f, 0.f, 1.f);
		glLineWidth(1);
		glBegin(GL_LINE_STRIP);
		glVertex2f(bl[0], bl[1]);
		glVertex2f(br[0], br[1]);
		glVertex2f(tr[0], tr[1]);
		glVertex2f(tl[0], tl[1]);
		glVertex2f(bl[0], bl[1]);
		glEnd();
	}
}

void RigidBodyRectangle::drawEdgeNormals()
{
	std::vector<Vector2d> edgeNormals = getEdgeNormals();
	std::vector<std::tuple<Vector2d, Vector2d>> edges = getEdges();
	int i = 0;
	for (auto tuple : edges) {
		// calculate point in the middle of the edge
		Vector2d middle = std::get<0>(tuple) + std::get<1>(tuple)/2;
		// draw from 'middle' to 'middle' + edgeNormal(i)
		Vector2d middlePlusNormal = middle + edgeNormals[i]/10.0;
		glPointSize(10);
		glColor3f(0.f, 1.f, 0.f);
		glLineWidth(1);
		glBegin(GL_LINES);
		glVertex2f(middle[0], middle[1]);
		glVertex2f(middlePlusNormal[0], middlePlusNormal[1]);
		glEnd();
		i++;
	}
}

std::vector<double> RigidBodyRectangle::computeAABB()
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

	coords.push_back(std::min_element(xcoords.cbegin(), xcoords.cend())[0] );
	coords.push_back(std::min_element(ycoords.cbegin(), ycoords.cend())[0] );
	coords.push_back(std::max_element(xcoords.cbegin(), xcoords.cend())[0] );
	coords.push_back(std::max_element(ycoords.cbegin(), ycoords.cend())[0] );

	return coords;
}

std::vector<int> RigidBodyRectangle::computeAABBcellAligned(int N)
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
	//	so: multiply by N and convert to integer.
	cellCoords.push_back(std::floor(x1 * N));
	cellCoords.push_back(std::floor(y1 * N));
	cellCoords.push_back(std::ceil(x2 * N));
	cellCoords.push_back(std::ceil(y2 * N));

	return cellCoords;
}

void RigidBodyRectangle::voxelize(int N)
{
	gridIndicesOccupiedPreviously = gridIndicesOccupied;
	gridIndicesOccupied.clear(); // remove old data of occupied grid cells.

	std::vector<int> cellCoords = computeAABBcellAligned(N);
	// NOTE:
	// cellCoords[0] = bottom left x value of bounding box
	// cellCoords[1] = bottom left y value of bounding box
	// cellCoords[2] = top right x value of bounding box
	// cellCoords[3] = top right y value of bounding box

	// 1) loop over each grid cell
		// 2) determine the 4 coordinates of the grid cell
		// 3) transform the grid cell coordinates to the body space of the square
		// 4) finally determine if the grid cell is inside or outside the square.
		// 5) save the grid cell coordinates that this square occupies in a member variable.
	
	// additionally: create macro for cellcoords <-> worldspace coordinates.


	int x_size = cellCoords[2] - cellCoords[0];
	int y_size = cellCoords[3] - cellCoords[1];
	for (double j = 0.0; j < y_size; j++) {
		for (double i = 0.0; i < x_size; i++) {
			Vector2d bl = Vector2d(((double)cellCoords[0])/N + i/N, ((double)cellCoords[1])/N + j/N);
			Vector2d br = bl + Vector2d(1.0/N, 0.0);
			Vector2d tl = bl + Vector2d(0.0, 1.0/N);
			Vector2d tr = bl + Vector2d(1.0/N, 1.0/N);

			// 3) and 4) are done in checkIfPointInSquare.
			if (checkIfPointInRectangle(bl,N) || checkIfPointInRectangle(br,N) || checkIfPointInRectangle(tl,N) || checkIfPointInRectangle(tr,N)) {
				// save grid cell index (bounding box bottomleft + i and j offsets)
				gridIndicesOccupied.push_back(Vector2i(cellCoords[0] + i, cellCoords[1] + j));
			}
		}
	}
}

std::vector<Vector2i> RigidBodyRectangle::getBoundaryCells(int N, int *solid)
{
	std::vector<Vector2i> bCells;
	gridIndicesCloseToBoundary.clear();
	// bCells should contain all the cells in gridIndicesOccupied that are
	// next to at least one cell with solid==0.

	for (Vector2i &cell : gridIndicesOccupied) {
		if (solid[IX(cell[0] - 1, cell[1])] == 0 || 
			solid[IX(cell[0], cell[1] - 1)] == 0 || 
			solid[IX(cell[0] + 1, cell[1])] == 0 || 
			solid[IX(cell[0], cell[1] + 1)] == 0) {
			
			if (solid[IX(cell[0] - 1, cell[1])] == 0)
				gridIndicesCloseToBoundary.insert({ cell[0] - 1, cell[1] });
			if (solid[IX(cell[0], cell[1] - 1)] == 0)
				gridIndicesCloseToBoundary.insert({ cell[0], cell[1] - 1 });
			if (solid[IX(cell[0] + 1, cell[1])] == 0)
				gridIndicesCloseToBoundary.insert({ cell[0] + 1, cell[1] });
			if (solid[IX(cell[0], cell[1] + 1)] == 0)
				gridIndicesCloseToBoundary.insert({ cell[0], cell[1] + 1 });

			bCells.push_back(cell);
		}
	}

	return bCells;
}

std::set<std::array<int, 2>> RigidBodyRectangle::getSurroundingCells(int N, int *solid)
{
	std::set<std::array<int,2>> sCells;
	// sCells should contain all the cells that are
	// next to at least one cell in gridIndicesCloseToBoundary and have solid==0,
	// but which themselves are NOT part of gridIndicesCloseToBoundary

	// NOTE! Call getBoundaryCells() first to ensure gridIndicesCloseToBoundary contains the correct cells!
	
	for (auto &cell : gridIndicesCloseToBoundary) {
		if (solid[IX(cell[0] - 1, cell[1])] == 0 ||
			solid[IX(cell[0], cell[1] - 1)] == 0 ||
			solid[IX(cell[0] + 1, cell[1])] == 0 ||
			solid[IX(cell[0], cell[1] + 1)] == 0) {

			if (solid[IX(cell[0] - 1, cell[1])] == 0)
				sCells.insert({ cell[0] - 1, cell[1] });
			if (solid[IX(cell[0], cell[1] - 1)] == 0)
				sCells.insert({ cell[0], cell[1] - 1 });
			if (solid[IX(cell[0] + 1, cell[1])] == 0)
				sCells.insert({ cell[0] + 1, cell[1] });
			if (solid[IX(cell[0], cell[1] + 1)] == 0)
				sCells.insert({ cell[0], cell[1] + 1 });
		}
	}

	// remove those cells from gridIndicesCloseToBoundary from sCells
	for (auto &cell : gridIndicesCloseToBoundary) {
		sCells.erase(cell);
	}

	return sCells;
}


bool RigidBodyRectangle::checkIfPointInRectangle(Vector2d &point, int N)
{
	Vector2d pCopy = point;
	// I assume the given point is in world coordinates.
	// we need to transform it to the object space of the square.

	// first translate to a point relative to the origin of the square (subtract square position)
	pCopy -= m_Position;

	// secondly, we rotate by the inverse (transpose) of the rotation of the square.
	pCopy = m_Rotation.transpose() * pCopy;

	// now we can check if the point is inside our outside the (unrotated) square.
	double N0 = N;
	int offset = 2;
	if (pCopy[0] < m_Size[0]/2 - offset/(N0+2) && pCopy[0] > -m_Size[0]/2 + offset/(N0+2))
		if (pCopy[1] < m_Size[1]/2 - offset /(N0+2) && pCopy[1] > -m_Size[1]/2 + offset /(N0+2))
			return true;
	return false;
}


std::vector<Vector2d> RigidBodyRectangle::getVertices()
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

std::vector<std::tuple<Vector2d, Vector2d>> RigidBodyRectangle::getEdges()
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

std::vector<Vector2d> RigidBodyRectangle::getEdgeNormals()
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
