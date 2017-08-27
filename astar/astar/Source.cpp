// A C++ Program to implement A* Search Algorithm
#include <bits.h>
#include<stdc++.h>
#include<stdio.h>
#include<conio.h>
#include<iostream>
#include<opencv2\highgui\highgui.hpp>
#include<opencv2\opencv.hpp>
#include<opencv2\core\core.hpp>
#include <windows.h>
#include <string.h>
#include <cstring>
#include <atlstr.h>
#include <math.h>

#define ROW 1000
#define COL 1000

using namespace std;
using namespace cv;

int x[1000];
int y[1000];
int xf[1000], yf[1000];
Mat skel;
Mat IMG;
int mat[1000][1000] = { 0 };
int startx = -1, starty = -1, endx = -1, endy = -1, c = 0;
int k = 0;
int jc = 0;
int pathc = 0;
int changex;
int changey;
//Point junctions[100];
int junctionCounter;
//int path[1000][1000] = { 0 };
//Point path[1000];
int pathx[1000], pathy[1000];
char orientation[1000] = {NULL};
int orientationCounter;
char turns[100] = { NULL };
int turnsCounter;
bool closedList[ROW][COL];
int closehandel=0;
CString PortSpecifier = "COM5";
struct Pt
{
	int x;
	int y;
};
Pt junctions[100];
Pt path[1000];

HANDLE hPort = CreateFile(PortSpecifier,GENERIC_WRITE,0,NULL,OPEN_EXISTING,0,NULL);

void findPath();
//void getOrientation();
void getOrientation2();
//void getTurns();
void getTurns2();
void findJunctions();

void thinningIteration(cv::Mat& im, int iter)
{
	cv::Mat marker = cv::Mat::zeros(im.size(), CV_8UC1);

	for (int i = 1; i < im.rows - 1; i++)
	{
		for (int j = 1; j < im.cols - 1; j++)
		{
			uchar p2 = im.at<uchar>(i - 1, j);
			uchar p3 = im.at<uchar>(i - 1, j + 1);
			uchar p4 = im.at<uchar>(i, j + 1);
			uchar p5 = im.at<uchar>(i + 1, j + 1);
			uchar p6 = im.at<uchar>(i + 1, j);
			uchar p7 = im.at<uchar>(i + 1, j - 1);
			uchar p8 = im.at<uchar>(i, j - 1);
			uchar p9 = im.at<uchar>(i - 1, j - 1);

			int A = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
				(p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
				(p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
				(p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
			int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
			int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
			int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

			if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
				marker.at<uchar>(i, j) = 1;
		}
	}

	im &= ~marker;
}

/**
* Function for thinning the given binary image
*
* @param  im  Binary image with range = 0-255
*/
void thinning(cv::Mat& im)
{
	im /= 255;

	cv::Mat prev = cv::Mat::zeros(im.size(), CV_8UC1);
	cv::Mat diff;

	do {
		thinningIteration(im, 0);
		thinningIteration(im, 1);
		cv::absdiff(im, prev, diff);
		im.copyTo(prev);
	} while (cv::countNonZero(diff) > 0);

	im *= 255;
}

//COORIDNATES FUNCTION--------------------------------------------------------------------------------------------
void mouseEvent(int event, int x, int y, int flags, void* param)
{
	IplImage* img = (IplImage*)param;
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		printf("%d,%d\n", x, y);
		if (c == 0)
		{
			startx = x;
			starty = y;
			printf("Starting point is : %d %d\n", startx, starty);
		}
		if (c == 1)
		{
			endx = x;
			endy = y;
			printf("Ending point is : %d %d\n", endx, endy);
			findPath();
		}
		c++;
	}
}
//Serial-Port DATA------------------------------------------------------------------------------------------------------
bool WriteComPort(CString PortSpecifier, CString data)
{
	DCB dcb;
	DWORD byteswritten;
/*	HANDLE hPort = CreateFile(

		PortSpecifier,
		GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		0,
		NULL
	);*/
	if (!GetCommState(hPort, &dcb))
		return false;
	dcb.BaudRate = CBR_19200; //9600 Baud
	dcb.ByteSize = 8; //8 data bits
	dcb.Parity = NOPARITY; //no parity
	dcb.StopBits = ONESTOPBIT; //1 stop
	if (!SetCommState(hPort, &dcb))
		return false;
	bool retVal = WriteFile(hPort, data, 1, &byteswritten, NULL);
//	CloseHandle(hPort); 
	return retVal;
}

// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type
typedef pair<double, pair<int, int>> pPair;

// A structure to hold the neccesary parameters
struct cell
{
	// Row and Column index of its parent
	// Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	int parent_i, parent_j;
	// f = g + h
	double f, g, h;
};
cell cellDetails[ROW][COL];

// A Utility Function to check whether given cell (row, col)
// is a valid cell or not.
bool isValid(int row, int col)
{
	// Returns true if row number and column number
	// is in range
	return (row >= 0) && (row < ROW) &&
		(col >= 0) && (col < COL);
}

// A Utility Function to check whether the given cell is
// blocked or not
bool isUnBlocked(int grid[][COL], int row, int col)
{
	// Returns true if the cell is not blocked else false
	if (grid[row][col] == 1)
		return (true);
	else
		return (false);
}

// A Utility Function to check whether destination cell has
// been reached or not
bool isDestination(int row, int col, Pair dest)
{
	if (row == dest.first && col == dest.second)
		return (true);
	else
		return (false);
}

// A Utility Function to calculate the 'h' heuristics.
double calculateHValue(int row, int col, Pair dest)
{
	// Return using the distance formula
	return ((double)sqrt((row - dest.first)*(row - dest.first)
		+ (col - dest.second)*(col - dest.second)));
}

// A Utility Function to trace the path from the source
// to destination
void tracePath(cell cellDetails[][COL], Pair dest)
{
	printf("\nThe Path is ");
	int row = dest.first;
	int col = dest.second;

	stack<Pair> Path;

	while (!(cellDetails[row][col].parent_i == row
		&& cellDetails[row][col].parent_j == col))
	{
		Path.push(make_pair(row, col));
		int temp_row = cellDetails[row][col].parent_i;
		int temp_col = cellDetails[row][col].parent_j;
		row = temp_row;
		col = temp_col;
	}

	Path.push(make_pair(row, col));
	while (!Path.empty())
	{
		pair<int, int> p = Path.top();
		Path.pop();
		//path[p.first][p.second] = 1;
		path[pathc].x = p.second;
		path[pathc].y = p.first;
		pathx[pathc] = p.second;
		pathy[pathc] = p.first;
		pathc++;
		printf("-> (%d,%d)\n ", p.first, p.second);
		IMG.at<uchar>(p.first, p.second) = 100;
	}

	return;
}

// A Function to find the shortest path between
// a given source cell to a destination cell according
// to A* Search Algorithm
void aStarSearch(int grid[][COL], Pair src, Pair dest)
{
	// If the source is out of range
	if (isValid(src.first, src.second) == false)
	{
		printf("Source is invalid\n");
		return;
	}

	// If the destination is out of range
	if (isValid(dest.first, dest.second) == false)
	{
		printf("Destination is invalid\n");
		return;
	}

	// Either the source or the destination is blocked
	if (isUnBlocked(grid, src.first, src.second) == false ||
		isUnBlocked(grid, dest.first, dest.second) == false)
	{
		printf("Source or the destination is blocked\n");
		return;
	}

	// If the destination cell is the same as source cell
	if (isDestination(src.first, src.second, dest) == true)
	{
		printf("We are already at the destination\n");
		return;
	}

	// Create a closed list and initialise it to false which means
	// that no cell has been included yet
	// This closed list is implemented as a boolean 2D array
	
	//bool closedList[ROW][COL];
	memset(closedList, false, sizeof(closedList));

	// Declare a 2D array of structure to hold the details
	//of that cell
	//cell cellDetails[ROW][COL];

	int i, j;

	for (i = 0; i<ROW; i++)
	{
		for (j = 0; j<COL; j++)
		{
			cellDetails[i][j].f = FLT_MAX;
			cellDetails[i][j].g = FLT_MAX;
			cellDetails[i][j].h = FLT_MAX;
			cellDetails[i][j].parent_i = -1;
			cellDetails[i][j].parent_j = -1;
		}
	}

	// Initialising the parameters of the starting node
	i = src.first, j = src.second;
	cellDetails[i][j].f = 0.0;
	cellDetails[i][j].g = 0.0;
	cellDetails[i][j].h = 0.0;
	cellDetails[i][j].parent_i = i;
	cellDetails[i][j].parent_j = j;

	/*
	Create an open list having information as-
	<f, <i, j>>
	where f = g + h,
	and i, j are the row and column index of that cell
	Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	This open list is implenented as a set of pair of pair.*/
	set<pPair> openList;

	// Put the starting cell on the open list and set its
	// 'f' as 0
	openList.insert(make_pair(0.0, make_pair(i, j)));

	// We set this boolean value as false as initially
	// the destination is not reached.
	bool foundDest = false;

	while (!openList.empty())
	{
		pPair p = *openList.begin();
		junctionCounter = 0;
		// Remove this vertex from the open list
		openList.erase(openList.begin());

		// Add this vertex to the open list
		i = p.second.first;
		j = p.second.second;
		closedList[i][j] = true;

		/*
		Generating all the 8 successor of this cell

		N.W   N   N.E
		\   |   /
		\  |  /
		W----Cell----E
		/ | \
		/   |  \
		S.W    S   S.E

		Cell-->Popped Cell (i, j)
		N -->  North       (i-1, j)
		S -->  South       (i+1, j)
		E -->  East        (i, j+1)
		W -->  West           (i, j-1)
		N.E--> North-East  (i-1, j+1)
		N.W--> North-West  (i-1, j-1)
		S.E--> South-East  (i+1, j+1)
		S.W--> South-West  (i+1, j-1)*/

		// To store the 'g', 'h' and 'f' of the 8 successors
		double gNew, hNew, fNew;

		//----------- 1st Successor (North) ------------

		// Only process this cell if this is a valid one
		if (isValid(i - 1, j) == true)
		{
			
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i - 1, j, dest) == true)
			{
				// Set the Parent of the destination cell
				cellDetails[i - 1][j].parent_i = i;
				cellDetails[i - 1][j].parent_j = j;
				printf("The destination cell is found\n");
				tracePath(cellDetails, dest);
				foundDest = true;
				return;
			}
			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (closedList[i - 1][j] == false &&
				isUnBlocked(grid, i - 1, j) == true)
			{
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i - 1, j, dest);
				fNew = gNew + hNew;
				junctionCounter++;
				// If it isn’t on the open list, add it to
				// the open list. Make the current square
				// the parent of this square. Record the
				// f, g, and h costs of the square cell
				//                OR
				// If it is on the open list already, check
				// to see if this path to that square is better,
				// using 'f' cost as the measure.
				if (cellDetails[i - 1][j].f == FLT_MAX ||
					cellDetails[i - 1][j].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_pair(i - 1, j)));

					// Update the details of this cell
					cellDetails[i - 1][j].f = fNew;
					cellDetails[i - 1][j].g = gNew;
					cellDetails[i - 1][j].h = hNew;
					cellDetails[i - 1][j].parent_i = i;
					cellDetails[i - 1][j].parent_j = j;
				}
			}
		}

		//----------- 2nd Successor (South) ------------

		// Only process this cell if this is a valid one
		if (isValid(i + 1, j) == true)
		{
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i + 1, j, dest) == true)
			{
				// Set the Parent of the destination cell
				cellDetails[i + 1][j].parent_i = i;
				cellDetails[i + 1][j].parent_j = j;
				printf("The destination cell is found\n");
				tracePath(cellDetails, dest);
				foundDest = true;
				return;
			}
			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (closedList[i + 1][j] == false &&
				isUnBlocked(grid, i + 1, j) == true)
			{
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i + 1, j, dest);
				fNew = gNew + hNew;
				junctionCounter++;
				// If it isn’t on the open list, add it to
				// the open list. Make the current square
				// the parent of this square. Record the
				// f, g, and h costs of the square cell
				//                OR
				// If it is on the open list already, check
				// to see if this path to that square is better,
				// using 'f' cost as the measure.
				if (cellDetails[i + 1][j].f == FLT_MAX ||
					cellDetails[i + 1][j].f > fNew)
				{
					openList.insert(make_pair(fNew, make_pair(i + 1, j)));
					// Update the details of this cell
					cellDetails[i + 1][j].f = fNew;
					cellDetails[i + 1][j].g = gNew;
					cellDetails[i + 1][j].h = hNew;
					cellDetails[i + 1][j].parent_i = i;
					cellDetails[i + 1][j].parent_j = j;
				}
			}
		}

		//----------- 3rd Successor (East) ------------

		// Only process this cell if this is a valid one
		if (isValid(i, j + 1) == true)
		{
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i, j + 1, dest) == true)
			{
				// Set the Parent of the destination cell
				cellDetails[i][j + 1].parent_i = i;
				cellDetails[i][j + 1].parent_j = j;
				printf("The destination cell is found\n");
				tracePath(cellDetails, dest);
				foundDest = true;
				return;
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (closedList[i][j + 1] == false &&
				isUnBlocked(grid, i, j + 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i, j + 1, dest);
				fNew = gNew + hNew;
				junctionCounter++;
				// If it isn’t on the open list, add it to
				// the open list. Make the current square
				// the parent of this square. Record the
				// f, g, and h costs of the square cell
				//                OR
				// If it is on the open list already, check
				// to see if this path to that square is better,
				// using 'f' cost as the measure.
				if (cellDetails[i][j + 1].f == FLT_MAX ||
					cellDetails[i][j + 1].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_pair(i, j + 1)));

					// Update the details of this cell
					cellDetails[i][j + 1].f = fNew;
					cellDetails[i][j + 1].g = gNew;
					cellDetails[i][j + 1].h = hNew;
					cellDetails[i][j + 1].parent_i = i;
					cellDetails[i][j + 1].parent_j = j;
				}
			}
		}

		//----------- 4th Successor (West) ------------

		// Only process this cell if this is a valid one
		if (isValid(i, j - 1) == true)
		{
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i, j - 1, dest) == true)
			{
				// Set the Parent of the destination cell
				cellDetails[i][j - 1].parent_i = i;
				cellDetails[i][j - 1].parent_j = j;
				printf("The destination cell is found\n");
				tracePath(cellDetails, dest);
				foundDest = true;
				return;
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (closedList[i][j - 1] == false &&
				isUnBlocked(grid, i, j - 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i, j - 1, dest);
				fNew = gNew + hNew;
				junctionCounter++;
				// If it isn’t on the open list, add it to
				// the open list. Make the current square
				// the parent of this square. Record the
				// f, g, and h costs of the square cell
				//                OR
				// If it is on the open list already, check
				// to see if this path to that square is better,
				// using 'f' cost as the measure.
				if (cellDetails[i][j - 1].f == FLT_MAX ||
					cellDetails[i][j - 1].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_pair(i, j - 1)));

					// Update the details of this cell
					cellDetails[i][j - 1].f = fNew;
					cellDetails[i][j - 1].g = gNew;
					cellDetails[i][j - 1].h = hNew;
					cellDetails[i][j - 1].parent_i = i;
					cellDetails[i][j - 1].parent_j = j;
				}
			}
		}

		//----------- 5th Successor (North-East) ------------

		// Only process this cell if this is a valid one
		if (isValid(i - 1, j + 1) == true)
		{
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i - 1, j + 1, dest) == true)
			{
				// Set the Parent of the destination cell
				cellDetails[i - 1][j + 1].parent_i = i;
				cellDetails[i - 1][j + 1].parent_j = j;
				printf("The destination cell is found\n");
				tracePath(cellDetails, dest);
				foundDest = true;
				return;
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (closedList[i - 1][j + 1] == false &&
				isUnBlocked(grid, i - 1, j + 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i - 1, j + 1, dest);
				fNew = gNew + hNew;
				junctionCounter++;
				// If it isn’t on the open list, add it to
				// the open list. Make the current square
				// the parent of this square. Record the
				// f, g, and h costs of the square cell
				//                OR
				// If it is on the open list already, check
				// to see if this path to that square is better,
				// using 'f' cost as the measure.
				if (cellDetails[i - 1][j + 1].f == FLT_MAX ||
					cellDetails[i - 1][j + 1].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_pair(i - 1, j + 1)));

					// Update the details of this cell
					cellDetails[i - 1][j + 1].f = fNew;
					cellDetails[i - 1][j + 1].g = gNew;
					cellDetails[i - 1][j + 1].h = hNew;
					cellDetails[i - 1][j + 1].parent_i = i;
					cellDetails[i - 1][j + 1].parent_j = j;
				}
			}
		}

		//----------- 6th Successor (North-West) ------------

		// Only process this cell if this is a valid one
		if (isValid(i - 1, j - 1) == true)
		{
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i - 1, j - 1, dest) == true)
			{
				// Set the Parent of the destination cell
				cellDetails[i - 1][j - 1].parent_i = i;
				cellDetails[i - 1][j - 1].parent_j = j;
				printf("The destination cell is found\n");
				tracePath(cellDetails, dest);
				foundDest = true;
				return;
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (closedList[i - 1][j - 1] == false &&
				isUnBlocked(grid, i - 1, j - 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i - 1, j - 1, dest);
				fNew = gNew + hNew;
				junctionCounter++;
				// If it isn’t on the open list, add it to
				// the open list. Make the current square
				// the parent of this square. Record the
				// f, g, and h costs of the square cell
				//                OR
				// If it is on the open list already, check
				// to see if this path to that square is better,
				// using 'f' cost as the measure.
				if (cellDetails[i - 1][j - 1].f == FLT_MAX ||
					cellDetails[i - 1][j - 1].f > fNew)
				{
					openList.insert(make_pair(fNew, make_pair(i - 1, j - 1)));
					// Update the details of this cell
					cellDetails[i - 1][j - 1].f = fNew;
					cellDetails[i - 1][j - 1].g = gNew;
					cellDetails[i - 1][j - 1].h = hNew;
					cellDetails[i - 1][j - 1].parent_i = i;
					cellDetails[i - 1][j - 1].parent_j = j;
				}
			}
		}

		//----------- 7th Successor (South-East) ------------

		// Only process this cell if this is a valid one
		if (isValid(i + 1, j + 1) == true)
		{
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i + 1, j + 1, dest) == true)
			{
				// Set the Parent of the destination cell
				cellDetails[i + 1][j + 1].parent_i = i;
				cellDetails[i + 1][j + 1].parent_j = j;
				printf("The destination cell is found\n");
				tracePath(cellDetails, dest);
				foundDest = true;
				return;
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (closedList[i + 1][j + 1] == false &&
				isUnBlocked(grid, i + 1, j + 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i + 1, j + 1, dest);
				fNew = gNew + hNew;
				junctionCounter++;
				// If it isn’t on the open list, add it to
				// the open list. Make the current square
				// the parent of this square. Record the
				// f, g, and h costs of the square cell
				//                OR
				// If it is on the open list already, check
				// to see if this path to that square is better,
				// using 'f' cost as the measure.
				if (cellDetails[i + 1][j + 1].f == FLT_MAX ||
					cellDetails[i + 1][j + 1].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_pair(i + 1, j + 1)));

					// Update the details of this cell
					cellDetails[i + 1][j + 1].f = fNew;
					cellDetails[i + 1][j + 1].g = gNew;
					cellDetails[i + 1][j + 1].h = hNew;
					cellDetails[i + 1][j + 1].parent_i = i;
					cellDetails[i + 1][j + 1].parent_j = j;
				}
			}
		}

		//----------- 8th Successor (South-West) ------------

		// Only process this cell if this is a valid one
		if (isValid(i + 1, j - 1) == true)
		{
			// If the destination cell is the same as the
			// current successor
			if (isDestination(i + 1, j - 1, dest) == true)
			{
				// Set the Parent of the destination cell
				cellDetails[i + 1][j - 1].parent_i = i;
				cellDetails[i + 1][j - 1].parent_j = j;
				printf("The destination cell is found\n");
				tracePath(cellDetails, dest);
				foundDest = true;
				return;
			}

			// If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
			else if (closedList[i + 1][j - 1] == false &&
				isUnBlocked(grid, i + 1, j - 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i + 1, j - 1, dest);
				fNew = gNew + hNew;
				junctionCounter++;
				// If it isn’t on the open list, add it to
				// the open list. Make the current square
				// the parent of this square. Record the
				// f, g, and h costs of the square cell
				//                OR
				// If it is on the open list already, check
				// to see if this path to that square is better,
				// using 'f' cost as the measure.
				if (cellDetails[i + 1][j - 1].f == FLT_MAX ||
					cellDetails[i + 1][j - 1].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_pair(i + 1, j - 1)));

					// Update the details of this cell
					cellDetails[i + 1][j - 1].f = fNew;
					cellDetails[i + 1][j - 1].g = gNew;
					cellDetails[i + 1][j - 1].h = hNew;
					cellDetails[i + 1][j - 1].parent_i = i;
					cellDetails[i + 1][j - 1].parent_j = j;
				}
			}
		}

	/*	if (junctionCounter >= 2)
		{
			junctions[jc].x = j;
			junctions[jc].y = i;
			jc++;
		}*/
		
	}

	// When the destination cell is not found and the open
	// list is empty, then we conclude that we failed to
	// reach the destiantion cell. This may happen when the
	// there is no way to destination cell (due to blockages)
	if (foundDest == false)
		printf("Failed to find the Destination Cell\n");
	return;
}

void main()
{
	Mat img = imread("DTrack1.jpg", 0);
	resize(img, img, Size(img.rows / 10, img.cols / 10));      //RESIZING
	bitwise_not(img, img);                                     //INVERTING IMG
	namedWindow("GSI", WINDOW_AUTOSIZE);
	imshow("GSI", img);
	

	//-----------------------------------------------------------------------------------------------------
	//Modified Conversion to binary
	for (int i = 0; i < img.rows; i++)          //ACCESSING PIXELS
	{
		for (int j = 0; j < img.cols; j++)
		{
			if (img.at<uchar>(i, j) < 150) //150 old value
			{
				img.at<uchar>(i, j) = 0;
			}
			if (img.at<uchar>(i, j) >= 150)
			{
				img.at<uchar>(i, j) = 255;
			}
		}
	}
	imshow("Mod bin", img);//new
	GaussianBlur(img, img, Size(9, 9), 50.5); //new
	imshow("blur", img);//new
	//-------------------------------------------------------------------------------------------------------------
	//SKELETONIZATION
	Mat skel(img.size(), CV_8UC1, Scalar(0));
	Mat temp(img.size(), CV_8UC1);
	Mat eroded;
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	Mat element2 = getStructuringElement(MORPH_RECT, Size(5, 5), Point(0, 0));
	Mat element3 = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
	erode(img, img, element);
	//namedWindow("Skeleton", WINDOW_AUTOSIZE);
	bool done;
	do
	{
		cv::erode(img, eroded, element);
		cv::dilate(eroded, temp, element); // temp = open(img)
		cv::subtract(img, temp, temp);
		cv::bitwise_or(skel, temp, skel);
		eroded.copyTo(img);
		done = (cv::countNonZero(img) == 0);
	} while (!done);
	
	dilate(skel, skel, element2);
	erode(skel, skel, element3);
	/*dilate(skel, skel, Mat(), Point(0, 0), 2, 1, 1);
	erode(skel, skel, Mat(), Point(0, 0), 1, 1, 1);*/
	cv::imshow("Skeleton", skel);
	//--------------------------------------------------------------------------------------------------------------
	int k = 0;
	for (int i = 0; i < skel.rows; i++)          //ACCESSING PIXELS FOR GETTING CORDS OF ENTIRE TRACK
	{

		for (int j = 0; j < skel.cols; j++)
		{
			if (skel.at<uchar>(i, j) > 0)
			{
				if (skel.at<uchar>(i, j) > 20)
				{
					skel.at<uchar>(i, j) = 255;

				/*	x[k] = j; //x-cords or columns
					y[k] = i; //y-cords	or rows
					mat[i][j] = 1;
					printf("%d , %d ,%d\n", x[k], y[k], k);
					k++;*/
				}
			}
		}
	}
	imshow("skel2", skel);

/*	cv::Mat bw;
	cv::cvtColor(skel, bw, CV_BGR2GRAY);
	cv::threshold(bw, bw, 10, 255, CV_THRESH_BINARY);*/

	//thinning(bw);
	thinning(skel);
	for (int i = 0; i < skel.rows; i++)          //ACCESSING PIXELS FOR GETTING CORDS OF ENTIRE TRACK
	{

		for (int j = 0; j < skel.cols; j++)
		{
			if (skel.at<uchar>(i, j) > 0)
			{
				if (skel.at<uchar>(i, j) == 255)
				{
					x[k] = j; //x-cords or columns
					y[k] = i; //y-cords	or rows
					mat[i][j] = 1;
					printf("%d , %d ,%d\n", x[k], y[k], k);
					k++;
				}
			}
		}
	}
//	cv::imshow("src", skel);
//	cv::imshow("dst", skel);
//	cv::imshow("dst", bw);
//	cv::waitKey(0);
	printf("\n\n");

	//--------------------------------------------------------------------------------------------------------------


	//-------------------------------------------------------------------------------------------------------------
	//For getting matrix in ot ints
	IMG = skel;
	for (int i = 0; i < 1000; i++)
	{
		for (int j = 0; j < 1000; j++)
		{
			if (mat[i][j] == 1)
			{
				printf("mat[%d][%d]=%d\n", i, j, mat[i][j]);
			}

		}
	}

	//COORDINATES
	IplImage copy = skel;  // Read image from file
	IplImage *img1 = &copy;
	//IplImage *img = cvLoadImage("track3.jpg", 1);
	cvNamedWindow("My Window", 1);
	cvSetMouseCallback("My Window", mouseEvent, &img1);   	//set the callback function for any mouse event
	cvShowImage("My Window", img1);      //show the image

	orientationCounter = 0;
	
	cvWaitKey(0);
}

void findPath()
{
	// Source is the left-most bottom-most corner
	Pair src = make_pair(starty, startx);

	// Destination is the left-most top-most corner
	Pair dest = make_pair(endy, endx);

	aStarSearch(mat, src, dest);
	findJunctions();
	for (int i = 0; i < 1000; i++)
	{
		if (pathx[i] != 0||pathy[i] !=0)
		{
			printf("(%d,%d)  ::  ", path[i].x, path[i].y);
			printf("(%d,%d)\n", pathx[i], pathy[i]);
		}	
	}

	for (int i = 0; i < 100; i++)
	{
		if (junctions[i].x != 0 && junctions[i].y != 0)
		{
			printf("junction[%d]=(%d,%d)\n", i, junctions[i].x, junctions[i].y);
		}
	}
	namedWindow("PATH", WINDOW_AUTOSIZE);
	imshow("PATH", IMG);
	getOrientation2();
}

/*void getOrientation()
{
	
	for (int i = 0, orientationCounter = 0; i < 1000; i++)
	{
		for (int j = 0; j < 100; j++)
		{
			if ((path[i].x == junctions[j].x && path[i].y == junctions[j].y) || i == 0)
			{
				if (path[i + 1].x == path[i].x + 1 && path[i + 1].y == path[i].y)
				{
					//right
					orientation[orientationCounter] = 'R';
					orientationCounter++;
				}
				if (path[i + 1].x == path[i].x - 1 && path[i + 1].y == path[i].y)
				{
					//left
					orientation[orientationCounter] = 'L';
					orientationCounter++;
				}
				if (path[i + 1].x == path[i].x  && path[i + 1].y == path[i].y + 1)
				{
					//down
					orientation[orientationCounter] = 'D';
					orientationCounter++;
				}
				if (path[i + 1].x == path[i].x && path[i + 1].y == path[i].y - 1)
				{
					//up
					orientation[orientationCounter] = 'U';
					orientationCounter++;
				}
				if (i == 0)
				{
					break;
				}
			}

		}
		if (i != 0)
		{
			if ((path[i + 1].x == path[i].x + 1 && ((path[i + 1].y == path[i].y + 1) || (path[i + 1].y == path[i].y - 1))) && (path[i - 1].x == path[i].x))
			{
				//right
				//orientation[orientationCounter] = 'r';
				orientation[orientationCounter] = 'R';
				printf("%d,%d\n", path[i].x, path[i].y);
				orientationCounter++;
			}
			if (path[i + 1].x == path[i].x - 1 && ((path[i + 1].y == path[i].y + 1) || (path[i + 1].y == path[i].y - 1)) && (path[i - 1].x == path[i].x))
			{
				//left
				//orientation[orientationCounter] = 'l';
				orientation[orientationCounter] = 'L';
				printf("%d,%d\n", path[i].x, path[i].y);
				orientationCounter++;
			}
			if ((((path[i + 1].x == path[i].x + 1) || (path[i + 1].x == path[i].x - 1)) && path[i + 1].y == path[i].y + 1) && (path[i - 1].y == path[i].y))
			{
				//down
				//orientation[orientationCounter] = 'd';
				orientation[orientationCounter] = 'D';
				printf("%d,%d\n", path[i].x, path[i].y);
				orientationCounter++;
			}
			if ((((path[i + 1].x == path[i].x + 1) || (path[i + 1].x == path[i].x - 1)) && path[i + 1].y == path[i].y - 1) && (path[i - 1].y == path[i].y))
			{
				//up
				//orientation[orientationCounter] = 'u';
				orientation[orientationCounter] = 'U';
				printf("%d,%d\n", path[i].x, path[i].y);
				orientationCounter++;
			}
		}
	}
	orientationCounter = 0;
	while (orientation[orientationCounter] != NULL)
	{
		printf("%c,", orientation[orientationCounter]);
		orientationCounter++;
	}
	getTurns();
}*/

void getOrientation2()
{

	for (int i = 0, orientationCounter = 0; i < 1000; i++)
	{
		for (int j = 0; j < 100; j++)
		{
			if ((pathx[i] == junctions[j].x && pathy[i] == junctions[j].y) || i == 0)
			{
				if (pathx[i + 1] == pathx[i] + 1 && pathy[i + 1] == pathy[i])
				{
					//right
					orientation[orientationCounter] = 'r';
					orientationCounter++;
				}
				if (pathx[i + 1] == pathx[i] - 1 && pathy[i + 1] == pathy[i])
				{
					//left
					orientation[orientationCounter] = 'l';
					orientationCounter++;
				}
				if (pathx[i + 1] == pathx[i] && pathy[i + 1] == pathy[i] + 1)
				{
					//down
					orientation[orientationCounter] = 'd';
					orientationCounter++;
				}
				if (pathx[i + 1] == pathx[i] && pathy[i + 1] == pathy[i] - 1)
				{
					//up
					orientation[orientationCounter] = 'u';
					orientationCounter++;
				}
				if (i == 0)
				{
					break;
				}
			}
		}


			if (i + 5 < 1000 && (pathx[i + 5] != 0 && pathy[i + 5] != 0))
			{
				changex = pathx[i + 5] - pathx[i];
				changey = pathy[i + 5] - pathy[i];
				printf("pathx[%d]=%d - pathx[%d]=%d & pathy[%d]=%d - pathy[%d]=%d>> cx=%d , cy=%d \n", i + 5, pathx[i + 5], i, pathx[i], i + 5, pathy[i + 5], i, pathy[i], changex, changey);

				if (changex >= 5 && abs(changey) >= 0)
				{
					//right
					//orientation[orientationCounter] = 'r';
					orientation[orientationCounter] = 'R';
				//	printf("%d,%d\n", pathx[i], pathy[i]);
					orientationCounter++;
				}
				if (changex <= -5 && abs(changey) >= 0)
				{
					//left
					//orientation[orientationCounter] = 'l';
					orientation[orientationCounter] = 'L';
				//	printf("%d,%d\n", pathx[i], pathy[i]);
					orientationCounter++;
				}
				if (changey >= 5 && abs(changex) >= 0)
				{
					//down
					//orientation[orientationCounter] = 'd';
					orientation[orientationCounter] = 'D';
				//	printf("%d,%d\n", pathx[i], pathy[i]);
					orientationCounter++;
				}
				if (changey <= -5 && abs(changex) >= 0)
				{
					//up
					//orientation[orientationCounter] = 'u';
					orientation[orientationCounter] = 'U';
				//	printf("%d,%d\n", pathx[i], pathy[i]);
					orientationCounter++;
				}
			}
		}
		orientationCounter = 0;
		while (orientation[orientationCounter] != NULL)
		{
			printf("%c,", orientation[orientationCounter]);
			orientationCounter++;
		}
		getTurns2();
	
}

/*void getTurns()
{
	orientationCounter = 0;
	turnsCounter = 0;
	while (orientation[orientationCounter] != NULL)
	{
		if ((orientation[orientationCounter] == 'D' && orientation[orientationCounter + 1] == 'R') || (orientation[orientationCounter] == 'R'&&orientation[orientationCounter + 1] == 'U'))
		{
			turns[turnsCounter] = 'l';
			turnsCounter++;
		}
		if ((orientation[orientationCounter] == 'D' && orientation[orientationCounter + 1] == 'L') || (orientation[orientationCounter] == 'L' && orientation[orientationCounter + 1] == 'U'))
		{
			turns[turnsCounter] = 'r';
			turnsCounter++;
		}
		if ((orientation[orientationCounter] == 'U'&&orientation[orientationCounter + 1] == 'R') || (orientation[orientationCounter] == 'R' && orientation[orientationCounter + 1] == 'D'))
		{
			turns[turnsCounter] = 'r';
			turnsCounter++;
		}
		if ((orientation[orientationCounter] == 'U'&&orientation[orientationCounter + 1] == 'L') || (orientation[orientationCounter] == 'L'&&orientation[orientationCounter + 1] == 'U'))
		{
			turns[turnsCounter] = 'l';
			turnsCounter++;
		}
		if (orientation[orientationCounter] == orientation[orientationCounter + 1])
		{
			turns[turnsCounter] = 's';
			turnsCounter++;
		}
		orientationCounter++;
	}
	printf("\n");
	turnsCounter = 0;
	bool value;
	while (turns[turnsCounter] != NULL)
	{
		printf("%c,", turns[turnsCounter]);
		value = WriteComPort("COM4", turns[turnsCounter]);
		//printf("%d\n", value);
		turnsCounter++;
	}
}*/


void getTurns2()
{
	orientationCounter = 0;
	turnsCounter = 0;
	int s = 0;
	while (orientation[orientationCounter] != NULL)
	{
		if (orientationCounter + 1 < 1000)
		{
			if ((orientation[orientationCounter] == 'U' && orientation[orientationCounter + 1] == 'R') || (orientation[orientationCounter] == 'L' && orientation[orientationCounter + 1] == 'U') || (orientation[orientationCounter] == 'D' && orientation[orientationCounter + 1] == 'L') || (orientation[orientationCounter] == 'R' && orientation[orientationCounter + 1] == 'D'))
			{
				//right
				turns[turnsCounter] = 'r';
				turnsCounter++;
			}
			if ((orientation[orientationCounter] == 'U' && orientation[orientationCounter + 1] == 'L') || (orientation[orientationCounter] == 'L' && orientation[orientationCounter + 1] == 'D') || (orientation[orientationCounter] == 'D' && orientation[orientationCounter + 1] == 'R') || (orientation[orientationCounter] == 'R' && orientation[orientationCounter + 1] == 'U'))
			{
				//left	
				turns[turnsCounter] = 'l';
				turnsCounter++;
			}
			if (((orientation[orientationCounter] == 'r') || (orientation[orientationCounter] == 'u') || (orientation[orientationCounter] == 'l') || (orientation[orientationCounter] == 'd'))&&orientationCounter!=0)
			{
				//straight
				turns[turnsCounter] = 's';
				turnsCounter++;
			}
			orientationCounter++;
		}
	}
	turns[turnsCounter] = '.';
	printf("\n");
	turnsCounter = 0;
	bool value;
	while (turns[turnsCounter] != NULL)
	{
		printf("%c,", turns[turnsCounter]);
		value = WriteComPort("COM4", turns[turnsCounter]);
		if (turns[turnsCounter] == '.')
		{
			CloseHandle(hPort);
		}
		//printf("%d\n", value);
		turnsCounter++;
		
	}
}


void findJunctions()
{

	int neighbour = 0;
	for (int i = 0; i < 1000; i++)
	{
		for (int j = 0; j < 1000; j++)
		{
			if (mat[i][j] != 0)
			{
				if (mat[i - 1][j] == 1)
				{//NORTH Neighbour
					neighbour++;
				}
				if (mat[i + 1][j] == 1)
				{//SOUTH Neighbour
					neighbour++;
				}
				if (mat[i][j+1] == 1)
				{//EAST Neighbour
					neighbour++;
				}
				if (mat[i][j-1] == 1)
				{//WEST Neighbour
					neighbour++;
				}
				if (neighbour >= 3)
				{
					junctions[jc].x = j;
					junctions[jc].y = i;
					jc++;
				}
				neighbour = 0;
			}
		}
	}
	return;
}


// Driver program to test above function
/*int main()
{
	/* Description of the Grid-
	1--> The cell is not blocked
	0--> The cell is blocked    
	int grid[ROW][COL] =
	{
		{ 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
		{ 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 },
		{ 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 },
		{ 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 },
		{ 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 },
		{ 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 },
		{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
		{ 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
		{ 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 }
	};

	// Source is the left-most bottom-most corner
	Pair src = make_pair(8, 0);

	// Destination is the left-most top-most corner
	Pair dest = make_pair(0, 0);

	aStarSearch(grid, src, dest);
	getch();
	return(0);
}*/