/* Here are implemented the functions defined in 
 * common_net.hpp


*/

// C++ related
#include <iostream>
#include <cstdlib>
#include <vector>
#include <math.h>


// Caffe related
//#include <caffe/caffe.hpp>
//#include <caffe/util/io.hpp>

// Project related
#include "Scenario.hpp"
#include "Supervisor.hpp"
#include "Common_net.hpp"
#include "Build_set.hpp"

// C++ related
using std::vector;
using std::cout;

// Caffe related
using caffe::Datum;


// Project related
using namespace Supervisor;
using namespace PathPlanning;

#ifdef DEBUG_LEV_2
const float MaxDiff = 50;
#endif

namespace NeuralNetwork
{


void AdjacentStatus(std::vector<float>& directionStatus, Pixel& current, Map& map, CoordinateList& direction) {
	
	if( direction.size() != directionStatus.size() ) {
			cout << "direction size does not match direction status size, check for inconsistences" << endl;
			exit(1);
	}
	
	for(int i = 0; i<directionStatus.size(); i++) {		
		Pixel next = current + direction[i];
			if ( OutOfBounds(map, next) ) {
				directionStatus[i] = 2;
			}
			else if( map.coord[next.x][next.y].sym != '.' ) {
				directionStatus[i] = 1;
			}
			else {
				directionStatus[i] = 0;
			}
	}

};


int getDirectionIndex(CoordinateList& direction, Pixel& nextPix) {
	
	int i;
	for(i = 0; i<direction.size(); i++) {
		if( direction[i] == nextPix ) {
			return i;
		}
	}

	return i;

};

			
vector<float> OccupancyData(Map& map, Pixel& current, Pixel& current_direction, int width, int height ) {
	
	vector<float> occupancyStatus;
	float data; 

	Pixel dir1(0,1);
	if( current_direction == dir1 ) {	
		Pixel LowSx(current.x + width/2, current.y);
		for(int vert = 0; vert < height; vert++) {
			for(int horiz = 0; horiz < width; horiz++) {
				Pixel nextPix(LowSx, -horiz, vert);
				if( OutOfBounds(map, nextPix) ) {
					data = 1;	
					occupancyStatus.push_back(data);
				}
				else if( detectCollision(map.coord[nextPix.x][nextPix.y]) ) {
					data = 1;
					occupancyStatus.push_back(data);
				}
				else {
					data = 0;
					occupancyStatus.push_back(data);
				}				
			}
		}
	return occupancyStatus;
	}

	Pixel dir2(1,0);
	if( current_direction == dir2 ) {
		Pixel LowSx(current.x, current.y - width/2);
		for(int vert = 0; vert < height; vert++) {
			for(int horiz = 0; horiz < width; horiz++) {
				Pixel nextPix(LowSx, vert, horiz);
				if( OutOfBounds(map, nextPix) ) {
					data = 1;	
					occupancyStatus.push_back(data);
				}
				else if( detectCollision(map.coord[nextPix.x][nextPix.y]) ) {
					data = 1;
					occupancyStatus.push_back(data);
				}
				else {
					data = 0;
					occupancyStatus.push_back(data);
				}	
			}		
		}
	return occupancyStatus;
	}
	
	Pixel dir3(0,-1);
	if( current_direction == dir3 ) {
		Pixel LowSx(current.x - width/2, current.y);
		for(int vert = 0; vert < height; vert++) {
			for(int horiz = 0; horiz < width; horiz++) {
			Pixel nextPix(LowSx, horiz, -vert);
				if( OutOfBounds(map, nextPix) ) {
					data = 1;	
					occupancyStatus.push_back(data);
				}
				else if( detectCollision(map.coord[nextPix.x][nextPix.y]) ) {
					data = 1;
					occupancyStatus.push_back(data);
				}
				else {
					data = 0;
					occupancyStatus.push_back(data);
				}		
			}
		}
	return occupancyStatus;
	}
	
	Pixel dir4(-1,0);
	if( current_direction == dir4 ) {
		Pixel LowSx(current.x, current.y + width/2);
		for(int vert = 0; vert < height; vert++) {
			for(int horiz = 0; horiz < width; horiz++) {
				Pixel nextPix(LowSx, -vert, -horiz);
				if( OutOfBounds(map, nextPix) ) {
					data = 1;	
					occupancyStatus.push_back(data);
				}
				else if( detectCollision(map.coord[nextPix.x][nextPix.y]) ) {
					data = 1;
					occupancyStatus.push_back(data);
				}
				else {
					data = 0;
					occupancyStatus.push_back(data);
				}				
			}	
		}
	return occupancyStatus;		
	}
	
	Pixel dir5(-1,-1);
	if( current_direction == dir5 ) {
		
		Pixel LowSx(current.x - width/2, current.y + width/2);
		for(int vert = 0; vert < height; vert++) {
			for(int horiz = 0; horiz < width; horiz++) {
				Pixel nextPix(LowSx, horiz, -horiz + vert);
				if( OutOfBounds(map, nextPix) ) {
					data = 1;	
					occupancyStatus.push_back(data);
				}
				else if( detectCollision(map.coord[nextPix.x][nextPix.y]) ) {
					data = 1;
					occupancyStatus.push_back(data);
				}
				else {
					data = 0;
					occupancyStatus.push_back(data);
				}		
			}
		}	
	return occupancyStatus;
	}
	
	
	
	Pixel dir6(1,1);
	if( current_direction == dir6 ) {	
		Pixel LowSx(current.x + width/2, current.y - width/2);
		for(int vert = 0; vert < height; vert++) {
			for(int horiz = 0; horiz < width; horiz++) {
				Pixel nextPix(LowSx, -horiz, horiz - vert);
				if( OutOfBounds(map, nextPix) ) {
					data = 1;	
					occupancyStatus.push_back(data);
				}
				else if( detectCollision(map.coord[nextPix.x][nextPix.y]) ) {
					data = 1;
					occupancyStatus.push_back(data);
				}
				else {
					data = 0;
					occupancyStatus.push_back(data);
				}		
			}
		}	
	return occupancyStatus;
	}

	Pixel dir7(-1,1);
	if( current_direction == dir7 ) {
		Pixel LowSx(current.x + width/2, current.y + width/2);
		for(int horiz = 0; horiz < width; horiz++) {
			for(int vert = 0; vert < height; vert++)  {
				Pixel nextPix(LowSx, -horiz, -horiz - vert);
				if( OutOfBounds(map, nextPix) ) {
					data = 2;	
					occupancyStatus.push_back(data);
				}
				else if( detectCollision(map.coord[nextPix.x][nextPix.y]) ) {
					data = 1;
					occupancyStatus.push_back(data);
				}
				else {
					data = 0;
					occupancyStatus.push_back(data);
				}		
			}
		}
	return occupancyStatus;
	}
	
	Pixel dir8(1,-1);
	if( current_direction == dir8 ) {
		Pixel LowSx(current.x - width/2, current.y - width/2);
		for(int vert = 0;  vert < height; vert++) {
			for(int horiz = 0; horiz < width; horiz++) {
				Pixel nextPix(LowSx, horiz+vert, horiz);
				if( OutOfBounds(map, nextPix) ) {
					data = 1;	
					occupancyStatus.push_back(data);
				}
				else if( detectCollision(map.coord[nextPix.x][nextPix.y]) ) {
					data = 1;
					occupancyStatus.push_back(data);
				}
				else {
					data = 0;
					occupancyStatus.push_back(data);
				}	
			}
		}
	return occupancyStatus;		
	}

};

vector<float> LocalMapToData(LocalMap& occupancy) {

	vector<float> occupancyStatus;
	
	for(int i; i < occupancy.size(); i++) {
		if( occupancy[i].sym == 'B' ) {
			float data = 2;
			occupancyStatus.push_back(data);
		}
		else if( occupancy[i].sym == 'T' || occupancy[i].sym == '@') {
			float data = 1;
			occupancyStatus.push_back(data);
		}
		else {
			float data = 0;
			occupancyStatus.push_back(data);
		}	
	}

	return occupancyStatus;
};

void shutdown(int sign)
{
	if (sign == SIGINT)
	{
		printf("Exit requested\n");
		exit(1);
	}
}

int getBlobSize(const float * array) {

	return sizeof(array) / sizeof(array[0]);
}

int getScenarioFromArchive(vector<Scenario> archive, string scenario_name) {

	int i = 0;
	for(i = 0; i < archive.size(); i++) {
		if( archive[i].getScenarioName() == scenario_name ) {
			break;
		}
	}

	if( i < archive.size() ) {
		return i;
	}
	else {
		cout << "Scenario not found in archive " << i << endl;
		exit(1);
	}

};

/* Function updates the occupancyMap map with respect to current position of the agent
 * in order to give to the neural a network an always consistent vector representing this map
 * I developed this solution, surely not valid in terms of code compactness, but  
 * good in terms of runtime computation
 */

// Assumption: Robot is in the middle of the bottom line of its local map
void OccupancyGrid(Map& map, LocalMap& occupancyMap, Pixel& current, Pixel& current_direction, int width, int height) {

	Pixel dir1(0,1);
	if( current_direction == dir1 ) {	
		Pixel LowSx(current.x + width/2, current.y);
		for(int vert = 0; vert < height; vert++) {
			for(int horiz = 0; horiz < width; horiz++) {
				Pixel nextPix(LowSx, -horiz, vert);
				if( OutOfBounds(map, nextPix) ) {
					PixelMap newPixel(nextPix, 'B' );
					occupancyMap.push_back(newPixel);
					break;
				}	
				PixelMap LocalPix(map.coord[nextPix.x][nextPix.y].pix,
							 map.coord[nextPix.x][nextPix.y].sym);	
				occupancyMap.push_back(LocalPix);			
			}
		}
		
	}

	Pixel dir2(1,0);
	if( current_direction == dir2 ) {
		Pixel LowSx(current.x, current.y - width/2);
		for(int vert = 0; vert < height; vert++) {
			for(int horiz = 0; horiz < width; horiz++) {
				Pixel nextPix(LowSx, vert, horiz);
				if( OutOfBounds(map, nextPix) ) {
					PixelMap newPixel(nextPix, 'B' );
					occupancyMap.push_back(newPixel);
					break;
				}
				PixelMap LocalPix(map.coord[nextPix.x][nextPix.y].pix,
							 map.coord[nextPix.x][nextPix.y].sym);	
				occupancyMap.push_back(LocalPix);
			}		
		}
		
	}
	
	Pixel dir3(0,-1);
	if( current_direction == dir3 ) {
		Pixel LowSx(current.x - width/2, current.y);
		for(int vert = 0; vert < height; vert++) {
			for(int horiz = 0; horiz < width; horiz++) {
			Pixel nextPix(LowSx, horiz, -vert);
				if( OutOfBounds(map, nextPix) ) {
					PixelMap newPixel(nextPix, 'B' );
					occupancyMap.push_back(newPixel);
					break;
				}
				PixelMap LocalPix(map.coord[nextPix.x][nextPix.y].pix,
							 map.coord[nextPix.x][nextPix.y].sym);	
				occupancyMap.push_back(LocalPix);
			}
		}
		
	}
	
	Pixel dir4(-1,0);
	if( current_direction == dir4 ) {
		Pixel LowSx(current.x, current.y + width/2);
		for(int vert = 0; vert < height; vert++) {
			for(int horiz = 0; horiz < width; horiz++) {
				Pixel nextPix(LowSx, -vert, -horiz);
				if( OutOfBounds(map, nextPix) ) {
					PixelMap newPixel(nextPix, 'B' );
					occupancyMap.push_back(newPixel);
					break;
				}
				PixelMap LocalPix(map.coord[nextPix.x][nextPix.y].pix,
							 map.coord[nextPix.x][nextPix.y].sym);	
				occupancyMap.push_back(LocalPix);	
			}	
		}
		
	}
	
	Pixel dir5(-1,-1);
	if( current_direction == dir5 ) {
		
		Pixel LowSx(current.x - width/2, current.y + width/2);
		for(int vert = 0; vert < height; vert++) {
			for(int horiz = 0; horiz < width; horiz++) {
				Pixel nextPix(LowSx, horiz, -horiz + vert);
				if( OutOfBounds(map, nextPix) ) {
					PixelMap newPixel(nextPix, 'B' );
					occupancyMap.push_back(newPixel);
					break;
				}
				PixelMap LocalPix(map.coord[nextPix.x][nextPix.y].pix,
							 map.coord[nextPix.x][nextPix.y].sym);	
				occupancyMap.push_back(LocalPix);	
			}
		}	
	}
	
	
	
	Pixel dir6(1,1);
	if( current_direction == dir6 ) {	
		Pixel LowSx(current.x + width/2, current.y - width/2);
		for(int vert = 0; vert < height; vert++) {
			for(int horiz = 0; horiz < width; horiz++) {
				Pixel nextPix(LowSx, -horiz, horiz - vert);
				if( OutOfBounds(map, nextPix) ) {
					PixelMap newPixel(nextPix, 'B' );
					occupancyMap.push_back(newPixel);
					break;
				}
				
				PixelMap LocalPix(map.coord[nextPix.x][nextPix.y].pix,
							 map.coord[nextPix.x][nextPix.y].sym);	
				occupancyMap.push_back(LocalPix);
			}
		}	
	}

	Pixel dir7(-1,1);
	if( current_direction == dir7 ) {
		Pixel LowSx(current.x + width/2, current.y + width/2);
		for(int horiz = 0; horiz < width; horiz++) {
			for(int vert = 0; vert < height; vert++)  {
				Pixel nextPix(LowSx, -horiz, -horiz - vert);
				if( OutOfBounds(map, nextPix) ) {
					PixelMap newPixel(nextPix, 'B' );
					occupancyMap.push_back(newPixel);
					break;
				}
				PixelMap LocalPix(map.coord[nextPix.x][nextPix.y].pix,
							 map.coord[nextPix.x][nextPix.y].sym);	
				occupancyMap.push_back(LocalPix);	
			}
		}
	}
	
	Pixel dir8(1,-1);
	if( current_direction == dir8 ) {
		Pixel LowSx(current.x - width/2, current.y - width/2);
		for(int vert = 0;  vert < height; vert++) {
			for(int horiz = 0; horiz < width; horiz++) {
				Pixel nextPix(LowSx, horiz+vert, horiz);
				if( OutOfBounds(map, nextPix) ) {
					PixelMap newPixel(nextPix, 'B' );
					occupancyMap.push_back(newPixel);
					break;
				}
				PixelMap LocalPix(map.coord[nextPix.x][nextPix.y].pix,
							 map.coord[nextPix.x][nextPix.y].sym);	
				occupancyMap.push_back(LocalPix);
			}
		}
		
	}
	
				
};


void PrintOccupancyGrid(Map& map, LocalMap& occupancyMap, string filename) {

	ofstream temp;
	temp.open(filename.c_str());
	if( temp.fail() ) {
		cout << "Print Map to file opening failed.\n";
		exit(1);
	}
	
	temp << "height " << map.height << endl;
	temp << "width " << map.width << endl;

	int x,y;
	for(y=0; y<map.height; y++) {
		for(x=0; x<map.width; x++) {
			
			if( findPixel(x, y, occupancyMap) ) {
				if( detectCollision(map.coord[x][y]) ) {
					temp << 'o';
				}

			}
			else {
				temp << map.coord[x][y].sym;
			}
			
		}
		temp << endl;
	}
	
	temp.close();	

};

float getModDistance(Pixel& current, Pixel& target) {

	float dy = current.y - target.y;
	float dx = current.x - target.x;

	return static_cast<float> ( DIST_WEIGHT * sqrt(pow(dx, 2) + pow(dy, 2)) );

};	

float getRelativeAngle(Pixel& current, Pixel& target) {

	float num = current.y - target.y;
	float den = current.x - target.x;

	return ANGLE_WEIGHT * atan2(num, den);

};


void DistanceMap(vector<float>& dist_state, Map& map, Pixel& current, CoordinateList& direction) {
	
	
	for(int i = 0; i < direction.size() / 2; i++) {
		int step = 0;
		float dist_dir = 0;
		Pixel toward_dir(current);

		while(  !detectCollision(map.coord[toward_dir.x][toward_dir.y]) &&
		        step < DISTANCE_RANGE ) {
		     dist_dir++;
		     toward_dir = toward_dir + direction[i];
			if( OutOfBounds(map, toward_dir) )
				break;

   		     step++;
			     
		}
	 	dist_state[i] = dist_dir * PROXIM_WEIGHT;
			
	}

		

	for(int i = direction.size() / 2; i < direction.size(); i++) {
		int step = 0;
		float dist_dir = 0;
		Pixel toward_dir(current);
		
		while(  !detectCollision(map.coord[toward_dir.x][toward_dir.y]) &&
	          step < DISTANCE_RANGE ) {
		     dist_dir += sqrt(2);
		     toward_dir = toward_dir + direction[i];
		     if( OutOfBounds(map, toward_dir) )
	   			break;

			step++;
		}
	 	dist_state[i] = dist_dir * PROXIM_WEIGHT;
			
	}

	
	
};

			
} // namespace NeuralNetwork
