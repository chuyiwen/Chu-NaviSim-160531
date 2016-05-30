#include "define.h"
#include "object.h"
#include "gen_graph.h"
#include "dijkstra.h"
#include "car.h"
#include <stdio.h>

int main()
{
	struct _INFO_NODE_ARC *_info_node_arc;
	struct _INFO_DIJKSTRA *_info_dijkstra;
	struct _INFO_CAR_MOVING *_info_car_moving;
	_info_node_arc = gen_lattice(10,40);
	//_info_node_arc =gen_radial(10,10,40);
	//_info_node_arc = gen_general_graph("/home/chuyiwen/Desktop/tokyo6.dat");
	//_info_dijkstra = dijkstra(*_info_node_arc,0,9999);
		
	car_move(*_info_node_arc);
	
	return 0;
}
