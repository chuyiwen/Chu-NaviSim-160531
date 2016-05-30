#ifndef GEN_GRAPH
#define GEN_GRAPH
#include "define.h"
#include "object.h"
#include "math.h"
#include "malloc.h"
#include <string.h>
#include <stdio.h>

struct _INFO_NODE_ARC *gen_lattice(int n,int length);
struct _INFO_NODE_ARC *gen_radial(int n,int m,int length);
struct _INFO_NODE_ARC *gen_general_graph(char *filename);

#endif // GEN_GRAPH

