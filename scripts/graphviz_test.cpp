#include <graphviz/gvc.h>

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>

int main(int argc, char * argv[]) {

    // set up a graphviz context
    GVC_t *gvc = gvContext();

    // Create a simple digraph
    Agraph_t *g = agopen("g", Agdirected, 0);


    gvLayout(gvc, g, "dot");
    Agnode_t *n = agnode(g, "n", 1);
    Agnode_t *m = agnode(g, "m", 1);
    Agedge_t *e = agedge(g, n, m, 0, 1);


    // Set an attribute - in this case one that affects the visible rendering
    agsafeset(n, "color", "red", "");

    // Compute a layout using layout engine from command line args
    gvLayoutJobs(gvc, g);

    // Write the graph according to -T and -o options
    gvRenderJobs(gvc, g);

    // Free layout data
    gvFreeLayout(gvc, g);

    // Free graph structures
    agclose(g);

    // close output file, free context, and return number of errors
    return gvFreeContext(gvc);


}
