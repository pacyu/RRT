#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "opencv2/core.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "obstacles.h"
#include "display.h"

#define  VERTEX_RADIUS    5
#define  EDGE_THICKNESS   3

const int   OBSTACLE_COLOR = 0x7799FF;
const int   BORDER_COLOR = 0x000000;
const int   EDGE_COLOR = 0x000000;
const int   SELECTED_COLOR = 0x008800;
const int   VERTEX_COLOR = 0xDD0000;


//Display-related variables
using namespace cv;


// Redraw all the obstacles and the edges and the vertices
void displayEnvironment(Environment *env) {
  Mat img(ENV_HEIGHT, ENV_WIDTH, CV_8UC3, Scalar(255, 255, 255));

	Child  *c = NULL; // A child of the current node

  // Draw all the obstacles
  for (int i=0; i<env->numObstacles; i++) {
    rectangle(img, { env->obstacles[i].x, ENV_HEIGHT - env->obstacles[i].y },
      { env->obstacles[i].x + env->obstacles[i].w, ENV_HEIGHT - env->obstacles[i].y + env->obstacles[i].h },
      Scalar(210, 180, 30), FILLED);
  }

  // Draw the tree
  if (env->rrt != NULL) {
   	// Go through all the nodes of the rrt and draw edges from them to their children
    for (int i=0; i<env->numNodes; i++) {
    	TreeNode  *n = env->rrt[i];
    	c = n->firstChild;
		  while (c != NULL) {
        line(img, { n->x - VERTEX_RADIUS, ENV_HEIGHT - (n->y + VERTEX_RADIUS) },
          { c->node->x - VERTEX_RADIUS, ENV_HEIGHT - (c->node->y + VERTEX_RADIUS) }, Scalar(0, 0, 0));
			  c = c->nextSibling;
		  }
	  }
	
    // Draw all the Nodes
   	for (int i=0; i<env->numNodes; i++) {
    	TreeNode  *n = env->rrt[i];
      circle(img, { n->x - VERTEX_RADIUS, ENV_HEIGHT - (n->y + VERTEX_RADIUS) },
				 	 VERTEX_RADIUS, Scalar(60, 10, 210), FILLED);
      circle(img, { n->x - VERTEX_RADIUS, ENV_HEIGHT - (n->y + VERTEX_RADIUS) },
        VERTEX_RADIUS, Scalar(0, 0, 0));
		}
	}

  // Draw the path
  if (env->path != NULL) {
   	// Go through all the nodes of the path and draw edges
    TreeNode *n = env->path[0];
    while (n != env->rrt[0]) {
    	TreeNode  *p = n->parent;
    	// Check if the line is more horizontal than vertical
    	
      line(img, { n->x - VERTEX_RADIUS, ENV_HEIGHT - (n->y + VERTEX_RADIUS) },
        { p->x - VERTEX_RADIUS, ENV_HEIGHT - (p->y + VERTEX_RADIUS) }, Scalar(20, 210, 120), 2);
    	
			n = p;
		}
	
    // Go through the nodes of the path and draw all the Nodes
    n = env->path[0];
    while (n != env->rrt[0]) {
      circle(img, { n->x - VERTEX_RADIUS, ENV_HEIGHT - (n->y + VERTEX_RADIUS) },
				 	 VERTEX_RADIUS, Scalar(20, 210, 120), FILLED);
      circle(img, { n->x - VERTEX_RADIUS, ENV_HEIGHT - (n->y + VERTEX_RADIUS) },
        VERTEX_RADIUS, Scalar(0, 0, 0));
			n = n->parent;
		}
	}

  imshow("RRT", img);
  if (waitKey(0) == 27) return;
}

