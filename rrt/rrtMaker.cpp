#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "obstacles.h"
#include "display.h"


// This is a helper function that determines whether or not line segment (v1---v2) intersects line segment (v3---v4)
unsigned char linesIntersect(short v1x, short v1y, short v2x, short v2y, short v3x, short v3y, short v4x, short v4y) {
  float uA = ((v4x-v3x)*(v1y-v3y) - (v4y-v3y)*(v1x-v3x)) / (float)(((v4y-v3y)*(v2x-v1x) - (v4x-v3x)*(v2y-v1y)));
  float uB = ((v2x-v1x)*(v1y-v3y) - (v2y-v1y)*(v1x-v3x)) / (float)(((v4y-v3y)*(v2x-v1x) - (v4x-v3x)*(v2y-v1y)));

  // If uA and uB are between 0-1, there is an intersection
  if (uA > 0 && uA < 1 && uB > 0 && uB < 1) 
		return 1;

	return 0;
}


float distance(short x1, short y1, short x2, short y2) {
	return sqrtf(powf((float)(x1 - x2), 2.f) + powf((float)(y1 - y2), 2.f));
}


unsigned char hittingObstacle(short x, short y, Obstacle obst) {
	if (x >= obst.x && x <= obst.x + obst.w && y <= obst.y && y >= obst.y - obst.h)
		return 1;

	return 0;
}

// Create a rrt using the growthAmount and maximumNodes parameters in the given environment.
void createRRT(Environment *env) {
	TreeNode* q_new = NULL;

	env->rrt = (TreeNode**)malloc(env->maximumNodes*sizeof(TreeNode*));
	for (unsigned short i = 0; i < env->maximumNodes; i++)
		env->rrt[i] = NULL;

	q_new = (TreeNode*)malloc(sizeof(TreeNode));
	if (q_new != NULL) {
		q_new->x = env->startX;
		q_new->y = env->startY;
		q_new->parent = NULL;
		q_new->firstChild = NULL;
		env->rrt[0] = q_new;
		env->numNodes++;
	}

	while (env->numNodes < env->maximumNodes) {
		short x = rand() % (ENV_WIDTH + 1);
		short y = rand() % (ENV_HEIGHT + 1);
		unsigned char is_hitting = 0;

		for (unsigned short j = 0; j < env->numObstacles; j++) {
			if (hittingObstacle(x, y, env->obstacles[j])) {
				is_hitting = 1;
				break;
			}
		}

		if (!is_hitting) {
			TreeNode* nearest = NULL;
			float     min_dis = 10e4f;
			is_hitting        = 0;

			for (unsigned short j = 0; j < env->numNodes; j++) {
				if (env->rrt[j] != NULL) {
					float dis = distance(x, y, env->rrt[j]->x, env->rrt[j]->y);
					if (min_dis > dis && dis > 0) {
						min_dis = dis;
						nearest = env->rrt[j];
					}
				}
			}

			if (nearest != NULL) {
				float angle = atanf((nearest->y - y) / (float)(nearest->x - x));
				float x_c = nearest->x + env->growthAmount * cosf(angle);
				float y_c = nearest->y + env->growthAmount * sinf(angle);

				for (unsigned short j = 0; j < env->numObstacles; j++) {
					if (hittingObstacle((short)x_c, (short)y_c, env->obstacles[j])
						|| linesIntersect(nearest->x, nearest->y, x_c, y_c, env->obstacles[j].x, ENV_HEIGHT - env->obstacles[j].y,
							env->obstacles[j].x, env->obstacles[j].h + (ENV_HEIGHT - env->obstacles[j].y))
						|| linesIntersect(nearest->x, nearest->y, x_c, y_c, env->obstacles[j].x, ENV_HEIGHT - env->obstacles[j].y,
							env->obstacles[j].x + env->obstacles[j].w, ENV_HEIGHT - env->obstacles[j].y)
						|| linesIntersect(nearest->x, nearest->y, x_c, y_c, env->obstacles[j].x, env->obstacles[j].y,
							env->obstacles[j].x + env->obstacles[j].w, env->obstacles[j].y)
						|| linesIntersect(nearest->x, nearest->y, x_c, y_c, env->obstacles[j].x + env->obstacles[j].w, ENV_HEIGHT - env->obstacles[j].y,
							env->obstacles[j].x + env->obstacles[j].w, env->obstacles[j].h + (ENV_HEIGHT - env->obstacles[j].y))) {
						is_hitting = 1;
						break;
					}
				}

				if (!is_hitting) {
					TreeNode* c = (TreeNode*)malloc(sizeof(TreeNode));
					if (c != NULL) {
						c->x = (short)x_c;
						c->y = (short)y_c;
						c->parent = nearest;
						c->firstChild = NULL;
					}

					Child* spl = (Child*)malloc(sizeof(Child));
					if (spl != NULL) {
						spl->node = c;
						spl->nextSibling = NULL;
					}

					if (nearest->firstChild != NULL) {
						Child* pc = nearest->firstChild;
						while (pc != NULL)
							pc = pc->nextSibling;
						pc = spl;
					}
					else
						nearest->firstChild = spl;
					
					env->rrt[env->numNodes] = c;
					env->numNodes++;
				}
			}
		}
	}
}


// Trace the path back from the node that is closest to the given (x,y) coordinate to the root
void tracePath(Environment *env, unsigned short x, unsigned short y) {
	TreeNode* nearest = NULL;
	float     min_dis = 10e4f;
	for (unsigned short i = 0; i < env->numNodes; i++) {
		float dis = distance(x, y, env->rrt[i]->x, env->rrt[i]->y);
		if (min_dis > dis) {
			min_dis = dis;
			nearest = env->rrt[i];
		}
	}

	if (nearest != NULL) {
		env->path = (TreeNode**)malloc(sizeof(TreeNode*));
		if (env->path != NULL)
			env->path[0] = nearest;
	}
}


// This procedure cleans up everything by creeing all alocated memory
void cleanupEverything(Environment *env) {
	free(env->obstacles);
	for (int i = 0; i < env->maximumNodes; i++) {
		TreeNode* p = env->rrt[i];
		Child* c = p->firstChild;
		while (c) {
			Child* n = c;
			c = c->nextSibling;
			free(n);
			n = NULL;
		}
		free(p);
		p = NULL;
	}
	free(env->rrt);
	env->rrt = NULL;
	free(env->path);
	env->path = NULL;
}


