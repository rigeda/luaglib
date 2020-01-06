// Lua-GL module to implement the Breadth First Search algorithm and the routing matrix object in C

#include "lua.h"
#include "lauxlib.h"

//#include "stdio.h"

#if defined __WIN32 || __WIN32__
    # include <windows.h>
    # define _EXPORT __declspec(dllexport)
#else
    # define _EXPORT
#endif

#define VERSION     "1.19.12.31"


// structure to store the visited coordinates
// each structure stores 100  coordinates
struct visited{
    int x[100];
    int y[100];
    struct visited *prev;   // pointer to previous visited structure
    struct visited *next;   // pointer to next visited structure
};

// structure to store routing matrix segments
struct seg{
    int x1;
    int y1;
    int x2;
    int y2;
    struct seg *prev;
    struct seg *next;
};

// structure to store routing matrix ports
struct port{
    int x;
    int y;
    struct port *prev;
    struct port *next;
};

// routing matrix data structure
struct routingMatrix{
    int minX;
    int maxX;
    int minY;
    int maxY;
    struct seg *hsegs;
    struct seg *vsegs;
    struct seg *blksegs;
    struct port *ports;
};

static int validateStep(struct routingMatrix*,int,int,int,int,int,int);

static int smaller3(int a,int b,int c){
    if(a<b)
        if(a<c)
            return a;
        else
            return c;
    else
        if(b<c)
            return b;
        else
            return c;
}

static int larger3(int a,int b,int c){
    if(a>b)
        if(a>c)
            return a;
        else
            return c;
    else
        if(b>c)
            return b;
        else
            return c;
}

// Function to check whether a coordinate in visited exists
static int visitDone(struct visited *vst,int x,int y){
    while(vst->prev)
        vst = vst->prev;    // rewind to 1st structure
    while(vst){
        for(int i=0;i<100;i++){
            if(vst->x[i]==x && vst->y[i]==y)
                return 1;
        }   // for i=1 to 100 ends here
        vst = vst->next;
    }   // while (vst) ends here
    return 0;
}

// Breadth First Search algorithm implementation for Lua-GL
static int BFS(lua_State *L) {
/*
Input parameters are:
rM: Routing Matrix
srcX: Start point x coordinate
srcY: Start point y coordinate
destX:	Destination point x coordinate
destY:	Destination point y coordinate
stepX:	Step size in the x direction to take
stepY:	Step size in the y direction to take
minX:	(OPTIONAL) minimum x coordinate allowed
minY:	(OPTIONAL) minimum y coordinate allowed
maxX:	(OPTIONAL) maximum x coordinate allowed
maxY: 	(OPTIONAL) maximum y coordinate allowed
*/
    // call is made BFS(rM,srcX,srcY,destX,destY,stepX,stepY,minX,minY,maxX,maxY)
    struct routingMatrix *rm;
    rm = (struct routingMatrix*)lua_touserdata(L,1);	// routing matrix userdata is the 1st argument
    printf("Got the routing matrix\n");

    int srcX,srcY,destX,destY,stepX,stepY,minX,minY,maxX,maxY;
    srcX = lua_tointeger(L,2);
    srcY = lua_tointeger(L,3);
    destX = lua_tointeger(L,4);
    destY = lua_tointeger(L,5);
    stepX = lua_tointeger(L,6);
    stepY = lua_tointeger(L,7);
    if(lua_isinteger(L,8))
        minX = lua_tointeger(L,8);
    else
        minX = smaller3(rm->minX - stepX,destX-stepX,srcX-stepX);
    if(lua_isinteger(L,9))
        minY = lua_tointeger(L,9);
    else
        minY = smaller3(rm->minY - stepY,destY-stepY,srcY-stepY);
    if(lua_isinteger(L,10))
        maxX = lua_tointeger(L,10);
    else
        maxX = larger3(rm->maxX + stepX,destX+stepX,srcX+stepX);
    if(lua_isinteger(L,11))
        maxY = lua_tointeger(L,11);
    else
        maxY = larger3(rm->maxY + stepY,destY+stepY,srcY+stepY);

    // align minX,maxX,minY,maxY with the stepX and stepY from srcX and srcY
    minX = srcX-((srcX-minX)/stepX)*stepX;    // integer arithmetic will align minX
    maxX = srcX+((maxX-srcX)/stepX)*stepX;
    minY = srcY-((srcY-minY)/stepY)*stepY;
    maxY = srcY+((maxY-srcY)/stepY)*stepY;

    printf("Got the parameters srcX=%d,srcY=%d,destX=%d,destY=%d,stepX=%d,stepY=%d,minX=%d,minY=%d,maxX=%d,maxY=%d\n",srcX,srcY,destX,destY,stepX,stepY,minX,minY,maxX,maxY);
    int delX[] = {-stepX,0,0,stepX};
    int delY[] = {0,-stepY,stepY,0};
    char stepStr[] = {'L','U','D','R'};

    int r,c,offx,offy;
    r = (maxY - minY)/stepY+1;
    c = (maxX - minX)/stepX+1;
    offx = -minX;
    offy = -minY;

    printf("Create visited structure with r=%d and c=%d...",r,c);
    int *vst = (int *)malloc(r * c * sizeof(int));
    for(int i = 0;i<r;i++)
        for(int j = 0;j<c;j++)
            *(vst+i*c+j) = 0;
    printf("DONE\n");

    // put the srcX,srcY in visited
    *(vst+(srcY+offy)/stepY*c+(srcX+offx)/stepX)=1;

    printf("Create the queue structure...");
    // structure for the que
    struct queue{
        int x;
        int y;
        char *c;    // pointer to the path string
        int clen;   // length of the path string
        struct queue *next; // pointer to the next item in the queue
    };
    struct queue *qtop;
    struct queue *qbot;
    struct queue *qtemp;
    char *c1;
    char *c2;
    // Create a 1st item in the queue at spot srcX and srcY
    qtop = (struct queue *)malloc(sizeof(struct queue));
    qbot = qtop;
    qtop->x = srcX;
    qtop->y = srcY;
    qtop->c = (char *)malloc(sizeof(char));
    c1 = qtop->c;
    qtop->clen = 1;
    //qtop->c[0] = '\0';
    c1[0] = '\0';
    printf("DONE\n");

    // now start the BFS loop
    printf("Start the BFS loop\n");
    while(qtop){
        if(qtop->x == destX && qtop->y == destY){
            // Free all the memory here
            printf("Reached destination. Start cleanup...");
            lua_pushinteger(L,0);   // 0 distance to destination
            lua_pushstring(L,qtop->c);
            // Clear the queue
            while(qtop){
                free(qtop->c);
                qtemp = qtop->next;
                free(qtop);
                qtop = qtemp;
            }   // while qtop ends here
            free(vst);
            printf("DONE\n");
            return 2;
        }
        // Add the adjacent cells int he que
        int nxtX,nxtY;
        for(int i=0;i<4;i++){
            // get coordinates for the ith adjacent cell
            nxtX = qtop->x + delX[i];
            nxtY = qtop->y + delY[i];
            //printf("Check nxtX=%d and nxtY=%d\n",nxtX,nxtY);
            if(nxtX >= minX && nxtX <= maxX && nxtY >= minY && nxtY <= maxY &&
              validateStep(rm,qtop->x,qtop->y,nxtX,nxtY,destX,destY) && (*(vst+(nxtY+offy)/stepY*c+(nxtX+offx)/stepX)==0)){
				// mark cell as visited and enqueue it
				//printf("Create new queue for coordinate x=%d,y=%d...",nxtX,nxtY);
				*(vst+(nxtY+offy)/stepY*c+(nxtX+offx)/stepX) = 1;
				qtemp = (struct queue *)malloc(sizeof(struct queue));
				qbot->next = qtemp;
				qtemp->clen = qtop->clen + 1;
				// copy the path string
				//printf("Create new string of len=%d..",qtemp->clen);
				qtemp->c = (char *)malloc((qtemp->clen)*sizeof(char));
				c1 = qtemp->c;
				c2 = qtop->c;
				//printf("Copy String...");
				for(int j=0;j<qtop->clen-1;j++){
                    //printf("j=%d,c2[j]=%c\n",j,c2[j]);
                    c1[j] = c2[j];
				}
                //printf("done copy...");
				qbot = qtemp;
				qbot->next = 0;
				qbot->x = nxtX;
				qbot->y = nxtY;
                c1[qbot->clen-2] = stepStr[i];
                c1[qbot->clen-1] = '\0';
                //printf("added x=%d,y=%d,c=%s,clen=%d,%d",nxtX,nxtY,qbot->c,strlen(qbot->c),qbot->clen);
                //getchar();
                //printf("DONE,i=%d\n",i);
            }   // if(nxtX >= minX && nxtX <= maxX ends
        }   // for i ends here
        //printf("Free qtop...");
        qtemp = qtop;
        qtop = qtop->next;
        free(qtemp);
        //printf("DONE\n");
    }   // while (qtop) ends here
    printf("Free all memory...");

    // Free all the memory here
    // Clear the queue
    while(qtop){
        free(qtop->c);
        qtemp = qtop->next;
        free(qtop);
        qtop = qtemp;
    }   // while qtop ends here
    // Clear the visited
    free(vst);
    printf("DONE\n");
    lua_pushnil(L);   // 0 distance to destination
    lua_pushstring(L,"Cannot reach destination");
    return 2;
}

/*
BFS with visited structure
// Breadth First Search algorithm implementation for Lua-GL
static int BFS(lua_State *L) {

Input parameters are:
rM: Routing Matrix
srcX: Start point x coordinate
srcY: Start point y coordinate
destX:	Destination point x coordinate
destY:	Destination point y coordinate
stepX:	Step size in the x direction to take
stepY:	Step size in the y direction to take
minX:	(OPTIONAL) minimum x coordinate allowed
minY:	(OPTIONAL) minimum y coordinate allowed
maxX:	(OPTIONAL) maximum x coordinate allowed
maxY: 	(OPTIONAL) maximum y coordinate allowed

    // call is made BFS(rM,srcX,srcY,destX,destY,stepX,stepY,minX,minY,maxX,maxY)
    struct routingMatrix *rm;
    rm = (struct routingMatrix*)lua_touserdata(L,1);	// routing matrix userdata is the 1st argument
    printf("Got the routing matrix\n");

    int srcX,srcY,destX,destY,stepX,stepY,minX,minY,maxX,maxY;
    srcX = lua_tointeger(L,2);
    srcY = lua_tointeger(L,3);
    destX = lua_tointeger(L,4);
    destY = lua_tointeger(L,5);
    stepX = lua_tointeger(L,6);
    stepY = lua_tointeger(L,7);
    if(lua_isinteger(L,8))
        minX = lua_tointeger(L,8);
    else
        minX = smaller3(rm->minX - stepX,destX-stepX,srcX-stepX);
    if(lua_isinteger(L,9))
        minY = lua_tointeger(L,9);
    else
        minY = smaller3(rm->minY - stepY,destY-stepY,srcY-stepY);
    if(lua_isinteger(L,10))
        maxX = lua_tointeger(L,10);
    else
        maxX = larger3(rm->maxX + stepX,destX+stepX,srcX+stepX);
    if(lua_isinteger(L,11))
        maxY = lua_tointeger(L,11);
    else
        maxY = larger3(rm->maxY + stepY,destY+stepY,srcY+stepY);

    printf("Got the parameters srcX=%d,srcY=%d,destX=%d,destY=%d,stepX=%d,stepY=%d,minX=%d,minY=%d,maxX=%d,maxY=%d\n",srcX,srcY,destX,destY,stepX,stepY,minX,minY,maxX,maxY);
    int delX[] = {-stepX,0,0,stepX};
    int delY[] = {0,-stepY,stepY,0};
    char stepStr[] = {'L','U','D','R'};

    printf("Create visited structure...");
    struct visited * vst;
    vst = (struct visited *)malloc(sizeof(struct visited));
    vst->prev = 0;
    vst->next = 0;
    printf("DONE\n");

    int index;
    index = 0;      // pointer to the visited arrays up till where it is filled
    // put the srcX,srcY in visited
    vst->x[index] = srcX;
    vst->y[index] = srcY;

    printf("Create the queue structure...");
    // structure for the que
    struct queue{
        int x;
        int y;
        char *c;    // pointer to the path string
        int clen;   // length of the path string
        struct queue *next; // pointer to the next item in the queue
    };
    struct queue *qtop;
    struct queue *qbot;
    struct queue *qtemp;
    char *c1;
    char *c2;
    // Create a 1st item in the queue at spot srcX and srcY
    qtop = (struct queue *)malloc(sizeof(struct queue));
    qbot = qtop;
    qtop->x = srcX;
    qtop->y = srcY;
    qtop->c = (char *)malloc(sizeof(char));
    c1 = qtop->c;
    qtop->clen = 1;
    //qtop->c[0] = '\0';
    c1[0] = '\0';
    printf("DONE\n");

    // now start the BFS loop
    printf("Start the BFS loop\n");
    while(qtop){
        if(qtop->x == destX && qtop->y == destY){
            // Free all the memory here
            printf("Reached destination. Start cleanup...");
            lua_pushinteger(L,0);   // 0 distance to destination
            lua_pushstring(L,qtop->c);
            // Clear the queue
            while(qtop){
                free(qtop->c);
                qtemp = qtop->next;
                free(qtop);
                qtop = qtemp;
            }   // while qtop ends here
            // Clear the visited
            struct visited *vnext;
            while(vst){
                vnext = vst->next;
                free(vst);
                vst = vnext;
            }
            printf("DONE\n");
            return 2;
        }
        // Add the adjacent cells int he que
        int nxtX,nxtY;
        for(int i=0;i<4;i++){
            // get coordinates for the ith adjacent cell
            nxtX = qtop->x + delX[i];
            nxtY = qtop->y + delY[i];

            if(nxtX >= minX && nxtX <= maxX && nxtY >= minY && nxtY <= maxY &&
              validateStep(rm,qtop->x,qtop->y,nxtX,nxtY,destX,destY) && (!visitDone(vst,nxtX,nxtY))){
				// mark cell as visited and enqueue it
				//printf("Add a step to the queue at step %d at index=%d...",qtop->clen,index);
				index = index+1;
				if(index > 99){
                    // create a new visited structure since this one is full
                    vst->next = (struct visited *)malloc(sizeof(struct visited));
                    vst->next->next = 0;
                    vst->next->prev = vst;
                    index = 0;
                    vst = vst->next;
				}
				vst->x[index] = nxtX;
				vst->y[index] = nxtY;
				//printf("Create new queue for coordinate x=%d,y=%d...",nxtX,nxtY);
				qtemp = (struct queue *)malloc(sizeof(struct queue));
				qbot->next = qtemp;
				qtemp->clen = qtop->clen + 1;
				// copy the path string
				//printf("Create new string of len=%d..",qtemp->clen);
				qtemp->c = (char *)malloc((qtemp->clen)*sizeof(char));
				c1 = qtemp->c;
				c2 = qtop->c;
				//printf("Copy String...");
				for(int j=0;j<qtop->clen-1;j++){
                    //printf("j=%d,c2[j]=%c\n",j,c2[j]);
                    c1[j] = c2[j];
				}
                //printf("done copy...");
				qbot = qtemp;
				qbot->next = 0;
				qbot->x = nxtX;
				qbot->y = nxtY;
                c1[qbot->clen-2] = stepStr[i];
                c1[qbot->clen-1] = '\0';
                //printf("added x=%d,y=%d,c=%s,clen=%d,%d\n",nxtX,nxtY,qbot->c,strlen(qbot->c),qbot->clen);
                //getchar();
                //printf("DONE\n");
            }   // if(nxtX >= minX && nxtX <= maxX ends
        }   // for i ends here
        qtemp = qtop;
        qtop = qtop->next;
        free(qtemp);
    }   // while (qtop) ends here

    // Free all the memory here
    // Clear the queue
    while(qtop){
        free(qtop->c);
        qtemp = qtop->next;
        free(qtop);
        qtop = qtemp;
    }   // while qtop ends here
    // Clear the visited
    struct visited *vnext;
    while(vst){
        vnext = vst->next;
        free(vst);
        vst = vnext;
    }
    lua_pushnil(L);   // 0 distance to destination
    lua_pushstring(L,"Cannot reach destination");
    return 2;
}*/

void fillLimits(lua_State *L, struct routingMatrix *rm,int x,int y){
	if (x < rm->minX)
        rm->minX = x;
	else if (x > rm->maxX)
		rm->maxX = x;
	if (y < rm->minY)
		rm->minY = y;
	else if (y > rm->maxY)
		rm->maxY = y;
    // update the minX, minY, maxX, maxY in the metatable __index table
    lua_pushlightuserdata(L,rm);
    lua_getmetatable(L,-1);
    lua_getfield(L,-1,"__index");
    lua_pushinteger(L,rm->minX);
    lua_setfield(L,-2,"minX");
    lua_pushinteger(L,rm->maxX);
    lua_setfield(L,-2,"maxX");
    lua_pushinteger(L,rm->minY);
    lua_setfield(L,-2,"minY");
    lua_pushinteger(L,rm->maxY);
    lua_setfield(L,-2,"maxY");
    lua_pop(L,2);
}

// Routing Matrix functions
// Function to add a connector segment to the routing matrix
static int addSegment(lua_State *L) {
    // parameters in lua passed like: function(rm,key,x1,y1,x2,y2)
    //printf("Get routing Matrix\n");
    struct routingMatrix *rm = (struct routingMatrix*)lua_touserdata(L,1);	// routing matrix userdata is the 1st argument
    //printf("Got routing matrix\n");
	// Get the coordinates of the segment from the stack
    int x1,y1,x2,y2;
    x1 = lua_tointeger(L,3);
    y1 = lua_tointeger(L,4);
    x2 = lua_tointeger(L,5);
    y2 = lua_tointeger(L,6);
    //printf("Got coordinates %d,%d,%d,%d\n",x1,y1,x2,y2);

	// Create a new segment structure
    struct seg *segment;
    segment = (struct seg*)malloc(sizeof(struct seg));
    //printf("Allocated memory for the segment\n");
    segment->x1 = x1;
    segment->y1 = y1;
    segment->x2 = x2;
    segment->y2 = y2;
    segment->prev = 0;

    // store the segment in the routing matrix structure
    if(x1==x2){
        // This is a vertical segment
        //printf("Add segment to vsegs..");
        segment->next = rm->vsegs;
        rm->vsegs = segment;
        if(segment->next)
            segment->next->prev = segment;
        //printf("DONE\n");
        // store segment into the registry referred by the key
        lua_pushvalue(L,2); // Push the key on top of the stack
        lua_pushlightuserdata(L,(void *)segment);    // push the pointer to the segment on the top of the stack
        lua_settable(L,LUA_REGISTRYINDEX);      // put it in the registry
        // store the key in the registry referred by the segment user data
        lua_pushlightuserdata(L,(void*)segment);
        lua_pushvalue(L,2);     // Push the key on top of the stack
        lua_settable(L,LUA_REGISTRYINDEX);
    }
    else if(y1==y2){
        // This is a horizontal segment
        //printf("Add segment to hsegs..");
        segment->next = rm->hsegs;
        rm->hsegs = segment;
        if(segment->next)
            segment->next->prev = segment;
        //printf("DONE\n");
        // store segment into the registry referred by the key
        lua_pushvalue(L,2); // Push the key on top of the stack
        lua_pushlightuserdata(L,(void *)segment);    // push the pointer to the segment on the top of the stack
        lua_settable(L,LUA_REGISTRYINDEX);      // put it in the registry
        // store the key in the registry referred by the segment user data
        lua_pushlightuserdata(L,(void*)segment);
        lua_pushvalue(L,2);     // Push the key on top of the stack
        lua_settable(L,LUA_REGISTRYINDEX);
    }
    fillLimits(L,rm,x1,y1);
    fillLimits(L,rm,x2,y2);
    lua_pushboolean(L,1);
    return 1;
}

// Function to remove a connector segment to the routing matrix
static int removeSegment(lua_State *L) {
	// parameters in lua passed like: function(rm,key)
    struct routingMatrix *rm;
    rm = (struct routingMatrix*)lua_touserdata(L,1);	// routing matrix userdata is the 1st argument
    //printf("Got routing matrix\n");
	// Get the pointer to the segment
	lua_pushvalue(L,2); // Push the key on top of the stack
	lua_gettable(L,LUA_REGISTRYINDEX);      // get the segment userdata from the registry
	struct seg *segment;
	segment = (struct seg*)lua_touserdata(L,-1);	// Get the pointer to the segment
	//printf("Got the segment pointer\n");
	if(!segment->prev){
        if(segment->next)
            segment->next->prev = 0;
		if(rm->hsegs == segment)
			rm->hsegs = segment->next;
		else if (rm->vsegs == segment)	// condition should ideally be redundant. This covers the case when blksegs segment is given by mistake
			rm->vsegs = segment->next;
	}
	else{
        if(segment->next)
            segment->next->prev = segment->prev;
		segment->prev->next = segment->next;
	}
	// Remove the entry from lua registry
	lua_pushvalue(L,2); // Push the key on top of the stack
	lua_pushnil(L);
	lua_settable(L,LUA_REGISTRYINDEX);      // clear the registry entry for the key
    // Remove the key pointed by the segment
    lua_pushlightuserdata(L,(void*)segment);
	lua_pushnil(L);
    lua_settable(L,LUA_REGISTRYINDEX);

	free(segment);	// free the memory from the segment structure
    lua_pushboolean(L,1);
    return 1;
}

// Function to add a blocking rectangle
static int addBlockingRectangle(lua_State *L){
    // parameters in lua passed like: function(rm,key,x1,y1,x2,y2)
    struct routingMatrix *rm = (struct routingMatrix*)lua_touserdata(L,1);	// routing matrix userdata is the 1st argument

	// Get the coordinates of the segment from the stack
    int x1,y1,x2,y2;
    x1 = lua_tointeger(L,3);
    y1 = lua_tointeger(L,4);
    x2 = lua_tointeger(L,5);
    y2 = lua_tointeger(L,6);
	// Create a new segment structure
    struct seg *segment;
    segment = (struct seg*)malloc(sizeof(struct seg));
    segment->x1 = x1;
    segment->y1 = y1;
    segment->x2 = x2;
    segment->y2 = y2;
    segment->prev = 0;

    // store the segment in the routing matrix structure
    segment->next = rm->blksegs;
    rm->blksegs = segment;
    if(segment->next)
        segment->next->prev = segment;
    // store segment into the registry referred by the key
    lua_pushvalue(L,2); // Push the key on top of the stack
    lua_pushlightuserdata(L,(void *)segment);    // push the pointer to the segment on the top of the stack
    lua_settable(L,LUA_REGISTRYINDEX);      // put it in the registry
    // store the key in the registry referred by the segment user data
    lua_pushlightuserdata(L,(void*)segment);
    lua_pushvalue(L,2);     // Push the key on top of the stack
    lua_settable(L,LUA_REGISTRYINDEX);

    fillLimits(L,rm,x1,y1);
    fillLimits(L,rm,x2,y2);
    lua_pushboolean(L,1);
    return 1;
}

// Function to remove a blocking rectangle from the routing matrix
static int removeBlockingRectangle(lua_State *L) {
	// parameters in lua passed like: function(rm,key)
    struct routingMatrix *rm;
    rm = (struct routingMatrix*)lua_touserdata(L,1);	// routing matrix userdata is the 1st argument
	// Get the pointer to the segment
	lua_pushvalue(L,2); // Push the key on top of the stack
	lua_gettable(L,LUA_REGISTRYINDEX);      // get the blocking rectangle userdata from the registry
	struct seg *segment;
	segment = (struct seg*)lua_touserdata(L,-1);	// Get the pointer to the segment
	if(!segment->prev){
        if(segment->next)
            segment->next->prev = 0;
		rm->blksegs = segment->next;
	}
	else{
        if(segment->next)
            segment->next->prev = segment->prev;
		segment->prev->next = segment->next;
	}
	// Remove the entry from lua registry
	lua_pushvalue(L,2); // Push the key on top of the stack
	lua_pushnil(L);
	lua_settable(L,LUA_REGISTRYINDEX);      // clear the registry entry for the key
    // Remove the key pointed by the segment
    lua_pushlightuserdata(L,(void*)segment);
	lua_pushnil(L);
    lua_settable(L,LUA_REGISTRYINDEX);

	free(segment);	// free the memory from the segment structure
    lua_pushboolean(L,1);
    return 1;
}

// Function to add port to routing matrix
static int addPort(lua_State *L) {
    // parameters in lua passed like: function(rm,key,x,y)
    struct routingMatrix *rm = (struct routingMatrix*)lua_touserdata(L,1);	// routing matrix userdata is the 1st argument

	// Get the coordinates of the port from the stack
    int x,y;
    x = lua_tointeger(L,3);
    y = lua_tointeger(L,4);
	// Create a new port structure
    struct port *prt;
    prt = (struct port*)malloc(sizeof(struct port));
    prt->x = x;
    prt->y = y;
    prt->prev = 0;
    // store the port in the routing matrix structure
    prt->next = rm->ports;
    rm->ports = prt;
    if (prt->next)
    prt->next->prev = prt;
    // store segment into the registry referred by the key
    lua_pushvalue(L,2); // Push the key on top of the stack
    lua_pushlightuserdata(L,(void *)prt);    // push the pointer to the segment on the top of the stack
    lua_settable(L,LUA_REGISTRYINDEX);      // put it in the registry
    // store the key in the registry referred by the prt user data
    lua_pushlightuserdata(L,(void*)prt);
    lua_pushvalue(L,2);     // Push the key on top of the stack
    lua_settable(L,LUA_REGISTRYINDEX);

    fillLimits(L,rm,x,y);
    lua_pushboolean(L,1);
    return 1;
}

// Function to remove a port from the routing matrix
static int removePort(lua_State *L) {
	// parameters in lua passed like: function(rm,key)
    struct routingMatrix *rm;
    rm = (struct routingMatrix*)lua_touserdata(L,1);	// routing matrix userdata is the 1st argument
	// Get the pointer to the port
	lua_pushvalue(L,2); // Push the key on top of the stack
	lua_gettable(L,LUA_REGISTRYINDEX);      // get the blocking rectangle userdata from the registry
	struct port *prt;
	prt = (struct port*)lua_touserdata(L,-1);	// Get the pointer to the port
	if(!prt->prev){
        if(prt->next)
            prt->next->prev = 0;
		rm->ports = prt->next;
	}
	else{
        if(prt->next)
            prt->next->prev = prt->prev;
		prt->prev->next = prt->next;
	}
	// Remove the entry from lua registry
	lua_pushvalue(L,2); // Push the key on top of the stack
	lua_pushnil(L);
	lua_settable(L,LUA_REGISTRYINDEX);      // clear the registry entry for the key
    // Remove the key pointed by the port
    lua_pushlightuserdata(L,(void*)prt);
	lua_pushnil(L);
    lua_settable(L,LUA_REGISTRYINDEX);

	free(prt);	// free the memory from the port structure
    lua_pushboolean(L,1);
    return 1;
}

// Reference https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
// To find orientation of ordered triplet (x1,y1; x2,y2; x3,y3).
// The function returns following values
//  0 --> All points are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
static int orientation(int x1,int y1,int x2,int y2,int x3,int y3){
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
    // for details of below formula.
    int val = (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2);

    if(val == 0)
        return 0;    // colinear
    else if(val > 0)
        return 1;   // clockwise
    else
        return 2;   // counter clockwise
}

static int larger(int a,int b){
    return (a>b?a:b);
}

static int smaller(int a,int b){
    return (a<b?a:b);
}
// Reference https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
// Given three colinear points x1,y1 xc,yc and x2,y2, the function checks if
// point xc,yc lies on line segment 'x1,y1 y2,y2'
static int onSegment(int x1,int y1,int xc,int yc,int x2,int y2) {
    if (xc <= larger(x1, x2) && xc >= smaller(x1, x2) &&
        yc <= larger(y1, y2) && yc >= smaller(y1, y2)) {
       return 1;
    }
    return 0;
}

// Reference https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
// The main function that returns true if line segment 'p1q1' and 'p2q2' intersect.
// p1 = (x1,y1)
// q1 = (x2,y2)
// p2 = (x3,y3)
// q2 = (x4,y4)
// Returns 5 if they intersect
// Returns 1 if p2 lies on p1 q1
// Returns 2 if q2 lies on p1 q1
// Returns 3 if p1 lies on p2 q2
// Returns 4 if q1 lies on p2 q2
// Returns 0 if no intersection
static int doIntersect(int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4){
    // Find the four orientations needed for general and special cases
    int o1 = orientation(x1,y1, x2,y2, x3,y3);
    int o2 = orientation(x1,y1, x2,y2, x4,y4);
    int o3 = orientation(x3,y3, x4,y4, x1,y1);
    int o4 = orientation(x3,y3, x4,y4, x2,y2);

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(x1,y1, x3,y3, x2,y2)) {
        return 1;
    }

    // p1, q1 and q2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(x1,y1, x4,y4, x2,y2)) {
        return 2;
    }

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(x3,y3, x1,y1, x4,y4)) {
        return 3;
    }

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(x3,y3, x2,y2, x4,y4)) {
        return 4;
    }

    // General case
    if (o1 != o2 && o3 != o4) {
        return 5;
    }

    return 0;	// Doesn't fall in any of the above cases
}

static int portExists(struct routingMatrix *rm,int x,int y){
    struct port *prt = rm->ports;
    while (prt){
        if (prt->x == x && prt->y == y)
            return 1;
        prt = prt->next;
    }
    return 0;
}

// Function to perform the validate step calculation
static int validateStep(struct routingMatrix *rm,int x1,int y1,int x2,int y2,int dstX,int dstY){
    /*
-- dstX and dstY is the final destination
-- The rules in order are as follows:
--[[
    # Check whether we are crossing a blocking segment, stepping on it or none. Crossing a blocking segment is not allowed. Stepping may be allowed if it is a port and final destination.
    # Check if this is a horizontal move then it should not be overlapping any horizontal segment
    # Check if this is a vertical move then it should not be overlapping any vertical segment
    # Check if x2,y2 is a port and x2==dstX and y2=dstY. x2,y2 can only be a port if it is the destination and not crossing a blocking segment
    # Check if stepping on segment end points. If it is not the destination then it is not allowed.
]]
    */
    struct seg * segs;
    segs = rm->blksegs;
    while(segs){
        // Segment 1 is x1,y1 x1,y2 of blksegs
        int intersect = doIntersect(segs->x1,segs->y1,segs->x1,segs->y2,x1,y1,x2,y2); // p1,q1,p2,q2
        // doIntersect:
        // Returns 5 if they intersect
        // Returns 1 if p2 lies on p1 q1
        // Returns 2 if q2 lies on p1 q1
        // Returns 3 if p1 lies on p2 q2
        // Returns 4 if q1 lies on p2 q2
        // Returns 0 if no intersection
        if (intersect && intersect > 2)	// 3,4,5 are not allowed i.e. crossing the blk segment (5), blk segment end lies on the step line (3,4)
            return 0;   // return false
        if (intersect) {	// case 1 and 2
            // This has to be a port and final destination
            if (x2 == dstX && y2 == dstY && portExists(rm,x2,y2))
                return 1;   // return true
            else
                return 0;   // return false
        }
        // Segment 2 is x1,y1 x2,y1 of blksegs
        intersect = doIntersect(segs->x1,segs->y1,segs->x2,segs->y1,x1,y1,x2,y2); // p1,q1,p2,q2
        if(intersect && intersect > 2)	    // 3,4,5 are not allowed i.e. crossing the blk segment (5), blk segment end lies on the step line (3,4)
            return 0;   // return false
        if(intersect){  // case 1 and 2
            // This has to be a port and final destination
            if (x2 == dstX && y2 == dstY && portExists(rm,x2,y2))
                return 1;   // return true
            else
                return 0;   // return false
        }
        // Segment 3 is x1,y2 x2,y2 of blksegs
        intersect = doIntersect(segs->x1,segs->y2,segs->x2,segs->y2,x1,y1,x2,y2); // p1,q1,p2,q2
        if(intersect && intersect > 2)	    // 3,4,5 are not allowed i.e. crossing the blk segment (5), blk segment end lies on the step line (3,4)
            return 0;   // return false
        if(intersect){  // case 1 and 2
            // This has to be a port and final destination
            if (x2 == dstX && y2 == dstY && portExists(rm,x2,y2))
                return 1;   // return true
            else
                return 0;   // return false
        }
        // Segment 4 is x2,y1 x2,y2 of blksegs
        intersect = doIntersect(segs->x2,segs->y1,segs->x2,segs->y2,x1,y1,x2,y2); // p1,q1,p2,q2
        if(intersect && intersect > 2)	    // 3,4,5 are not allowed i.e. crossing the blk segment (5), blk segment end lies on the step line (3,4)
            return 0;   // return false
        if(intersect){  // case 1 and 2
            // This has to be a port and final destination
            if (x2 == dstX && y2 == dstY && portExists(rm,x2,y2))
                return 1;   // return true
            else
                return 0;   // return false
        }

        // Get the next blocking rectangle
        segs = segs->next;
    }   // while(segs) ends here
    int vmove,hmove;
    if(x1==x2)
        vmove = 1;
    else
        vmove = 0;
    if(y1==y2)
        hmove = 1;
    else
        hmove = 0;

    segs = rm->vsegs;
    while(segs){
        if (vmove && segs->x1 == x1 && ((y2 > smaller(segs->y1,segs->y2) && y2 < larger(segs->y1,segs->y2)) ||
          (y1 > smaller(segs->y1,segs->y2) && y1 < larger(segs->y1,segs->y2))))
            // cannot do vertical move on a vertical segment
            return 0;   // return false
        if (segs->x1 == x2 && (segs->y1 == y2  || segs->y2 == y2)) {
            // stepping on end point (only allowed if that is the destination)
            if(x2 == dstX && y2 == dstY)
                return 1;   // return true
            else
                return 0;   // return false
        }
        segs = segs->next;
    }   // while(segs) ends here
    segs = rm->hsegs;
    while(segs){
        if (hmove && segs->y1 == y1 && ((x2 > smaller(segs->x1,segs->x2) && x2 < larger(segs->x1,segs->x2)) ||
          (x1 > smaller(segs->x1,segs->x2) && x1 < larger(segs->x1,segs->x2))))
            // cannot do horizontal move on a horizontal segment
            return 0;   // return false
        if (segs->y1 == y2 && (segs->x1 == x2 || segs->x2 == x2)){
            // stepping on end point (only allowed if that is the destination)
            if(x2 == dstX && y2 == dstY)
                return 1;   // return true
            else
                return 0;   // return false
        }
        segs = segs->next;
    }   // while (segs) ends here

    if (portExists(rm,x2,y2)){
        if(x2 == dstX && y2 == dstY)
            return 1;   // return true
        else
            return 0;   // return false
    }   // if (portExists(rm,x2,y2)) ends
    return 1;   // return true
}

// function to check for a valid step using the routing matrix
static int validStep(lua_State *L){
    // The function call is made like: function(rm,x1,y1,x2,y2,dstX,dstY)
    struct routingMatrix *rm;
    rm = (struct routingMatrix*)lua_touserdata(L,1);	// routing matrix userdata is the 1st argument
	// Get the coordinates from the stack
    int x1,y1,x2,y2,dstX,dstY;
    x1 = lua_tointeger(L,2);
    y1 = lua_tointeger(L,3);
    x2 = lua_tointeger(L,4);
    y2 = lua_tointeger(L,5);
    dstX = lua_tointeger(L,6);
    dstY = lua_tointeger(L,7);
    if (validateStep(rm,x1,y1,x2,y2,dstX,dstY)){
        lua_pushboolean(L,1);   // return true
        return 1;
    }
    else{
        lua_pushboolean(L,0);   // return false
        return 1;
    }
}

// Function to collect the remove matrix structur
static int routingMatrixGC(lua_State *L) {
    struct routingMatrix *rm = (struct routingMatrix*)lua_touserdata(L,1);	// routing matrix userdata is the 1st argument
    // Free memory associated with all vsegs
    struct seg * segment = rm->vsegs;
    while(segment){
        // Get the key associated with the segment
        lua_pushlightuserdata(L,(void*)segment);
        lua_gettable(L,LUA_REGISTRYINDEX);      // get the key from the registry
        // Remove the entry from lua registry
        lua_pushnil(L);
        lua_settable(L,LUA_REGISTRYINDEX);      // clear the registry entry for the key
        // Remove the key pointed by the segment
        lua_pushlightuserdata(L,(void*)segment);
        lua_pushnil(L);
        lua_settable(L,LUA_REGISTRYINDEX);

        free(segment);	// free the memory from the segment structure

        segment = segment->next;
    }
    // Free memory associated with all hsegs
    segment = rm->hsegs;
    while(segment){
        // Get the key associated with the segment
        lua_pushlightuserdata(L,(void*)segment);
        lua_gettable(L,LUA_REGISTRYINDEX);      // get the key from the registry
        // Remove the entry from lua registry
        lua_pushnil(L);
        lua_settable(L,LUA_REGISTRYINDEX);      // clear the registry entry for the key
        // Remove the key pointed by the segment
        lua_pushlightuserdata(L,(void*)segment);
        lua_pushnil(L);
        lua_settable(L,LUA_REGISTRYINDEX);

        free(segment);	// free the memory from the segment structure

        segment = segment->next;
    }
    // Free memory associated with all blksegs
    segment = rm->blksegs;
    while(segment){
        // Get the key associated with the segment
        lua_pushlightuserdata(L,(void*)segment);
        lua_gettable(L,LUA_REGISTRYINDEX);      // get the key from the registry
        // Remove the entry from lua registry
        lua_pushnil(L);
        lua_settable(L,LUA_REGISTRYINDEX);      // clear the registry entry for the key
        // Remove the key pointed by the segment
        lua_pushlightuserdata(L,(void*)segment);
        lua_pushnil(L);
        lua_settable(L,LUA_REGISTRYINDEX);

        free(segment);	// free the memory from the segment structure

        segment = segment->next;
    }
    // Free the memory associated with ports
    struct port* prt= rm->ports;
    while(prt){
        // Get the key associated with the segment
        lua_pushlightuserdata(L,(void*)prt);
        lua_gettable(L,LUA_REGISTRYINDEX);      // get the key from the registry
        // Remove the entry from lua registry
        lua_pushnil(L);
        lua_settable(L,LUA_REGISTRYINDEX);      // clear the registry entry for the key
        // Remove the key pointed by the segment
        lua_pushlightuserdata(L,(void*)prt);
        lua_pushnil(L);
        lua_settable(L,LUA_REGISTRYINDEX);

        free(prt);	// free the memory from the segment structure

        prt = prt->next;
    }
    // Free memory associated with the routing matrix
    free(rm);
    return 0;
}

// Function to create and return a new routing Matrix structure
static int newRoutingMatrix(lua_State *L) {
	// Create a new routing matrix structure
	struct routingMatrix * rm;
	rm  = (struct routingMatrix*)malloc(sizeof(struct routingMatrix));
	rm->blksegs = 0;
	rm->hsegs = 0;
	rm->maxX = 0;
	rm->vsegs = 0;
	rm->maxY = 0;
	rm->minX = 0;
	rm->minY = 0;
	rm->ports = 0;

	lua_pushlightuserdata(L,(void *)rm);    // push the pointer on the top of the stack

    lua_newtable(L);    // metatable for rm with the functions -- maybe we can create a common table for all and put it in the registry and reuse it

    static const struct luaL_Reg funcs[] = {
        {"addSegment", addSegment},
        {"removeSegment", removeSegment},
        {"addBlockingRectangle", addBlockingRectangle},
        {"removeBlockingRectangle", removeBlockingRectangle},
        {"addPort", addPort},
        {"removePort", removePort},
        {"validStep", validStep},
        {NULL, NULL}
    };
	luaL_newlib(L, funcs); // Table with all the functions is now on top of the stack
	// create minX, maxX, minY and maxY in __index
	lua_pushinteger(L,0);
	lua_setfield(L,-2,"minX");
	lua_pushinteger(L,0);
	lua_setfield(L,-2,"minY");
	lua_pushinteger(L,0);
	lua_setfield(L,-2,"maxX");
	lua_pushinteger(L,0);
	lua_setfield(L,-2,"maxY");

	lua_setfield(L,-2,"__index");   // put the table with the functions into the metatable with key as __index
	lua_pushcfunction(L,routingMatrixGC);
	lua_setfield(L,-2,"__gc");		// set the __gc meta index in the metatable to the function routingMatrixGC
	lua_setmetatable(L,-2);     // set the table on top to be the metatable of rm (routing matrix userdata)

	// Now only the rm userdata is left on the top
	return 1;	// Return routing matrix userdata
}


int _EXPORT luaopen_luaglib_crouter(lua_State *L) {

    static const struct luaL_Reg funcs[] = {
        {"BFS", BFS},
        {"newRoutingMatrix", newRoutingMatrix},
        {NULL, NULL}
    };
	luaL_newlib(L, funcs); // Just returns the module as a table

	// Add the _VERSION key and version string into the module table
	lua_pushstring(L,"_VERSION");
	lua_pushstring(L,VERSION);
	lua_rawset(L,-3);

    return 1;
}

