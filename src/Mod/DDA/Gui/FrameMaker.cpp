#include "FrameMaker.h"
#include <cmath>
#include <Inventor/SbTesselator.h>
#include <Inventor/nodes/SoText2.h>
#include <Inventor/nodes/SoFont.h>
#include <Inventor/nodes/SoTransform.h>
#include "Triangulate.h"

namespace DDAGui{
/////////////////////////////////////////////////
/////
/////  graph block
/////
////////////////////////////////////////////////
using namespace std;

GraphBlock::GraphBlock()
{
	////////////////////////////////
	// block graph
	////////////////////////////////
	SoSeparator* polygon = new SoSeparator();
	polygon->ref();


	norm = new SoNormal();
	norm->ref();
	norm->vector.setValue(0 , 0 , 1.0);
    polygon->addChild(norm);

	normalBinding = new SoNormalBinding();
	normalBinding->ref();
	normalBinding->value = SoNormalBinding.PER_FACE;
	polygon->addChild(normalBinding);

	polygonColors = new SoMaterial() ;
	polygonColors->ref();
	polygon->addChild(polygonColors);

	materialBinding = new SoMaterialBinding();
	materialBinding->ref();
	materialBinding->value = SoMaterialBinding.PER_VERTEX;
    polygon->addChild(materialBinding);

	polygonCoords = new SoCoordinate3();
	polygonCoords->ref();
	polygon->addChild(polygonCoords);

	faceset = new SoIndexedFaceSet();
	faceset->ref();
	polygon->addChild(faceset);

	////////////////////////////////
	// block boundary lines
	////////////////////////////////
	SoSeparator* lines = new SoSeparator() ;

	lineColor = new SoBaseColor();
	lineColor->rgb.setValue(0,0,0);
    lineColor->ref();
	lines->addChild(lineColor);

	lineCoords = new SoCoordinate3();
	lineCoords->ref();
	lines->addChild(lineCoords);

	lineset = new SoLineSet();
	lineset->ref();
	lines->addChild(lineset);

	//////////////////////////////////
	// whole block graph
	//////////////////////////////////
	rootGraphBlock = new SoSeparator();
	rootGraphBlock->ref();
    rootGraphBlock->addChild(lines);
 	rootGraphBlock->addChild(polygon);
}

GraphBlock::~GraphBlock()
{

	// block graph
// 	polygon->removeAllChildren();
	norm->unref();
	normalBinding->unref();
	polygonColors->unref();
	materialBinding->unref();
	polygonCoords->unref();
	faceset->unref();
//     polygon->unref();

	// block boundary lines
    lineColor->unref();
	lineCoords->unref();
	lineset->unref();

    rootGraphBlock->unref();
}

////////////////////////////////////////////////
////
////  measured point graph
////
////////////////////////////////////////////////
GraphMeasuredPoints::GraphMeasuredPoints()
{
	rootGraphMPts = new SoSeparator() ;
	rootGraphMPts->ref();

    lineColor = new SoBaseColor();
    lineColor->rgb.setValue(1.0,0,0);
    lineColor->ref();
    rootGraphMPts->addChild(lineColor);


//     float pts[][3] = {{100.0 , 100.0 , 1} , {100.0 , 0.0 , 1} , {0.0 , 100.0 , 1}};
	meausuredPointsCoords = new SoCoordinate3() ;
	meausuredPointsCoords->ref();
	rootGraphMPts->addChild(meausuredPointsCoords);
//     meausuredPointsCoords->point.setValues(0 , 3 , pts);

	measuredPointsLineset = new SoLineSet() ;
	measuredPointsLineset->ref();
	rootGraphMPts->addChild(measuredPointsLineset);
//     measuredPointsLineset->numVertices.setValue(3);

	mpts = new vector<vector<MeasuredPoint>>();
}

GraphMeasuredPoints::~GraphMeasuredPoints()
{
	delete mpts;
	rootGraphMPts->unref();
    lineColor->unref();
	meausuredPointsCoords->unref();
	measuredPointsLineset->unref();
}

/////////////////////////////////////////////////
/////
/////  scaler
/////
////////////////////////////////////////////////
SoSeparator* Scaler::rootGraphBlock = NULL;
int Scaler::scale = 10;
SoSeparator* Scaler::createScaler( const std::vector<double>& windowInfo , int scale)
{
    Scaler::scale = scale;

    if (rootGraphBlock)
        rootGraphBlock->removeAllChildren();
    else
        rootGraphBlock = new SoSeparator;

    double width = windowInfo[1]-windowInfo[0];
    double height = windowInfo[3]-windowInfo[2];
    double centerX = (windowInfo[0]+windowInfo[1])/2;
    double centerY = (windowInfo[2]+windowInfo[3])/2;
    double LBX = centerX - width*0.7;
    double LBY = centerY - height*0.3;
    SoTransform* trans = new SoTransform;
    trans->translation.setValue(LBX , LBY , 0);
    rootGraphBlock->addChild(trans);

    rootGraphBlock->addChild(_createScalerBar(windowInfo));
    rootGraphBlock->addChild(_createTexts(windowInfo));
    return rootGraphBlock;
}

SoSeparator* Scaler::_createScalerBar( const std::vector<double>& windowInfo )
{
    SoSeparator* root = new SoSeparator;

    SoNormal* norm = new SoNormal;
    SbVec3f norms[] = {SbVec3f(0,0,1.0), SbVec3f(0,0,1.0) , SbVec3f(0,0,1.0) , SbVec3f(0,0,1.0)};
    norm->vector.setValues(0,4,norms);
    root->addChild(norm);

    SoNormalBinding* normalBinding = new SoNormalBinding();
    normalBinding->value = SoNormalBinding.PER_FACE;
    root->addChild(normalBinding);


    float colors[16][3] = {  {0,0,1} , {0,0,1} , {0,1,1} , {0,1,1} , {0,1,1} , {0,1,1} , {0,1,0} , {0,1,0} 
        , {0,1,0} , {0,1,0} , {1,1,0} , {1,1,0} , {1,1,0} , {1,1,0} , {1,0,0} , {1,0,0} };

    SoMaterial* polygonColors = new SoMaterial() ;
    polygonColors->diffuseColor.setValues(0 , 16 , colors);
    root->addChild(polygonColors);

    SoMaterialBinding* materialBinding = new SoMaterialBinding();
    materialBinding->value = SoMaterialBinding.PER_VERTEX;
    root->addChild(materialBinding);

    double width = windowInfo[1]-windowInfo[0];
    double height = windowInfo[3]-windowInfo[2];
    double centerX = (windowInfo[0]+windowInfo[1])/2;
    double centerY = (windowInfo[2]+windowInfo[3])/2;

    double detY = 0.6*height;
    double detX = detY*0.1; 

    float pts[16][3] = { {0,0,0 }  ,      {detX,0,0 } ,       {detX,detY*0.25,0}, {0,detY*0.25,0}\
                        ,{0,detY*0.25,0 },{detX,detY*0.25,0 },{detX,detY*0.5 ,0}, {0,detY*0.5,0 }\
                        ,{0,detY*0.5 ,0 },{detX,detY*0.5 ,0 },{detX,detY*0.75,0}, {0,detY*0.75,0}
                        ,{0,detY*0.75,0 },{detX,detY*0.75,0 },{detX,detY,0 },     {0,detY,0 }};

    SoCoordinate3* polygonCoords = new SoCoordinate3();
    polygonCoords->point.setValues(0 , 16 , pts);
    root->addChild(polygonCoords);

    SoFaceSet* face = new SoFaceSet();
    int nums[] = {4 ,4 , 4, 4};
    face->numVertices.setValues(0 , 4 , nums);
    root->addChild(face);
    return root;
}

SoSeparator* Scaler::_createTexts( const std::vector<double>& windowInfo )
{
    SoSeparator* textRoot = new SoSeparator;
    double width = windowInfo[1]-windowInfo[0];
    double height = windowInfo[3]-windowInfo[2];
    double detY = height;
    double detX = 0.12*height;

    double disInterval = height/float(scale);


    SoFont* font = new SoFont;
    font->name.setValue("Times-Roman");
    font->size.setValue(15);

    for( int i=0 ; i<10 ; i++)
    {
        SoSeparator* g = new SoSeparator;

        SoTransform* t = new SoTransform;
        t->translation.setValue(detX , detY*0.06*i , 0);
        g->addChild(t);

        SoBaseColor* c = new SoBaseColor;
        c->rgb.setValue(0,0,0);
        g->addChild(c);

        g->addChild(font);

        SoText2* text = new SoText2;
        char str[100];
        sprintf(str , "%lf" , disInterval*0.1*i);
        text->string.setValue(str);
        g->addChild(text);

        textRoot->addChild(g);
    }
    return textRoot;
}

/////////////////////////////////////////////////
/////
/////  scene graph
/////
////////////////////////////////////////////////

SceneGraph::SceneGraph()
{
//	blocks = new vector<GraphBlock>();
    blocks = NULL;
    graphMeasuredPoints = NULL;

	rootScene = new SoSeparator();
	rootScene->ref();
}

SceneGraph::~SceneGraph()
{
    rootScene->unref();
	delete []blocks ;
	delete graphMeasuredPoints ;
}

void SceneGraph::resetRootScene(const vector<int> schema , const vector<double> windowInfo )
{
    rootScene->removeAllChildren();

    /////////////////////////////////
    //  blocks
    /////////////////////////////////
    blocksNum = schema[0];
    if(blocks){
        delete []blocks;
        blocks = NULL;
    }

    blocks = new GraphBlock[blocksNum];
    for(int i=0 ; i<schema[0] ; i++) 
    {
        assert(blocks[i].rootGraphBlock);
        rootScene->addChild(blocks[i].rootGraphBlock);
    }

    /////////////////////////////////
    //  measured points
    /////////////////////////////////
    if(graphMeasuredPoints){
        delete graphMeasuredPoints; 
        graphMeasuredPoints=NULL;
    }
    graphMeasuredPoints = new GraphMeasuredPoints;
    assert(graphMeasuredPoints);
 	graphMeasuredPoints->mpts->resize(schema[3]);
  	rootScene->addChild(graphMeasuredPoints->rootGraphMPts);

    /////////////////////////////////
    //  scaler
    /////////////////////////////////
    rootScene->addChild(Scaler::createScaler(windowInfo , 10));
}

/////////////////////////////////////////////////
/////
/////  block updater
/////
////////////////////////////////////////////////
vector<double> BlockUpdatater::windowInfo = vector<double>();
double BlockUpdatater::scale = 0;  // scale for color
FrameData* BlockUpdatater::originalGraph = NULL;
vector<SbVec3f> BlockUpdatater::coords = vector<SbVec3f>();
int BlockUpdatater::tmpCount=0;

void BlockUpdatater::reset(vector<double> windowInfo , double scale , FrameData* originalGraph)
{
	BlockUpdatater::windowInfo = windowInfo;
	BlockUpdatater::scale = scale;
	BlockUpdatater::originalGraph = originalGraph;
    _calcWindowInfoFromFrame();
}

void BlockUpdatater::_calcWindowInfoFromFrame()
{
    assert(originalGraph!=NULL && originalGraph->blocks.size()>0) ;
    const vector<Block>& blocks = originalGraph->blocks;
    windowInfo[0] = windowInfo[1] = originalGraph->blocks[0].vertices[0].first;
    windowInfo[2] = windowInfo[3] = originalGraph->blocks[0].vertices[0].second;
    for(int i=0 ; i<blocks.size() ; i++)
    {
        const vector<pair<double,double>>& pts = blocks[i].vertices;
        for(vector<pair<double,double>>::const_iterator it = pts.begin() ; it!=pts.end() ; it++){
            if(it->first < windowInfo[0]) windowInfo[0] = it->first;
            if(it->first > windowInfo[1]) windowInfo[1] = it->first;
            if(it->second< windowInfo[2]) windowInfo[2] = it->second;
            if(it->second> windowInfo[3]) windowInfo[3] = it->second;
        }
    }
}

void BlockUpdatater::update(FrameData* frame , GraphBlock* graphBlock , int blockIdx)
{
 	_updatePolygon(frame , graphBlock , blockIdx);
	_updateLines(frame , graphBlock , blockIdx);
}


void BlockUpdatater::_updatePolygonColor(FrameData* frame , GraphBlock* graphBlock , int blockIdx , SbColor* colors)
{
	const vector<pair<double,double>>& pts1 = frame->blocks[blockIdx].vertices;
	const vector<pair<double,double>>& originPts = originalGraph->blocks[blockIdx].vertices;

	assert(pts1.size()==originPts.size());
	for(int i=0 ; i<pts1.size();i++)
	{
		const pair<double , double>& p1= pts1[i];
		const pair<double , double>& p2= originPts[i];
		double tmp = sqrt((p1.first-p2.first)*(p1.first-p2.first) 
			+(p1.second-p2.second)*(p1.second-p2.second));
		double t1 = tmp/scale;
		if (t1>1) t1=1;
		if(t1>0.75)                 { t1-=0.75 ; colors[i].setValue( 1    , 1-4*t1 , 0      );}
		else if(t1<=0.75 && t1>0.5) { t1-=0.5  ; colors[i].setValue( 4*t1 , 1      , 0      );}
		else if(t1<=0.5 && t1>0.25) { t1-=0.25 ; colors[i].setValue( 0    , 1      , 1-4*t1 );}
		else						{			 colors[i].setValue( 0    , 4*t1   , 1      );}
	}
#ifdef DEBUGFRAMEMAKER
	char tmpStr[200];
	sprintf(tmpStr , "%d colors got.\n" , colors);
#endif
}

int BlockUpdatater::_getVectorUp(const SbVec3f& p1 , const SbVec3f& p2 , const SbVec3f& p3)
{
// 	double ax = p1[0]-p2[0];
// 	double ay = p1[1]-p2[1];
// 	double bx = p2[0]-p3[0];
// 	double by = p2[1]-p3[1];
//	double tmp = ax*by-ay*bx;  // cross vector v1*v2
//  return tmp>0?1:-1
	return (p1[0]-p2[0])*(p2[1]-p3[1])-(p1[1]-p2[1])*(p2[0]-p3[0])>0?1:-1;
}


bool BlockUpdatater::_ifConcave(const vector<SbVec3f>& pts)
{
	assert(pts.size()>2);
	int size = pts.size();
	if(pts[0]==pts[size-1])
		size-=1;
	
	double res = _getVectorUp(pts[0] , pts[1] , pts[2]);
	for( int i=0 ;i<size; i++)
	{
		if( _getVectorUp(pts[i],pts[(i+1)%size] , pts[(i+2)%size])!=res)
			return false;
	}
	return true;
}

void BlockUpdatater::_triangulationCallback(void * v0, void * v1, void * v2, void * cbdata)
{
	SbVec3f * vtx0 = (SbVec3f *)v0;
	SbVec3f * vtx1 = (SbVec3f *)v1;
	SbVec3f * vtx2 = (SbVec3f *)v2;

	coords.push_back(*vtx0);
	coords.push_back(*vtx1);
	coords.push_back(*vtx2);
#ifdef DEBUGFRAMEMAKER
    msg("one triangle is tesselated out.\n");
#endif
}

void BlockUpdatater::_triangulate(vector<SbVec3f>& pts)
{
	assert(pts.size()>3);
	coords.clear();

    int size = pts.size();
    SbVec3f p1 = pts[0];
    SbVec3f p2 = pts.back();

//     char str[200];
//     sprintf(str , "(%f , %f)<--->(%f , %f)\n" , p1[0] , p1[1] , p2[0] , p2[1]);
//     msg(str);
// 
//     if(p1[0]==p2[0] && p1[1]==p2[1]) 
//     {size-=1; msg("first and last point are same");}
// 

    Vector2dVector vec2d;
    vec2d.resize(size);
    for( int i=0; i<size ; i++)
        vec2d[i].Set(pts[i][0],pts[i][1]);

    Vector2dVector res;
    Triangulate::Process( vec2d , res);
    coords.resize(res.size());
    for( int i=0; i<res.size() ; i++)
    {
        coords[i][0] = res[i].GetX();
        coords[i][1] = res[i].GetY();
        coords[i][2] = -1;
    }

// 	SbTesselator tesselator(_triangulationCallback , NULL);
// 	tesselator.beginPolygon();
// 	for(int i=0 ; i<size ; i++)
// 		tesselator.addVertex(pts[i] , &pts[i]);
// 	tesselator.endPolygon();
	assert(int(coords.size())%3==0);
// #ifdef DEBUGFRAMEMAKER
// 	char tmpStr[200];
// 	sprintf(tmpStr , "tesselate done. %d triangles are tesselated out.\n", int(coords.size())/3);
// 	msg(tmpStr);
// #endif
}

int find_pos(const vector<SbVec3f>& pts , SbVec3f p)
{
	// if found , return n (n>0) , else return -1
	for(int i=0 ; i<pts.size() ; i++)
		if( pts[i]==p )
			return i;
	return -1;
}

void BlockUpdatater::_calcColors4ConcavePolygon(const vector<SbVec3f>& pts , SbColor* colors , SbColor* resultColors)
{
	// calculate colors for points after tesselation

	double* percents = new double[coords.size()+1];
	double sum = 0;

	for(int i=0 ; i<coords.size() ; i++)
	{
		int pos = find_pos(pts , coords[i]);
		if( pos>0 ){ resultColors[i]=colors[pos]; continue;}
		else // the point is not in pts
		{
			// calculate distance to every point in original convex polygon
			for(int j=0 ; j<pts.size() ; j++)
			{
				percents[j] = (coords[i][0]-pts[j][0])*(coords[i][0]-pts[j][0])
					+(coords[i][1]-pts[j][1])*(coords[i][1]-pts[j][1]);
				sum+=percents[j];
			}
			for(int j=0 ; j<pts.size() ; j++)
				if( percents[j]==0)	percents[j]=sum;
				else percents[j] = sum/percents[j];

			// get percent for every point in original convex polygon to the current coords[i]
			sum=0;
			for(int j=0 ; j<pts.size() ; j++)
				sum+=percents[j];
			for(int j=0 ; j<pts.size() ; j++)
				percents[j]/=sum;

			// get color for this point
			SbColor& color = resultColors[i];
			color[0]=color[1]=color[2]=0;
			for(int j=0 ; j<pts.size() ; j++)
				for( int k=0 ; k<3 ; k++)
					color[k]+=percents[j]*colors[j][k];
		}
	}
	delete []percents;
}

void BlockUpdatater::_updatePolygon(FrameData* frame , GraphBlock* graphBlock , int blockIdx)
{
	const vector<pair<double,double>>& tmpPts = frame->blocks[blockIdx].vertices;
    assert(tmpPts.size()>2);

	SbColor* colors = new SbColor[tmpPts.size()+1];
	_updatePolygonColor(frame , graphBlock , blockIdx , colors);
	
	SbVec3f* pts = new SbVec3f[tmpPts.size()+1];
	for(int i=0;i<tmpPts.size();i++)
	{
		pts[i][0] = tmpPts[i].first;
		pts[i][1] = tmpPts[i].second;
		pts[i][2] = -1;
	}
	vector<SbVec3f> vecPts(pts , pts+tmpPts.size());
	assert(vecPts.size()==tmpPts.size());


//     char tmpStr[200];
	if(_ifConcave(vecPts))
	{
        //sprintf(tmpStr , "\t%d-th polygon is convex.\n" , blockIdx);
        //msg(tmpStr);

        graphBlock->norm->vector.setNum(tmpPts.size());
        SbVec3f * p = graphBlock->norm->vector.startEditing();
        for(int i=0 ; i<tmpPts.size(); i++) p[i].setValue(0,0,1.0);
        graphBlock->norm->vector.finishEditing();

        graphBlock->polygonCoords->point.setValues(0 , tmpPts.size() , pts);
		
        graphBlock->faceset->coordIndex.setNum(tmpPts.size()+1);
        int* pIdx = graphBlock->faceset->coordIndex.startEditing();
        for(int i=0; i<tmpPts.size(); i++) pIdx[i]=i;
        pIdx[tmpPts.size()] = SO_END_FACE_INDEX;
        graphBlock->faceset->coordIndex.finishEditing();

		graphBlock->polygonColors->diffuseColor.setValues(0 , tmpPts.size() , colors);
	}
	else // the polygon is concave
	{
		_triangulate(vecPts);
        graphBlock->polygonCoords->point.setNum(coords.size());
        SbVec3f* pCoord = graphBlock->polygonCoords->point.startEditing();
        for (int i=0 ; i<coords.size() ; i++)
            pCoord[i].setValue(coords[i][0] , coords[i][1] , coords[i][2]);
        graphBlock->polygonCoords->point.finishEditing();

        assert(int(coords.size())%3==0);
        const int size = int(coords.size())/3;
        graphBlock->faceset->coordIndex.setNum(size*4);
        int* pIdx = graphBlock->faceset->coordIndex.startEditing();
        for(int i=0 , j=0; i<coords.size(); i++)
        {
            pIdx[j++]=i;
            if((i+1)%3==0)
                pIdx[j++] = SO_END_FACE_INDEX;
        }
        graphBlock->faceset->coordIndex.finishEditing();

        graphBlock->norm->vector.setNum(coords.size());
        SbVec3f * p = graphBlock->norm->vector.startEditing();
        for(int i=0 ; i<coords.size(); i++) p[i].setValue(0,0,1.0);
        graphBlock->norm->vector.finishEditing();
		
		SbColor *TColors = new SbColor[coords.size()+1];
		_calcColors4ConcavePolygon(vecPts , colors , TColors);
		graphBlock->polygonColors->diffuseColor.setValues(0 , coords.size() , TColors);
		delete []TColors;
	}
// 
//     sprintf(tmpStr , "%d-th polygon was parsed out.\n" , blockIdx);
//     msg(tmpStr);
// 
	delete []pts;
	delete []colors;
}
void BlockUpdatater::_updateLines(FrameData* frame , GraphBlock* graphBlock , int blockIdx)
{
	const vector<pair<double , double>>& pts = frame->blocks[blockIdx].vertices;
	const int size = pts.size();

	SbVec3f* TmpPts = new SbVec3f[size+1];
	for( int i=0 ; i<size ; i++)
	{
		TmpPts[i][0] = pts[i].first;
		TmpPts[i][1] = pts[i].second;
		TmpPts[i][2] = 0;
	}
    TmpPts[size] = TmpPts[0];

	if( pts.front()==pts.back())
	{
		graphBlock->lineCoords->point.setValues(0 , size , TmpPts);
        int nums[] = {size};
		graphBlock->lineset->numVertices.setValues(0 , 1 , nums);
	}
	else
	{
		graphBlock->lineCoords->point.setValues(0 , size+1 , TmpPts);
        int nums[] = {size+1};
		graphBlock->lineset->numVertices.setValues(0 , 1 , nums);
	}
    delete []TmpPts;
}
//////////////////////////////////////////////////////
/////
///// Points Updater
/////
//////////////////////////////////////////////////////
void PointsUpdater::update(FrameData* frame , SceneGraph* sceneGraph)
{
    const int num = frame->measuredPoints.size();
    GraphMeasuredPoints* graphMPts = sceneGraph->graphMeasuredPoints;
    const int steps = (*(graphMPts->mpts))[0].size();

    SoCoordinate3* coords = graphMPts->meausuredPointsCoords;
    coords->point.setNum(num*(steps+1));
    SbVec3f* p = coords->point.startEditing();
    for( int i=0 ; i<num ; i++)
    {
        vector<MeasuredPoint>& tmpPts = (*(graphMPts->mpts))[i];
        tmpPts.push_back( frame->measuredPoints[i] );

        for(int j=0 ; j<steps+1 ; j++)
            p[i*(steps+1)+j].setValue(tmpPts[j].x , tmpPts[j].y , 0.5);
    }
    coords->point.finishEditing();
    
    graphMPts->measuredPointsLineset->numVertices.setNum(num);
    int* pNum = graphMPts->measuredPointsLineset->numVertices.startEditing();
    for(int i=0 ; i<num ; i++)  pNum[i] = steps+1;
    graphMPts->measuredPointsLineset->numVertices.finishEditing();
}



/////////////////////////////////////////////////
/////
/////  frame maker
/////
////////////////////////////////////////////////



FrameMaker::FrameMaker(void)
{
	_rootSceneGraph = new SceneGraph();
	_scale = 2;
}

FrameMaker::~FrameMaker(void)
{
	delete _rootSceneGraph;
}

bool FrameMaker::reset(FrameDataTaker* reader)
{
	_firstFrame = reader->getNextGraph();
#ifdef DEBUGFRAMEMAKER
    msg("first frame get done.\n");
#endif
    BlockUpdatater::reset(reader->getWindowInfo() , _scale , &_firstFrame);
#ifdef DEBUGFRAMEMAKER
    msg("frameMaker reset done.\n");
#endif
    _rootSceneGraph->resetRootScene(reader->getSchema() , BlockUpdatater::getWindowInfo());
#ifdef DEBUGFRAMEMAKER
    msg("root secene reset done.\n");
#endif
	updateGraph(_firstFrame);
#ifdef DEBUGFRAMEMAKER
    msg("first frame draw done.\n");
#endif

	return true;
}

bool FrameMaker::setRootSceneNode(SoSeparator* node)
{
	_freeCADrootNode = node;
	_freeCADrootNode->addChild(_rootSceneGraph->rootScene);
#ifdef DEBUGFRAMEMAKER
	msg("frameMaker set root freecad node done.\n");
#endif
	return true;
}

bool FrameMaker::updateGraph(FrameData frame)
{
	assert(frame.blocks.size()>0);
	int size = frame.blocks.size();
	for( int i=0 ; i<size ; i++)
        BlockUpdatater::update(&frame , &(_rootSceneGraph->blocks[i]) , i);

    PointsUpdater::update(&frame , _rootSceneGraph);

#ifdef DEBUGFRAMEMAKER
	msg("frameMaker parse one graph done.\n");
#endif
    return true;
}
}   // namespace DDAGui