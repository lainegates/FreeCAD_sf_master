#pragma once

#include <Windows.h>

# include <Inventor/SbVec3f.h>
# include <Inventor/nodes/SoBaseColor.h>
# include <Inventor/nodes/SoCoordinate3.h>
# include <Inventor/nodes/SoLightModel.h>
# include <Inventor/nodes/SoMaterial.h>
# include <Inventor/nodes/SoMaterialBinding.h>
# include <Inventor/nodes/SoSeparator.h>
# include <Inventor/nodes/SoTransform.h>
# include <Inventor/nodes/SoRotation.h>
# include <Inventor/nodes/SoCoordinate3.h>
# include <Inventor/nodes/SoDrawStyle.h>
# include <Inventor/nodes/SoIndexedFaceSet.h>
# include <Inventor/nodes/SoShapeHints.h>
# include <Inventor/nodes/SoPointSet.h>
# include <Inventor/nodes/SoPolygonOffset.h>
# include <Inventor/nodes/SoNormal.h>
# include <Inventor/nodes/SoNormalBinding.h>

#include <vector>
#include <Inventor\nodes\SoIndexedFaceSet.h>
#include <Inventor\nodes\SoFaceSet.h>
#include "Inventor\nodes\SoLineSet.h"
#include "GraphTools.h"

namespace DDAGui{
	using namespace std;
	class GraphBlock
	{
	public:
		GraphBlock();
		~GraphBlock();

	public:
		// whole graph scene node
		SoSeparator* rootGraphBlock;

		// block graph
// 		SoSeparator* polygon;
		SoNormal* norm;
		SoNormalBinding* normalBinding;
		SoMaterial* polygonColors;
		SoMaterialBinding* materialBinding;
		SoCoordinate3* polygonCoords;
		SoIndexedFaceSet* faceset;

		// block boundary lines
        SoBaseColor* lineColor;
		SoCoordinate3* lineCoords;
		SoLineSet* lineset;

	};

	class GraphMeasuredPoints
	{
	public:
		GraphMeasuredPoints();
		~GraphMeasuredPoints();

//     public:
//         void setMeasuredPointsNum(int num){mpts->resize(num);}
// 
	public:
		SoSeparator* rootGraphMPts;
		SoCoordinate3* meausuredPointsCoords;
		SoLineSet* measuredPointsLineset;
        SoBaseColor* lineColor;

		vector<vector<MeasuredPoint>>* mpts;
	};

	class Scaler
	{
	private:
		Scaler();
		~Scaler(){}

	public:
		static SoSeparator* createScaler( const std::vector<double>& windowInfo , int scale);
     
    private:
        static SoSeparator* _createScalerBar(const std::vector<double>& windowInfo);
        static SoSeparator* _createTexts(const std::vector<double>& windowInfo);

	private:
        // whole graph scene node
        static SoSeparator* rootGraphBlock;
        static int scale;
	};


	class SceneGraph
	{
	public:
		SceneGraph();
		~SceneGraph();

	public:
		void resetRootScene(const vector<int> graphSchema , const vector<double> windowInfo );

	public:
		GraphBlock *blocks;
        int blocksNum;
		SoSeparator* rootScene;
		GraphMeasuredPoints* graphMeasuredPoints;
	};

	class BlockUpdatater
	{
    private:
        BlockUpdatater();
        ~BlockUpdatater();
	public:
		static void reset(vector<double> windowInfo , double scale , FrameData* originalGraph);
		static void update(FrameData* frame , GraphBlock* graphBlock , int blockIdx);
        static const vector<double>& getWindowInfo(){ return windowInfo;}
    private:
		static void _updatePolygonColor(FrameData* frame , GraphBlock* graphBlock , int blockIdx , SbColor* colors);
		static void _updatePolygon(FrameData* frame , GraphBlock* graphBlock , int blockIdx);
		static void _updateLines(FrameData* frame , GraphBlock* graphBlock , int blockIdx);
		static bool _ifConcave(const vector<SbVec3f>& pts);
		static int _getVectorUp(const SbVec3f& p1 , const SbVec3f& p2 , const SbVec3f& p3);
		static void _triangulate(vector<SbVec3f>&);
		static void _triangulationCallback(void * v0, void * v1, void * v2, void * cbdata);
		static void _calcColors4ConcavePolygon(const vector<SbVec3f>& pts , SbColor* colors, SbColor* resultColors);
        static void _calcWindowInfoFromFrame();
	private:
		static vector<double> windowInfo;
		static double scale;  // scale for color
		static FrameData* originalGraph;
		static vector<SbVec3f> coords;
		static int tmpCount; // used when triangulation
	};

    class PointsUpdater
    {
    private:
        PointsUpdater();
        ~PointsUpdater();
    public:
        static void update(FrameData* frame , SceneGraph* sceneGraph);
    };

// 	#define DEBUGFRAMEMAKER

	class FrameMaker
	{
	public:
		FrameMaker(void);
		~FrameMaker(void);

	public:
		bool reset(FrameDataTaker* reader);
		bool setRootSceneNode(SoSeparator* node);
		bool updateGraph(FrameData frame);

    private:
		SoSeparator* _freeCADrootNode;
		SceneGraph* _rootSceneGraph;
		FrameData _firstFrame;
		double _scale;
	};
}// DDAGui