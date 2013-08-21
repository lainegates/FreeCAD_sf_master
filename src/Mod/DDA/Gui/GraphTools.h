#pragma once
#include<vector>
#include <utility>
#include "FrameReader.h"

namespace DDAGui
{
	using namespace std;
	class DDAPoint
	{
	public:
		DDAPoint(double x , double y , int blockNo);
		DDAPoint(){}
	public:
		double x;
		double y;
		double speedX;
		double speedY;
		int blockNo;
	};

	class FixedPoint: public DDAPoint
	{
	public:
		FixedPoint(double x , double y , int blockNo); 
		FixedPoint();
	};

	class LoadingPoint: public DDAPoint
	{
	public:
		LoadingPoint(double x , double y , int blockNo);
		LoadingPoint();
	};

	class MeasuredPoint: public DDAPoint
	{
	public:
		MeasuredPoint(double x , double y , int blockNo);
		MeasuredPoint();
	public:
		double u;
		double v;
		double r;
		double stressX;
		double stressY;
		double stressXY;
	};


	class Block
	{
	public:
		vector<pair<double,double>> vertices;   // keep double precision
		int blockIdx;
		int startNo;
		int endNo;
		double parameters[8];
		double stressX;
		double stressY;
		double stressXY;
	};

	class FrameData
	{
	public:
		void reset();

	public:
        int frameNo;
		vector<Block> blocks;
		vector<FixedPoint> fixedPoints;
		vector<LoadingPoint> loadingPoints;
		vector<MeasuredPoint> measuredPoints;

	};


// 	#define DEBUGFRAMEDATATAKER

	class FrameDataTaker
	{
	public:
		FrameDataTaker();
		~FrameDataTaker();

	public:
		bool setFile(char* filename);
		bool setStepInterval(int stepInterval);
        bool hasNextFrame(){return _frameReader->hasNextFrame();}
		FrameData getNextGraph();
        void setCurrentFrameIdx(int frameIdx);

		const vector<int> getSchema();
		const vector<double> getWindowInfo();

	private:
		bool initBlocksSchema();
		bool parseBlocksSchema(const string& blocksSchema);
		bool parseBlocksVertices(const string& blocksData);
		bool parseBlocksStress(const string& blocksStress);
		bool parse1Point(istringstream& dataIn , DDAPoint& point);
		bool parsePoints(const string& pointsData);

	private:
		FrameData* _graph;
		FrameData* _originGraph; // the 0-th frame
		FrameReader* _frameReader;
	};
}