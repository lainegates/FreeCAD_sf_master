
#include <sstream>
#include <assert.h>
#include "GraphTools.h"
using namespace DDAGui;

////////////////////////////////
///
///  find n-th '\n
///
////////////////////////////////
int find_nth_endl(const string& str , int startPostion , int n)
{
	int num=0;
	int pos=startPostion;

	while(num<n)
	{
        int t = pos;
		pos = str.find('\n' , pos+1);
		if(pos==string::npos)
            return -1;
		num++;
	}
	return pos;
}


////////////////////////////////
///////   DDA Data tools
////////////////////////////////

DDAPoint::DDAPoint(double x , double y , int blockNo)
{
	this->x = x;
	this->y = y;
	this->blockNo = blockNo;
}

FixedPoint::FixedPoint(double x , double y , int blockNo):DDAPoint(x , y , blockNo){}
FixedPoint::FixedPoint():DDAPoint(){}

LoadingPoint::LoadingPoint(double x , double y , int blockNo):DDAPoint(x , y , blockNo){}
LoadingPoint::LoadingPoint():DDAPoint(){}

MeasuredPoint::MeasuredPoint(double x , double y , int blockNo):DDAPoint(x , y , blockNo){}
MeasuredPoint::MeasuredPoint():DDAPoint(){}

void FrameData::reset()
{
	blocks.clear();
	fixedPoints.clear();
	loadingPoints.clear();
	measuredPoints.clear();
}


/////////////////////////////////////////////////////////
/////       
/////	FrameDataTaker
/////
/////////////////////////////////////////////////////////


FrameDataTaker::FrameDataTaker()
{
	_graph = new FrameData();
	_frameReader = new FrameReader();
}

FrameDataTaker::~FrameDataTaker()
{
	if(_graph){ delete _graph; _graph=NULL;}
	if(_frameReader){ delete _frameReader; _frameReader=NULL;}
}

const vector<int> FrameDataTaker::getSchema()
{
	return _frameReader->getSchema();
}
const vector<double> FrameDataTaker::getWindowInfo()
{
	return _frameReader->getWindowInfo();
}

bool FrameDataTaker::setFile(char* filename)
{
	return _frameReader->setFile(filename);
}

bool FrameDataTaker::setStepInterval(int stepInterval)
{
	//_frameReader-
	return true;
}
void FrameDataTaker::setCurrentFrameIdx(int frameIdx)
{
    bool res = _frameReader->setCurrentFrameIndex(frameIdx);
    assert(res);
}

FrameData FrameDataTaker::getNextGraph()
{
	const string frame = _frameReader->getNextFrame();
    assert( frame.size()>2);
    if(frame.size()<2)  // no file any more
    {
        return FrameData();
    }

	const vector<int> schema = _frameReader->getSchema();
	int pos=0;

	// parse blocks schema
	initBlocksSchema();

	// parse blocks vertices 
	{
		pos = find_nth_endl(frame , 0 , schema[5]+1);
		assert(pos!=-1);
		if( pos==-1) return FrameData();
		parseBlocksVertices(frame.substr(0 , pos+1));
	}
	
	// parse points data
	{
		int tmpPos = pos+1;
		pos = find_nth_endl(frame , pos+1 , (schema[1]+schema[2]+schema[3])*2+schema[3]*2);
		assert(pos!=-1);
		if( pos==-1) return FrameData();
		parsePoints(frame.substr(tmpPos , pos-tmpPos+1));
	}

	// real time for this step
	{
		pos = find_nth_endl(frame , pos+1 , 1);
	}

	// parse block stress
	{
		parseBlocksStress(frame.substr(pos+1 , frame.size()-pos));
	}

	return *_graph;
}

bool FrameDataTaker::initBlocksSchema()
{
	const vector<int> schema = _frameReader->getSchema();
	const string blocksInfo = _frameReader->getBlocksSchema();
	int num=0;
	int pos=0;

	while(pos<=blocksInfo.size())
	{
		pos = blocksInfo.find('\n' , pos) ;
		if(pos==string::npos) break;
		else
		{
			num++;
			pos++;
		}
	}
	assert( num == schema[0]);
	if(num!=schema[0]) return false;


#ifdef DEBUGFRAMEDATATAKER
	char tmpStr[200];
	sprintf(tmpStr , "record num : %d , blocks num : %d\n" , num , schema[0]);
	msg(tmpStr);
#endif

	_graph->blocks.resize(schema[0]);
	parseBlocksSchema( blocksInfo );
	return true;
}

bool FrameDataTaker::parseBlocksSchema(const string& blocksData)
{
	assert(blocksData.size()>0);
	vector<Block>& blocks = _graph->blocks;
	istringstream dataIn(blocksData);
	int blockIdx = 0;
	int t1 , t2;
	char str[200];

	msg("start to parse blocks schema.\n");
	while(blockIdx<blocks.size())
	{
		assert(!dataIn.eof());
		if (dataIn.eof()) return false;

		dataIn>>t1;
		dataIn>>t2;

#ifdef DEBUGFRAMEDATATAKER
		sprintf(str , "%d %d\n" , t1 , t2);
		msg(str);
#endif	
		
		blocks[blockIdx].startNo=t1;
		blocks[blockIdx].endNo=t2;
		blockIdx++;
	}
	msg("blocks schema parsed done.\n");
	return true;
}

bool FrameDataTaker::parseBlocksVertices(const string& blocksData)
{
//     msgString(blocksData);
	assert(blocksData.size()>5);

	istringstream dataIn(blocksData);
	assert( dataIn.good());
	vector<Block>& blocks = _graph->blocks;

	char tmpStr[200];
	double t1 , t2;
	dataIn>>tmpStr>>t1; // <<<step>>> n
    _graph->frameNo = int(t1);

    msg(tmpStr);
	sprintf(tmpStr , "start to parse blocks vertices for frame index : %d.\n" , int(t1));
	msg(tmpStr);
	for(int t=0 ; t<blocks.size() ; t++)
	{
		assert(!dataIn.eof());
		Block& block = blocks[t];
		int size = block.endNo - block.startNo+1;
		block.vertices.resize(size);
		for (int i=0;i<size;i++)
		{
//             sprintf(tmpStr , "frame Index : %d , vertex No. %d\n" ,t , i );
//             msg(tmpStr);
			assert(!dataIn.eof());
			if (dataIn.eof()) return false;
			dataIn>>t1>>t2;
			block.vertices[i].first = t1;
			block.vertices[i].second = t2;

#ifdef DEBUGFRAMEDATATAKER
			sprintf(tmpStr , "%lf %lf\n" , t1 , t2);
			msg(tmpStr);
#endif		
		}
		
		for(int i=0 ; i<8; i++)
		{
			assert(!dataIn.eof());
			if (dataIn.eof()) return false;
			dataIn>>t1;
			block.parameters[i]=t1;
		}

#ifdef DEBUGFRAMEDATATAKER
		const double* paras = block.parameters;
		sprintf(tmpStr , "%lf %lf\n%lf %lf\n%lf %lf\n%lf %lf\n" , paras[0] , paras[1] , paras[2] , paras[3]
			 , paras[4] , paras[5] , paras[6] , paras[7]);
		msg(tmpStr);
#endif	
	
	}
	msg("block vertices parsed done.\n");
	return true;
}

bool FrameDataTaker::parseBlocksStress(const string& blocksStress)
{
	istringstream dataIn(blocksStress);
	vector<Block>& blocks = _graph->blocks;

	char tmpStr[200];
	msg("start to parse blocks vertices.\n");
	for(int t=0 ; t<blocks.size() ; t++)
	{
		double t1 , t2 , t3;
		Block& block = blocks[t];
		assert(!dataIn.eof());
		if (dataIn.eof()) return false;
		dataIn>>t1>>t2>>t3;
		block.stressX = t1;
		block.stressY = t2;
		block.stressXY = t3;

#ifdef DEBUGFRAMEDATATAKER
		sprintf(tmpStr , "%lf %lf %lf\n" ,t1 , t2 , t3);
		msg(tmpStr);
#endif	
	
	}
	msg("block stress parsed done.\n");
	return true;
}

bool FrameDataTaker::parse1Point(istringstream& dataIn , DDAPoint& point)
{
	double t1 , t2 , t3 , t4 , t5;
	assert(!dataIn.eof());
	if (dataIn.eof()) return false;
	dataIn>>t1>>t2>>t3>>t4>>t5;
	point.x=t1;
	point.y=t2;
	point.blockNo=int(t3);
	point.speedX=t4;
	point.speedY=t5;


#ifdef DEBUGFRAMEDATATAKER
	char tmpStr[200];
	sprintf(tmpStr , "%lf %lf %lf\n%lf %lf\n" ,t1 , t2 , t3 , t4 , t5);
	msg(tmpStr);
#endif

	return true;
}

bool FrameDataTaker::parsePoints(const string& pointsData)
{
	istringstream dataIn(pointsData);	
	const vector<int> schema = _frameReader->getSchema();

	char tmpStr[200];
	msg("start to parse points.\n");

	{
		// fixed points
		vector<FixedPoint>& fps = _graph->fixedPoints;
		_graph->fixedPoints.resize(schema[1]);
		for(int i=0; i<schema[1];i++)  // schema[1], fixed points num
		{
			assert(!dataIn.eof());
			if (dataIn.eof()) return false;
			if(!parse1Point(dataIn , fps[i])) return false;
		}
		sprintf(tmpStr , "%d fixed points parsed done.\n" , schema[1]);
		msg(tmpStr);
	}

	{
		// loading points
		vector<LoadingPoint>& lps = _graph->loadingPoints;
		_graph->loadingPoints.resize(schema[2]);
		for(int i=0; i<schema[2];i++)  // schema[2], loading points num
		{
			assert(!dataIn.eof());
			if (dataIn.eof()) return false;
			if(!parse1Point(dataIn , lps[i])) return false;
		}
		sprintf(tmpStr , "%d loading points parsed done.\n" , schema[2]);
		msg(tmpStr);
	}

	{
		// measured points
		vector<MeasuredPoint>& mps = _graph->measuredPoints;
		_graph->measuredPoints.resize(schema[3]);
		for(int i=0; i<schema[3];i++)  // schema[3], measured points num
		{
			assert(!dataIn.eof());
			if (dataIn.eof()) return false;
			if(!parse1Point(dataIn , mps[i])) return false;
		}

		double t1 , t2 , t3 , t4 , t5 , t6;
		for(int i=0; i<schema[3];i++)
		{
			assert(!dataIn.eof());
			if (dataIn.eof()) return false;
			dataIn>>t1>>t2>>t3>>t4>>t5>>t6;
			mps[i].u=t1;
			mps[i].v=t2;
			mps[i].r=t3;
			mps[i].stressX=t4;
			mps[i].stressY=t5;
			mps[i].stressXY=t6;
		}
		sprintf(tmpStr , "%d measured points parsed done.\n" , schema[3]);
		msg(tmpStr);
	}
	return true;
}


