#pragma once

// #include <QtCore/QString>
// #include <Qtcore/QStringList>
// #include <QtCore/QTextStream>
// #include <Qtcore/QFile>
// #include <qtcore/QTextStream>
#include <vector>
#include <utility>
#include <string>
#include <fstream>
#include <stdio.h>
#include <errno.h>

namespace DDAGui{
	//#define DEBUGFRAMEREADER


	using namespace std;
    void msg(const char* str);
    void msgString(const string& str);
// 	void msgQString(QString str);

	typedef std::pair<int , int> FrameNo_Pos;
class FrameReader
{
public:
	FrameReader(void);
	~FrameReader(void);
	
	const string getNextFrame();
	const std::string getBlocksSchema();
	bool setFile( char* filename);
    bool hasNextFrame(){return (_frameIdx==_frames.size()-1)?false:true;}
	const std::vector<int> getSchema();
	const std::vector<double> getWindowInfo();
    bool setCurrentFrameIndex(int frameIndex);

private:
    bool _seekPostionAndcalcFrame(long long pos);
	bool readNewbuffer();
	bool scanFrames();
	bool initAnimation();
	bool initBlocksSchema();
//	inline void updateBufferSize();
	char* refreshBuffer();

private:
	long long _1frameSize;
    long long _firstFramePos;
//     long long _fileSize;
	int _framesNum;
	std::vector<std::pair<int , int>> _frames;
	int _frameIdx;

	char* _buffer;
	char* _bufferPos ;
	long long _bufferSize ;

// 	ifstream _file;
    FILE* _file;
	long long _filePos;
	std::vector<int> _schema;
	std::vector<double> _windowInfo;
	std::string _blocksSchema;
};

} //DDAGui