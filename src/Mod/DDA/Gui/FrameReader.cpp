#include "FrameReader.h"

#include <assert.h>
#include <cmath>
namespace DDAGui{
void msg(const char* str);
// void msgQString(const QString str);
}

using namespace DDAGui;
using namespace std;
//void msg2File(const char* str);



FrameReader::FrameReader(void)
{
    _file = NULL;
	_frameIdx=-1;

	_1frameSize = 1024*200;
	_framesNum = 7;
	_bufferSize = _1frameSize*_framesNum;

	_buffer = new char[4*1024*1024]; // default 4MB
	_frames.clear();
	_bufferPos=0 ;
}

FrameReader::~FrameReader(void)
{
	if(_buffer)
	{
		delete _buffer;
		_buffer = NULL;
	}

    if(_file)
    {
        fclose(_file);
        _file = NULL;
    }
}

bool FrameReader::setFile(char* filename)
{
    _file = fopen(filename , "rb");
//	_file.open(filename , ios::binary| ios::in);
	char str[200];

	if (!_file)
	{
		sprintf(str , "file \'%s\' open failed.\n" , filename);
		msg(str);
		return false;
	}
	
	sprintf(str , "file \'%s\' open successfully.\n" , filename);
	msg(str);
	
	if( !initAnimation()) return false;
	if( !initBlocksSchema()) return false;
	if( !readNewbuffer()) return false;
	if( !scanFrames()) return false;
    assert(_frames.size()>0);
    _1frameSize = _frames[1].second - _frames[0].second;
	return true;
}

char* FrameReader::refreshBuffer()
{
    char *newBuffer;
	if( fabs(double(_1frameSize*(_framesNum+2) - _bufferSize)) > _1frameSize)  // confirm 1 frame's spare space
	{
		_bufferSize = _1frameSize*_framesNum;
		assert(_buffer);
		newBuffer = new char[_1frameSize*(_framesNum+2)];
		msg("buffer refresh done.\n");
	}
	else
    {
        newBuffer = _buffer;
		msg("buffer doesn't need refresh.\n");
    }
    return newBuffer;
}

bool FrameReader::readNewbuffer()
{
//     if (_file.eof()) { msg("input file ends.\n");return false;}
    clearerr(_file);

    int tmpSize = _bufferSize;
    char* newBuffer = refreshBuffer();

	char* p = newBuffer;
	if(_frames.size()>0)   // the last frame may not be complete, concatenate the half-break content to its rest comtent.
	{
		char* start = _buffer + _frames.back().second;
		char* end = _buffer + tmpSize;
		while ( start<end)
		{
			*p = *start;
			start++;
			p++;
		}
	}
    
    if(!feof(_file))
    {
        int readCount = fread(p , sizeof(char) , _bufferSize , _file);
        assert(readCount>=0);
        p+=readCount;
// 	    _file.read(p , _bufferSize);
//        p+=int(_file.gcount());
    }
    *p = '\0';
    _bufferSize = p-newBuffer;

    if(newBuffer!=_buffer)
    {
        delete []_buffer;
        _buffer = newBuffer;
        newBuffer = NULL;
    }

	char str[200];
	sprintf(str , "read content for length : %d\n" , _bufferSize);
	msg(str);
	return _bufferSize>5; // the number is not important
}

const string FrameReader::getNextFrame()
{
	char str[200];

#ifdef DEBUGFRAMEREADER
	sprintf(str ,"current frame index : %d frames size : %d\n" , _frameIdx , _frames.size());
	msg(str);
#endif

	assert(_frameIdx<= int(_frames.size()));
	if(_frameIdx==_frames.size()-1){ msg("file ends. exiting.\n");return string("");}
	_frameIdx++;
	if(_frameIdx==_frames.size()-1)
	{ 
		if(!readNewbuffer()) return string(""); 
		scanFrames();
        _frameIdx = 0;
	}
	
	std::pair<int , int>& info = _frames[_frameIdx];
	sprintf(str , "\tstart to take %dth frame.\n" , info.first);
	msg(str);

    if(_frameIdx<_frames.size()-1)
		return string(_buffer+info.second , _frames[_frameIdx+1].second - info.second);
	else  // the last whole frame
		return string(_buffer+info.second , _bufferSize - info.second);
}
const std::string FrameReader::getBlocksSchema()
{
	return _blocksSchema;
}

bool FrameReader::scanFrames()
{
	_frameIdx=-1;
	char tmpContent[200];
	int lastFrameNo=-1 , lastPos=0;
    _frames.clear();

//     FILE* f = fopen("D:/tmpStore.txt" , "wb");
//     fwrite(_buffer , sizeof(char) , _bufferSize , f);
//     fclose(f);
// 
	for( int i=0 ; i<_bufferSize ; i++ )
	{
		if(_buffer[i]=='<')
		{
			int tmpPos = i;
			int tmpNo = lastFrameNo+1;
			if( i+10<_bufferSize)   // <<<step>>> n
			{
				i+=10;
				assert( _buffer[i]==' ');
				int t=i+1;
				char num[10];
				int j=0;
				while (t<_bufferSize && isdigit(_buffer[t]))  // '<<<step>>> n' may not complete
				{

#ifdef DEBUGFRAMEREADER
					sprintf(tmpContent , "%c" , _buffer[t]);
					msg(tmpContent);
#endif				
					
					num[j++] = _buffer[t++];
				}
				num[j]='\0';
// 				sprintf(tmpContent , "i: %d , t: %d\n" , i , t);
// 				msg(tmpContent);

				tmpNo = atoi(num);
				if (lastFrameNo!=-1)
                    assert(tmpNo <= lastFrameNo+1);

#ifdef DEBUGFRAMEREADER
				sprintf(tmpContent , "\ttmpFrameNo : %s %d <--> lastFrameNo+1 : %d\n" , num ,tmpNo , lastFrameNo+1);
				msg(tmpContent);
#endif		

			}

#ifdef DEBUGFRAMEREADER
			sprintf(tmpContent , "(frame No %d , buffer pos: %d)\n" , tmpNo , tmpPos);
			msg(tmpContent);
#endif		
			
			_frames.push_back(FrameNo_Pos(tmpNo , tmpPos));
			lastFrameNo = tmpNo;
			lastPos = tmpPos;
		}
	}

//     string tmpStr(_buffer , _frames[2].second);


    if(_frames.size()>1)
        _1frameSize = (_frames.back().second -_frames.front().second)/(_frames.size()-1);

	return true;
}

bool FrameReader::initAnimation()
{
	_schema.resize(7);
	double t;
// 	for( int i=0 ; i<7 ; i++)   // 0-7 schema of data.dg ,  7-11 window info of data.dg
// 	{
// 		_file>>t;
// 		_schema[i] = t;
// 	}
    fscanf(_file , "%d %d %d %d %d %d %d\n" , &_schema[0] , &_schema[1] , &_schema[2] , &_schema[3]
                        , &_schema[4] , &_schema[5] , &_schema[6]);

	_windowInfo.resize(4);
// 	for( int i=0 ; i<4 ; i++)   // 0-7 schema of data.dg ,  7-11 window info of data.dg
// 	{
// 		_file>>t;
// 		_windowInfo[i] = t;
// 	}
    fscanf(_file , "%lf %lf %lf %lf\n" , &_windowInfo[0] , &_windowInfo[1] 
                        , &_windowInfo[2] ,  &_windowInfo[3]);


	msg("animation schma get done.\n");
	return true;
}

bool FrameReader::initBlocksSchema()
{
	_blocksSchema.clear();
	char str[201];
//     fgets(str , 200 , _file);// the current character is '\n'
// 	_file.getline(str , 200);// the current character is '\n'
	for(int i=0 ; i<_schema[0] ; i++)
	{
// 		_file.getline(str , 200);
        fgets(str , 200 , _file);
//		if(strlen(str)==0) continue; // the first character is '\n' for this loop.
		int len = strlen(str);
		str[len-1]='\n';
		_blocksSchema.append(str , len);
	}

    _firstFramePos = _ftelli64(_file);
//     _file.seekg(0 , _file.end);
//     _fileSize = _file.tellg();
/*    _file.seekg(_firstFramePos);*/

#ifdef DEBUGFRAMEREADER
	msg(_blocksSchema.c_str());
	sprintf(str , "block schema get done.\n");
	msg(str);
#endif

	return true;
}

const std::vector<int> FrameReader::getSchema()
{
	return _schema;
}

const std::vector<double> FrameReader::getWindowInfo()
{
	return _windowInfo;
}

bool FrameReader::_seekPostionAndcalcFrame(long long pos)
{
    bool res = _fseeki64( _file , pos , 0)!=-1;
    assert(res);
    assert(ferror(_file)==0);

    _frames.clear();
    readNewbuffer();
    return(scanFrames());
}

bool FrameReader::setCurrentFrameIndex(int frameIndex)
{
//#ifdef DEBUGFRAMEREADER
    char tmpStr[200];
    sprintf(tmpStr , "set start frame %d in frame reader\n" , frameIndex);
    msg(tmpStr);
//#endif



//     _file.clear();
    if(frameIndex>=_frames.front().first && frameIndex<_frames.back().first)
    {
        for(int i=0 ; i<_frames.size() ; i++)
            if( frameIndex== _frames[i].first){
                _frameIdx = i-1;
                return true;
            }
    }
    else{
        long long newPos = _firstFramePos;
        if(frameIndex>0)
            newPos += (frameIndex-1)*_1frameSize;

        bool res= _seekPostionAndcalcFrame(newPos);
        assert(res);


        // frameIndex < _frames.front().first
        while( _frames.size()>0 && frameIndex < _frames.front().first){
            int frameDifference = _frames.front().first- frameIndex ;
            newPos -= frameDifference*_1frameSize;
            if( newPos < _firstFramePos )
                newPos = _firstFramePos;
            bool res= _seekPostionAndcalcFrame(newPos);
            assert(res);
        }

        // frameIndex > _frames.back().first
        while( _frames.size()>0 && frameIndex > _frames.back().first-1){
            int frameDifference = frameIndex - _frames.back().first;
            if(frameDifference<2)
                newPos += _1frameSize;
            else
                newPos += (frameDifference-1)*_1frameSize;
            bool res= _seekPostionAndcalcFrame(newPos);
            assert(res);
        }

        assert(frameIndex>=_frames.front().first && frameIndex<_frames.back().first);
        for(int i=0 ; i<_frames.size() ; i++)
            if( frameIndex== _frames[i].first){
                _frameIdx = i-1;
                return true;
            }
    }
    return false;
}
